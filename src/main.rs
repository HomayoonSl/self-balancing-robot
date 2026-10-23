#![no_std]
#![no_main]
#![deny(unsafe_code)]

mod control;
mod drivers;
mod utils;

use core::fmt::Write;

use cortex_m::peripheral::SCB;
use cortex_m_rt::entry;
use embedded_hal::digital::{InputPin, OutputPin};
use stm32f1xx_hal::{adc, pac, prelude::*, timer::Timer};

use control::{apply_cmd, cal_fail_text, calibrate, deadzone_comp, Filter, Wheel};
use drivers::{bmi_init, read_imu, read_raw_angle, BitI2c, Uart};
use utils::{BlackBox, Rec, Ticker};

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    SCB::sys_reset()
}

// ================================================================ Configuration

const CPU_HZ: u32 = 8_000_000;
const BAUD: u32 = 57_600;
const BIT_CYCLES: u32 = CPU_HZ / BAUD;
const I2C_HALF_US: u32 = 3;
const AS_ADDR: u8 = 0x36;
const BMI_ADDR: u8 = 0x69;
const PWM_HZ: u32 = 20_000;
const SLOW_DECAY: bool = true;
const LED_ACTIVE_HIGH: bool = true;

//const BENCH_MODE: bool = false;
//const BENCH_DRIVE: bool = false;
//const BENCH_V_MAX: f32 = 2.5;
//const BENCH_VBAT_MV: u32 = 5500;
const DUMP_ON_STOP: bool = true;

const DT_US: u32 = 5000;
const DT_MS: u32 = 5;
const DT: f32 = 0.005;

const ACC_UP_AXIS: usize = 2;
const ACC_UP_SIGN: f32 = 1.0;
const ACC_FWD_AXIS: usize = 0;
const ACC_FWD_SIGN: f32 = 1.0;
const GYRO_FLIP: bool = false;
const GYRO_PITCH_AXIS: usize = 3 - ACC_UP_AXIS - ACC_FWD_AXIS;
const _: () = assert!(ACC_UP_AXIS < 3 && ACC_FWD_AXIS < 3 && ACC_UP_AXIS != ACC_FWD_AXIS);

const ACC_LSB_PER_G: f32 = 16384.0;
const GYRO_LSB_PER_DPS: f32 = 32.8;
const GYRO_RAD_PER_LSB: f32 = 0.017_453_293 / GYRO_LSB_PER_DPS;

const ENC_SIGN: f32 = -1.0;
const RAD_PER_COUNT: f32 = 6.283_185_3 / 4096.0;

const KP_ACC: f32 = 0.5;
const KI_BIAS: f32 = 0.0625;

const K_THETA: f32 = 33.0;
const K_RATE: f32 = 3.1;
const K_WHEEL: f32 = 0.73;
const K_WHEEL_RATE: f32 = 0.49;
const DZ_POS_V: f32 = 0.40;
const DZ_NEG_V: f32 = 0.30;
const DZ_SOFT_V: f32 = 0.40;
const V_LIMIT_V: f32 = 5.0;

const BOOT_WAIT_MS: u32 = 5000;
const RELEASE_DELAY_MS: u32 = 3000;
const STILL_P2P_DPS: f32 = 20.0;
const ENGAGE_RAD: f32 = 0.17;
const FALL_RAD: f32 = 0.50;
const RUNAWAY_RATE: f32 = 40.0;
const RUNAWAY_TICKS: u32 = 60;
const WHEEL_LIMIT_RAD: f32 = 70.0;
const VBAT_MIN_V: f32 = 4.8;

//const VDDA_MV: u32 = 3300;
const ADC_TO_V: f32 = (3.3 / 4095.0) * 4.0;
const BB_LEN: usize = 400;

// ================================================================ Helpers

fn delay_us(us: u32) {
    cortex_m::asm::delay(us * (CPU_HZ / 1_000_000));
}

fn delay_ms(ms: u32) {
    for _ in 0..ms {
        delay_us(1000);
    }
}

fn drive<P: OutputPin>(p: &mut P, high: bool) {
    let _ = if high { p.set_high() } else { p.set_low() };
}

fn level<P: InputPin>(p: &mut P) -> bool {
    p.is_high().unwrap_or(false)
}

fn gyro_sign() -> f32 {
    let cyclic = (ACC_FWD_AXIS + 3 - ACC_UP_AXIS) % 3 == 1;
    let parity = if cyclic { 1.0 } else { -1.0 };
    let flip = if GYRO_FLIP { -1.0 } else { 1.0 };
    ACC_UP_SIGN * ACC_FWD_SIGN * parity * flip
}

fn vbat_avg(read_adc: &mut impl FnMut() -> u16, n: u32) -> f32 {
    let mut s = 0u32;
    for _ in 0..n {
        s += read_adc() as u32;
    }
    let avg_raw = s as f32 / n as f32;
    avg_raw * ADC_TO_V
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Outcome {
    Fell,
    Runaway,
    Travel,
    Sensor,
    Fault,
    Battery,
}

impl Outcome {
    fn name(self) -> &'static str {
        match self {
            Outcome::Fell => "pitch limit",
            Outcome::Runaway => "wheel speed runaway",
            Outcome::Travel => "wheel travel limit",
            Outcome::Sensor => "sensor read failures",
            Outcome::Fault => "DRV8833 nFAULT",
            Outcome::Battery => "battery below minimum",
        }
    }
}

struct RunStats {
    outcome: Outcome,
    engaged_ticks: u32,
    overruns: u32,
    max_busy_us: u32,
    max_pitch_cd: i32,
}

// ================================================================ Control Routine

#[allow(clippy::too_many_arguments)]
fn run_balance<S1, D1, S2, D2, S, F>(
    imu: &mut BitI2c<S1, D1>,
    enc: &mut BitI2c<S2, D2>,
    nsleep: &mut S,
    nfault: &mut F,
    set1: &mut impl FnMut(u16),
    set2: &mut impl FnMut(u16),
    read_adc: &mut impl FnMut() -> u16,
    max_duty: u16,
    cal: &control::Calib,
    bb: &mut BlackBox<BB_LEN>,
) -> RunStats
where
    S1: drivers::Line,
    D1: drivers::Line,
    S2: drivers::Line,
    D2: drivers::Line,
    S: OutputPin,
    F: InputPin,
{
    let cyc_per_us = CPU_HZ / 1_000_000;
    let mut stats = RunStats {
        outcome: Outcome::Fell,
        engaged_ticks: 0,
        overruns: 0,
        max_busy_us: 0,
        max_pitch_cd: 0,
    };

    let mut raw = match read_raw_angle(enc, AS_ADDR, I2C_HALF_US) {
        Some(r) => r,
        None => {
            stats.outcome = Outcome::Sensor;
            return stats;
        }
    };

    let mut filt = Filter::new(
        cal,
        gyro_sign(),
        KP_ACC,
        KI_BIAS,
        DT,
        GYRO_RAD_PER_LSB,
        ACC_UP_SIGN,
        ACC_LSB_PER_G,
        ACC_FWD_SIGN,
        GYRO_PITCH_AXIS,
        ACC_UP_AXIS,
        ACC_FWD_AXIS,
    );
    let mut wheel = Wheel::new(raw, ENC_SIGN, RAD_PER_COUNT, DT);
    let mut vbat = vbat_avg(read_adc, 8);

    set1(0);
    set2(0);
    drive(nsleep, true);
    delay_ms(5);

    let mut tk = Ticker::new(DT_US * cyc_per_us);
    let mut n: u32 = 0;
    let mut engaged = false;
    let (mut theta, mut rate) = (0.0f32, 0.0f32);
    let mut imu_err = 0u32;
    let mut enc_err = 0u32;
    let mut run_cnt = 0u32;

    let outcome = loop {
        if tk.wait() {
            stats.overruns += 1;
        }
        let t_in = cortex_m::peripheral::DWT::cycle_count();
        n += 1;

        match read_raw_angle(enc, AS_ADDR, I2C_HALF_US) {
            Some(r) => {
                raw = r;
                enc_err = 0;
            }
            None => enc_err += 1,
        }
        let (mut wang, mut wrate) = wheel.update(raw);
        match read_imu(imu, BMI_ADDR, I2C_HALF_US) {
            Some(s) => {
                imu_err = 0;
                let (th, ra) = filt.update(&s);
                theta = th;
                rate = ra;
            }
            None => imu_err += 1,
        }
        if imu_err > 3 || enc_err > 3 {
            break Outcome::Sensor;
        }

        if !engaged && n * DT_MS >= RELEASE_DELAY_MS {
            if theta.abs() < ENGAGE_RAD {
                engaged = true;
                wheel = Wheel::new(raw, ENC_SIGN, RAD_PER_COUNT, DT);
                wang = 0.0;
                wrate = 0.0;
            } else if theta.abs() > FALL_RAD {
                break Outcome::Fell;
            }
        }

        if engaged {
            stats.engaged_ticks += 1;
            if theta.abs() > FALL_RAD {
                break Outcome::Fell;
            }
            if wrate.abs() > RUNAWAY_RATE {
                run_cnt += 1;
            } else {
                run_cnt = 0;
            }
            if run_cnt > RUNAWAY_TICKS {
                break Outcome::Runaway;
            }
            if wang.abs() > WHEEL_LIMIT_RAD {
                break Outcome::Travel;
            }
            if !level(nfault) {
                break Outcome::Fault;
            }
            if vbat < VBAT_MIN_V {
                break Outcome::Battery;
            }

            let v_lin = K_THETA * theta + K_RATE * rate + K_WHEEL * wang + K_WHEEL_RATE * wrate;
            let v = deadzone_comp(v_lin, DZ_POS_V, DZ_NEG_V, DZ_SOFT_V).clamp(-V_LIMIT_V, V_LIMIT_V);
            apply_cmd(set1, set2, max_duty, (v * 1000.0) as i32, (vbat * 1000.0) as u32, SLOW_DECAY);

            let pitch_cd = theta * 5729.578;
            let pa = pitch_cd.abs() as i32;
            if pa > stats.max_pitch_cd {
                stats.max_pitch_cd = pa;
            }
            bb.push(Rec {
                pitch_cd: pitch_cd as i16,
                rate_dd: (rate * 572.958) as i16,
                wheel_cr: (wang * 100.0) as i16,
                wheel_rate_cr: (wrate * 100.0) as i16,
                v_mv: (v * 1000.0) as i16,
            });
        }

        if n % 10 == 0 {
            let vm = read_adc() as f32 * ADC_TO_V;
            vbat += 0.3 * (vm - vbat);
        }
        let busy = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(t_in) / cyc_per_us;
        if busy > stats.max_busy_us {
            stats.max_busy_us = busy;
        }
    };

    set1(0);
    set2(0);
    drive(nsleep, false);
    stats.outcome = outcome;
    stats
}

// ================================================================ Entry Point

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    let mut rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain(&mut rcc);
    let mut gpioa = dp.GPIOA.split(&mut rcc);
    let mut gpiob = dp.GPIOB.split(&mut rcc);
    let mut gpioc = dp.GPIOC.split(&mut rcc);

    let mut pc13 = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    let mut pa3 = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
    let mut status_led = |on: bool| drive(&mut pc13, !on);
    let mut ready_led = |on: bool| drive(&mut pa3, on == LED_ACTIVE_HIGH);
    status_led(false);
    ready_led(false);

    let mut nsleep = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);
    drive(&mut nsleep, false);
    let mut nfault = gpiob.pb1.into_floating_input(&mut gpiob.crl);

    let mut tx = gpioa.pa9.into_push_pull_output(&mut gpioa.crh);
    drive(&mut tx, true);

    let pa6 = gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl);
    let pa7 = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let pwm = Timer::new(dp.TIM3, &mut rcc).pwm_hz((pa6, pa7), &mut afio.mapr, PWM_HZ.Hz());
    let max_duty: u16 = pwm.get_max_duty();
    let (mut ch_in1, mut ch_in2) = pwm.split();
    ch_in1.set_duty(0);
    ch_in2.set_duty(0);
    ch_in1.enable();
    ch_in2.enable();
    let mut set1 = |d: u16| ch_in1.set_duty(d);
    let mut set2 = |d: u16| ch_in2.set_duty(d);

    let mut adc1 = adc::Adc::new(dp.ADC1, &mut rcc);
    adc1.set_sample_time(adc::SampleTime::T_239);
    let mut ch0 = gpioa.pa0.into_analog(&mut gpioa.crl);
    let mut read_adc = || -> u16 { adc1.read(&mut ch0).unwrap_or(0) };

    let scl1 = gpiob.pb6.into_open_drain_output(&mut gpiob.crl);
    let sda1 = gpiob.pb7.into_open_drain_output(&mut gpiob.crl);
    let scl2 = gpiob.pb10.into_open_drain_output(&mut gpiob.crh);
    let sda2 = gpiob.pb11.into_open_drain_output(&mut gpiob.crh);
    let mut imu = BitI2c::new(scl1, sda1, I2C_HALF_US, CPU_HZ);
    let mut enc = BitI2c::new(scl2, sda2, I2C_HALF_US, CPU_HZ);

    let sck = gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh);
    let miso = gpiob.pb14.into_floating_input(&mut gpiob.crh);
    let mosi = gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh);
    let mut sd_cs = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
    sd_cs.set_high();

    let spi2 = stm32f1xx_hal::spi::Spi::new(
        dp.SPI2,
        (Some(sck), Some(miso), Some(mosi)),
        stm32f1xx_hal::spi::Mode {
            polarity: stm32f1xx_hal::spi::Polarity::IdleLow,
            phase: stm32f1xx_hal::spi::Phase::CaptureOnFirstTransition,
        },
        400.kHz(),
        &mut rcc,
    );

    let delay = cp.SYST.delay(&rcc.clocks);
    let spi_device = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi2, sd_cs).unwrap();
    let mut sd_logger = utils::SdLogger::<_, _, BB_LEN>::new(spi_device, delay);

    delay_ms(100);
    let mut con = Uart::new(tx, BIT_CYCLES);
    let _ = write!(con, "\r\n# === Modular Self-balancing controller v1 ===\r\n");

    if !bmi_init(&mut imu, BMI_ADDR, I2C_HALF_US) {
        let _ = con.write_str("# BMI160 init failed\r\n");
        loop {
            status_led(true);
            delay_ms(100);
            status_led(false);
            delay_ms(100);
        }
    }
    let _ = con.write_str("# BMI160 configured\r\n");
    let vb0 = vbat_avg(&mut read_adc, 16);
    let _ = write!(con, "# VBAT {:.3} V\r\n", vb0);
    let mut bb = BlackBox::<BB_LEN>::new();
    loop {
        let _ = con.write_str("# hold the robot upright and still\r\n");
        for i in 0..(BOOT_WAIT_MS / 250) {
            status_led(i % 2 == 0);
            delay_ms(250);
        }
        status_led(false);

        let cal = match calibrate(
            &mut imu,
            BMI_ADDR,
            I2C_HALF_US,
            DT_US,
            CPU_HZ,
            STILL_P2P_DPS,
            GYRO_LSB_PER_DPS,
            ACC_UP_AXIS,
            ACC_UP_SIGN,
            ACC_FWD_AXIS,
            ACC_FWD_SIGN,
            ACC_LSB_PER_G,
            GYRO_PITCH_AXIS,
            gyro_sign(),
            GYRO_RAD_PER_LSB,
        ) {
            Ok(c) => c,
            Err(e) => {
                let _ = write!(con, "# calibration failed: {}\r\n", cal_fail_text(e));
                for _ in 0..10 {
                    status_led(true);
                    delay_ms(100);
                    status_led(false);
                    delay_ms(100);
                }
                continue;
            }
        };

        ready_led(true);
        let _ = write!(con, "# READY: release the robot within {} ms\r\n", RELEASE_DELAY_MS);
        let stats = run_balance(
            &mut imu,
            &mut enc,
            &mut nsleep,
            &mut nfault,
            &mut set1,
            &mut set2,
            &mut read_adc,
            max_duty,
            &cal,
            &mut bb,
        );
        ready_led(false);

        let _ = write!(
            con,
            "# stop: {} | engaged {} ms | overruns {} | max loop {} us | max pitch {} cdeg\r\n",
            stats.outcome.name(),
            stats.engaged_ticks * DT_MS,
            stats.overruns,
            stats.max_busy_us,
            stats.max_pitch_cd
        );
        
        if DUMP_ON_STOP && stats.engaged_ticks > 0 {
            bb.dump(&mut con, DT_MS);

            let _ = con.write_str("# dumping logs to MicroSD...\r\n");
            match sd_logger.dump_blackbox(&bb, DT_MS) {
                Ok(_) => {
                    let _ = con.write_str("# SD write successful\r\n");
                }
                Err(e) => {
                    let _ = write!(con, "# SD write failed: {}\r\n", e);
                }
            }
        }
        bb = BlackBox::new();
        delay_ms(1000);
    }
}
