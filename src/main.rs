#![no_std]
#![no_main]
#![deny(unsafe_code)]

mod control;
mod drivers;
mod utils;

use cortex_m::peripheral::SCB;
use cortex_m_rt::entry;
use embedded_hal::digital::{InputPin, OutputPin};
use stm32f1xx_hal::{adc, pac, prelude::*, timer::Timer};

use control::{apply_cmd, calibrate, deadzone_comp, Filter, Wheel};
use drivers::{bmi_init, read_imu, read_raw_angle, BitI2c};
use utils::{Rec, StreamLogger, Ticker};

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    SCB::sys_reset()
}

// ================================================================ Configuration

const CPU_HZ: u32 = 48_000_000;
const I2C_HALF_US: u32 = 3;
const AS_ADDR: u8 = 0x36;
const BMI_ADDR: u8 = 0x69;
const PWM_HZ: u32 = 20_000;
const SLOW_DECAY: bool = true;
const LED_ACTIVE_HIGH: bool = true;

const DT_US: u32 = 5000;
const DT_MS: u32 = 5;
const DT: f32 = 0.005;

const ACC_UP_AXIS: usize = 2;
const ACC_UP_SIGN: f32 = 1.0;
const ACC_FWD_AXIS: usize = 0;
const ACC_FWD_SIGN: f32 = 1.0;
const GYRO_FLIP: bool = false;
const GYRO_PITCH_AXIS: usize = 3 - ACC_UP_AXIS - ACC_FWD_AXIS;

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
const ADC_TO_V: f32 = (3.3 / 4095.0) * 4.0;

// ظرفیت بافر حلقوی رم برای جذب تاخیرهای احتمالی کارت SD (حدود ۲.۵ ثانیه)
const FIFO_LEN: usize = 512;

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

// بافر صف حلقوی
struct LogQueue<const N: usize> {
    buf: [Rec; N],
    head: usize,
    tail: usize,
}

impl<const N: usize> LogQueue<N> {
    const fn new() -> Self {
        Self {
            buf: [utils::blackbox::REC_ZERO; N],
            head: 0,
            tail: 0,
        }
    }

    fn push(&mut self, r: Rec) {
        let next = (self.head + 1) % N;
        if next != self.tail {
            self.buf[self.head] = r;
            self.head = next;
        }
    }

    fn pop(&mut self) -> Option<Rec> {
        if self.head == self.tail {
            None
        } else {
            let r = self.buf[self.tail];
            self.tail = (self.tail + 1) % N;
            Some(r)
        }
    }
}

// ================================================================ Entry Point

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    let mut flash = dp.FLASH.constrain();

    // Configure 48 MHz SYSCLK from 8 MHz HSE crystal in stm32f1xx-hal 0.11.0
    let mut rcc = dp.RCC.freeze(
        stm32f1xx_hal::rcc::Config::hse(8.MHz())
            .sysclk(48.MHz())
            .pclk1(24.MHz()),
        &mut flash.acr,
    );
    let clocks = rcc.clocks;
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

    // فرکانس SPI روی ۶ مگاهرتز برای نوشتن سریع روی کارت SD
    let spi2 = stm32f1xx_hal::spi::Spi::new(
        dp.SPI2,
        (Some(sck), Some(miso), Some(mosi)),
        stm32f1xx_hal::spi::Mode {
            polarity: stm32f1xx_hal::spi::Polarity::IdleLow,
            phase: stm32f1xx_hal::spi::Phase::CaptureOnFirstTransition,
        },
        6.MHz(),
        &mut rcc,
    );

    let delay = cp.SYST.delay(&clocks);
    let spi_device = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi2, sd_cs).unwrap();
    let mut logger = StreamLogger::new(spi_device, delay);

    if !bmi_init(&mut imu, BMI_ADDR, I2C_HALF_US) {
        loop {
            status_led(true);
            delay_ms(100);
            status_led(false);
            delay_ms(100);
        }
    }

    // چشمک‌زن اولیه انتظار برای استقرار ربات
    for i in 0..(BOOT_WAIT_MS / 250) {
        status_led(i % 2 == 0);
        delay_ms(250);
    }
    status_led(false);

    // کالیبراسیون اولیه سنسورها
    let cal = loop {
        match calibrate(
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
            Ok(c) => break c,
            Err(_) => {
                for _ in 0..6 {
                    status_led(true);
                    delay_ms(80);
                    status_led(false);
                    delay_ms(80);
                }
            }
        }
    };

    // ایجاد فایل CSV روی کارت حافظه پیش از شروع کنترل
    if logger.open_session().is_err() {
        loop {
            status_led(true);
            delay_ms(500);
            status_led(false);
            delay_ms(500);
        }
    }

    ready_led(true);

    let cyc_per_us = CPU_HZ / 1_000_000;
    let mut raw = read_raw_angle(&mut enc, AS_ADDR, I2C_HALF_US).unwrap_or(0);
    let mut filt = Filter::new(
        &cal,
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
    let mut vbat = vbat_avg(&mut read_adc, 8);

    set1(0);
    set2(0);
    drive(&mut nsleep, true);
    delay_ms(5);

    let mut tk = Ticker::new(DT_US * cyc_per_us);
    let mut n: u32 = 0;
    let mut logged_rows: u32 = 0;
    let mut engaged = false;
    let (mut theta, mut rate) = (0.0f32, 0.0f32);
    let mut run_cnt = 0u32;
    let mut queue = LogQueue::<FIFO_LEN>::new();

    // حلقه اصلی تعادل و ثبت داده
    loop {
        tk.wait();
        n += 1;

        if let Some(r) = read_raw_angle(&mut enc, AS_ADDR, I2C_HALF_US) {
            raw = r;
        }
        let (mut wang, mut wrate) = wheel.update(raw);

        if let Some(s) = read_imu(&mut imu, BMI_ADDR, I2C_HALF_US) {
            let (th, ra) = filt.update(&s);
            theta = th;
            rate = ra;
        }

        // بررسی شرط ورود اولیه به تعادل
        if !engaged && n * DT_MS >= RELEASE_DELAY_MS {
            if theta.abs() < ENGAGE_RAD {
                engaged = true;
                wheel = Wheel::new(raw, ENC_SIGN, RAD_PER_COUNT, DT);
                wang = 0.0;
                wrate = 0.0;
            } else if theta.abs() > FALL_RAD {
                break;
            }
        }

        if engaged {
            // شرط پایان: افتادن یا برداشتن ربات از روی زمین (Runaway چرخ‌ها)
            if theta.abs() > FALL_RAD {
                break;
            }
            if wrate.abs() > RUNAWAY_RATE {
                run_cnt += 1;
            } else {
                run_cnt = 0;
            }
            if run_cnt > RUNAWAY_TICKS || wang.abs() > WHEEL_LIMIT_RAD || !level(&mut nfault) || vbat < VBAT_MIN_V {
                break;
            }

            let v_lin = K_THETA * theta + K_RATE * rate + K_WHEEL * wang + K_WHEEL_RATE * wrate;
            let v = deadzone_comp(v_lin, DZ_POS_V, DZ_NEG_V, DZ_SOFT_V).clamp(-V_LIMIT_V, V_LIMIT_V);
            apply_cmd(&mut set1, &mut set2, max_duty, (v * 1000.0) as i32, (vbat * 1000.0) as u32, SLOW_DECAY);

            let pitch_cd = (theta * 5729.578) as i16;
            let rec = Rec {
                pitch_cd,
                rate_dd: (rate * 572.958) as i16,
                wheel_cr: (wang * 100.0) as i16,
                wheel_rate_cr: (wrate * 100.0) as i16,
                v_mv: (v * 1000.0) as i16,
            };

            queue.push(rec);

            // در صورتی که داده‌ای در صف باشد، به صورت پیوسته روی فایل SD ریخته می‌شود
            if let Some(item) = queue.pop() {
                let _ = logger.log_record(logged_rows, &item);
                logged_rows += 1;
            }
        }

        if n % 10 == 0 {
            let vm = read_adc() as f32 * ADC_TO_V;
            vbat += 0.3 * (vm - vbat);
        }
    }

    // متوقف کردن آنی موتورها
    set1(0);
    set2(0);
    drive(&mut nsleep, false);
    ready_led(false);

    // تخلیه داده‌های باقیمانده در صف رم و بستن نهایی فایل روی کارت SD
    while let Some(item) = queue.pop() {
        let _ = logger.log_record(logged_rows, &item);
        logged_rows += 1;
    }
    logger.close_session();

    // علامت پایان ایمن: چشمک‌زن سریع ممتد به معنی اتمام نوشتن و امکان قطع تغذیه
    loop {
        status_led(true);
        delay_ms(80);
        status_led(false);
        delay_ms(80);
    }
}
