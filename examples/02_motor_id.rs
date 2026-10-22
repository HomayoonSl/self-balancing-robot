#![no_std]
#![no_main]
#![deny(unsafe_code)]

use core::fmt::{self, Write};

use cortex_m::peripheral::{DWT, SCB};
use cortex_m_rt::entry;
use embedded_hal::digital::{InputPin, OutputPin};
use stm32f1xx_hal::{adc, pac, prelude::*, timer::Timer};

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    SCB::sys_reset()
}

// ============================================================ configuration
const CPU_HZ: u32 = 8_000_000;
const BAUD: u32 = 115_200;
const BIT_CYCLES: u32 = (CPU_HZ + BAUD / 2) / BAUD; // 69 → 115942 baud (0.65 %)

const I2C_HALF_US: u32 = 3;
const AS_ADDR: u8 = 0x36;

const PWM_HZ: u32 = 20_000;
const SAMPLE_PERIOD_US: u32 = 10_000;   // 100 Hz
const TEST_DURATION_MS: u32 = 120_000;  // hard cap

const VDDA_MV: u32 = 3300;
const DIV_R1_K: u32 = 20;
const DIV_R2_K: u32 = 10;

const TEST_ID: u8 = 2;
const RUN_ID: u8 = 1;

// Chirp parameters
const CHIRP_F0_HZ: f32 = 0.1;
const CHIRP_F1_HZ: f32 = 15.0;
const CHIRP_AMP_MV: i32 = 400;

// Doublet
const DBL_AMP_MV: i32 = 800;
const DBL_PULSE_MS: u32 = 80;
const DBL_GAP_MS: u32 = 400;
const DBL_REPS: u32 = 5;

// Fine staircase (14 levels, mV)
const FINE_MV: [i32; 14] = [
    100, 150, 200, 250, 300, 400, 500,
    650, 800, 1000, 1250, 1500, 2000, 3000,
];
const FINE_STEP_MS: u32 = 1_200;

// Log staircase (10 levels, mV)
const LOG_MV: [i32; 10] = [100, 160, 250, 400, 630, 1000, 1600, 2500, 4000, 5000];
const LOG_DRIVE_MS: u32 = 800;
const LOG_COAST_MS: u32 = 400;

// Phase boundaries (ms, cumulative)
const T_P0: u32 = 3_000;
const T_P1: u32 = 19_800;
const T_P2: u32 = 36_600;
const T_P3: u32 = 50_600;
const T_P4: u32 = 74_600;
const T_P5: u32 = 96_600;
const T_P6: u32 = 104_600;
const T_P7: u32 = 116_600;
const T_END: u32 = 118_600;

// ============================================================ timing
#[inline(always)]
fn wait_since(start: u32, cycles: u32) {
    while DWT::cycle_count().wrapping_sub(start) < cycles {}
}
fn delay_us(us: u32) {
    wait_since(DWT::cycle_count(), us * (CPU_HZ / 1_000_000));
}
fn delay_ms(ms: u32) {
    for _ in 0..ms {
        delay_us(1000);
    }
}

// ============================================================ pin helpers
#[inline(always)]
fn drive<P: OutputPin>(p: &mut P, high: bool) {
    let _ = if high { p.set_high() } else { p.set_low() };
}
#[inline(always)]
fn level<P: InputPin>(p: &mut P) -> bool {
    p.is_high().unwrap_or(false)
}

trait Line: OutputPin + InputPin {}
impl<T: OutputPin + InputPin> Line for T {}

fn blink_forever<P: OutputPin>(led: &mut P, half_ms: u32) -> ! {
    loop {
        drive(led, false);
        delay_ms(half_ms);
        drive(led, true);
        delay_ms(half_ms);
    }
}

// ============================================================ UART
struct Uart<TX: OutputPin> {
    tx: TX,
}
impl<TX: OutputPin> Write for Uart<TX> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for b in s.bytes() {
            let start = DWT::cycle_count();
            drive(&mut self.tx, false);
            wait_since(start, BIT_CYCLES);
            for i in 0..8u32 {
                drive(&mut self.tx, (b >> i) & 1 == 1);
                wait_since(start, BIT_CYCLES * (i + 2));
            }
            drive(&mut self.tx, true);
            wait_since(start, BIT_CYCLES * 10);
        }
        Ok(())
    }
}

fn wait_start_bit<P: InputPin>(rx: &mut P, timeout_ms: u32) -> bool {
    let start = DWT::cycle_count();
    let timeout_cycles = timeout_ms * 1000 * (CPU_HZ / 1_000_000);
    while DWT::cycle_count().wrapping_sub(start) < timeout_cycles {
        if !level(rx) {
            return true;
        }
        delay_us(1);
    }
    false
}

// ============================================================ I2C
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum I2cError {
    SclStuckLow,
    Nack,
}

struct BitI2c<SCL: Line, SDA: Line> {
    scl: SCL,
    sda: SDA,
}

#[inline(always)]
fn t() {
    delay_us(I2C_HALF_US);
}

impl<SCL: Line, SDA: Line> BitI2c<SCL, SDA> {
    fn new(mut scl: SCL, mut sda: SDA) -> Self {
        drive(&mut scl, true);
        drive(&mut sda, true);
        Self { scl, sda }
    }
    fn scl_high(&mut self) -> Result<(), I2cError> {
        drive(&mut self.scl, true);
        let mut waited_us = 0u32;
        while !level(&mut self.scl) {
            if waited_us >= 2000 {
                return Err(I2cError::SclStuckLow);
            }
            delay_us(10);
            waited_us += 10;
        }
        Ok(())
    }
    fn start(&mut self) -> Result<(), I2cError> {
        drive(&mut self.sda, true);
        t();
        self.scl_high()?;
        t();
        drive(&mut self.sda, false);
        t();
        drive(&mut self.scl, false);
        t();
        Ok(())
    }
    fn stop(&mut self) -> Result<(), I2cError> {
        drive(&mut self.sda, false);
        t();
        self.scl_high()?;
        t();
        drive(&mut self.sda, true);
        t();
        Ok(())
    }
    fn write_byte(&mut self, b: u8) -> Result<bool, I2cError> {
        for i in (0..8).rev() {
            drive(&mut self.sda, (b >> i) & 1 == 1);
            t();
            self.scl_high()?;
            t();
            drive(&mut self.scl, false);
            t();
        }
        drive(&mut self.sda, true);
        t();
        self.scl_high()?;
        t();
        let ack = !level(&mut self.sda);
        drive(&mut self.scl, false);
        t();
        Ok(ack)
    }
    fn read_byte(&mut self, ack: bool) -> Result<u8, I2cError> {
        drive(&mut self.sda, true);
        let mut v = 0u8;
        for _ in 0..8 {
            t();
            self.scl_high()?;
            t();
            v = (v << 1) | (level(&mut self.sda) as u8);
            drive(&mut self.scl, false);
            t();
        }
        drive(&mut self.sda, !ack);
        t();
        self.scl_high()?;
        t();
        drive(&mut self.scl, false);
        t();
        drive(&mut self.sda, true);
        Ok(v)
    }
    fn read_regs(&mut self, addr: u8, reg: u8, buf: &mut [u8]) -> Result<(), I2cError> {
        let r = self.read_regs_inner(addr, reg, buf);
        let _ = self.stop();
        r
    }
    fn read_regs_inner(&mut self, addr: u8, reg: u8, buf: &mut [u8]) -> Result<(), I2cError> {
        self.start()?;
        if !self.write_byte(addr << 1)? {
            return Err(I2cError::Nack);
        }
        if !self.write_byte(reg)? {
            return Err(I2cError::Nack);
        }
        self.start()?;
        if !self.write_byte((addr << 1) | 1)? {
            return Err(I2cError::Nack);
        }
        let n = buf.len();
        for i in 0..n {
            buf[i] = self.read_byte(i + 1 < n)?;
        }
        Ok(())
    }
}

fn read_status_angle<SCL: Line, SDA: Line>(
    bus: &mut BitI2c<SCL, SDA>,
) -> Result<(u8, u16), I2cError> {
    let mut d = [0u8; 3];
    bus.read_regs(AS_ADDR, 0x0B, &mut d)?;
    Ok((d[0], (((d[1] & 0x0F) as u16) << 8) | d[2] as u16))
}

fn read_agc_mag<SCL: Line, SDA: Line>(bus: &mut BitI2c<SCL, SDA>) -> Result<(u8, u16), I2cError> {
    let mut d = [0u8; 3];
    bus.read_regs(AS_ADDR, 0x1A, &mut d)?;
    Ok((d[0], (((d[1] & 0x0F) as u16) << 8) | d[2] as u16))
}

// ============================================================ battery
fn adc_to_mv(adc: u32) -> u32 {
    adc * VDDA_MV * (DIV_R1_K + DIV_R2_K) / (DIV_R2_K * 4095)
}
fn vbat_mv_avg(read_adc: &mut impl FnMut() -> u16, n: u32) -> u32 {
    let mut s = 0u32;
    for _ in 0..n {
        s += read_adc() as u32;
    }
    adc_to_mv(s / n)
}

// ============================================================ motor modes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum MotorMode {
    Brake = 0,
    Coast = 1,
    DriveFwd = 2,
    DriveRev = 3,
}

fn apply_brake(set1: &mut impl FnMut(u16), set2: &mut impl FnMut(u16), max: u16) {
    set1(max);
    set2(max);
}
fn apply_coast(set1: &mut impl FnMut(u16), set2: &mut impl FnMut(u16)) {
    set1(0);
    set2(0);
}
fn apply_drive(
    set1: &mut impl FnMut(u16),
    set2: &mut impl FnMut(u16),
    max: u16,
    cmd_mv: i32,
    vbat_mv: u32,
) -> (u16, MotorMode) {
    let mag_mv = cmd_mv.unsigned_abs() as u64;
    let mut mag = (mag_mv * max as u64 / vbat_mv.max(1) as u64) as u32;
    if mag > max as u32 {
        mag = max as u32;
    }
    let m = mag as u16;
    if cmd_mv >= 0 {
        set1(max);
        set2(max - m);
        (m, MotorMode::DriveFwd)
    } else {
        set2(max);
        set1(max - m);
        (m, MotorMode::DriveRev)
    }
}

// ============================================================ state
struct ChirpState {
    initialized: bool,
    e: f32,
    phase: f32,
    exp_k_dt: f32,
    phase_scale: f32,
}
impl ChirpState {
    const fn new() -> Self {
        Self {
            initialized: false,
            e: 1.0,
            phase: 0.0,
            exp_k_dt: 1.0,
            phase_scale: 0.0,
        }
    }
    fn start(&mut self, dur_ms: u32) {
        let dt_s = SAMPLE_PERIOD_US as f32 * 1e-6;
        let t_tot = dur_ms as f32 * 1e-3;
        let k = libm::logf(CHIRP_F1_HZ / CHIRP_F0_HZ) / t_tot;
        // Incremental: e_{n+1} = e_n * exp(k*dt)  → only 1 sinf per sample.
        self.exp_k_dt = libm::expf(k * dt_s);
        self.phase_scale = 2.0 * core::f32::consts::PI * CHIRP_F0_HZ * dt_s;
        self.e = 1.0;
        self.phase = 0.0;
        self.initialized = true;
    }
    fn next_mv(&mut self) -> i32 {
        self.e *= self.exp_k_dt;
        self.phase += self.phase_scale * self.e;
        (CHIRP_AMP_MV as f32 * libm::sinf(self.phase)) as i32
    }
}

struct SysIdState {
    chirp: ChirpState,
}
impl SysIdState {
    const fn new() -> Self {
        Self { chirp: ChirpState::new() }
    }
}

// ============================================================ phase machine
fn phase_state(
    elapsed_ms: u32,
    vbat_mv: u32,
    st: &mut SysIdState,
) -> (i32, MotorMode, u8, u8) {
    if elapsed_ms < T_P0 {
        // Phase 0: REST — 1.5s brake, then 1.5s coast baseline
        if elapsed_ms < 1_500 {
            (0, MotorMode::Brake, 0, 0)
        } else {
            (0, MotorMode::Coast, 0, 1)
        }
    } else if elapsed_ms < T_P1 {
        // Phase 1: Fine forward staircase
        let idx = ((elapsed_ms - T_P0) / FINE_STEP_MS) as usize;
        let amp = FINE_MV[idx.min(13)];
        (amp, MotorMode::DriveFwd, 1, idx as u8)
    } else if elapsed_ms < T_P2 {
        // Phase 2: Fine reverse staircase
        let idx = ((elapsed_ms - T_P1) / FINE_STEP_MS) as usize;
        let amp = FINE_MV[idx.min(13)];
        (-amp, MotorMode::DriveRev, 2, idx as u8)
    } else if elapsed_ms < T_P3 {
        // Phase 3: Dynamic step transients
        let sub = elapsed_ms - T_P2;
        let half_v = (vbat_mv / 2) as i32;
        let eight_v = (vbat_mv * 8 / 10) as i32;
        let six_v = (vbat_mv * 6 / 10) as i32;
        if sub < 3_000 {
            (half_v, MotorMode::DriveFwd, 3, 0)
        } else if sub < 5_000 {
            (0, MotorMode::Brake, 3, 1)
        } else if sub < 8_000 {
            (eight_v, MotorMode::DriveFwd, 3, 2)
        } else if sub < 10_000 {
            (0, MotorMode::Brake, 3, 3)
        } else if sub < 13_000 {
            (-six_v, MotorMode::DriveRev, 3, 4)
        } else {
            (0, MotorMode::Brake, 3, 5)
        }
    } else if elapsed_ms < T_P4 {
        // Phase 4: Log-stair with coast (2 passes × 10 levels)
        let sub = elapsed_ms - T_P3;
        let step_ms = LOG_DRIVE_MS + LOG_COAST_MS;
        let pass_idx = sub / (10 * step_ms);
        let within = sub % (10 * step_ms);
        let amp_idx = (within / step_ms) as usize;
        let local = within % step_ms;
        let amp = LOG_MV[amp_idx.min(9)];
        let signed = if pass_idx == 0 { amp } else { -amp };
        let coast_bit = if local < LOG_DRIVE_MS { 0 } else { 1 };
        let sub_id = (pass_idx * 20 + amp_idx as u32 * 2 + coast_bit) as u8;
        if local < LOG_DRIVE_MS {
            let mode = if signed >= 0 { MotorMode::DriveFwd } else { MotorMode::DriveRev };
            (signed, mode, 4, sub_id)
        } else {
            (0, MotorMode::Coast, 4, sub_id)
        }
    } else if elapsed_ms < T_P5 {
        // Phase 5: Chirp
        let sub = elapsed_ms - T_P4;
        let chirp_dur_ms = T_P5 - T_P4 - 2_000;
        if sub < chirp_dur_ms {
            if !st.chirp.initialized {
                st.chirp.start(chirp_dur_ms);
            }
            let mv = st.chirp.next_mv();
            let mode = if mv >= 0 { MotorMode::DriveFwd } else { MotorMode::DriveRev };
            (mv, mode, 5, 0)
        } else {
            (0, MotorMode::Coast, 5, 1)
        }
    } else if elapsed_ms < T_P6 {
        // Phase 6: Doublet
        let sub = elapsed_ms - T_P5;
        let dbl_period: u32 = 2 * (DBL_PULSE_MS + DBL_GAP_MS);
        let dbl_total = DBL_REPS * dbl_period;
        if sub < dbl_total {
            let rep = sub / dbl_period;
            let local = sub % dbl_period;
            let (cmd, is_coast) = if local < DBL_PULSE_MS {
                (DBL_AMP_MV, false)
            } else if local < DBL_PULSE_MS + DBL_GAP_MS {
                (0, true)
            } else if local < 2 * DBL_PULSE_MS + DBL_GAP_MS {
                (DBL_AMP_MV, false)
            } else {
                (0, true)
            };
            let sub_id = (rep * 2 + if local < dbl_period / 2 { 0 } else { 1 }) as u8;
            let mode = if is_coast { MotorMode::Coast } else { MotorMode::DriveFwd };
            (cmd, mode, 6, sub_id)
        } else {
            (0, MotorMode::Coast, 6, 10)
        }
    } else if elapsed_ms < T_P7 {
        // Phase 7: Coast-down friction
        let sub = elapsed_ms - T_P6;
        let driven = (vbat_mv * 3 / 4) as i32;
        if sub < 3_000 {
            (driven, MotorMode::DriveFwd, 7, 0)
        } else if sub < 6_000 {
            (0, MotorMode::Coast, 7, 1)
        } else if sub < 9_000 {
            (-driven, MotorMode::DriveRev, 7, 2)
        } else {
            (0, MotorMode::Coast, 7, 3)
        }
    } else if elapsed_ms < T_END {
        (0, MotorMode::Brake, 8, 0)
    } else {
        (0, MotorMode::Brake, 9, 0)
    }
}

// ============================================================ main
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

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    drive(&mut led, true);

    let mut nsleep = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);
    drive(&mut nsleep, false);
    let mut nfault = gpiob.pb1.into_floating_input(&mut gpiob.crl);

    let mut tx = gpioa.pa9.into_push_pull_output(&mut gpioa.crh);
    drive(&mut tx, true);
    let mut rx = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);

    let pa6 = gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl);
    let pa7 = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let pwm = Timer::new(dp.TIM3, &mut rcc).pwm_hz((pa6, pa7), &mut afio.mapr, PWM_HZ.Hz());
    let max_duty: u16 = pwm.get_max_duty();
    let (mut ch1, mut ch2) = pwm.split();
    ch1.set_duty(0);
    ch2.set_duty(0);
    ch1.enable();
    ch2.enable();
    let mut set1 = |d: u16| ch1.set_duty(d);
    let mut set2 = |d: u16| ch2.set_duty(d);

    let mut adc1 = adc::Adc::new(dp.ADC1, &mut rcc);
    adc1.set_sample_time(adc::SampleTime::T_239);
    let mut ch0 = gpioa.pa0.into_analog(&mut gpioa.crl);
    let mut read_adc = || -> u16 { adc1.read(&mut ch0).unwrap_or(0) };

    let scl2 = gpiob.pb10.into_open_drain_output(&mut gpiob.crh);
    let sda2 = gpiob.pb11.into_open_drain_output(&mut gpiob.crh);
    let mut enc = BitI2c::new(scl2, sda2);

    delay_ms(50);
    let mut w = Uart { tx };

    // Initial brake safety
    apply_brake(&mut set1, &mut set2, max_duty);

    // Banner (all `#`-prefixed → pandas will skip via comment="#")
    let _ = w.write_str("# === Rich sysid v2 (STUM32, 115200) ===\r\n");
    let _ = write!(w, "# test_id={} run_id={} baud={} sample_us={}\r\n",
                   TEST_ID, RUN_ID, BAUD, SAMPLE_PERIOD_US);

    // ---- arm ----
    let _ = w.write_str("# --- preflight ---\r\n");
    let mut n_fail = 0u32;

    if !level(&mut rx) {
        let _ = w.write_str("# [FAIL] UART RX: PA10 reads low at idle\r\n");
        n_fail += 1;
    } else {
        let mut rx_ok = false;
        for i in 0..20u32 {
            let _ = write!(w, "# send any character to start ({} s left)\r\n", 20 - i);
            drive(&mut led, i % 2 == 1);
            if wait_start_bit(&mut rx, 1000) {
                rx_ok = true;
                break;
            }
        }
        delay_ms(5);
        drive(&mut led, true);
        if rx_ok {
            let _ = w.write_str("# [PASS] UART RX: start bit detected\r\n");
        } else {
            let _ = w.write_str("# [FAIL] UART RX: no character\r\n");
            n_fail += 1;
        }
    }

    match read_status_angle(&mut enc) {
        Ok((st, a)) => {
            if st & 0x20 == 0 {
                let _ = write!(w, "# [FAIL] AS5600: magnet not detected (STATUS 0x{:02X})\r\n", st);
                n_fail += 1;
            } else {
                let _ = write!(w, "# [PASS] AS5600: responds, angle {}\r\n", a);
            }
            if st & 0x10 != 0 {
                let _ = w.write_str("# [WARN] AS5600: magnet too weak\r\n");
            }
            if st & 0x08 != 0 {
                let _ = w.write_str("# [WARN] AS5600: magnet too strong\r\n");
            }
        }
        Err(e) => {
            let _ = write!(w, "# [FAIL] AS5600: read failed {:?}\r\n", e);
            n_fail += 1;
        }
    }
    if let Ok((agc, mag)) = read_agc_mag(&mut enc) {
        let _ = write!(w, "# [INFO] AS5600: AGC {} MAGNITUDE {}\r\n", agc, mag);
    }

    let vb = vbat_mv_avg(&mut read_adc, 16);
    let _ = write!(w, "# [INFO] VBAT: {}.{:03} V\r\n", vb / 1000, vb % 1000);
    if vb < 5_000 {
        let _ = w.write_str("# [FAIL] VBAT below 5.0 V\r\n");
        n_fail += 1;
    }

    drive(&mut nsleep, true);
    delay_ms(5);
    let fault_clear = level(&mut nfault);
    drive(&mut nsleep, false);
    if fault_clear {
        let _ = w.write_str("# [PASS] DRV8833: nFAULT high\r\n");
    } else {
        let _ = w.write_str("# [FAIL] DRV8833: nFAULT low\r\n");
        n_fail += 1;
    }

    if n_fail > 0 {
        let _ = w.write_str("# preflight failed: nothing will move\r\n");
        blink_forever(&mut led, 100);
    }

    // ---- second arm ----
    let _ = w.write_str("# [ACTION] Robot on stand, wheel free to spin.\r\n");
    let _ = w.write_str("# [ACTION] Send any character within 30 s to start\r\n");
    delay_ms(1000);
    if !wait_start_bit(&mut rx, 30_000) {
        let _ = w.write_str("# not armed\r\n");
        blink_forever(&mut led, 500);
    }
    delay_ms(5);

    // ---- CSV header (first non-# line = header for pandas) ----
    let _ = w.write_str("step,t_us,cmd_mv,ccr,adc,ang,st,mode\r\n");

    // ---- main loop ----
    let t0 = DWT::cycle_count();
    let cyc_per_us = CPU_HZ / 1_000_000;
    let period_cyc = SAMPLE_PERIOD_US * cyc_per_us;
    let mut n = 0u32;
    let mut state = SysIdState::new();
    let mut n_overruns = 0u32;
    let mut n_i2c = 0u32;
    let mut fault = false;

    drive(&mut nsleep, true);
    delay_ms(5);

    loop {
        let due = n.wrapping_mul(period_cyc);
        while DWT::cycle_count().wrapping_sub(t0) < due {}

        let now = DWT::cycle_count();
        let t_us = now.wrapping_sub(t0) / cyc_per_us;
        let elapsed_ms = t_us / 1000;

        if elapsed_ms >= TEST_DURATION_MS {
            break;
        }
        if now.wrapping_sub(t0) > due + period_cyc {
            n_overruns += 1;
        }
        if !level(&mut nfault) {
            fault = true;
            break;
        }

        let vb_now = vbat_mv_avg(&mut read_adc, 1);
        let (cmd_mv, mode, phase, sub_step) = phase_state(elapsed_ms, vb_now, &mut state);

        let ccr = match mode {
            MotorMode::Brake => {
                apply_brake(&mut set1, &mut set2, max_duty);
                0u16
            }
            MotorMode::Coast => {
                apply_coast(&mut set1, &mut set2);
                0u16
            }
            MotorMode::DriveFwd | MotorMode::DriveRev => {
                let (d, _) = apply_drive(&mut set1, &mut set2, max_duty, cmd_mv, vb_now);
                d
            }
        };

        let (st, ang) = match read_status_angle(&mut enc) {
            Ok(v) => v,
            Err(_) => {
                n_i2c += 1;
                (0xFFu8, 0xFFFFu16)
            }
        };
        let adc_val = read_adc();

        // step = phase*100 + sub_step
        let step = (phase as u32) * 100 + (sub_step as u32);

        let _ = write!(
            w,
            "{},{},{},{},{},{},{},{}\r\n",
            step, t_us, cmd_mv, ccr, adc_val, ang, st, mode as u8
        );

        n = n.wrapping_add(1);
    }

    // Final safety brake
    apply_brake(&mut set1, &mut set2, max_duty);
    drive(&mut nsleep, false);

    let _ = w.write_str("# --- sysid complete, outputs off ---\r\n");
    let _ = write!(
        w,
        "# [INFO] samples={} overruns={} i2c_errors={} fault={}\r\n",
        n, n_overruns, n_i2c, fault as u8
    );
    if fault {
        let _ = w.write_str("# [FAIL] DRV8833 fault asserted\r\n");
    }
    if n_overruns > 0 {
        let _ = w.write_str("# [WARN] timing overruns: increase SAMPLE_PERIOD_US\r\n");
    }

    blink_forever(&mut led, if fault { 100 } else { 500 });
}
