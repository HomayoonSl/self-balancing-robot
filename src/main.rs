#![no_std]
#![no_main]
#![deny(unsafe_code)]

use core::fmt::{self, Write};

use cortex_m::peripheral::{DWT, SCB};
use cortex_m_rt::entry;
use embedded_hal::digital::{InputPin, OutputPin};
use stm32f1xx_hal::{adc, pac, prelude::*, timer::Timer};

// Reset instead of halting: hardware PWM keeps running after a halt.
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    SCB::sys_reset()
}

// ================================================================ configuration

// Platform
const CPU_HZ: u32 = 8_000_000;
const BAUD: u32 = 57_600;
const BIT_CYCLES: u32 = CPU_HZ / BAUD;
const I2C_HALF_US: u32 = 3;
const AS_ADDR: u8 = 0x36;
const BMI_ADDR: u8 = 0x69;
const PWM_HZ: u32 = 20_000;
const SLOW_DECAY: bool = true;
const LED_ACTIVE_HIGH: bool = true;

// Run modes
const BENCH_MODE: bool = false;
const BENCH_DRIVE: bool = false;
const BENCH_V_MAX: f32 = 2.5;
const BENCH_VBAT_MV: u32 = 5500;
const DUMP_ON_STOP: bool = true;

// Loop timing
const DT_US: u32 = 5000;
const DT_MS: u32 = 5;
const DT: f32 = 0.005;

// IMU mounting (set with BENCH_MODE). Axis index: 0 = X, 1 = Y, 2 = Z.
const ACC_UP_AXIS: usize = 2;
const ACC_UP_SIGN: f32 = 1.0;
const ACC_FWD_AXIS: usize = 0;
const ACC_FWD_SIGN: f32 = 1.0;
const GYRO_FLIP: bool = false;
const GYRO_PITCH_AXIS: usize = 3 - ACC_UP_AXIS - ACC_FWD_AXIS;
const _: () = assert!(ACC_UP_AXIS < 3 && ACC_FWD_AXIS < 3 && ACC_UP_AXIS != ACC_FWD_AXIS);
const ACC_LSB_PER_G: f32 = 16384.0;
const GYRO_LSB_PER_DPS: f32 = 32.8; // +-1000 dps range
const GYRO_RAD_PER_LSB: f32 = 0.017_453_293 / GYRO_LSB_PER_DPS;

// Encoder: positive motor command must give positive wheel angle.
const ENC_SIGN: f32 = -1.0;
const RAD_PER_COUNT: f32 = 6.283_185_3 / 4096.0;

// Estimator (complementary filter with bias adaptation)
const KP_ACC: f32 = 0.5;
const KI_BIAS: f32 = 0.0625;

// Controller: V = K_THETA*pitch + K_RATE*pitch_rate + K_WHEEL*wheel + K_WHEEL_RATE*wheel_rate
const K_THETA: f32 = 33.0;
const K_RATE: f32 = 3.1;
const K_WHEEL: f32 = 0.73;
const K_WHEEL_RATE: f32 = 0.49;
const DZ_POS_V: f32 = 0.40;
const DZ_NEG_V: f32 = 0.30;
const DZ_SOFT_V: f32 = 0.40;
const V_LIMIT_V: f32 = 5.0;

// Sequencing and safety
const BOOT_WAIT_MS: u32 = 5000;
const RELEASE_DELAY_MS: u32 = 3000;
const STILL_P2P_DPS: f32 = 20.0;
const ENGAGE_RAD: f32 = 0.17;
const FALL_RAD: f32 = 0.50;
const RUNAWAY_RATE: f32 = 40.0;
const RUNAWAY_TICKS: u32 = 60;
const WHEEL_LIMIT_RAD: f32 = 70.0;
const VBAT_MIN_V: f32 = 4.8;

// Battery divider on PA0
const VDDA_MV: u32 = 3300;
const DIV_R1_K: u32 = 20;
const DIV_R2_K: u32 = 10;
const ADC_TO_V: f32 =
    (VDDA_MV as f32) * ((DIV_R1_K + DIV_R2_K) as f32) / ((DIV_R2_K as f32) * 4095.0) / 1000.0;

const BB_LEN: usize = 400;

// ================================================================ helpers

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

fn drive<P: OutputPin>(p: &mut P, high: bool) {
    let _ = if high { p.set_high() } else { p.set_low() };
}

fn level<P: InputPin>(p: &mut P) -> bool {
    p.is_high().unwrap_or(false)
}

trait Line: OutputPin + InputPin {}
impl<T: OutputPin + InputPin> Line for T {}

fn absf(x: f32) -> f32 {
    if x < 0.0 { -x } else { x }
}

fn clampf(x: f32, lo: f32, hi: f32) -> f32 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

struct Ticker {
    next: u32,
    period: u32,
}

impl Ticker {
    fn new(period: u32) -> Self {
        Self { next: DWT::cycle_count().wrapping_add(period), period }
    }
    // Waits for the next tick; returns true if the tick was already late by more than one period.
    fn wait(&mut self) -> bool {
        let late = (DWT::cycle_count().wrapping_sub(self.next) as i32) > (self.period as i32);
        while (DWT::cycle_count().wrapping_sub(self.next) as i32) < 0 {}
        self.next = self.next.wrapping_add(self.period);
        late
    }
}

// ---------------------------------------------------------------- bit-banged UART TX

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

// ---------------------------------------------------------------- bit-banged I2C

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum I2cError {
    SclStuckLow,
    Nack,
}

struct BitI2c<SCL: Line, SDA: Line> {
    scl: SCL,
    sda: SDA,
}

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

    fn write_regs(&mut self, addr: u8, reg: u8, data: &[u8]) -> Result<(), I2cError> {
        let r = self.write_regs_inner(addr, reg, data);
        let _ = self.stop();
        r
    }

    fn write_regs_inner(&mut self, addr: u8, reg: u8, data: &[u8]) -> Result<(), I2cError> {
        self.start()?;
        if !self.write_byte(addr << 1)? {
            return Err(I2cError::Nack);
        }
        if !self.write_byte(reg)? {
            return Err(I2cError::Nack);
        }
        for &b in data {
            if !self.write_byte(b)? {
                return Err(I2cError::Nack);
            }
        }
        Ok(())
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

    fn read_reg(&mut self, addr: u8, reg: u8) -> Result<u8, I2cError> {
        let mut b = [0u8; 1];
        self.read_regs(addr, reg, &mut b)?;
        Ok(b[0])
    }
}

// ---------------------------------------------------------------- IMU

#[derive(Clone, Copy)]
struct ImuRaw {
    g: [i16; 3],
    a: [i16; 3],
}

fn bmi_init<SCL: Line, SDA: Line>(bus: &mut BitI2c<SCL, SDA>) -> bool {
    if bus.read_reg(BMI_ADDR, 0x00) != Ok(0xD1) {
        return false;
    }
    let mut ok = true;
    ok &= bus.write_regs(BMI_ADDR, 0x7E, &[0x11]).is_ok(); // accel normal mode
    delay_ms(10);
    ok &= bus.write_regs(BMI_ADDR, 0x7E, &[0x15]).is_ok(); // gyro normal mode
    delay_ms(100);
    ok &= bus.write_regs(BMI_ADDR, 0x40, &[0x2B]).is_ok(); // ACC_CONF: 800 Hz, normal
    ok &= bus.write_regs(BMI_ADDR, 0x41, &[0x03]).is_ok(); // ACC_RANGE: +-2 g
    ok &= bus.write_regs(BMI_ADDR, 0x42, &[0x2B]).is_ok(); // GYR_CONF: 800 Hz, normal
    ok &= bus.write_regs(BMI_ADDR, 0x43, &[0x01]).is_ok(); // GYR_RANGE: +-1000 dps
    delay_ms(20);
    ok && bus.read_reg(BMI_ADDR, 0x03) == Ok(0x14)
}

fn read_imu<SCL: Line, SDA: Line>(bus: &mut BitI2c<SCL, SDA>) -> Option<ImuRaw> {
    let mut d = [0u8; 12];
    bus.read_regs(BMI_ADDR, 0x0C, &mut d).ok()?;
    let mut g = [0i16; 3];
    let mut a = [0i16; 3];
    for k in 0..3 {
        g[k] = i16::from_le_bytes([d[2 * k], d[2 * k + 1]]);
        a[k] = i16::from_le_bytes([d[6 + 2 * k], d[7 + 2 * k]]);
    }
    Some(ImuRaw { g, a })
}

// Sign of the pitch rate (positive = leaning forward) on the derived gyro axis.
fn gyro_sign() -> f32 {
    let cyclic = (ACC_FWD_AXIS + 3 - ACC_UP_AXIS) % 3 == 1;
    let parity = if cyclic { 1.0 } else { -1.0 };
    let flip = if GYRO_FLIP { -1.0 } else { 1.0 };
    ACC_UP_SIGN * ACC_FWD_SIGN * parity * flip
}

struct Calib {
    bias_rad: f32,
    theta0: f32,
}

#[derive(Clone, Copy)]
enum CalFail {
    Read,
    Moving,
    Axis,
}

fn cal_fail_text(e: CalFail) -> &'static str {
    match e {
        CalFail::Read => "IMU read failed",
        CalFail::Moving => "robot not still (gyro peak-to-peak above limit)",
        CalFail::Axis => "gravity not on the configured up axis, or tilt above about 23 deg",
    }
}

fn calibrate<SCL: Line, SDA: Line>(imu: &mut BitI2c<SCL, SDA>) -> Result<Calib, CalFail> {
    let n: i32 = 400;
    let mut gsum = [0i32; 3];
    let mut asum = [0i32; 3];
    let mut gmin = [i16::MAX; 3];
    let mut gmax = [i16::MIN; 3];
    let mut tk = Ticker::new(DT_US * (CPU_HZ / 1_000_000));
    for _ in 0..n {
        tk.wait();
        let s = read_imu(imu).ok_or(CalFail::Read)?;
        for k in 0..3 {
            gsum[k] += s.g[k] as i32;
            asum[k] += s.a[k] as i32;
            gmin[k] = gmin[k].min(s.g[k]);
            gmax[k] = gmax[k].max(s.g[k]);
        }
    }
    let limit = (STILL_P2P_DPS * GYRO_LSB_PER_DPS) as i32;
    for k in 0..3 {
        if (gmax[k] as i32 - gmin[k] as i32) > limit {
            return Err(CalFail::Moving);
        }
    }
    let nf = n as f32;
    let au = ACC_UP_SIGN * (asum[ACC_UP_AXIS] as f32) / nf / ACC_LSB_PER_G;
    let af = ACC_FWD_SIGN * (asum[ACC_FWD_AXIS] as f32) / nf / ACC_LSB_PER_G;
    if au < 0.85 || au > 1.15 || absf(af) > 0.4 {
        return Err(CalFail::Axis);
    }
    let g_mean = (gsum[GYRO_PITCH_AXIS] as f32) / nf;
    Ok(Calib { bias_rad: gyro_sign() * g_mean * GYRO_RAD_PER_LSB, theta0: -af / au })
}

// Complementary filter with gyro bias adaptation; pitch positive = leaning forward.
struct Filter {
    theta_abs: f32,
    bias: f32,
    theta0: f32,
    sign: f32,
}

impl Filter {
    fn new(cal: &Calib) -> Self {
        Self { theta_abs: cal.theta0, bias: cal.bias_rad, theta0: cal.theta0, sign: gyro_sign() }
    }

    // Returns (pitch relative to the calibrated upright in rad, bias-corrected pitch rate in rad/s).
    fn update(&mut self, s: &ImuRaw) -> (f32, f32) {
        let meas = self.sign * (s.g[GYRO_PITCH_AXIS] as f32) * GYRO_RAD_PER_LSB;
        let au = ACC_UP_SIGN * (s.a[ACC_UP_AXIS] as f32);
        let af = ACC_FWD_SIGN * (s.a[ACC_FWD_AXIS] as f32);
        let mut e = 0.0f32;
        if au > 0.5 * ACC_LSB_PER_G {
            e = -af / au - self.theta_abs;
        }
        self.bias -= KI_BIAS * e * DT;
        let rate = meas - self.bias;
        self.theta_abs += (rate + KP_ACC * e) * DT;
        (self.theta_abs - self.theta0, rate)
    }
}

// ---------------------------------------------------------------- wheel encoder

fn wrap_delta(a: u16, b: u16) -> i32 {
    let d = (b as i32 - a as i32) & 0x0FFF;
    if d >= 2048 { d - 4096 } else { d }
}

fn read_raw_angle<SCL: Line, SDA: Line>(bus: &mut BitI2c<SCL, SDA>) -> Option<u16> {
    let mut d = [0u8; 2];
    bus.read_regs(AS_ADDR, 0x0C, &mut d).ok()?;
    Some((((d[0] & 0x0F) as u16) << 8) | d[1] as u16)
}

struct Wheel {
    prev_raw: u16,
    counts: i32,
    p1: f32,
    p2: f32,
    rate: f32,
}

impl Wheel {
    fn new(raw: u16) -> Self {
        Self { prev_raw: raw, counts: 0, p1: 0.0, p2: 0.0, rate: 0.0 }
    }

    // Returns (wheel angle relative to the body in rad, filtered wheel rate in rad/s).
    fn update(&mut self, raw: u16) -> (f32, f32) {
        self.counts += wrap_delta(self.prev_raw, raw);
        self.prev_raw = raw;
        let b = ENC_SIGN * (self.counts as f32) * RAD_PER_COUNT;
        let r = (b - self.p2) / (2.0 * DT);
        self.rate += 0.5 * (r - self.rate);
        self.p2 = self.p1;
        self.p1 = b;
        (b, self.rate)
    }
}

// ---------------------------------------------------------------- battery and drive

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

fn deadzone_comp(v: f32) -> f32 {
    let dz = if v >= 0.0 { DZ_POS_V } else { DZ_NEG_V };
    v + dz * clampf(v / DZ_SOFT_V, -1.0, 1.0)
}

// Returns the signed active duty in timer counts (positive = IN1 side).
fn apply_cmd(
    set1: &mut impl FnMut(u16),
    set2: &mut impl FnMut(u16),
    max: u16,
    cmd_mv: i32,
    vbat_mv: u32,
) -> i32 {
    let mag_mv = cmd_mv.unsigned_abs() as u64;
    let mut mag = (mag_mv * max as u64 / vbat_mv.max(1) as u64) as u32;
    if mag > max as u32 {
        mag = max as u32;
    }
    let m = mag as u16;
    if SLOW_DECAY {
        if cmd_mv >= 0 {
            set1(max);
            set2(max - m);
        } else {
            set2(max);
            set1(max - m);
        }
    } else if cmd_mv >= 0 {
        set1(m);
        set2(0);
    } else {
        set1(0);
        set2(m);
    }
    if cmd_mv >= 0 { m as i32 } else { -(m as i32) }
}

// ---------------------------------------------------------------- black box

#[derive(Clone, Copy)]
struct Rec {
    pitch_cd: i16,
    rate_dd: i16,
    wheel_cr: i16,
    wheel_rate_cr: i16,
    v_mv: i16,
}

const REC_ZERO: Rec = Rec { pitch_cd: 0, rate_dd: 0, wheel_cr: 0, wheel_rate_cr: 0, v_mv: 0 };

struct BlackBox {
    buf: [Rec; BB_LEN],
    head: usize,
    filled: usize,
}

impl BlackBox {
    fn new() -> Self {
        Self { buf: [REC_ZERO; BB_LEN], head: 0, filled: 0 }
    }
    fn push(&mut self, r: Rec) {
        self.buf[self.head] = r;
        self.head = (self.head + 1) % BB_LEN;
        if self.filled < BB_LEN {
            self.filled += 1;
        }
    }
}

fn dump_blackbox<W: Write>(w: &mut W, bb: &BlackBox) {
    let _ = write!(w, "# blackbox: {} rows, {} ms per row, last row is the stop\r\n", bb.filled, DT_MS);
    let _ = w.write_str("row,pitch_cdeg,rate_ddps,wheel_crad,wheel_rate_crad_s,v_mv\r\n");
    let start = (bb.head + BB_LEN - bb.filled) % BB_LEN;
    for i in 0..bb.filled {
        let r = bb.buf[(start + i) % BB_LEN];
        let _ = write!(w, "{},{},{},{},{},{}\r\n", i, r.pitch_cd, r.rate_dd, r.wheel_cr, r.wheel_rate_cr, r.v_mv);
    }
}

// ---------------------------------------------------------------- balance run

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
    cal: &Calib,
    bb: &mut BlackBox,
) -> RunStats
where
    S1: Line,
    D1: Line,
    S2: Line,
    D2: Line,
    S: OutputPin,
    F: InputPin,
{
    let cyc_per_us = CPU_HZ / 1_000_000;
    let mut stats = RunStats { outcome: Outcome::Fell, engaged_ticks: 0, overruns: 0, max_busy_us: 0, max_pitch_cd: 0 };

    let mut raw = match read_raw_angle(enc) {
        Some(r) => r,
        None => {
            stats.outcome = Outcome::Sensor;
            return stats;
        }
    };
    let mut filt = Filter::new(cal);
    let mut wheel = Wheel::new(raw);
    let mut vbat = vbat_mv_avg(read_adc, 8) as f32 / 1000.0;

    // Driver awake with both inputs low (coast) until the controller engages.
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
        let t_in = DWT::cycle_count();
        n += 1;

        // Wheel first (slow states), IMU last (fast states) to minimise IMU age.
        match read_raw_angle(enc) {
            Some(r) => {
                raw = r;
                enc_err = 0;
            }
            None => enc_err += 1,
        }
        let (mut wang, mut wrate) = wheel.update(raw);
        match read_imu(imu) {
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
            if absf(theta) < ENGAGE_RAD {
                engaged = true;
                wheel = Wheel::new(raw);
                wang = 0.0;
                wrate = 0.0;
            } else if absf(theta) > FALL_RAD {
                break Outcome::Fell;
            }
        }

        if engaged {
            stats.engaged_ticks += 1;
            if absf(theta) > FALL_RAD {
                break Outcome::Fell;
            }
            if absf(wrate) > RUNAWAY_RATE {
                run_cnt += 1;
            } else {
                run_cnt = 0;
            }
            if run_cnt > RUNAWAY_TICKS {
                break Outcome::Runaway;
            }
            if absf(wang) > WHEEL_LIMIT_RAD {
                break Outcome::Travel;
            }
            if !level(nfault) {
                break Outcome::Fault;
            }
            if vbat < VBAT_MIN_V {
                break Outcome::Battery;
            }

            let v_lin = K_THETA * theta + K_RATE * rate + K_WHEEL * wang + K_WHEEL_RATE * wrate;
            let v = clampf(deadzone_comp(v_lin), -V_LIMIT_V, V_LIMIT_V);
            apply_cmd(set1, set2, max_duty, (v * 1000.0) as i32, (vbat * 1000.0) as u32);

            let pitch_cd = theta * 5729.578;
            let pa = absf(pitch_cd) as i32;
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
        let busy = DWT::cycle_count().wrapping_sub(t_in) / cyc_per_us;
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

// ---------------------------------------------------------------- bench mode

#[allow(clippy::too_many_arguments)]
fn bench_loop<S1, D1, S2, D2, S, W>(
    w: &mut W,
    imu: &mut BitI2c<S1, D1>,
    enc: &mut BitI2c<S2, D2>,
    nsleep: &mut S,
    set1: &mut impl FnMut(u16),
    set2: &mut impl FnMut(u16),
    max_duty: u16,
    cal: &Calib,
) -> !
where
    S1: Line,
    D1: Line,
    S2: Line,
    D2: Line,
    S: OutputPin,
    W: Write,
{
    let mut filt = Filter::new(cal);
    let mut raw = read_raw_angle(enc).unwrap_or(0);
    let mut wheel = Wheel::new(raw);
    set1(0);
    set2(0);
    drive(nsleep, BENCH_DRIVE);
    let _ = w.write_str("# bench mode: wheel free; motor follows pitch terms only if BENCH_DRIVE\r\n");
    let mut tk = Ticker::new(DT_US * (CPU_HZ / 1_000_000));
    let mut n = 0u32;
    let mut last = ImuRaw { g: [0; 3], a: [0; 3] };
    let (mut theta, mut rate) = (0.0f32, 0.0f32);
    let mut v = 0.0f32;
    loop {
        tk.wait();
        n += 1;
        if let Some(r) = read_raw_angle(enc) {
            raw = r;
        }
        let (wang, _) = wheel.update(raw);
        if let Some(s) = read_imu(imu) {
            last = s;
            let (th, ra) = filt.update(&s);
            theta = th;
            rate = ra;
        }
        if BENCH_DRIVE {
            v = clampf(deadzone_comp(K_THETA * theta + K_RATE * rate), -BENCH_V_MAX, BENCH_V_MAX);
            apply_cmd(set1, set2, max_duty, (v * 1000.0) as i32, BENCH_VBAT_MV);
        }
        if n % 40 == 0 {
            let _ = write!(
                w,
                "acc_mg {} {} {} gyro_ddps {} {} {} pitch_cdeg {} rate_ddps {} wheel_crad {} v_mv {}\r\n",
                (last.a[0] as i32) * 1000 / 16384,
                (last.a[1] as i32) * 1000 / 16384,
                (last.a[2] as i32) * 1000 / 16384,
                (last.g[0] as i32) * 100 / 328,
                (last.g[1] as i32) * 100 / 328,
                (last.g[2] as i32) * 100 / 328,
                (theta * 5729.578) as i32,
                (rate * 572.958) as i32,
                (wang * 100.0) as i32,
                (v * 1000.0) as i32
            );
        }
    }
}

// ---------------------------------------------------------------- main

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

    // Status LED (PC13, active low) and ready LED (PA3).
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

    // Hardware PWM: TIM3 CH1 on PA6 (IN1+IN3), CH2 on PA7 (IN2+IN4).
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

    // Battery voltage through the divider on PA0.
    let mut adc1 = adc::Adc::new(dp.ADC1, &mut rcc);
    adc1.set_sample_time(adc::SampleTime::T_239);
    let mut ch0 = gpioa.pa0.into_analog(&mut gpioa.crl);
    let mut read_adc = || -> u16 { adc1.read(&mut ch0).unwrap_or(0) };

    // IMU bus (PB6/PB7) and encoder bus (PB10/PB11), both bit-banged.
    let scl1 = gpiob.pb6.into_open_drain_output(&mut gpiob.crl);
    let sda1 = gpiob.pb7.into_open_drain_output(&mut gpiob.crl);
    let scl2 = gpiob.pb10.into_open_drain_output(&mut gpiob.crh);
    let sda2 = gpiob.pb11.into_open_drain_output(&mut gpiob.crh);
    let mut imu = BitI2c::new(scl1, sda1);
    let mut enc = BitI2c::new(scl2, sda2);

    delay_ms(100);
    let mut con = Uart { tx };
    let _ = write!(con, "\r\n# === Self-balancing controller v1 (serial {} 8N1) ===\r\n", BAUD);
    let _ = write!(
        con,
        "# imu mapping: up axis {} sign {}, forward axis {} sign {}, gyro pitch axis {}, gyro sign {}\r\n",
        ACC_UP_AXIS,
        ACC_UP_SIGN as i32,
        ACC_FWD_AXIS,
        ACC_FWD_SIGN as i32,
        GYRO_PITCH_AXIS,
        gyro_sign() as i32
    );

    if !bmi_init(&mut imu) {
        let _ = con.write_str("# BMI160 init failed (address 0x69, CHIP_ID 0xD1, PMU_STATUS 0x14)\r\n");
        loop {
            status_led(true);
            delay_ms(100);
            status_led(false);
            delay_ms(100);
        }
    }
    let _ = con.write_str("# BMI160 configured\r\n");
    let vb0 = vbat_mv_avg(&mut read_adc, 16);
    let _ = write!(con, "# VBAT {} mV (nominal divider assumed; verify with a multimeter)\r\n", vb0);

    let mut bb = BlackBox::new();
    loop {
        let _ = con.write_str("# hold the robot upright and still\r\n");
        for i in 0..(BOOT_WAIT_MS / 250) {
            status_led(i % 2 == 0);
            delay_ms(250);
        }
        status_led(false);

        let cal = match calibrate(&mut imu) {
            Ok(c) => c,
            Err(e) => {
                let _ = write!(con, "# calibration failed: {}\r\n", cal_fail_text(e));
                if BENCH_MODE {
                    Calib { bias_rad: 0.0, theta0: 0.0 }
                } else {
                    for _ in 0..10 {
                        status_led(true);
                        delay_ms(100);
                        status_led(false);
                        delay_ms(100);
                    }
                    continue;
                }
            }
        };
        let _ = write!(
            con,
            "# calibrated: gyro bias {} mdps, upright offset {} cdeg\r\n",
            (cal.bias_rad * 57295.78) as i32,
            (cal.theta0 * 5729.578) as i32
        );

        if BENCH_MODE {
            bench_loop(&mut con, &mut imu, &mut enc, &mut nsleep, &mut set1, &mut set2, max_duty, &cal);
        }

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
            dump_blackbox(&mut con, &bb);
        }
        bb = BlackBox::new();
        delay_ms(1000);
    }
}
