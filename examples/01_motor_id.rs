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

// ---------------------------------------------------------------- configuration

const CPU_HZ: u32 = 8_000_000;
const BAUD: u32 = 57_600;
const BIT_CYCLES: u32 = CPU_HZ / BAUD;

const I2C_HALF_US: u32 = 3;
const AS_ADDR: u8 = 0x36;

// Stimulus: triangle sweep 0 -> +max -> -max -> +max -> 0, in equal voltage steps.
const V_MAX_MV: i32 = 5000;
const STEP_MV: i32 = 250;
const DWELL_MS: u32 = 600;
const SAMPLE_PERIOD_US: u32 = 5000;
const MAX_SAMPLES: usize = 160;
const SLOW_DECAY: bool = true;
const PWM_HZ: u32 = 20_000;

// Battery divider on PA0 (top and bottom resistor in kilo-ohm), nominal ADC reference.
const VDDA_MV: u32 = 3300;
const DIV_R1_K: u32 = 20;
const DIV_R2_K: u32 = 10;

const _: () = assert!((DWELL_MS * 1000 / SAMPLE_PERIOD_US) as usize <= MAX_SAMPLES);

// ---------------------------------------------------------------- timing

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

// ---------------------------------------------------------------- pin helpers

fn drive<P: OutputPin>(p: &mut P, high: bool) {
    let _ = if high { p.set_high() } else { p.set_low() };
}

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

fn wait_start_bit<P: InputPin>(rx: &mut P, timeout_ms: u32) -> bool {
    for _ in 0..(timeout_ms * 20) {
        if !level(rx) {
            return true;
        }
        delay_us(50);
    }
    false
}

// ---------------------------------------------------------------- reporting

struct Fixed(i32, i32);

impl fmt::Display for Fixed {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let d = self.1 as u32;
        let a = self.0.unsigned_abs();
        let sign = if self.0 < 0 { "-" } else { "" };
        let w: usize = if d == 1000 { 3 } else { 1 };
        write!(f, "{}{}.{:0w$}", sign, a / d, a % d, w = w)
    }
}

struct Ctx<W: Write> {
    w: W,
    n_pass: u32,
    n_fail: u32,
    n_warn: u32,
}

impl<W: Write> Ctx<W> {
    fn new(w: W) -> Self {
        Self { w, n_pass: 0, n_fail: 0, n_warn: 0 }
    }
    fn emit(&mut self, tag: &str, name: &str, args: fmt::Arguments<'_>) {
        let _ = write!(self.w, "[{}] {}: ", tag, name);
        let _ = self.w.write_fmt(args);
        let _ = self.w.write_str("\r\n");
    }
    fn pass(&mut self, name: &str, args: fmt::Arguments<'_>) {
        self.n_pass += 1;
        self.emit("PASS", name, args);
    }
    fn fail(&mut self, name: &str, args: fmt::Arguments<'_>) {
        self.n_fail += 1;
        self.emit("FAIL", name, args);
    }
    fn warn(&mut self, name: &str, args: fmt::Arguments<'_>) {
        self.n_warn += 1;
        self.emit("WARN", name, args);
    }
    fn info(&mut self, name: &str, args: fmt::Arguments<'_>) {
        self.emit("INFO", name, args);
    }
    fn text(&mut self, s: &str) {
        let _ = self.w.write_str(s);
        let _ = self.w.write_str("\r\n");
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

// ---------------------------------------------------------------- AS5600

fn read_status_angle<SCL: Line, SDA: Line>(bus: &mut BitI2c<SCL, SDA>) -> Result<(u8, u16), I2cError> {
    let mut d = [0u8; 3];
    bus.read_regs(AS_ADDR, 0x0B, &mut d)?;
    Ok((d[0], (((d[1] & 0x0F) as u16) << 8) | d[2] as u16))
}

fn read_agc_mag<SCL: Line, SDA: Line>(bus: &mut BitI2c<SCL, SDA>) -> Result<(u8, u16), I2cError> {
    let mut d = [0u8; 3];
    bus.read_regs(AS_ADDR, 0x1A, &mut d)?;
    Ok((d[0], (((d[1] & 0x0F) as u16) << 8) | d[2] as u16))
}

// ---------------------------------------------------------------- battery voltage

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

// ---------------------------------------------------------------- stimulus

// Step index i in 0..=6n -> level index k in -n..=n (triangle path).
fn sweep_k(i: i32, n: i32) -> Option<i32> {
    if i < 0 || i > 6 * n {
        None
    } else if i <= n {
        Some(i)
    } else if i <= 3 * n {
        Some(n - (i - n))
    } else if i <= 5 * n {
        Some(-n + (i - 3 * n))
    } else {
        Some(n - (i - 5 * n))
    }
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
        // Forward: IN1 held high, IN2 low for the active fraction (brake otherwise).
        if cmd_mv >= 0 {
            set1(max);
            set2(max - m);
        } else {
            set2(max);
            set1(max - m);
        }
    } else {
        // Fast decay: active leg PWM, other leg low (coast otherwise).
        if cmd_mv >= 0 {
            set1(m);
            set2(0);
        } else {
            set1(0);
            set2(m);
        }
    }
    if cmd_mv >= 0 { m as i32 } else { -(m as i32) }
}

#[derive(Clone, Copy)]
struct Sample {
    t_us: u32,
    adc: u16,
    ang: u16,
    st: u8,
}

struct SweepStats {
    steps: u32,
    overruns: u32,
    i2c_errs: u32,
    fault: bool,
    adc_min: u16,
}

#[allow(clippy::too_many_arguments)]
fn run_sweep<W, SCL, SDA, S, F>(
    c: &mut Ctx<W>,
    enc: &mut BitI2c<SCL, SDA>,
    nsleep: &mut S,
    nfault: &mut F,
    set1: &mut impl FnMut(u16),
    set2: &mut impl FnMut(u16),
    read_adc: &mut impl FnMut() -> u16,
    max_duty: u16,
) -> SweepStats
where
    W: Write,
    SCL: Line,
    SDA: Line,
    S: OutputPin,
    F: InputPin,
{
    let n = V_MAX_MV / STEP_MV;
    let mut stats = SweepStats { steps: 0, overruns: 0, i2c_errs: 0, fault: false, adc_min: u16::MAX };
    let cyc_per_us = CPU_HZ / 1_000_000;
    let period_cycles = SAMPLE_PERIOD_US * cyc_per_us;
    let n_samples = ((DWELL_MS * 1000 / SAMPLE_PERIOD_US) as usize).min(MAX_SAMPLES);
    let mut buf = [Sample { t_us: 0, adc: 0, ang: 0, st: 0 }; MAX_SAMPLES];

    let _ = write!(
        c.w,
        "# max_duty={} pwm_hz={} step_mv={} dwell_ms={} period_us={} slow_decay={}\r\n",
        max_duty, PWM_HZ, STEP_MV, DWELL_MS, SAMPLE_PERIOD_US, SLOW_DECAY as u8
    );
    c.text("step,t_us,cmd_mv,ccr,adc,ang,st");

    drive(nsleep, true);
    delay_ms(5);
    let t_origin = DWT::cycle_count();
    let mut step = 0i32;
    'outer: loop {
        let k = match sweep_k(step, n) {
            Some(k) => k,
            None => break,
        };
        let cmd_mv = k * STEP_MV;
        let vb = vbat_mv_avg(read_adc, 4);
        let ccr = apply_cmd(set1, set2, max_duty, cmd_mv, vb);
        let t_step0 = DWT::cycle_count();

        let mut got = 0usize;
        while got < n_samples {
            let due = period_cycles * (got as u32);
            while DWT::cycle_count().wrapping_sub(t_step0) < due {}
            let now = DWT::cycle_count();
            if now.wrapping_sub(t_step0) > due + period_cycles {
                stats.overruns += 1;
            }
            if !level(nfault) {
                stats.fault = true;
                break 'outer;
            }
            let (st, ang) = match read_status_angle(enc) {
                Ok(v) => v,
                Err(_) => {
                    stats.i2c_errs += 1;
                    (0xFFu8, 0xFFFFu16)
                }
            };
            let a = read_adc();
            if a > 0 && a < stats.adc_min {
                stats.adc_min = a;
            }
            buf[got] = Sample { t_us: now.wrapping_sub(t_origin) / cyc_per_us, adc: a, ang, st };
            got += 1;
        }

        // The step level stays applied while the buffer is transmitted.
        for s in &buf[..got] {
            let _ = write!(
                c.w,
                "{},{},{},{},{},{},{}\r\n",
                step, s.t_us, cmd_mv, ccr, s.adc, s.ang, s.st
            );
        }
        step += 1;
        stats.steps += 1;
    }

    set1(0);
    set2(0);
    drive(nsleep, false);
    stats
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

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    drive(&mut led, true);

    // Driver stays asleep until the sweep starts.
    let mut nsleep = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);
    drive(&mut nsleep, false);
    let mut nfault = gpiob.pb1.into_floating_input(&mut gpiob.crl);

    let mut tx = gpioa.pa9.into_push_pull_output(&mut gpioa.crh);
    drive(&mut tx, true);
    let mut rx = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);

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

    // Encoder on the second bit-banged bus.
    let scl2 = gpiob.pb10.into_open_drain_output(&mut gpiob.crh);
    let sda2 = gpiob.pb11.into_open_drain_output(&mut gpiob.crh);
    let mut enc = BitI2c::new(scl2, sda2);

    delay_ms(50);
    let mut c = Ctx::new(Uart { tx });

    c.text("");
    c.text("# === Motor staircase identification v1 ===");
    let _ = write!(c.w, "# serial {} 8N1\r\n", BAUD);

    // Start trigger (also an RX test).
    let rx_idle_high = level(&mut rx);
    let mut rx_ok = false;
    if !rx_idle_high {
        c.fail("UART RX", format_args!("PA10 reads low at idle: nothing will move"));
    } else {
        for i in 0..20u32 {
            let _ = write!(c.w, "# send any character to start preflight ({} s left)\r\n", 20 - i);
            drive(&mut led, i % 2 == 1);
            if wait_start_bit(&mut rx, 1000) {
                rx_ok = true;
                break;
            }
        }
        delay_ms(5);
        drive(&mut led, true);
        if rx_ok {
            c.pass("UART RX", format_args!("start bit detected"));
        } else {
            c.fail("UART RX", format_args!("no character received: nothing will move"));
        }
    }

    // Preflight.
    c.text("# --- preflight ---");
    match read_status_angle(&mut enc) {
        Ok((st, a)) => {
            let md = st & 0x20 != 0;
            let ml = st & 0x10 != 0;
            let mh = st & 0x08 != 0;
            if !md {
                c.fail("AS5600", format_args!("magnet not detected (STATUS 0x{:02X})", st));
            } else {
                c.pass("AS5600", format_args!("responds, raw angle {}", a));
            }
            if ml {
                c.warn("AS5600", format_args!("magnet too weak: angle may drop out during the sweep"));
            }
            if mh {
                c.warn("AS5600", format_args!("magnet too strong"));
            }
        }
        Err(e) => c.fail("AS5600", format_args!("read failed: {:?}", e)),
    }
    if let Ok((agc, mag)) = read_agc_mag(&mut enc) {
        c.info("AS5600", format_args!("AGC {} MAGNITUDE {}", agc, mag));
    }

    let vb = vbat_mv_avg(&mut read_adc, 16);
    c.info("VBAT", format_args!("{} V (nominal divider and 3.3 V reference assumed)", Fixed(vb as i32, 1000)));
    if vb < (V_MAX_MV as u32) + 500 {
        c.fail(
            "VBAT",
            format_args!(
                "below {} V: no headroom for a {} V command (USB-only power, discharged battery, or divider fault)",
                Fixed(V_MAX_MV + 500, 1000),
                Fixed(V_MAX_MV, 1000)
            ),
        );
    } else if vb > 9000 {
        c.warn("VBAT", format_args!("above 9 V: divider or reference error"));
    }

    drive(&mut nsleep, true);
    delay_ms(5);
    let fault_clear = level(&mut nfault);
    drive(&mut nsleep, false);
    if fault_clear {
        c.pass("DRV8833", format_args!("nFAULT high after wake"));
    } else {
        c.fail("DRV8833", format_args!("nFAULT low after wake: VM missing, output short, or over-temperature"));
    }

    if c.n_fail > 0 {
        c.text("# preflight failed: nothing will move");
        blink_forever(&mut led, 100);
    }

    c.text("# [ACTION] Robot on a stand with the wheel free to spin.");
    c.text("# [ACTION] Send any character within 30 s to start the sweep; otherwise nothing moves.");
    delay_ms(1000);
    if !wait_start_bit(&mut rx, 30_000) {
        c.text("# not armed: exiting without motion");
        blink_forever(&mut led, 500);
    }
    delay_ms(5);

    let stats = run_sweep(
        &mut c,
        &mut enc,
        &mut nsleep,
        &mut nfault,
        &mut set1,
        &mut set2,
        &mut read_adc,
        max_duty,
    );

    c.text("# --- sweep finished, outputs off ---");
    c.info(
        "sweep",
        format_args!(
            "steps={} overruns={} i2c_errors={} fault={}",
            stats.steps, stats.overruns, stats.i2c_errs, stats.fault as u8
        ),
    );
    if stats.adc_min != u16::MAX {
        c.info("VBAT", format_args!("minimum during sweep {} V", Fixed(adc_to_mv(stats.adc_min as u32) as i32, 1000)));
    }
    if stats.fault {
        c.fail("DRV8833", format_args!("nFAULT asserted, sweep aborted"));
    }
    if stats.overruns > 0 {
        c.warn("timing", format_args!("{} sample periods overran: increase SAMPLE_PERIOD_US", stats.overruns));
    }
    if stats.i2c_errs > 0 {
        c.warn("AS5600", format_args!("{} failed reads (angle 65535 in the data)", stats.i2c_errs));
    }

    let half = if stats.fault { 100 } else { 500 };
    blink_forever(&mut led, half);
}
