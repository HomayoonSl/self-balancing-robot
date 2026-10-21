#![no_std]
#![no_main]
#![deny(unsafe_code)]

use core::fmt::{self, Write};

use cortex_m::peripheral::DWT;
use cortex_m_rt::entry;
use embedded_hal::digital::{InputPin, OutputPin};
use panic_halt as _;
use stm32f1xx_hal::{pac, prelude::*};

// Core clock assumed after constrain() without PLL configuration (HSI, 8 MHz).
// If the terminal shows garbage, this assumption is wrong.
const CPU_HZ: u32 = 8_000_000;
const BAUD: u32 = 9_600;
const BIT_CYCLES: u32 = CPU_HZ / BAUD;

const I2C_HALF_US: u32 = 5;
const BMI_ADDR: u8 = 0x68;
const AS_ADDR: u8 = 0x36;

const MOTOR_DUTY_PCT: u32 = 40;
const MOTOR_BURST_MS: u32 = 150;
const MOTOR_MIN_COUNTS: i32 = 100;

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

    fn lines(&mut self) -> (bool, bool) {
        (level(&mut self.scl), level(&mut self.sda))
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

    fn recover(&mut self) {
        for _ in 0..9 {
            if level(&mut self.sda) {
                break;
            }
            drive(&mut self.scl, false);
            t();
            drive(&mut self.scl, true);
            t();
        }
        let _ = self.stop();
    }

    fn probe(&mut self, addr: u8) -> Result<bool, I2cError> {
        self.start()?;
        let r = self.write_byte(addr << 1);
        let _ = self.stop();
        r
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

// ---------------------------------------------------------------- bus checks

fn diag_bus<W: Write, SCL: Line, SDA: Line>(
    c: &mut Ctx<W>,
    label: &str,
    bus: &mut BitI2c<SCL, SDA>,
) -> bool {
    let (mut scl, mut sda) = bus.lines();
    if !(scl && sda) {
        c.warn(label, format_args!("idle SCL={} SDA={}, attempting recovery", scl as u8, sda as u8));
        bus.recover();
        let l = bus.lines();
        scl = l.0;
        sda = l.1;
    }
    if scl && sda {
        c.pass(label, format_args!("idle lines high (pull-ups present, no short to GND)"));
        true
    } else {
        c.fail(
            label,
            format_args!(
                "line held low SCL={} SDA={}: short to GND, missing pull-up, or device holding the bus",
                scl as u8, sda as u8
            ),
        );
        false
    }
}

fn diag_scan<W: Write, SCL: Line, SDA: Line>(
    c: &mut Ctx<W>,
    label: &str,
    bus: &mut BitI2c<SCL, SDA>,
    expected: u8,
) -> bool {
    let mut found = [0u8; 16];
    let mut n = 0usize;
    for addr in 0x08u8..0x78 {
        match bus.probe(addr) {
            Ok(true) => {
                if n < found.len() {
                    found[n] = addr;
                    n += 1;
                }
            }
            Ok(false) => {}
            Err(e) => {
                c.fail(label, format_args!("scan aborted: {:?}", e));
                return false;
            }
        }
    }
    let _ = write!(c.w, "[INFO] {} devices:", label);
    for i in 0..n {
        let _ = write!(c.w, " 0x{:02X}", found[i]);
    }
    if n == 0 {
        let _ = write!(c.w, " none");
    }
    let _ = c.w.write_str("\r\n");
    let present = found[..n].contains(&expected);
    if present {
        c.pass(label, format_args!("expected device 0x{:02X} acknowledges", expected));
    } else {
        c.fail(label, format_args!("no acknowledge from 0x{:02X}", expected));
    }
    if n > usize::from(present) {
        c.warn(label, format_args!("unexpected additional devices on the bus"));
    }
    present
}

// ---------------------------------------------------------------- BMI160

fn diag_bmi160<W: Write, SCL: Line, SDA: Line>(c: &mut Ctx<W>, bus: &mut BitI2c<SCL, SDA>) {
    let name = "BMI160";
    match bus.read_reg(BMI_ADDR, 0x00) {
        Ok(0xD1) => c.pass(name, format_args!("CHIP_ID 0xD1")),
        Ok(v) => {
            c.fail(name, format_args!("CHIP_ID 0x{:02X}, expected 0xD1", v));
            return;
        }
        Err(e) => {
            c.fail(name, format_args!("CHIP_ID read failed: {:?}", e));
            return;
        }
    }

    // Power up accelerometer and gyroscope, then set ranges (+-2 g, +-2000 dps).
    let w1 = bus.write_regs(BMI_ADDR, 0x7E, &[0x11]);
    delay_ms(10);
    let w2 = bus.write_regs(BMI_ADDR, 0x7E, &[0x15]);
    delay_ms(100);
    let w3 = bus.write_regs(BMI_ADDR, 0x41, &[0x03]);
    let w4 = bus.write_regs(BMI_ADDR, 0x43, &[0x00]);
    if w1.is_err() || w2.is_err() || w3.is_err() || w4.is_err() {
        c.fail(name, format_args!("configuration writes failed"));
        return;
    }

    match bus.read_reg(BMI_ADDR, 0x03) {
        Ok(0x14) => c.pass(name, format_args!("PMU_STATUS 0x14 (accel and gyro in normal mode)")),
        Ok(v) => c.fail(name, format_args!("PMU_STATUS 0x{:02X}, expected 0x14", v)),
        Err(e) => c.fail(name, format_args!("PMU_STATUS read failed: {:?}", e)),
    }
    match bus.read_reg(BMI_ADDR, 0x02) {
        Ok(0) => c.pass(name, format_args!("ERR_REG clear")),
        Ok(v) => c.fail(name, format_args!("ERR_REG 0x{:02X}", v)),
        Err(e) => c.fail(name, format_args!("ERR_REG read failed: {:?}", e)),
    }

    let mut gsum = [0i32; 3];
    let mut asum = [0i32; 3];
    let mut gmin = [i16::MAX; 3];
    let mut gmax = [i16::MIN; 3];
    let mut good = 0i32;
    for _ in 0..32 {
        let mut d = [0u8; 12];
        if bus.read_regs(BMI_ADDR, 0x0C, &mut d).is_err() {
            }
        good += 1;
        for k in 0..3 {
            let g = i16::from_le_bytes([d[2 * k], d[2 * k + 1]]);
            let a = i16::from_le_bytes([d[6 + 2 * k], d[7 + 2 * k]]);
            gsum[k] += g as i32;
            asum[k] += a as i32;
            gmin[k] = gmin[k].min(g);
            gmax[k] = gmax[k].max(g);
        }
        delay_ms(12);
    }
    if good < 24 {
        c.fail(name, format_args!("only {} of 32 sample reads succeeded", good));
        return;
    }
    let gm = [gsum[0] / good, gsum[1] / good, gsum[2] / good];
    let am = [asum[0] / good, asum[1] / good, asum[2] / good];

    c.info(
        name,
        format_args!(
            "accel [g] x={} y={} z={}",
            Fixed(am[0] * 1000 / 16384, 1000),
            Fixed(am[1] * 1000 / 16384, 1000),
            Fixed(am[2] * 1000 / 16384, 1000)
        ),
    );
    c.info(
        name,
        format_args!(
            "gyro mean [dps] x={} y={} z={}",
            Fixed(gm[0] * 100 / 164, 10),
            Fixed(gm[1] * 100 / 164, 10),
            Fixed(gm[2] * 100 / 164, 10)
        ),
    );

    let m2: i64 = am.iter().map(|&v| (v as i64) * (v as i64)).sum::<i64>();
    let g2: i64 = 16384 * 16384;
    if m2 > g2 * 7225 / 10000 && m2 < g2 * 13225 / 10000 {
        c.pass(name, format_args!("accel magnitude within 0.85..1.15 g at rest"));
    } else {
        c.fail(name, format_args!("accel magnitude outside 0.85..1.15 g: sensor fault, motion, or wrong range"));
    }

    let bias_ok = gm.iter().all(|v| v.abs() <= 82);
    let p2p_ok = (0..3).all(|k| (gmax[k] as i32 - gmin[k] as i32) <= 50);
    if bias_ok && p2p_ok {
        c.pass(name, format_args!("gyro bias below 5 dps and noise below 3 dps peak-to-peak"));
    } else {
        c.warn(name, format_args!("gyro bias or noise above heuristic limits: keep the robot still and repeat"));
    }
}

// ---------------------------------------------------------------- AS5600

fn wrap_delta(a: u16, b: u16) -> i32 {
    let d = (b as i32 - a as i32) & 0x0FFF;
    if d >= 2048 { d - 4096 } else { d }
}

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

fn diag_as5600<W: Write, SCL: Line, SDA: Line>(c: &mut Ctx<W>, bus: &mut BitI2c<SCL, SDA>) -> bool {
    let name = "AS5600";
    let (status, angle0) = match read_status_angle(bus) {
        Ok(v) => v,
        Err(e) => {
            c.fail(name, format_args!("register read failed: {:?}", e));
            return false;
        }
    };
    c.pass(
        name,
        format_args!("raw angle {} ({} deg), STATUS 0x{:02X}", angle0, Fixed(angle0 as i32 * 3600 / 4096, 10), status),
    );

    let md = status & 0x20 != 0;
    let ml = status & 0x10 != 0;
    let mh = status & 0x08 != 0;
    let mut magnet_ok = false;
    if !md {
        c.fail(name, format_args!("magnet not detected: check presence, diametric polarity, air gap"));
    } else if ml {
        c.warn(name, format_args!("magnet too weak: reduce the air gap"));
    } else if mh {
        c.warn(name, format_args!("magnet too strong: increase the air gap"));
    } else {
        c.pass(name, format_args!("magnet detected, field within range"));
        magnet_ok = true;
    }
    if let Ok((agc, mag)) = read_agc_mag(bus) {
        c.info(name, format_args!("AGC {} MAGNITUDE {}", agc, mag));
    }

    // Stationary jitter.
    let mut lo = 0i32;
    let mut hi = 0i32;
    for _ in 0..20 {
        if let Ok((_, a)) = read_status_angle(bus) {
            let d = wrap_delta(angle0, a);
            lo = lo.min(d);
            hi = hi.max(d);
        }
        delay_ms(5);
    }
    if hi - lo <= 6 {
        c.pass(name, format_args!("stationary jitter {} counts", hi - lo));
    } else {
        c.warn(name, format_args!("stationary jitter {} counts: vibration, loose magnet, or noise", hi - lo));
    }

    // Manual rotation.
    c.text("[ACTION] Rotate the wheel by hand for about 6 seconds, starting after this line.");
    delay_ms(1000);
    let t0 = DWT::cycle_count();
    let mut prev = angle0;
    let mut total = 0i32;
    let mut tmin = 0i32;
    let mut tmax = 0i32;
    let mut errs = 0u32;
    let (mut seen_ml, mut seen_mh, mut seen_lost) = (false, false, false);
    let (mut agc_min, mut agc_max, mut mag_min, mut mag_max) = (255u8, 0u8, 4095u16, 0u16);
    let mut iter = 0u32;
    while DWT::cycle_count().wrapping_sub(t0) < 6 * CPU_HZ {
        match read_status_angle(bus) {
            Ok((st, a)) => {
                total += wrap_delta(prev, a);
                prev = a;
                tmin = tmin.min(total);
                tmax = tmax.max(total);
                seen_ml |= st & 0x10 != 0;
                seen_mh |= st & 0x08 != 0;
                seen_lost |= st & 0x20 == 0;
            }
            Err(_) => errs += 1,
        }
        if iter % 8 == 0 {
            if let Ok((agc, mag)) = read_agc_mag(bus) {
                agc_min = agc_min.min(agc);
                agc_max = agc_max.max(agc);
                mag_min = mag_min.min(mag);
                mag_max = mag_max.max(mag);
            }
        }
        iter += 1;
        delay_ms(2);
    }
    let span = tmax - tmin;
    c.info(
        name,
        format_args!(
            "travel span {} deg, net {} deg (positive = raw angle increasing), read errors {}",
            Fixed(span * 3600 / 4096, 10),
            Fixed(total * 3600 / 4096, 10),
            errs
        ),
    );
    c.info(name, format_args!("AGC {}..{} MAGNITUDE {}..{}", agc_min, agc_max, mag_min, mag_max));
    if span >= 1024 && errs == 0 {
        c.pass(name, format_args!("rotation tracked over at least 90 deg without read errors"));
    } else {
        c.fail(name, format_args!("rotation not tracked (wheel not turned, magnet not on the shaft, or read errors)"));
    }
    if seen_lost || seen_ml || seen_mh {
        c.warn(name, format_args!("magnet status left the valid window during rotation: centering or air gap"));
    }
    if mag_max > 0 && (mag_max - mag_min) as u32 * 100 / mag_max as u32 > 30 {
        c.warn(name, format_args!("MAGNITUDE varies more than 30 percent over rotation: eccentric or tilted magnet (heuristic limit)"));
    }
    magnet_ok && span >= 1024
}

// ---------------------------------------------------------------- DRV8833

fn diag_drv<W: Write, S: OutputPin, F: InputPin>(
    c: &mut Ctx<W>,
    nsleep_floating_high: bool,
    nsleep: &mut S,
    nfault: &mut F,
) -> bool {
    let name = "DRV8833";
    let mut ok = true;
    if nsleep_floating_high {
        c.fail(name, format_args!("PB0 (nSLEEP) reads high while floating: a pull-up exists, the driver would wake at reset"));
        ok = false;
    } else {
        c.pass(name, format_args!("PB0 (nSLEEP) reads low while floating (absence of a pull-down is not detectable here)"));
    }
    drive(nsleep, false);
    delay_ms(2);
    let high_asleep = level(nfault);
    drive(nsleep, true);
    delay_ms(5);
    let high_awake = level(nfault);
    drive(nsleep, false);
    if !high_asleep {
        c.fail(name, format_args!("nFAULT (PB1) low while asleep: no pull-up on the line, or shorted to GND"));
        ok = false;
    } else if !high_awake {
        c.fail(name, format_args!("nFAULT low after wake: VM missing or too low, output short, or over-temperature"));
        ok = false;
    } else {
        c.pass(name, format_args!("nFAULT high asleep and awake"));
    }
    ok
}

// ---------------------------------------------------------------- motor + encoder

fn pwm_burst<A: OutputPin, B: OutputPin>(hot: &mut A, other: &mut B, duty_pct: u32, ms: u32) {
    drive(other, false);
    let on = 1000 * duty_pct / 100;
    let off = 1000 - on;
    for _ in 0..ms {
        drive(hot, true);
        delay_us(on);
        drive(hot, false);
        delay_us(off);
    }
    drive(hot, false);
}

fn diag_motor<W, A, B, S, F, SCL, SDA>(
    c: &mut Ctx<W>,
    in1: &mut A,
    in2: &mut B,
    nsleep: &mut S,
    nfault: &mut F,
    enc: &mut BitI2c<SCL, SDA>,
) where
    W: Write,
    A: OutputPin,
    B: OutputPin,
    S: OutputPin,
    F: InputPin,
    SCL: Line,
    SDA: Line,
{
    let name = "Motor";
    let a0 = match read_status_angle(enc) {
        Ok((_, a)) => a,
        Err(_) => {
            c.fail(name, format_args!("encoder read failed"));
            return;
        }
    };
    drive(nsleep, true);
    delay_ms(5);

    pwm_burst(in1, in2, MOTOR_DUTY_PCT, MOTOR_BURST_MS);
    let fault_fwd = !level(nfault);
    delay_ms(300);
    let a1 = read_status_angle(enc).map(|v| v.1);

    pwm_burst(in2, in1, MOTOR_DUTY_PCT, MOTOR_BURST_MS);
    let fault_rev = !level(nfault);
    delay_ms(300);
    let a2 = read_status_angle(enc).map(|v| v.1);

    drive(nsleep, false);
    drive(in1, false);
    drive(in2, false);

    let (a1, a2) = match (a1, a2) {
        (Ok(x), Ok(y)) => (x, y),
        _ => {
            c.fail(name, format_args!("encoder read failed after drive"));
            return;
        }
    };
    let d_fwd = wrap_delta(a0, a1);
    let d_rev = wrap_delta(a1, a2);
    c.info(
        name,
        format_args!(
            "IN1-driven burst: {} deg, IN2-driven burst: {} deg",
            Fixed(d_fwd * 3600 / 4096, 10),
            Fixed(d_rev * 3600 / 4096, 10)
        ),
    );
    if fault_fwd || fault_rev {
        c.fail(name, format_args!("nFAULT asserted during drive: overcurrent or thermal event"));
    }
    if d_fwd.abs() >= MOTOR_MIN_COUNTS && d_rev.abs() >= MOTOR_MIN_COUNTS && (d_fwd > 0) != (d_rev > 0) {
        c.pass(name, format_args!("both directions move the encoder with opposite signs"));
        if d_fwd > 0 {
            c.info(name, format_args!("polarity: IN1-driven burst increases the raw angle"));
        } else {
            c.info(name, format_args!("polarity: IN1-driven burst decreases the raw angle (invert in firmware)"));
        }
    } else {
        c.fail(
            name,
            format_args!("motion missing or same sign: motor not turning, IN1/IN2 wiring, encoder not on the driven axle, or burst too weak"),
        );
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
    let mut gpioa = dp.GPIOA.split(&mut rcc);
    let mut gpiob = dp.GPIOB.split(&mut rcc);
    let mut gpioc = dp.GPIOC.split(&mut rcc);

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    drive(&mut led, true); // active low: off

    let mut tx = gpioa.pa9.into_push_pull_output(&mut gpioa.crh);
    drive(&mut tx, true);
    let mut rx = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);

    let mut in1 = gpioa.pa6.into_push_pull_output(&mut gpioa.crl);
    let mut in2 = gpioa.pa7.into_push_pull_output(&mut gpioa.crl);
    drive(&mut in1, false);
    drive(&mut in2, false);

    let mut nsleep_probe = gpiob.pb0.into_floating_input(&mut gpiob.crl);
    let pb0_floating_high = level(&mut nsleep_probe);
    let mut nsleep = nsleep_probe.into_push_pull_output(&mut gpiob.crl);
    drive(&mut nsleep, false);
    let mut nfault = gpiob.pb1.into_floating_input(&mut gpiob.crl);

    let scl1 = gpiob.pb6.into_open_drain_output(&mut gpiob.crl);
    let sda1 = gpiob.pb7.into_open_drain_output(&mut gpiob.crl);
    let scl2 = gpiob.pb10.into_open_drain_output(&mut gpiob.crh);
    let sda2 = gpiob.pb11.into_open_drain_output(&mut gpiob.crh);
    let mut bus1 = BitI2c::new(scl1, sda1);
    let mut bus2 = BitI2c::new(scl2, sda2);

    delay_ms(50);
    let mut c = Ctx::new(Uart { tx });

    c.text("");
    c.text("=== Self-balancing robot: hardware diagnostic v1 ===");
    c.text("Serial 9600 8N1. TX: bit-banged on PA9. RX: start-bit detection on PA10.");

    // Serial RX test doubles as the start trigger.
    let rx_idle_high = level(&mut rx);
    let mut rx_ok = false;
    if !rx_idle_high {
        c.fail("UART RX", format_args!("PA10 reads low at idle: short to GND or adapter holding the line low"));
        delay_ms(3000);
    } else {
        for i in 0..20u32 {
            let _ = write!(c.w, "Send any character to start ({} s left)\r\n", 20 - i);
            drive(&mut led, i % 2 == 1);
            if wait_start_bit(&mut rx, 1000) {
                rx_ok = true;
                break;
            }
        }
        delay_ms(5);
        if rx_ok {
            c.pass("UART RX", format_args!("start bit detected on PA10"));
        } else {
            c.fail("UART RX", format_args!("no start bit within 20 s: PA10 to adapter TX, GND, baud 9600"));
        }
    }
    drive(&mut led, true);

    c.text("--- I2C1 (PB6 SCL, PB7 SDA): BMI160 ---");
    if diag_bus(&mut c, "I2C1 lines", &mut bus1) && diag_scan(&mut c, "I2C1 scan", &mut bus1, BMI_ADDR) {
        diag_bmi160(&mut c, &mut bus1);
    }

    c.text("--- I2C2 (PB10 SCL, PB11 SDA): AS5600 ---");
    let mut as_ok = false;
    if diag_bus(&mut c, "I2C2 lines", &mut bus2) && diag_scan(&mut c, "I2C2 scan", &mut bus2, AS_ADDR) {
        as_ok = diag_as5600(&mut c, &mut bus2);
    }

    c.text("--- DRV8833 (PB0 nSLEEP, PB1 nFAULT) ---");
    let drv_ok = diag_drv(&mut c, pb0_floating_high, &mut nsleep, &mut nfault);

    c.text("--- Motor and encoder integration ---");
    if rx_ok && as_ok && drv_ok {
        c.text("[ACTION] The wheel must spin freely (robot on a stand). Do not press keys until now.");
        c.text("[ACTION] Send any character within 15 s to run the motor test; otherwise it is skipped.");
        delay_ms(1000);
        if wait_start_bit(&mut rx, 15_000) {
            delay_ms(5);
            diag_motor(&mut c, &mut in1, &mut in2, &mut nsleep, &mut nfault, &mut bus2);
        } else {
            c.info("Motor", format_args!("skipped (no character received)"));
        }
    } else {
        c.warn("Motor", format_args!("skipped: requires UART RX, AS5600 and DRV8833 checks to pass"));
    }

    drive(&mut nsleep, false);
    drive(&mut in1, false);
    drive(&mut in2, false);

    c.text("=== Summary ===");
    let (p, f, w) = (c.n_pass, c.n_fail, c.n_warn);
    let _ = write!(c.w, "PASS={} FAIL={} WARN={}\r\n", p, f, w);
    c.text(if f == 0 { "Result: no failures" } else { "Result: failures present, see FAIL lines" });

    let half = if f == 0 { 500 } else { 100 };
    loop {
        drive(&mut led, false);
        delay_ms(half);
        drive(&mut led, true);
        delay_ms(half);
    }
}
