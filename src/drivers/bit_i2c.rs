use cortex_m::peripheral::DWT;
use embedded_hal::digital::{InputPin, OutputPin};

pub trait Line: OutputPin + InputPin {}
impl<T: OutputPin + InputPin> Line for T {}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2cError {
    SclStuckLow,
    Nack,
}

pub struct BitI2c<SCL: Line, SDA: Line> {
    pub scl: SCL,
    pub sda: SDA,
    half_cycles: u32,
    timeout_cycles: u32,
}

impl<SCL: Line, SDA: Line> BitI2c<SCL, SDA> {
    pub fn new(mut scl: SCL, mut sda: SDA, half_us: u32, cpu_hz: u32) -> Self {
        Self::drive_pin(&mut scl, true);
        Self::drive_pin(&mut sda, true);
        let cyc_per_us = cpu_hz / 1_000_000;
        let half_cycles = half_us * cyc_per_us;
        let timeout_cycles = 2000 * cyc_per_us; // 2 ms clock stretching timeout
        Self {
            scl,
            sda,
            half_cycles,
            timeout_cycles,
        }
    }

    #[inline(always)]
    fn drive_pin<P: OutputPin>(p: &mut P, high: bool) {
        let _ = if high { p.set_high() } else { p.set_low() };
    }

    #[inline(always)]
    fn level_pin<P: InputPin>(p: &mut P) -> bool {
        p.is_high().unwrap_or(false)
    }

    #[inline(always)]
    fn delay_cycles(&self, cycles: u32) {
        let start = DWT::cycle_count();
        while DWT::cycle_count().wrapping_sub(start) < cycles {}
    }

    #[inline(always)]
    fn half_delay(&self) {
        self.delay_cycles(self.half_cycles);
    }

    pub fn scl_high(&mut self) -> Result<(), I2cError> {
        Self::drive_pin(&mut self.scl, true);
        let start = DWT::cycle_count();
        while !Self::level_pin(&mut self.scl) {
            if DWT::cycle_count().wrapping_sub(start) >= self.timeout_cycles {
                return Err(I2cError::SclStuckLow);
            }
        }
        Ok(())
    }

    pub fn start(&mut self) -> Result<(), I2cError> {
        Self::drive_pin(&mut self.sda, true);
        self.half_delay();
        self.scl_high()?;
        self.half_delay();
        Self::drive_pin(&mut self.sda, false);
        self.half_delay();
        Self::drive_pin(&mut self.scl, false);
        self.half_delay();
        Ok(())
    }

    pub fn stop(&mut self) -> Result<(), I2cError> {
        Self::drive_pin(&mut self.sda, false);
        self.half_delay();
        self.scl_high()?;
        self.half_delay();
        Self::drive_pin(&mut self.sda, true);
        self.half_delay();
        Ok(())
    }

    pub fn write_byte(&mut self, b: u8) -> Result<bool, I2cError> {
        for i in (0..8).rev() {
            Self::drive_pin(&mut self.sda, (b >> i) & 1 == 1);
            self.half_delay();
            self.scl_high()?;
            self.half_delay();
            Self::drive_pin(&mut self.scl, false);
            self.half_delay();
        }
        Self::drive_pin(&mut self.sda, true);
        self.half_delay();
        self.scl_high()?;
        self.half_delay();
        let ack = !Self::level_pin(&mut self.sda);
        Self::drive_pin(&mut self.scl, false);
        self.half_delay();
        Ok(ack)
    }

    pub fn read_byte(&mut self, ack: bool) -> Result<u8, I2cError> {
        Self::drive_pin(&mut self.sda, true);
        let mut v = 0u8;
        for _ in 0..8 {
            self.half_delay();
            self.scl_high()?;
            self.half_delay();
            v = (v << 1) | (Self::level_pin(&mut self.sda) as u8);
            Self::drive_pin(&mut self.scl, false);
            self.half_delay();
        }
        Self::drive_pin(&mut self.sda, !ack);
        self.half_delay();
        self.scl_high()?;
        self.half_delay();
        Self::drive_pin(&mut self.scl, false);
        self.half_delay();
        Self::drive_pin(&mut self.sda, true);
        Ok(v)
    }

    pub fn write_regs(&mut self, addr: u8, reg: u8, data: &[u8], _half_us: u32) -> Result<(), I2cError> {
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

    pub fn read_regs(&mut self, addr: u8, reg: u8, buf: &mut [u8], _half_us: u32) -> Result<(), I2cError> {
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

    pub fn read_reg(&mut self, addr: u8, reg: u8, _half_us: u32) -> Result<u8, I2cError> {
        let mut b = [0u8; 1];
        self.read_regs(addr, reg, &mut b, _half_us)?;
        Ok(b[0])
    }
}
