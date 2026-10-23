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
}

impl<SCL: Line, SDA: Line> BitI2c<SCL, SDA> {
    pub fn new(mut scl: SCL, mut sda: SDA, _half_us: u32, _cpu_hz: u32) -> Self {
        let _ = if cfg!(debug_assertions) { 0 } else { 0 };
        Self::drive_pin(&mut scl, true);
        Self::drive_pin(&mut sda, true);
        Self { scl, sda }
    }

    fn drive_pin<P: OutputPin>(p: &mut P, high: bool) {
        let _ = if high { p.set_high() } else { p.set_low() };
    }

    fn level_pin<P: InputPin>(p: &mut P) -> bool {
        p.is_high().unwrap_or(false)
    }

    pub fn start(&mut self, _half_us: u32) -> Result<(), I2cError> {
        Self::drive_pin(&mut self.sda, true);
        cortex_m::asm::delay(_half_us * 48); 
        self.scl_high(_half_us)?;
        cortex_m::asm::delay(_half_us * 48);
        Self::drive_pin(&mut self.sda, false);
        cortex_m::asm::delay(_half_us * 48);
        Self::drive_pin(&mut self.scl, false);
        cortex_m::asm::delay(_half_us * 48);
        Ok(())
    }

    pub fn stop(&mut self, _half_us: u32) -> Result<(), I2cError> {
        Self::drive_pin(&mut self.sda, false);
        cortex_m::asm::delay(_half_us * 48);
        self.scl_high(_half_us)?;
        cortex_m::asm::delay(_half_us * 48);
        Self::drive_pin(&mut self.sda, true);
        cortex_m::asm::delay(_half_us * 48);
        Ok(())
    }

    pub fn scl_high(&mut self, _half_us: u32) -> Result<(), I2cError> {
        Self::drive_pin(&mut self.scl, true);
        let mut waited_us = 0u32;
        while !Self::level_pin(&mut self.scl) {
            if waited_us >= 2000 {
                return Err(I2cError::SclStuckLow);
            }
            cortex_m::asm::delay(480);
            waited_us += 10;
        }
        Ok(())
    }

    pub fn write_byte(&mut self, b: u8, _half_us: u32) -> Result<bool, I2cError> {
        for i in (0..8).rev() {
            Self::drive_pin(&mut self.sda, (b >> i) & 1 == 1);
            cortex_m::asm::delay(_half_us * 48);
            self.scl_high(_half_us)?;
            cortex_m::asm::delay(_half_us * 48);
            Self::drive_pin(&mut self.scl, false);
            cortex_m::asm::delay(_half_us * 48);
        }
        Self::drive_pin(&mut self.sda, true);
        cortex_m::asm::delay(_half_us * 48);
        self.scl_high(_half_us)?;
        cortex_m::asm::delay(_half_us * 48);
        let ack = !Self::level_pin(&mut self.sda);
        Self::drive_pin(&mut self.scl, false);
        cortex_m::asm::delay(_half_us * 48);
        Ok(ack)
    }

    pub fn read_byte(&mut self, ack: bool, _half_us: u32) -> Result<u8, I2cError> {
        Self::drive_pin(&mut self.sda, true);
        let mut v = 0u8;
        for _ in 0..8 {
            cortex_m::asm::delay(_half_us * 48);
            self.scl_high(_half_us)?;
            cortex_m::asm::delay(_half_us * 48);
            v = (v << 1) | (Self::level_pin(&mut self.sda) as u8);
            Self::drive_pin(&mut self.scl, false);
            cortex_m::asm::delay(_half_us * 48);
        }
        Self::drive_pin(&mut self.sda, !ack);
        cortex_m::asm::delay(_half_us * 48);
        self.scl_high(_half_us)?;
        cortex_m::asm::delay(_half_us * 48);
        Self::drive_pin(&mut self.scl, false);
        cortex_m::asm::delay(_half_us * 48);
        Self::drive_pin(&mut self.sda, true);
        Ok(v)
    }

    pub fn write_regs(&mut self, addr: u8, reg: u8, data: &[u8], _half_us: u32) -> Result<(), I2cError> {
        let r = self.write_regs_inner(addr, reg, data, _half_us);
        let _ = self.stop(_half_us);
        r
    }

    fn write_regs_inner(&mut self, addr: u8, reg: u8, data: &[u8], _half_us: u32) -> Result<(), I2cError> {
        self.start(_half_us)?;
        if !self.write_byte(addr << 1, _half_us)? {
            return Err(I2cError::Nack);
        }
        if !self.write_byte(reg, _half_us)? {
            return Err(I2cError::Nack);
        }
        for &b in data {
            if !self.write_byte(b, _half_us)? {
                return Err(I2cError::Nack);
            }
        }
        Ok(())
    }

    pub fn read_regs(&mut self, addr: u8, reg: u8, buf: &mut [u8], _half_us: u32) -> Result<(), I2cError> {
        let r = self.read_regs_inner(addr, reg, buf, _half_us);
        let _ = self.stop(_half_us);
        r
    }

    fn read_regs_inner(&mut self, addr: u8, reg: u8, buf: &mut [u8], _half_us: u32) -> Result<(), I2cError> {
        self.start(_half_us)?;
        if !self.write_byte(addr << 1, _half_us)? {
            return Err(I2cError::Nack);
        }
        if !self.write_byte(reg, _half_us)? {
            return Err(I2cError::Nack);
        }
        self.start(_half_us)?;
        if !self.write_byte((addr << 1) | 1, _half_us)? {
            return Err(I2cError::Nack);
        }
        let n = buf.len();
        for i in 0..n {
            buf[i] = self.read_byte(i + 1 < n, _half_us)?;
        }
        Ok(())
    }

    pub fn read_reg(&mut self, addr: u8, reg: u8, _half_us: u32) -> Result<u8, I2cError> {
        let mut b = [0u8; 1];
        self.read_regs(addr, reg, &mut b, _half_us)?;
        Ok(b[0])
    }
}
