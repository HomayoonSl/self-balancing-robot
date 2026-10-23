use core::fmt::{self, Write};
use embedded_hal::digital::OutputPin;

pub struct Uart<TX: OutputPin> {
    pub tx: TX,
    pub bit_cycles: u32,
}

impl<TX: OutputPin> Uart<TX> {
    pub fn new(tx: TX, bit_cycles: u32) -> Self {
        Self { tx, bit_cycles }
    }
}

impl<TX: OutputPin> Write for Uart<TX> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for b in s.bytes() {
            let start = cortex_m::peripheral::DWT::cycle_count();
            let _ = self.tx.set_low();
            while cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start) < self.bit_cycles {}
            
            for i in 0..8u32 {
                if (b >> i) & 1 == 1 {
                    let _ = self.tx.set_high();
                } else {
                    let _ = self.tx.set_low();
                }
                while cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start) < self.bit_cycles * (i + 2) {}
            }
            let _ = self.tx.set_high();
            while cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start) < self.bit_cycles * 10 {}
        }
        Ok(())
    }
}
