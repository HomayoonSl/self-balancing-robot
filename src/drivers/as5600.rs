use super::bit_i2c::{BitI2c, Line};

pub fn wrap_delta(a: u16, b: u16) -> i32 {
    let d = (b as i32 - a as i32) & 0x0FFF;
    if d >= 2048 { d - 4096 } else { d }
}

pub fn read_raw_angle<SCL: Line, SDA: Line>(bus: &mut BitI2c<SCL, SDA>, as_addr: u8, half_us: u32) -> Option<u16> {
    let mut d = [0u8; 2];
    bus.read_regs(as_addr, 0x0C, &mut d, half_us).ok()?;
    Some((((d[0] & 0x0F) as u16) << 8) | d[1] as u16)
}
