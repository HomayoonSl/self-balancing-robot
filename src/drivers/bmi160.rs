use super::bit_i2c::{BitI2c, Line};

#[derive(Clone, Copy)]
pub struct ImuRaw {
    pub g: [i16; 3],
    pub a: [i16; 3],
}

pub fn bmi_init<SCL: Line, SDA: Line>(bus: &mut BitI2c<SCL, SDA>, bmi_addr: u8, half_us: u32) -> bool {
    if bus.read_reg(bmi_addr, 0x00, half_us) != Ok(0xD1) {
        return false;
    }
    let mut ok = true;
    ok &= bus.write_regs(bmi_addr, 0x7E, &[0x11], half_us).is_ok();
    cortex_m::asm::delay(800_000);
    ok &= bus.write_regs(bmi_addr, 0x7E, &[0x15], half_us).is_ok();
    cortex_m::asm::delay(8_000_000);
    ok &= bus.write_regs(bmi_addr, 0x40, &[0x2B], half_us).is_ok();
    ok &= bus.write_regs(bmi_addr, 0x41, &[0x03], half_us).is_ok();
    ok &= bus.write_regs(bmi_addr, 0x42, &[0x2B], half_us).is_ok();
    ok &= bus.write_regs(bmi_addr, 0x43, &[0x01], half_us).is_ok();
    cortex_m::asm::delay(1_600_000);
    ok && bus.read_reg(bmi_addr, 0x03, half_us) == Ok(0x14)
}

pub fn read_imu<SCL: Line, SDA: Line>(bus: &mut BitI2c<SCL, SDA>, bmi_addr: u8, half_us: u32) -> Option<ImuRaw> {
    let mut d = [0u8; 12];
    bus.read_regs(bmi_addr, 0x0C, &mut d, half_us).ok()?;
    let mut g = [0i16; 3];
    let mut a = [0i16; 3];
    for k in 0..3 {
        g[k] = i16::from_le_bytes([d[2 * k], d[2 * k + 1]]);
        a[k] = i16::from_le_bytes([d[6 + 2 * k], d[7 + 2 * k]]);
    }
    Some(ImuRaw { g, a })
}
