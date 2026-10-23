pub fn deadzone_comp(v: f32, dz_pos: f32, dz_neg: f32, dz_soft: f32) -> f32 {
    let dz = if v >= 0.0 { dz_pos } else { dz_neg };
    let norm = v / dz_soft;
    let clamped = if norm < -1.0 {
        -1.0
    } else if norm > 1.0 {
        1.0
    } else {
        norm
    };
    v + dz * clamped
}

pub fn apply_cmd(
    set1: &mut impl FnMut(u16),
    set2: &mut impl FnMut(u16),
    max: u16,
    cmd_mv: i32,
    vbat_mv: u32,
    slow_decay: bool,
) -> i32 {
    let mag_mv = cmd_mv.unsigned_abs() as u64;
    let mut mag = (mag_mv * max as u64 / vbat_mv.max(1) as u64) as u32;
    if mag > max as u32 {
        mag = max as u32;
    }
    let m = mag as u16;
    if slow_decay {
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
    if cmd_mv >= 0 {
        m as i32
    } else {
        -(m as i32)
    }
}
