use crate::drivers::{read_imu, ImuRaw};
use crate::drivers::BitI2c;
use crate::drivers::Line;

pub struct Calib {
    pub bias_rad: f32,
    pub theta0: f32,
}

#[derive(Clone, Copy)]
pub enum CalFail {
    Read,
    Moving,
    Axis,
}

pub fn cal_fail_text(e: CalFail) -> &'static str {
    match e {
        CalFail::Read => "IMU read failed",
        CalFail::Moving => "robot not still (gyro peak-to-peak above limit)",
        CalFail::Axis => "gravity not on the configured up axis, or tilt above about 23 deg",
    }
}

pub fn calibrate<SCL: Line, SDA: Line>(
    imu: &mut BitI2c<SCL, SDA>,
    bmi_addr: u8,
    half_us: u32,
    dt_us: u32,
    cpu_hz: u32,
    still_p2p_dps: f32,
    gyro_lsb_per_dps: f32,
    acc_up_axis: usize,
    acc_up_sign: f32,
    acc_fwd_axis: usize,
    acc_fwd_sign: f32,
    acc_lsb_per_g: f32,
    gyro_pitch_axis: usize,
    gyro_sign: f32,
    gyro_rad_per_lsb: f32,
) -> Result<Calib, CalFail> {
    let n: i32 = 400;
    let mut gsum = [0i32; 3];
    let mut asum = [0i32; 3];
    let mut gmin = [i16::MAX; 3];
    let mut gmax = [i16::MIN; 3];
    
    let period = dt_us * (cpu_hz / 1_000_000);

    for _ in 0..n {
        let start = cortex_m::peripheral::DWT::cycle_count();
        let s = read_imu(imu, bmi_addr, half_us).ok_or(CalFail::Read)?;
        for k in 0..3 {
            gsum[k] += s.g[k] as i32;
            asum[k] += s.a[k] as i32;
            gmin[k] = gmin[k].min(s.g[k]);
            gmax[k] = gmax[k].max(s.g[k]);
        }
        while cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start) < period {}
    }

    let limit = (still_p2p_dps * gyro_lsb_per_dps) as i32;
    for k in 0..3 {
        if (gmax[k] as i32 - gmin[k] as i32) > limit {
            return Err(CalFail::Moving);
        }
    }

    let nf = n as f32;
    let au = acc_up_sign * (asum[acc_up_axis] as f32) / nf / acc_lsb_per_g;
    let af = acc_fwd_sign * (asum[acc_fwd_axis] as f32) / nf / acc_lsb_per_g;
    if au < 0.85 || au > 1.15 || af.abs() > 0.4 {
        return Err(CalFail::Axis);
    }
    let g_mean = (gsum[gyro_pitch_axis] as f32) / nf;
    Ok(Calib {
        bias_rad: gyro_sign * g_mean * gyro_rad_per_lsb,
        theta0: -af / au,
    })
}

pub struct Filter {
    theta_abs: f32,
    bias: f32,
    theta0: f32,
    sign: f32,
    kp_acc: f32,
    ki_bias: f32,
    dt: f32,
    gyro_rad_per_lsb: f32,
    acc_up_sign: f32,
    acc_lsb_per_g: f32,
    acc_fwd_sign: f32,
    gyro_pitch_axis: usize,
    acc_up_axis: usize,
    acc_fwd_axis: usize,
}

impl Filter {
    pub fn new(
        cal: &Calib,
        gyro_sign: f32,
        kp_acc: f32,
        ki_bias: f32,
        dt: f32,
        gyro_rad_per_lsb: f32,
        acc_up_sign: f32,
        acc_lsb_per_g: f32,
        acc_fwd_sign: f32,
        gyro_pitch_axis: usize,
        acc_up_axis: usize,
        acc_fwd_axis: usize,
    ) -> Self {
        Self {
            theta_abs: cal.theta0,
            bias: cal.bias_rad,
            theta0: cal.theta0,
            sign: gyro_sign,
            kp_acc,
            ki_bias,
            dt,
            gyro_rad_per_lsb,
            acc_up_sign,
            acc_lsb_per_g,
            acc_fwd_sign,
            gyro_pitch_axis,
            acc_up_axis,
            acc_fwd_axis,
        }
    }

    pub fn update(&mut self, s: &ImuRaw) -> (f32, f32) {
        let meas = self.sign * (s.g[self.gyro_pitch_axis] as f32) * self.gyro_rad_per_lsb;
        let au = self.acc_up_sign * (s.a[self.acc_up_axis] as f32);
        let af = self.acc_fwd_sign * (s.a[self.acc_fwd_axis] as f32);
        let mut e = 0.0f32;
        if au > 0.5 * self.acc_lsb_per_g {
            e = -af / au - self.theta_abs;
        }
        self.bias -= self.ki_bias * e * self.dt;
        let rate = meas - self.bias;
        self.theta_abs += (rate + self.kp_acc * e) * self.dt;
        (self.theta_abs - self.theta0, rate)
    }
}
