pub struct Wheel {
    prev_raw: u16,
    counts: i32,
    p1: f32,
    p2: f32,
    rate: f32,
    enc_sign: f32,
    rad_per_count: f32,
    dt: f32,
}

impl Wheel {
    pub fn new(raw: u16, enc_sign: f32, rad_per_count: f32, dt: f32) -> Self {
        Self {
            prev_raw: raw,
            counts: 0,
            p1: 0.0,
            p2: 0.0,
            rate: 0.0,
            enc_sign,
            rad_per_count,
            dt,
        }
    }

    pub fn update(&mut self, raw: u16) -> (f32, f32) {
        self.counts += crate::drivers::wrap_delta(self.prev_raw, raw);
        self.prev_raw = raw;
        let b = self.enc_sign * (self.counts as f32) * self.rad_per_count;
        let r = (b - self.p2) / (2.0 * self.dt);
        self.rate += 0.5 * (r - self.rate);
        self.p2 = self.p1;
        self.p1 = b;
        (b, self.rate)
    }
}
