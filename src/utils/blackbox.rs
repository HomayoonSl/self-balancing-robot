use core::fmt::Write;

#[derive(Clone, Copy)]
pub struct Rec {
    pub pitch_cd: i16,
    pub rate_dd: i16,
    pub wheel_cr: i16,
    pub wheel_rate_cr: i16,
    pub v_mv: i16,
}

pub const REC_ZERO: Rec = Rec {
    pitch_cd: 0,
    rate_dd: 0,
    wheel_cr: 0,
    wheel_rate_cr: 0,
    v_mv: 0,
};

pub struct BlackBox<const N: usize> {
    pub buf: [Rec; N],
    pub head: usize,
    pub filled: usize,
}

impl<const N: usize> BlackBox<N> {
    pub fn new() -> Self {
        Self {
            buf: [REC_ZERO; N],
            head: 0,
            filled: 0,
        }
    }

    pub fn push(&mut self, r: Rec) {
        self.buf[self.head] = r;
        self.head = (self.head + 1) % N;
        if self.filled < N {
            self.filled += 1;
        }
    }

    pub fn dump<W: Write>(&self, w: &mut W, dt_ms: u32) {
        let _ = write!(
            w,
            "# blackbox: {} rows, {} ms per row, last row is the stop\r\n",
            self.filled, dt_ms
        );
        let _ = w.write_str("row,pitch_cdeg,rate_ddps,wheel_crad,wheel_rate_crad_s,v_mv\r\n");
        let start = (self.head + N - self.filled) % N;
        for i in 0..self.filled {
            let r = self.buf[(start + i) % N];
            let _ = write!(
                w,
                "{},{},{},{},{},{}\r\n",
                i, r.pitch_cd, r.rate_dd, r.wheel_cr, r.wheel_rate_cr, r.v_mv
            );
        }
    }
}
