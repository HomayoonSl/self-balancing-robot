use cortex_m::peripheral::DWT;

pub struct Ticker {
    next: u32,
    period: u32,
}

impl Ticker {
    pub fn new(period: u32) -> Self {
        Self {
            next: DWT::cycle_count().wrapping_add(period),
            period,
        }
    }

    pub fn wait(&mut self) -> bool {
        let diff = DWT::cycle_count().wrapping_sub(self.next) as i32;
        let late = diff > 0;
        while (DWT::cycle_count().wrapping_sub(self.next) as i32) < 0 {}
        self.next = self.next.wrapping_add(self.period);
        late
    }
}
