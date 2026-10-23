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
