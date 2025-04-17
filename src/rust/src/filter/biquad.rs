use core::f32::consts::PI;
use crate::math::abs::Absf;
use crate::math::trig::Trig;

struct BiquadHelper {
    omega: f32,
    cos: f32,
    sin: f32,
    alpha: f32,
    a0: f32,
}

impl BiquadHelper {
    fn new(quality_factor: f32, cutoff: f32, dt: f32) -> Self {
        let omega: f32 = 2.0 * PI * cutoff * dt;
        let sin_omega: f32 = omega.sinf_unchecked();
        let cos_omega: f32 = omega.cosf_unchecked();
        let alpha: f32 = sin_omega / (2.0 * quality_factor);
        let a0: f32 = 1.0 / (1.0 + alpha);

        Self {
            omega,
            cos: cos_omega,
            sin: sin_omega,
            alpha,
            a0,
        }
    }
}

#[derive(Copy, Clone)]
#[repr(C)]
struct BiquadGain {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
}

impl BiquadGain {
    pub fn notch_gains(q: f32, cutoff: f32, dt: f32) -> Self {
        let helper = BiquadHelper::new(q, cutoff, dt);
        let b1 = -2.0 * helper.cos * helper.a0;

        Self {
            b0: helper.a0,
            b1,
            b2: helper.a0,
            a1: b1,
            a2: (1.0 - helper.alpha) * helper.a0,
        }
    }

    pub fn update_notch_gains(&mut self, q: f32, cutoff: f32, dt: f32) {
        *self = Self::notch_gains(q, cutoff, dt);
    }

    pub fn peak_gains(magnitude: f32, q: f32, cutoff: f32, dt: f32) -> Self {
        let helper = BiquadHelper::new(q, cutoff, dt);
        let b1 = -2.0 * helper.cos * helper.a0;

        Self {
            b0: (1.0 + helper.alpha * magnitude) * helper.a0,
            b1,
            b2: (1.0 - helper.alpha * magnitude) * helper.a0,
            a1: b1,
            a2: (1.0 - helper.alpha) * helper.a0,
        }
    }

    pub fn update_peak_gains(&mut self, magnitude: f32, q: f32, cutoff: f32, dt: f32) {
        *self = Self::peak_gains(magnitude, q, cutoff, dt);
    }

    pub fn second_order_lowpass_gains(q: f32, cutoff: f32, dt: f32) -> Self {
        let helper = BiquadHelper::new(q, cutoff, dt);
        let b1 = (1.0 - helper.cos) * helper.a0;
        let b0 = b1 * 0.5;

        Self {
            b0,
            b1,
            b2: b0,
            a1: -2.0 * helper.cos * helper.a0,
            a2: (1.0 - helper.alpha) * helper.a0,
        }
    }

    pub fn update_second_order_lowpass_gains(&mut self, q: f32, cutoff: f32, dt: f32) {
        *self = Self::second_order_lowpass_gains(q, cutoff, dt);
    }

    pub fn second_order_highpass_gains(q: f32, cutoff: f32, dt: f32) -> Self {
        let helper = BiquadHelper::new(q, cutoff, dt);
        let b1 = -(1.0 + helper.cos) * helper.a0;
        let b0 = -b1 * 0.5;

        Self {
            b0,
            b1,
            b2: b0,
            a1: -2.0 * helper.cos * helper.a0,
            a2: (1.0 - helper.alpha) * helper.a0,
        }
    }

    pub fn update_second_order_highpass_gains(&mut self, q: f32, cutoff: f32, dt: f32) {
        *self = Self::second_order_highpass_gains(q, cutoff, dt);
    }

    pub fn sliding_order_gains(q: f32, cutoff: f32, shift: f32, dt: f32) -> Self {
        let helper = BiquadHelper::new(q, cutoff, dt);
        let cos = helper.cos;
        let sin = helper.sin;
        let alpha = helper.alpha;
        let a0 = helper.a0;

        let transform = 2.0 * sin / (cos + 1.0);
        let transformed_a0 = 1.0 / (2.0 + transform);

        let first_b0 = transform * transformed_a0;
        let first_b1 = first_b0;
        let first_a1 = (transform - 2.0) * transformed_a0;

        let second_b1 = (1.0 - cos) * a0;
        let second_b0 = second_b1 * 0.5;
        let second_b2 = second_b0;
        let second_a1 = -2.0 * cos * a0;
        let second_a2 = (1.0 - alpha) * a0;

        let recip_shift = 1.0 - shift;

        Self {
            b0: first_b0 * shift + second_b0 * recip_shift,
            b1: first_b1 * shift + second_b1 * recip_shift,
            b2: second_b2 * recip_shift,
            a1: first_a1 * shift + second_a1 * recip_shift,
            a2: second_a2 * recip_shift,
        }
    }

    pub fn update_sliding_order_gains(&mut self, q: f32, cutoff: f32, shift: f32, dt: f32) {
        *self = Self::sliding_order_gains(q, cutoff, shift, dt);
    }

    fn pt1_a1(omega: f32) -> f32 {
        -1.0 / ((6.0 + omega * (6.0 + omega * (3.0 + omega))) * 0.16666666)
    }

    pub fn pt1_second_order_gains(q: f32, cutoff: f32, shift: f32, dt: f32) -> Self {
        let helper = BiquadHelper::new(q, cutoff, dt);
        let omega = helper.omega;
        let cos = helper.cos;
        let alpha = helper.alpha;
        let a0 = helper.a0;

        let pt1_a1 = Self::pt1_a1(omega);
        let pt1_b0 = pt1_a1 + 1.0;

        let second_b1 = (1.0 - cos) * a0;
        let second_b0 = second_b1 * 0.5;
        let second_b2 = second_b0;
        let second_a1 = -2.0 * cos * a0;
        let second_a2 = (1.0 - alpha) * a0;

        let recip_shift = 1.0 - shift;
        Self {
            b0: pt1_b0 * shift + second_b0 * recip_shift,
            b1: second_b1 * recip_shift,
            b2: second_b2 * recip_shift,
            a1: pt1_a1 * shift + second_a1 * recip_shift,
            a2: second_a2 * recip_shift,
        }
    }

    pub fn update_pt1_second_order_gains(&mut self, q: f32, cutoff: f32, shift: f32, dt: f32) {
        *self = Self::pt1_second_order_gains(q, cutoff, shift, dt);
    }
}

#[derive(Copy, Clone)]
#[repr(C)]
struct BiquadState {
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl BiquadState {
    fn new() -> Self {
        Self {
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
    }

    pub fn apply(&mut self, gains: &BiquadGain, input: f32) -> f32 {
        let output = gains.b0 * input + gains.b1 * self.x1 + gains.b2 * self.x2
            - gains.a1 * self.y1
            - gains.a2 * self.y2;

        self.x2 = self.x1;
        self.x1 = input;

        self.y2 = self.y1;
        self.y1 = output;

        output
    }
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct Biquad {
    gains: BiquadGain,
    state: BiquadState,
}

impl Biquad {
    pub fn apply(&mut self, input: f32) -> f32 {
        self.state.apply(&self.gains, input)
    }

    pub fn reset(&mut self) {
        self.state.reset();
    }

    pub fn new_notch(q: f32, cutoff: f32, dt: f32) -> Self {
        Self {
            gains: BiquadGain::notch_gains(q, cutoff, dt),
            state: BiquadState::new(),
        }
    }

    pub fn update_notch(&mut self, q: f32, cutoff: f32, dt: f32) {
        self.gains.update_notch_gains(q, cutoff, dt);
    }

    pub fn notch_q_from_center_and_end_freq(center_freq: f32, end_freq: f32) -> f32 {
        center_freq * end_freq / (center_freq * center_freq - end_freq * end_freq).absf()
    }

    pub fn new_peak(magnitude: f32, q: f32, cutoff: f32, dt: f32) -> Self {
        Self {
            gains: BiquadGain::peak_gains(magnitude, q, cutoff, dt),
            state: BiquadState::new(),
        }
    }

    pub fn update_peak(&mut self, magnitude: f32, q: f32, cutoff: f32, dt: f32) {
        self.gains.update_peak_gains(magnitude, q, cutoff, dt);
    }

    pub fn new_second_order_lowpass(q: f32, cutoff: f32, dt: f32) -> Self {
        Self {
            gains: BiquadGain::second_order_lowpass_gains(q, cutoff, dt),
            state: BiquadState::new(),
        }
    }

    pub fn update_second_order_lowpass(&mut self, q: f32, cutoff: f32, dt: f32) {
        self.gains.update_second_order_lowpass_gains(q, cutoff, dt);
    }

    pub fn new_second_order_highpass(q: f32, cutoff: f32, dt: f32) -> Self {
        Self {
            gains: BiquadGain::second_order_highpass_gains(q, cutoff, dt),
            state: BiquadState::new(),
        }
    }

    pub fn update_second_highpass(&mut self, q: f32, cutoff: f32, dt: f32) {
        self.gains.update_second_order_highpass_gains(q, cutoff, dt);
    }

    pub fn new_sliding_order(q: f32, cutoff: f32, shift: f32, dt: f32) -> Self {
        Self {
            gains: BiquadGain::sliding_order_gains(q, cutoff, shift, dt),
            state: BiquadState::new(),
        }
    }

    pub fn update_sliding_order(&mut self, q: f32, cutoff: f32, shift: f32, dt: f32) {
        self.gains.update_sliding_order_gains(q, cutoff, shift, dt);
    }

    pub fn new_pt1_second_order(q: f32, cutoff: f32, shift: f32, dt: f32) -> Self {
        Self {
            gains: BiquadGain::pt1_second_order_gains(q, cutoff, shift, dt),
            state: BiquadState::new(),
        }
    }

    pub fn update_pt1_second_order(&mut self, q: f32, cutoff: f32, shift: f32, dt: f32) {
        self.gains.update_pt1_second_order_gains(q, cutoff, shift, dt);
    }
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct Vec3Biquad {
    gains: BiquadGain,
    state: [BiquadState; 3],
}

impl Vec3Biquad {
    pub fn apply(&mut self, input: [f32; 3]) -> [f32; 3] {
        [
            self.state[0].apply(&self.gains, input[0]),
            self.state[1].apply(&self.gains, input[1]),
            self.state[2].apply(&self.gains, input[2]),
        ]
    }

    pub fn new_notch(q: f32, cutoff: f32, dt: f32) -> Self {
        Self {
            gains: BiquadGain::notch_gains(q, cutoff, dt),
            state: [BiquadState::new(); 3],
        }
    }

    pub fn new_second_order_lowpass(q: f32, cutoff: f32, dt: f32) -> Self {
        Self {
            gains: BiquadGain::second_order_lowpass_gains(q, cutoff, dt),
            state: [BiquadState::new(); 3],
        }
    }

    pub fn new_sliding_order(q: f32, cutoff: f32, shift: f32, dt: f32) -> Self {
        Self {
            gains: BiquadGain::sliding_order_gains(q, cutoff, shift, dt),
            state: [BiquadState::new(); 3],
        }
    }

    pub fn new_pt1_second_order(q: f32, cutoff: f32, shift: f32, dt: f32) -> Self {
        Self {
            gains: BiquadGain::pt1_second_order_gains(q, cutoff, shift, dt),
            state: [BiquadState::new(); 3],
        }
    }

    pub fn reset(&mut self) {
        self.state[0].reset();
        self.state[1].reset();
        self.state[2].reset();
    }

    pub fn update_notch(&mut self, q: f32, cutoff: f32, dt: f32) {
        self.gains.update_notch_gains(q, cutoff, dt);
    }

    pub fn update_second_order_lowpass(&mut self, q: f32, cutoff: f32, dt: f32) {
        self.gains.update_second_order_lowpass_gains(q, cutoff, dt);
    }

    pub fn update_sliding_order(&mut self, q: f32, cutoff: f32, shift: f32, dt: f32) {
        self.gains.update_sliding_order_gains(q, cutoff, shift, dt);
    }

    pub fn update_pt1_second_order(&mut self, q: f32, cutoff: f32, shift: f32, dt: f32) {
        self.gains.update_pt1_second_order_gains(q, cutoff, shift, dt);
    }
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct FirstOrderLowpassFilter {
    b0: f32,
    b1: f32,
    a1: f32,
    x: f32,
    y: f32,
}

impl FirstOrderLowpassFilter {
    pub fn update_cutoff(&mut self, cutoff: f32, dt: f32) {
        let omega: f32 = 2.0 * PI * cutoff * dt;
        let sin_omega: f32 = omega.sinf_unchecked();
        let cos_omega: f32 = omega.cosf_unchecked();
        let transform = 2.0 * sin_omega / (cos_omega + 1.0);
        let transformed_a0 = 1.0 / (2.0 + transform);

        self.b0 = transform * transformed_a0;
        self.b1 = self.b0;
        self.a1 = (transform - 2.0) * transformed_a0;
    }

    pub fn new(cutoff: f32, dt: f32) -> Self {
        let mut lowpass = Self {
            b0: 0.0,
            b1: 0.0,
            a1: 0.0,
            x: 0.0,
            y: 0.0,
        };

        lowpass.update_cutoff(cutoff, dt);

        lowpass
    }

    pub fn reset(&mut self) {
        self.x = 0.0;
        self.y = 0.0;
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        let output = self.b0 * input + self.b1 * self.x - self.a1 * self.y;

        self.x = input;
        self.y = output;

        output
    }
}

#[no_mangle] pub extern "C" fn q_from_center_and_end_freq(center_freq: f32, cutoff_freq: f32) -> f32 {
    Biquad::notch_q_from_center_and_end_freq(center_freq, cutoff_freq)
}

#[cfg(test)]
mod biquad_tests {
    use super::*;

    const LOOPRATE: f32 = 8000.0;
    const DT: f32 = 1.0 / LOOPRATE;

    const Q: f32 = 1.0;
    const CUTOFF: f32 = 100.0;
    const INPUT: f32 = 1000.0;
    const SHIFT: f32 = 0.5;

    #[test]
    fn notch_gains() {
        let notch = Biquad::new_notch(Q, CUTOFF, DT);
        let gains = notch.gains;

        assert_eq!(gains.b0, 0.96225137);
        assert_eq!(gains.b1, -1.91857);
        assert_eq!(gains.b2, 0.96225137);
        assert_eq!(gains.a1, -1.91857);
        assert_eq!(gains.a2, 0.92450273);
    }

    #[test]
    fn notch_apply() {
        let mut notch = Biquad::new_notch(Q, CUTOFF, DT);
        assert_eq!(notch.apply(INPUT), 962.25134);
    }

    #[test]
    fn peak_gains() {
        let peak = Biquad::new_peak(0.01, Q, CUTOFF, DT);
        let gains = peak.gains;

        assert_eq!(gains.b0, 0.9626289);
        assert_eq!(gains.b1, -1.91857);
        assert_eq!(gains.b2, 0.9618738);
        assert_eq!(gains.a1, -1.91857);
        assert_eq!(gains.a2, 0.92450273);
    }

    #[test]
    fn peak_apply() {
        let mut peak = Biquad::new_peak(0.01, Q, CUTOFF, DT);
        assert_eq!(peak.apply(INPUT), 962.6289);
    }

    #[test]
    fn first_order_lowpass_gains() {
        let lowpass = FirstOrderLowpassFilter::new(CUTOFF, DT);

        assert_eq!(lowpass.b0, 0.037804723);
        assert_eq!(lowpass.b1, 0.037804723);
        assert_eq!(lowpass.a1, -0.92439055);
    }

    #[test]
    fn first_order_lowpass_apply() {
        let mut lowpass = FirstOrderLowpassFilter::new(CUTOFF, DT);
        assert_eq!(lowpass.apply(INPUT), 37.80472);
    }

    #[test]
    fn second_order_lowpass_gains() {
        let lowpass = Biquad::new_second_order_lowpass(Q, CUTOFF, DT);
        let gains = lowpass.gains;

        assert_eq!(gains.b0, 0.0014831626);
        assert_eq!(gains.b1, 0.0029663253);
        assert_eq!(gains.b2, 0.0014831626);
        assert_eq!(gains.a1, -1.91857);
        assert_eq!(gains.a2, 0.92450273);
    }

    #[test]
    fn second_order_lowpass_apply() {
        let mut lowpass = Biquad::new_second_order_lowpass(Q, CUTOFF, DT);
        assert_eq!(lowpass.apply(INPUT), 1.4831626);
    }

    #[test]
    fn second_order_highpass_gains() {
        let highpass = Biquad::new_second_order_highpass(Q, CUTOFF, DT);
        let gains = highpass.gains;

        assert_eq!(gains.b0, 0.96076816);
        assert_eq!(gains.b1, -1.9215363);
        assert_eq!(gains.b2, 0.96076816);
        assert_eq!(gains.a1, -1.91857);
        assert_eq!(gains.a2, 0.92450273);
    }

    #[test]
    fn second_order_highpass_apply() {
        let mut highpass = Biquad::new_second_order_highpass(Q, CUTOFF, DT);
        assert_eq!(highpass.apply(INPUT), 960.7682);
    }

    #[test]
    fn sliding_order_gains() {
        let lowpass = Biquad::new_sliding_order(Q, CUTOFF, SHIFT, DT);
        let gains = lowpass.gains;

        assert_eq!(gains.b0, 0.019643942);
        assert_eq!(gains.b1, 0.020385524);
        assert_eq!(gains.b2, 0.0007415813);
        assert_eq!(gains.a1, -1.4214803);
        assert_eq!(gains.a2, 0.46225137);
    }

    #[test]
    fn sliding_order_apply() {
        let mut lowpass = Biquad::new_sliding_order(Q, CUTOFF, SHIFT, DT);
        assert_eq!(lowpass.apply(INPUT), 19.643942);
    }

    #[test]
    fn pt1_second_order_gains() {
        let lowpass = Biquad::new_pt1_second_order(Q, CUTOFF, SHIFT, DT);
        let gains = lowpass.gains;

        assert_eq!(gains.b0, 0.038508248);
        assert_eq!(gains.b1, 0.0014831626);
        assert_eq!(gains.b2, 0.0007415813);
        assert_eq!(gains.a1, -1.4215183);
        assert_eq!(gains.a2, 0.46225137);
    }

    #[test]
    fn pt1_second_order_apply() {
        let mut lowpass = Biquad::new_pt1_second_order(Q, CUTOFF, SHIFT, DT);
        assert_eq!(lowpass.apply(INPUT), 38.508247);
    }
}
