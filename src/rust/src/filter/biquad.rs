use core::f32::consts::PI;
use crate::math::abs::Absf;
use crate::math::trig::Trig;

#[derive(Copy, Clone)]
#[repr(C)]
struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl Biquad {
    fn apply(&mut self, input: f32) -> f32 {
        let output = self.b0 * input + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;

        self.x2 = self.x1;
        self.x1 = input;

        self.y2 = self.y1;
        self.y1 = output;

        output
    }

    fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
    }

    fn empty() -> Self {
        Self {
            b0: 0.0,
            b1: 0.0,
            b2: 0.0,
            a1: 0.0,
            a2: 0.0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }
}

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
pub struct NotchFilter {
    biquad: Biquad,
}

impl NotchFilter {
    pub fn update_cutoff(&mut self, quality_factor: f32, cutoff: f32, dt: f32) {
        let helper = BiquadHelper::new(quality_factor, cutoff, dt);
        let cos = helper.cos;
        let alpha = helper.alpha;
        let a0 = helper.a0;

        self.biquad.b0 = a0;
        self.biquad.b1 = -2.0 * cos * a0;
        self.biquad.b2 = a0;
        self.biquad.a1 = self.biquad.b1;
        self.biquad.a2 = (1.0 - alpha) * a0;
    }

    pub fn new(quality_factor: f32, cutoff: f32, dt: f32) -> Self {
        let mut notch = Self {
            biquad: Biquad::empty(),
        };
        notch.update_cutoff(quality_factor, cutoff, dt);

        notch
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        self.biquad.apply(input)
    }

    pub fn reset(&mut self) {
        self.biquad.reset();
    }

    pub fn q_from_center_and_end_freq(center_freq: f32, end_freq: f32) -> f32 {
        center_freq * end_freq / (center_freq * center_freq - end_freq * end_freq).absf()
    }
}

#[derive(Copy, Clone)]
pub struct PeakFilter {
    biquad: Biquad,
}

impl PeakFilter {
    pub fn update_cutoff(&mut self, magnitude: f32, quality_factor: f32, cutoff: f32, dt: f32) {
        let helper = BiquadHelper::new(quality_factor, cutoff, dt);
        let cos = helper.cos;
        let alpha = helper.alpha;
        let a0 = helper.a0;

        self.biquad.b0 = (1.0 + alpha * magnitude) * a0;
        self.biquad.b1 = -2.0 * cos * a0;
        self.biquad.b2 = (1.0 - alpha * magnitude) * a0;
        self.biquad.a1 = self.biquad.b1;
        self.biquad.a2 = (1.0 - alpha) * a0;
    }

    pub fn new(magnitude: f32, quality_factor: f32, cutoff: f32, dt: f32) -> Self {
        let mut peak = Self {
            biquad: Biquad::empty(),
        };
        peak.update_cutoff(magnitude, quality_factor, cutoff, dt);

        peak
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        self.biquad.apply(input)
    }

    pub fn reset(&mut self) {
        self.biquad.reset();
    }

    pub fn q_from_center_and_end_freq(center_freq: f32, end_freq: f32) -> f32 {
        center_freq * end_freq / (center_freq * center_freq - end_freq * end_freq).absf()
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

#[derive(Copy, Clone)]
#[repr(C)]
pub struct SecondOrderLowpassFilter {
    biquad: Biquad,
}

impl SecondOrderLowpassFilter {
    pub fn update_cutoff(&mut self, quality_factor: f32, cutoff: f32, dt: f32) {
        let helper = BiquadHelper::new(quality_factor, cutoff, dt);
        let cos = helper.cos;
        let alpha = helper.alpha;
        let a0 = helper.a0;

        self.biquad.b1 = (1.0 - cos) * a0;
        self.biquad.b0 = self.biquad.b1 * 0.5;
        self.biquad.b2 = self.biquad.b0;
        self.biquad.a1 = -2.0 * cos * a0;
        self.biquad.a2 = (1.0 - alpha) * a0;
    }

    pub fn new(quality_factor: f32, cutoff: f32, dt: f32) -> Self {
        let mut lowpass = Self {
            biquad: Biquad::empty(),
        };
        lowpass.update_cutoff(quality_factor, cutoff, dt);

        lowpass
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        self.biquad.apply(input)
    }

    pub fn reset(&mut self) {
        self.biquad.reset();
    }
}

#[derive(Copy, Clone)]
pub struct SecondOrderHighpassFilter {
    biquad: Biquad,
}

impl SecondOrderHighpassFilter {
    pub fn update_cutoff(&mut self, quality_factor: f32, cutoff: f32, dt: f32) {
        let helper = BiquadHelper::new(quality_factor, cutoff, dt);
        let cos = helper.cos;
        let alpha = helper.alpha;
        let a0 = helper.a0;

        self.biquad.b1 = -(1.0 + cos) * a0;
        self.biquad.b0 = -self.biquad.b1 * 0.5;
        self.biquad.b2 = self.biquad.b0;
        self.biquad.a1 = -2.0 * cos * a0;
        self.biquad.a2 = (1.0 - alpha) * a0;
    }

    pub fn new(quality_factor: f32, cutoff: f32, dt: f32) -> Self {
        let mut lowpass = Self {
            biquad: Biquad::empty(),
        };
        lowpass.update_cutoff(quality_factor, cutoff, dt);

        lowpass
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        self.biquad.apply(input)
    }

    pub fn reset(&mut self) {
        self.biquad.reset();
    }
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct SlidingOrderLowpassFilter {
    biquad: Biquad,
}

impl SlidingOrderLowpassFilter {
    pub fn update_cutoff(&mut self, quality_factor: f32, cutoff: f32, shift: f32, dt: f32) {
        let helper = BiquadHelper::new(quality_factor, cutoff, dt);
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

        self.biquad.b0 = first_b0 * shift + second_b0 * recip_shift;
        self.biquad.b1 = first_b1 * shift + second_b1 * recip_shift;
        self.biquad.b2 = second_b2 * recip_shift;
        self.biquad.a1 = first_a1 * shift + second_a1 * recip_shift;
        self.biquad.a2 = second_a2 * recip_shift;
    }

    pub fn new(quality_factor: f32, cutoff: f32, shift: f32, dt: f32) -> Self {
        let mut lowpass = Self {
            biquad: Biquad::empty(),
        };
        lowpass.update_cutoff(quality_factor, cutoff, shift, dt);

        lowpass
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        self.biquad.apply(input)
    }

    pub fn reset(&mut self) {
        self.biquad.reset();
    }
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PtSecondOrderLowpassFilter {
    biquad: Biquad,
}

impl PtSecondOrderLowpassFilter {
    pub fn update_cutoff(&mut self, quality_factor: f32, cutoff: f32, shift: f32, dt: f32) {
        let helper = BiquadHelper::new(quality_factor, cutoff, dt);
        let omega = helper.omega;
        let cos = helper.cos;
        let alpha = helper.alpha;
        let a0 = helper.a0;

        let pt1_a1 = PtSecondOrderLowpassFilter::pt1_a1(omega);
        let pt1_b0 = pt1_a1 + 1.0;

        let second_b1 = (1.0 - cos) * a0;
        let second_b0 = second_b1 * 0.5;
        let second_b2 = second_b0;
        let second_a1 = -2.0 * cos * a0;
        let second_a2 = (1.0 - alpha) * a0;

        let recip_shift = 1.0 - shift;

        self.biquad.b0 = pt1_b0 * shift + second_b0 * recip_shift;
        self.biquad.b1 = second_b1 * recip_shift;
        self.biquad.b2 = second_b2 * recip_shift;
        self.biquad.a1 = pt1_a1 * shift + second_a1 * recip_shift;
        self.biquad.a2 = second_a2 * recip_shift;
    }

    pub fn new(quality_factor: f32, cutoff: f32, shift: f32, dt: f32) -> Self {
        let mut lowpass = Self {
            biquad: Biquad::empty(),
        };
        lowpass.update_cutoff(quality_factor, cutoff, shift, dt);

        lowpass
    }

    fn pt1_a1(omega: f32) -> f32 {
        -1.0 / ((6.0 + omega * (6.0 + omega * (3.0 + omega))) * 0.16666666)
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        self.biquad.apply(input)
    }

    pub fn reset(&mut self) {
        self.biquad.reset();
    }
}

#[no_mangle] pub extern "C" fn q_from_center_and_end_freq(center_freq: f32, cutoff_freq: f32) -> f32 {
    NotchFilter::q_from_center_and_end_freq(center_freq, cutoff_freq)
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

    #[cfg(test)]
    mod notch_tests {
        use super::*;

        #[test]
        fn notch_gains() {
            // given
            let notch = NotchFilter::new(Q, CUTOFF, DT);

            // then
            assert_eq!(notch.biquad.b0, 0.96225137);
            assert_eq!(notch.biquad.b1, -1.91857);
            assert_eq!(notch.biquad.b2, 0.96225137);
            assert_eq!(notch.biquad.a1, -1.91857);
            assert_eq!(notch.biquad.a2, 0.92450273);
        }

        #[test]
        fn notch_apply() {
            // given
            let mut notch = NotchFilter::new(Q, CUTOFF, DT);

            // then
            assert_eq!(notch.apply(INPUT), 962.25134);
        }
    }

    #[cfg(test)]
    mod peak_tests {
        use super::*;

        #[test]
        fn peak_gains() {
            // given
            let peak = PeakFilter::new(0.01, Q, CUTOFF, DT);

            // then
            assert_eq!(peak.biquad.b0, 0.9626289);
            assert_eq!(peak.biquad.b1, -1.91857);
            assert_eq!(peak.biquad.b2, 0.9618738);
            assert_eq!(peak.biquad.a1, -1.91857);
            assert_eq!(peak.biquad.a2, 0.92450273);
        }

        #[test]
        fn peak_apply() {
            // given
            let mut peak = PeakFilter::new(0.01, Q, CUTOFF, DT);

            // then
            assert_eq!(peak.apply(INPUT), 962.6289);
        }
    }

    #[cfg(test)]
    mod first_order_lowpass_tests {
        use super::*;

        #[test]
        fn gains() {
            // given
            let lowpass = FirstOrderLowpassFilter::new(CUTOFF, DT);

            // then
            assert_eq!(lowpass.b0, 0.037804723);
            assert_eq!(lowpass.b1, 0.037804723);
            assert_eq!(lowpass.a1, -0.92439055);
        }

        #[test]
        fn apply() {
            // given
            let mut lowpass = FirstOrderLowpassFilter::new(CUTOFF, DT);

            // then
            assert_eq!(lowpass.apply(INPUT), 37.80472);
        }
    }

    #[cfg(test)]
    mod second_order_lowpass_tests {
        use super::*;

        #[test]
        fn lowpass_gains() {
            // given
            let lowpass = SecondOrderLowpassFilter::new(Q, CUTOFF, DT);

            // then
            assert_eq!(lowpass.biquad.b0, 0.0014831626);
            assert_eq!(lowpass.biquad.b1, 0.0029663253);
            assert_eq!(lowpass.biquad.b2, 0.0014831626);
            assert_eq!(lowpass.biquad.a1, -1.91857);
            assert_eq!(lowpass.biquad.a2, 0.92450273);
        }

        #[test]
        fn lowpass_apply() {
            // given
            let mut lowpass = SecondOrderLowpassFilter::new(Q, CUTOFF, DT);

            // then
            assert_eq!(lowpass.apply(INPUT), 1.4831626);
        }
    }

    #[cfg(test)]
    mod second_order_highpass_tests {
        use super::*;

        #[test]
        fn highpass_gains() {
            // given
            let highpass = SecondOrderHighpassFilter::new(Q, CUTOFF, DT);

            // then
            assert_eq!(highpass.biquad.b0, 0.96076816);
            assert_eq!(highpass.biquad.b1, -1.9215363);
            assert_eq!(highpass.biquad.b2, 0.96076816);
            assert_eq!(highpass.biquad.a1, -1.91857);
            assert_eq!(highpass.biquad.a2, 0.92450273);
        }

        #[test]
        fn highpass_apply() {
            // given
            let mut highpass = SecondOrderHighpassFilter::new(Q, CUTOFF, DT);

            // then
            assert_eq!(highpass.apply(INPUT), 960.7682);
        }
    }

    #[cfg(test)]
    mod sliding_order_tests {
        use super::*;

        #[test]
        fn sliding_order_gains() {
            // given
            let lowpass = SlidingOrderLowpassFilter::new(Q, CUTOFF, SHIFT, DT);

            // then
            assert_eq!(lowpass.biquad.b0, 0.019643942);
            assert_eq!(lowpass.biquad.b1, 0.020385524);
            assert_eq!(lowpass.biquad.b2, 0.0007415813);
            assert_eq!(lowpass.biquad.a1, -1.4214803);
            assert_eq!(lowpass.biquad.a2, 0.46225137);
        }

        #[test]
        fn sliding_order_apply() {
            // given
            let mut lowpass = SlidingOrderLowpassFilter::new(Q, CUTOFF, SHIFT, DT);

            // then
            assert_eq!(lowpass.apply(INPUT), 19.643942);
        }
    }

    #[cfg(test)]
    mod pt_second_order_tests {
        use super::*;

        #[test]
        fn pt_second_order_gains() {
            // given
            let lowpass = PtSecondOrderLowpassFilter::new(Q, CUTOFF, SHIFT, DT);

            // then
            assert_eq!(lowpass.biquad.b0, 0.038508248);
            assert_eq!(lowpass.biquad.b1, 0.0014831626);
            assert_eq!(lowpass.biquad.b2, 0.0007415813);
            assert_eq!(lowpass.biquad.a1, -1.4215183);
            assert_eq!(lowpass.biquad.a2, 0.46225137);
        }

        #[test]
        fn pt_second_apply() {
            // given
            let mut lowpass = PtSecondOrderLowpassFilter::new(Q, CUTOFF, SHIFT, DT);

            // then
            assert_eq!(lowpass.apply(INPUT), 38.508247);
        }
    }
}
