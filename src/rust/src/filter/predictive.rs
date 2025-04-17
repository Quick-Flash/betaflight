use crate::filter::biquad::{NotchFilter, SecondOrderLowpassFilter};

// TODO make this use a notch and second order lowpass that uses a 3d array biquad (just the state is an array, this will reduce CPU load).

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PredictiveNotchFilter {
    notch: NotchFilter,
    second_order_lowpass: SecondOrderLowpassFilter,
    predictive_weight: f32,
    weight: f32
}

impl PredictiveNotchFilter {
    pub fn new(notch_cutoff: f32, notch_q: f32, lowpass_cutoff: f32, lowpass_q: f32, predictive_weight: f32, weight: f32, dt: f32) -> Self {
        Self {
            notch: NotchFilter::new(notch_q, notch_cutoff, dt),
            second_order_lowpass: SecondOrderLowpassFilter::new(lowpass_q, lowpass_cutoff, dt),
            predictive_weight,
            weight,
        }
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        let notched = self.notch.apply(input);
        let prediction = self.second_order_lowpass.apply(input - notched) * self.predictive_weight;
        (notched + prediction) * self.weight
    }

    pub fn update_cutoff(&mut self, q: f32, cutoff: f32, weight: f32, dt: f32) {
        self.notch.update_cutoff(q, cutoff, dt);
        self.weight = weight;
    }

    pub fn copy_gains(&mut self, gains: &PredictiveNotchFilter) {
        self.notch.copy_gains(&gains.notch);
        self.weight = gains.weight;
    }

    pub fn reset(&mut self) {
        self.notch.reset();
        self.second_order_lowpass.reset();
    }
}

#[cfg(test)]
mod predictive_filter_tests {
    use super::*;

    const LOOPRATE: f32 = 8000.0;
    const DT: f32 = 1.0 / LOOPRATE;
    const INPUT: f32 = 1000.0;

    const MAIN_CUTOFF: f32 = 100.0;
    const MAIN_Q: f32 = 0.707;
    const LEFTOVER_CUTOFF: f32 = 15.0;
    const LEFTOVER_Q: f32 = 0.5;

    const WEIGHT: f32 = 1.0;
    const PREDICTIVE_WEIGHT: f32 = 1.0;

    #[cfg(test)]
    mod second_order_predictive_notch_tests {
        use super::*;

        #[test]
        fn second_order_predictive_notch_apply() {
            // given
            let mut predictive_notch = PredictiveNotchFilter::new(MAIN_CUTOFF, MAIN_Q, LEFTOVER_CUTOFF, LEFTOVER_Q, WEIGHT, PREDICTIVE_WEIGHT, DT);

            // then
            assert_eq!(predictive_notch.apply(INPUT), 947.4315);
        }
    }
}