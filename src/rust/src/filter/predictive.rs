use crate::filter::biquad::{Biquad, Vec3Biquad};

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PredictiveNotchFilter {
    notch: Biquad,
    second_order_lowpass: Biquad,
    predictive_strength: f32,
    crossfade_amount: f32
}

impl PredictiveNotchFilter {
    pub fn new(notch_cutoff: f32, notch_q: f32, lowpass_cutoff: f32, lowpass_q: f32, predictive_weight: f32, weight: f32, dt: f32) -> Self {
        Self {
            notch: Biquad::new_notch(notch_q, notch_cutoff, dt),
            second_order_lowpass: Biquad::new_second_order_lowpass(lowpass_q, lowpass_cutoff, dt),
            predictive_strength: predictive_weight,
            crossfade_amount: weight,
        }
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        let notched = self.notch.apply(input);
        let prediction = self.second_order_lowpass.apply(input - notched) * self.predictive_strength;

        // Crossfade between the raw input and the predicted+notched signal
        input * (1.0 - self.crossfade_amount) + (notched + prediction) * self.crossfade_amount
    }

    pub fn update_cutoff(&mut self, q: f32, cutoff: f32, weight: f32, dt: f32) {
        self.notch.update_notch(q, cutoff, dt);
        self.crossfade_amount = weight;
    }

    pub fn reset(&mut self) {
        self.notch.reset();
        self.second_order_lowpass.reset();
    }
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct Vec3PredictiveNotchFilter {
    notch: Vec3Biquad,
    second_order_lowpass: Vec3Biquad,
    predictive_strength: f32,
    crossfade_amount: f32,
}

impl Vec3PredictiveNotchFilter {
    pub fn new(
        notch_cutoff: f32,
        notch_q: f32,
        lowpass_cutoff: f32,
        lowpass_q: f32,
        predictive_weight: f32,
        crossfade_amount: f32,
        dt: f32,
    ) -> Self {
        Self {
            notch: Vec3Biquad::new_notch(notch_q, notch_cutoff, dt),
            second_order_lowpass: Vec3Biquad::new_second_order_lowpass(lowpass_q, lowpass_cutoff, dt),
            predictive_strength: predictive_weight,
            crossfade_amount,
        }
    }

    pub fn apply(&mut self, input: [f32; 3]) -> [f32; 3] {
        let notched = self.notch.apply(input);
        let mut output = [0.0; 3];

        let residual = [
            input[0] - notched[0],
            input[1] - notched[1],
            input[2] - notched[2],
        ];
        let predicted = self.second_order_lowpass.apply(residual);

        for i in 0..3 {
            output[i] = input[i] * (1.0 - self.crossfade_amount)
                + (notched[i] + predicted[i] * self.predictive_strength) * self.crossfade_amount;
        }

        output
    }

    pub fn update_cutoff(&mut self, q: f32, cutoff: f32, crossfade: f32, dt: f32) {
        self.notch.update_notch(q, cutoff, dt);
        self.crossfade_amount = crossfade;
    }

    pub fn reset(&mut self) {
        self.notch.reset();
        self.second_order_lowpass.reset();
    }
}

#[no_mangle] pub extern "C" fn predictive_notch_vec_filter_init(
    filter: *mut Vec3PredictiveNotchFilter,
    notch_cutoff: f32,
    notch_q: f32,
    predictive_cutoff: f32,
    predictive_q: f32,
    predictive_weight: f32,
    crossfade_amount: f32,
    dt: f32
)
{
    unsafe {
        *filter = Vec3PredictiveNotchFilter::new(notch_cutoff, notch_q, predictive_cutoff, predictive_q, predictive_weight, crossfade_amount, dt);
    }
}

#[link_section = ".tcm_code"]
#[no_mangle] pub extern "C" fn predictive_notch_vec_filter_update(
    filter: *mut Vec3PredictiveNotchFilter,
    notch_cutoff: f32,
    notch_q: f32,
    crossfade_amount: f32,
    dt: f32
)
{
    unsafe {
        (*filter).update_cutoff(notch_cutoff, notch_q, crossfade_amount, dt);
    }
}

#[link_section = ".tcm_code"]
#[no_mangle] pub extern "C" fn predictive_notch_vec_filter_apply(
    filter: *mut Vec3PredictiveNotchFilter,
    input: &mut [f32; 3],
)
{
    unsafe {
        *input = (*filter).apply(*input);
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