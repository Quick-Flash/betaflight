use crate::filter::ptn::{Pt1Filter, Pt2Filter, Pt3Filter};
use crate::filter::biquad::{Biquad, FirstOrderLowpassFilter};

#[repr(C)]
pub struct GyroFilters {
    lowpass1: [PredictiveLowpass; 3],
    lowpass2: [PredictiveLowpass; 3],
}

impl GyroFilters {
    #[inline(never)]
    pub fn new(
        cutoff1: f32,
        predictive_cutoff1: f32,
        cutoff_q1: f32,
        predictive_q1: f32,
        cutoff_shift1: f32,
        predictive_shift1: f32,
        predictive_weight1: f32,

        cutoff2: f32,
        predictive_cutoff2: f32,
        cutoff_q2: f32,
        predictive_q2: f32,
        cutoff_shift2: f32,
        predictive_shift2: f32,
        predictive_weight2: f32,

        lowpass_variants: &GyroLowpassVariants,
        dt: f32
    ) -> Self {
        Self {
            lowpass1: [
                PredictiveLowpass::new(&lowpass_variants.lowpass1, &lowpass_variants.predictive1, cutoff1, predictive_cutoff1, cutoff_q1, predictive_q1, cutoff_shift1, predictive_shift1, predictive_weight1, dt),
                PredictiveLowpass::new(&lowpass_variants.lowpass1, &lowpass_variants.predictive1, cutoff1, predictive_cutoff1, cutoff_q1, predictive_q1, cutoff_shift1, predictive_shift1, predictive_weight1, dt),
                PredictiveLowpass::new(&lowpass_variants.lowpass1, &lowpass_variants.predictive1, cutoff1, predictive_cutoff1, cutoff_q1, predictive_q1, cutoff_shift1, predictive_shift1, predictive_weight1, dt),
            ],
            lowpass2: [
                PredictiveLowpass::new(&lowpass_variants.lowpass2, &lowpass_variants.predictive2, cutoff2, predictive_cutoff2, cutoff_q2, predictive_q2, cutoff_shift2, predictive_shift2, predictive_weight2, dt),
                PredictiveLowpass::new(&lowpass_variants.lowpass2, &lowpass_variants.predictive2, cutoff2, predictive_cutoff2, cutoff_q2, predictive_q2, cutoff_shift2, predictive_shift2, predictive_weight2, dt),
                PredictiveLowpass::new(&lowpass_variants.lowpass2, &lowpass_variants.predictive2, cutoff2, predictive_cutoff2, cutoff_q2, predictive_q2, cutoff_shift2, predictive_shift2, predictive_weight2, dt),
            ],
        }
    }

    pub fn apply(&mut self, lowpass_variants: &GyroLowpassVariants, input: [f32; 3]) -> [f32; 3] {
        let mut output = [0.0; 3];
        for i in 0..3 {
            let first_pass = self.lowpass1[i].apply(&lowpass_variants.lowpass1, &lowpass_variants.predictive1, input[i]);
            output[i] = self.lowpass2[i].apply(&lowpass_variants.lowpass2, &lowpass_variants.predictive2, first_pass);
        }

        output
    }
}

#[no_mangle] pub extern "C" fn gyro_filter_init(
    filter: *mut GyroFilters,
    cutoff1: f32,
    predictive_cutoff1: f32,
    cutoff_q1: f32,
    predictive_q1: f32,
    cutoff_shift1: f32,
    predictive_shift1: f32,
    predictive_weight1: f32,

    cutoff2: f32,
    predictive_cutoff2: f32,
    cutoff_q2: f32,
    predictive_q2: f32,
    cutoff_shift2: f32,
    predictive_shift2: f32,
    predictive_weight2: f32,

    lowpass_variants: &GyroLowpassVariants,
    dt: f32
)
{
    unsafe {
        *filter = GyroFilters::new(cutoff1, predictive_cutoff1, cutoff_q1, predictive_q1, cutoff_shift1, predictive_shift1, predictive_weight1,
                                   cutoff2, predictive_cutoff2, cutoff_q2, predictive_q2, cutoff_shift2, predictive_shift2, predictive_weight2,
                                   lowpass_variants, dt
        );
    }
}

#[link_section = ".tcm_code"]
#[no_mangle] pub extern "C" fn gyro_filter_apply(
    filter: *mut GyroFilters,
    lowpass_variants: &GyroLowpassVariants,
    input: *mut [f32; 3],
)
{
    unsafe {
        *input = (*filter).apply(lowpass_variants, *input);
    }
}

#[repr(C)]
pub struct GyroLowpassVariants {
    lowpass1: LowpassVariant,
    predictive1: LowpassVariant,
    lowpass2: LowpassVariant,
    predictive2: LowpassVariant,
}

impl GyroLowpassVariants {
    pub fn new(variant1: LowpassVariant, pred_variant1: LowpassVariant, variant2: LowpassVariant, pred_variant2: LowpassVariant) -> Self {
        Self {
            lowpass1: variant1,
            predictive1: pred_variant1,
            lowpass2: variant2,
            predictive2: pred_variant2,
        }
    }
}

#[no_mangle] pub extern "C" fn gyro_lowpass_init(gyro_variants: *mut GyroLowpassVariants, variant1: LowpassVariant, pred_variant1: LowpassVariant, variant2: LowpassVariant, pred_variant2: LowpassVariant) {
    unsafe {
        *gyro_variants = GyroLowpassVariants::new(variant1, pred_variant1, variant2, pred_variant2);
    }
}

#[repr(C)]
struct PredictiveLowpass {
    lowpass_filter: LowpassFilter,
    predictive_lowpass: LowpassFilter,
    predictive_weight: f32,
}

impl PredictiveLowpass {
    pub fn new(variant: &LowpassVariant, predictive_variant: &LowpassVariant, cutoff: f32, predictive_cutoff: f32, cutoff_q: f32, predictive_q: f32, shift: f32, predictive_shift: f32, predictive_weight: f32, dt: f32) -> Self {
        Self {
            lowpass_filter: LowpassFilter::new(variant, cutoff, cutoff_q, shift, dt),
            predictive_lowpass: LowpassFilter::new(predictive_variant, predictive_cutoff, predictive_q, predictive_shift, dt),
            predictive_weight,
        }
    }

    pub fn apply(&mut self, filter_variant: &LowpassVariant, predictive_variant: &LowpassVariant, input: f32) -> f32 {
        unsafe {
            let filtered = self.lowpass_filter.apply(filter_variant, input);
            let predictive = if *predictive_variant != LowpassVariant::Off {
                self.predictive_lowpass.apply(predictive_variant, input - filtered) * self.predictive_weight
            } else {
                0.0
            };

            filtered + predictive
        }
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LowpassVariant {
    Off,
    Pt1,
    Pt2,
    Pt3,
    FirstOrder,
    SecondOrder,
    SlidingOrder,
    PtSecondOrder,
}

#[repr(C)]
pub union LowpassFilter {
    pub off: NoFilter,
    pub pt1: Pt1Filter,
    pub pt2: Pt2Filter,
    pub pt3: Pt3Filter,
    pub first_order: FirstOrderLowpassFilter,
    pub second_order: Biquad,
    pub sliding_order: Biquad,
    pub pt_second_order: Biquad,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct NoFilter;

impl LowpassFilter {
    fn new(variant: &LowpassVariant, cutoff: f32, q: f32, shift: f32, dt: f32) -> Self {
        match variant {
            LowpassVariant::Off => LowpassFilter { off: NoFilter },
            LowpassVariant::Pt1 => LowpassFilter { pt1: Pt1Filter::new(cutoff, dt) },
            LowpassVariant::Pt2 => LowpassFilter { pt2: Pt2Filter::new(cutoff, dt) },
            LowpassVariant::Pt3 => LowpassFilter { pt3: Pt3Filter::new(cutoff, dt) },
            LowpassVariant::FirstOrder => LowpassFilter { first_order: FirstOrderLowpassFilter::new(cutoff, dt) },
            LowpassVariant::SecondOrder => LowpassFilter { second_order: Biquad::new_second_order_lowpass(q, cutoff, dt) },
            LowpassVariant::SlidingOrder => LowpassFilter { sliding_order: Biquad::new_sliding_order(q, cutoff, shift, dt) },
            LowpassVariant::PtSecondOrder => LowpassFilter { pt_second_order: Biquad::new_pt1_second_order(q, cutoff, shift, dt) },
        }
    }

    unsafe fn apply(&mut self, variant: &LowpassVariant, input: f32) -> f32 {
        match variant {
            LowpassVariant::Off => input,
            LowpassVariant::Pt1 => self.pt1.apply(input),
            LowpassVariant::Pt2 => self.pt2.apply(input),
            LowpassVariant::Pt3 => self.pt3.apply(input),
            LowpassVariant::FirstOrder => self.first_order.apply(input),
            LowpassVariant::SecondOrder => self.second_order.apply(input),
            LowpassVariant::SlidingOrder => self.sliding_order.apply(input),
            LowpassVariant::PtSecondOrder =>self.pt_second_order.apply(input),
        }
    }
}

#[cfg(test)]
mod gyro_filter_tests {
    use std::mem::MaybeUninit;
    use super::*;

    const LOOPRATE: f32 = 8000.0;
    const DT: f32 = 1.0 / LOOPRATE;

    const CUTOFF1: f32 = 100.0;
    const PREDICTIVE_CUTOFF1: f32 = 10.0;
    const CUTOFF_Q1: f32 = 0.707;
    const PREDICTIVE_Q1: f32 = 1.0;
    const CUTOFF_SHIFT1: f32 = 0.5;
    const PREDICTIVE_SHIFT1: f32 = 0.5;
    const PREDICTIVE_WEIGHT1: f32 = 0.5;

    const CUTOFF2: f32 = 120.0;
    const PREDICTIVE_CUTOFF2: f32 = 5.0;
    const CUTOFF_Q2: f32 = 0.707;
    const PREDICTIVE_Q2: f32 = 1.0;
    const CUTOFF_SHIFT2: f32 = 0.5;
    const PREDICTIVE_SHIFT2: f32 = 0.5;
    const PREDICTIVE_WEIGHT2: f32 = 0.5;

    #[test]
    fn off() {
        // given
        let values: [f32; 8] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let expected_outputs: [[f32; 3]; 8] = [
            [1.0, 1.0, 1.0],
            [2.0, 2.0, 2.0],
            [3.0, 3.0, 3.0],
            [4.0, 4.0, 4.0],
            [5.0, 5.0, 5.0],
            [6.0, 6.0, 6.0],
            [7.0, 7.0, 7.0],
            [8.0, 8.0, 8.0],
        ];
        let variants = GyroLowpassVariants::new(LowpassVariant::Off, LowpassVariant::Off, LowpassVariant::Off, LowpassVariant::Off);
        let mut filter = GyroFilters::new(
            CUTOFF1,
            PREDICTIVE_CUTOFF1,
            CUTOFF_Q1,
            PREDICTIVE_Q1,
            CUTOFF_SHIFT1,
            PREDICTIVE_SHIFT1,
            PREDICTIVE_WEIGHT1,
            CUTOFF2,
            PREDICTIVE_CUTOFF2,
            CUTOFF_Q2,
            PREDICTIVE_Q2,
            CUTOFF_SHIFT2,
            PREDICTIVE_SHIFT2,
            PREDICTIVE_WEIGHT2,

            &variants,
            DT,
        );
        let mut outputs: [[f32; 3]; 8] = [[0.0; 3]; 8];

        // when
        for (i, value) in values.iter().enumerate() {
            outputs[i] = filter.apply(&variants, [*value, *value, *value]);
        }

        // then
        assert_eq!(outputs, expected_outputs);
    }

    #[test]
    fn pt1() {
        // given
        let values: [f32; 8] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let expected_outputs: [[f32; 3]; 8] = [
            [0.007252392, 0.007252392, 0.007252392],
            [0.02780703, 0.02780703, 0.02780703],
            [0.06666092, 0.06666092, 0.06666092],
            [0.1278913, 0.1278913, 0.1278913],
            [0.21477383, 0.21477383, 0.21477383],
            [0.32988766, 0.32988766, 0.32988766],
            [0.47520816, 0.47520816, 0.47520816],
            [0.6521895, 0.6521895, 0.6521895]
        ];
        let variants = GyroLowpassVariants::new(LowpassVariant::Pt1, LowpassVariant::Pt1, LowpassVariant::Pt1, LowpassVariant::Pt1);
        let mut filter = GyroFilters::new(
            CUTOFF1,
            PREDICTIVE_CUTOFF1,
            CUTOFF_Q1,
            PREDICTIVE_Q1,
            CUTOFF_SHIFT1,
            PREDICTIVE_SHIFT1,
            PREDICTIVE_WEIGHT1,
            CUTOFF2,
            PREDICTIVE_CUTOFF2,
            CUTOFF_Q2,
            PREDICTIVE_Q2,
            CUTOFF_SHIFT2,
            PREDICTIVE_SHIFT2,
            PREDICTIVE_WEIGHT2,

            &variants,
            DT,
        );
        let mut outputs: [[f32; 3]; 8] = [[0.0; 3]; 8];

        // when
        for (i, value) in values.iter().enumerate() {
            outputs[i] = filter.apply(&variants, [*value, *value, *value]);
        }

        // then
        assert_eq!(outputs, expected_outputs);
    }

    #[test]
    fn c_pt1() {
        // given
        let values: [f32; 8] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let expected_outputs: [[f32; 3]; 8] = [
            [0.007252392, 0.007252392, 0.007252392],
            [0.02780703, 0.02780703, 0.02780703],
            [0.06666092, 0.06666092, 0.06666092],
            [0.1278913, 0.1278913, 0.1278913],
            [0.21477383, 0.21477383, 0.21477383],
            [0.32988766, 0.32988766, 0.32988766],
            [0.47520816, 0.47520816, 0.47520816],
            [0.6521895, 0.6521895, 0.6521895]
        ];
        let mut variants: MaybeUninit<GyroLowpassVariants> = MaybeUninit::uninit();
        gyro_lowpass_init(variants.as_mut_ptr(), LowpassVariant::Pt1, LowpassVariant::Pt1, LowpassVariant::Pt1, LowpassVariant::Pt1);
        let variants = unsafe { variants.assume_init() };

        let mut filter: MaybeUninit<GyroFilters> = MaybeUninit::uninit();
        gyro_filter_init(
            filter.as_mut_ptr(),
            CUTOFF1,
            PREDICTIVE_CUTOFF1,
            CUTOFF_Q1,
            PREDICTIVE_Q1,
            CUTOFF_SHIFT1,
            PREDICTIVE_SHIFT1,
            PREDICTIVE_WEIGHT1,
            CUTOFF2,
            PREDICTIVE_CUTOFF2,
            CUTOFF_Q2,
            PREDICTIVE_Q2,
            CUTOFF_SHIFT2,
            PREDICTIVE_SHIFT2,
            PREDICTIVE_WEIGHT2,

            &variants,
            DT,
        );

        let mut filter = unsafe { filter.assume_init() };

        let mut outputs: [[f32; 3]; 8] = [[0.0; 3]; 8];

        // when
        for (i, value) in values.iter().enumerate() {
            outputs[i] = [*value, *value, *value];
            gyro_filter_apply(&mut filter, &variants, &mut outputs[i]);
        }

        // then
        assert_eq!(outputs, expected_outputs);
    }
}
