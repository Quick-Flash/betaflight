use crate::filter::ptn::*;
use crate::filter::biquad::*;
use crate::filter::predictive::*;

#[repr(C)]
pub struct GyroFilters {
    lowpass1: [LowpassFilter; 3],
    lowpass2: [LowpassFilter; 3],
}

impl GyroFilters {
    pub fn new(
        cutoff1: f32,
        predictive_cutoff1: f32,
        cutoff_q1: f32,
        predictive_q1: f32,

        cutoff2: f32,
        predictive_cutoff2: f32,
        cutoff_q2: f32,
        predictive_q2: f32,

        lowpass_variants: &GyroLowpassVariants,
        dt: f32
    ) -> Self {
        Self {
            lowpass1: [
                LowpassFilter::new(&lowpass_variants.lowpass1, cutoff1, predictive_cutoff1, cutoff_q1, predictive_q1, dt),
                LowpassFilter::new(&lowpass_variants.lowpass1, cutoff1, predictive_cutoff1, cutoff_q1, predictive_q1, dt),
                LowpassFilter::new(&lowpass_variants.lowpass1, cutoff1, predictive_cutoff1, cutoff_q1, predictive_q1, dt),
            ],
            lowpass2: [
                LowpassFilter::new(&lowpass_variants.lowpass2, cutoff2, predictive_cutoff2, cutoff_q2, predictive_q2, dt),
                LowpassFilter::new(&lowpass_variants.lowpass2, cutoff2, predictive_cutoff2, cutoff_q2, predictive_q2, dt),
                LowpassFilter::new(&lowpass_variants.lowpass2, cutoff2, predictive_cutoff2, cutoff_q2, predictive_q2, dt),
            ],
        }
    }

    pub fn apply(&mut self, lowpass_variants: &GyroLowpassVariants, input: [f32; 3]) -> [f32; 3] {
        let mut output = [0.0; 3];
        for i in 0..3 {
            unsafe {
                let first_pass = self.lowpass1[i].apply(&lowpass_variants.lowpass1, input[i]);
                output[i] = self.lowpass2[i].apply(&lowpass_variants.lowpass2, first_pass);
            }
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

    cutoff2: f32,
    predictive_cutoff2: f32,
    cutoff_q2: f32,
    predictive_q2: f32,

    lowpass_variants: &GyroLowpassVariants,
    dt: f32
)
{
    unsafe {
        *filter = GyroFilters::new(cutoff1, predictive_cutoff1, cutoff_q1, predictive_q1,
                                   cutoff2, predictive_cutoff2, cutoff_q2, predictive_q2,
                                   lowpass_variants, dt
        );
    }
}

#[link_section = ".tcm_code"]
#[inline]
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
    lowpass2: LowpassVariant,
}

impl GyroLowpassVariants {
    pub fn new(variant1: LowpassVariant, variant2: LowpassVariant) -> Self {
        Self {
            lowpass1: variant1,
            lowpass2: variant2,
        }
    }
}

#[no_mangle] pub extern "C" fn gyro_lowpass_init(gyro_variants: *mut GyroLowpassVariants, variant1: LowpassVariant, variant2: LowpassVariant) {
    unsafe {
        *gyro_variants = GyroLowpassVariants::new(variant1, variant2);
    }
}


#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum LowpassVariant {
    Off,
    Pt1,
    Pt2,
    Pt3,
    FirstOrder,
    SecondOrder,
    PredictivePt1,
    PredictiveFirstOrder,
    PredictiveSecondOrder,
}

#[repr(C)]
pub union LowpassFilter {
    pub off: NoFilter,
    pub pt1: Pt1Filter,
    pub pt2: Pt2Filter,
    pub pt3: Pt3Filter,
    pub first_order: FirstOrderLowpassFilter,
    pub second_order: SecondOrderLowpassFilter,
    pub predictive_pt1: PredictivePt1Filter,
    pub predictive_first_order: PredictiveFirstOrderFilter,
    pub predictive_second_order: PredictiveSecondOrderFilter,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct NoFilter;

impl LowpassFilter {
    fn new(variant: &LowpassVariant, cutoff: f32, predictive_cutoff: f32, cutoff_q: f32, predictive_q: f32, dt: f32) -> Self {
        match variant {
            LowpassVariant::Off => LowpassFilter { off: NoFilter },
            LowpassVariant::Pt1 => LowpassFilter { pt1: Pt1Filter::new(cutoff, dt) },
            LowpassVariant::Pt2 => LowpassFilter { pt2: Pt2Filter::new(cutoff, dt) },
            LowpassVariant::Pt3 => LowpassFilter { pt3: Pt3Filter::new(cutoff, dt) },
            LowpassVariant::FirstOrder => LowpassFilter { first_order: FirstOrderLowpassFilter::new(cutoff, dt) },
            LowpassVariant::SecondOrder => LowpassFilter { second_order: SecondOrderLowpassFilter::new(cutoff_q, cutoff, dt) },
            LowpassVariant::PredictivePt1 => LowpassFilter { predictive_pt1: PredictivePt1Filter::new(cutoff, predictive_cutoff, dt) },
            LowpassVariant::PredictiveFirstOrder => LowpassFilter { predictive_first_order: PredictiveFirstOrderFilter::new(cutoff, predictive_cutoff, dt) },
            LowpassVariant::PredictiveSecondOrder => LowpassFilter { predictive_second_order: PredictiveSecondOrderFilter::new(cutoff, cutoff_q, predictive_cutoff, predictive_q, dt) },
        }
    }

    pub unsafe fn apply(&mut self, variant: &LowpassVariant, input: f32) -> f32 {
        match variant {
            LowpassVariant::Off => input,
            LowpassVariant::Pt1 => self.pt1.apply(input),
            // LowpassVariant::Pt1 => input * 0.5,
            LowpassVariant::Pt2 => self.pt2.apply(input),
            LowpassVariant::Pt3 => self.pt3.apply(input),
            LowpassVariant::FirstOrder => self.first_order.apply(input),
            LowpassVariant::SecondOrder => self.second_order.apply(input),
            LowpassVariant::PredictivePt1 => self.predictive_pt1.apply(input),
            LowpassVariant::PredictiveFirstOrder => self.predictive_first_order.apply(input),
            LowpassVariant::PredictiveSecondOrder => self.predictive_second_order.apply(input),
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

    const CUTOFF2: f32 = 120.0;
    const PREDICTIVE_CUTOFF2: f32 = 5.0;
    const CUTOFF_Q2: f32 = 0.707;
    const PREDICTIVE_Q2: f32 = 1.0;

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
        let variants = GyroLowpassVariants::new(LowpassVariant::Off, LowpassVariant::Off);
        let mut filter = GyroFilters::new(
            CUTOFF1,
            PREDICTIVE_CUTOFF1,
            CUTOFF_Q1,
            PREDICTIVE_Q1,
            CUTOFF2,
            PREDICTIVE_CUTOFF2,
            CUTOFF_Q2,
            PREDICTIVE_Q2,
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
            [0.0067857387, 0.0067857387, 0.0067857387],
            [0.026020745, 0.026020745, 0.026020745],
            [0.06238588, 0.06238588, 0.06238588],
            [0.11970341, 0.11970341, 0.11970341],
            [0.20104726, 0.20104726, 0.20104726],
            [0.3088408, 0.3088408, 0.3088408],
            [0.4449436, 0.4449436, 0.4449436],
            [0.61072826, 0.61072826, 0.61072826],
        ];
        let variants = GyroLowpassVariants::new(LowpassVariant::Pt1, LowpassVariant::Pt1);
        let mut filter = GyroFilters::new(
            CUTOFF1,
            PREDICTIVE_CUTOFF1,
            CUTOFF_Q1,
            PREDICTIVE_Q1,
            CUTOFF2,
            PREDICTIVE_CUTOFF2,
            CUTOFF_Q2,
            PREDICTIVE_Q2,
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
            [0.0067857387, 0.0067857387, 0.0067857387],
            [0.026020745, 0.026020745, 0.026020745],
            [0.06238588, 0.06238588, 0.06238588],
            [0.11970341, 0.11970341, 0.11970341],
            [0.20104726, 0.20104726, 0.20104726],
            [0.3088408, 0.3088408, 0.3088408],
            [0.4449436, 0.4449436, 0.4449436],
            [0.61072826, 0.61072826, 0.61072826],
        ];
        let mut variants: MaybeUninit<GyroLowpassVariants> = MaybeUninit::uninit();
        gyro_lowpass_init(variants.as_mut_ptr(), LowpassVariant::Pt1, LowpassVariant::Pt1);
        let variants = unsafe { variants.assume_init() };

        let mut filter: MaybeUninit<GyroFilters> = MaybeUninit::uninit();
        gyro_filter_init(
            filter.as_mut_ptr(),
            CUTOFF1,
            PREDICTIVE_CUTOFF1,
            CUTOFF_Q1,
            PREDICTIVE_Q1,
            CUTOFF2,
            PREDICTIVE_CUTOFF2,
            CUTOFF_Q2,
            PREDICTIVE_Q2,
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

    // #[test]
    // fn below_min() {
    //     // given
    //     let mut nmf = NonLinearMedian::new(10.0);
    //     let output: f32;
    //
    //     // when
    //     nmf.apply(7.0);
    //     nmf.apply(6.0);
    //     nmf.apply(5.0);
    //     output = nmf.apply(0.0);
    //
    //     // then
    //     assert_eq!(output, 5.0);
    // }
    //
    // #[test]
    // fn above_max() {
    //     // given
    //     let mut nmf = NonLinearMedian::new(0.0);
    //     let output: f32;
    //
    //     // when
    //     nmf.apply(5.0);
    //     nmf.apply(6.0);
    //     nmf.apply(7.0);
    //     output = nmf.apply(10.0);
    //
    //     // then
    //     assert_eq!(output, 7.0);
    // }
    //
    // #[test]
    // fn first_input() {
    //     // given
    //     let mut nmf = NonLinearMedian::new(50.0);
    //     let output: f32;
    //
    //     // when
    //     output = nmf.apply(10.0);
    //
    //     // then
    //     assert_eq!(output, 50.0);
    // }
}
