use crate::filter::biquad::{FirstOrderLowpassFilter, NotchFilter, SecondOrderLowpassFilter};
use crate::filter::ptn::Pt1Filter;

pub struct Pt1PredictiveNotchFilter {
    notch: NotchFilter,
    pt1: Pt1Filter,
}

impl Pt1PredictiveNotchFilter {
    pub fn new(notch_cutoff: f32, notch_q: f32, pt1_cutoff: f32, dt: f32) -> Self {
        Self {
            notch: NotchFilter::new(notch_q, notch_cutoff, dt),
            pt1: Pt1Filter::new(pt1_cutoff, dt),
        }
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        let notched = self.notch.apply(input);
        notched + self.pt1.apply(input - notched)
    }

    pub fn reset(&mut self) {
        self.notch.reset();
        self.pt1.reset();
    }
}

pub struct FirstOrderPredictiveNotchFilter {
    notch: NotchFilter,
    first_order_lowpass: FirstOrderLowpassFilter,
}

impl FirstOrderPredictiveNotchFilter {
    pub fn new(notch_cutoff: f32, notch_q: f32, lowpass_cutoff: f32, dt: f32) -> Self {
        Self {
            notch: NotchFilter::new(notch_q, notch_cutoff, dt),
            first_order_lowpass: FirstOrderLowpassFilter::new(lowpass_cutoff, dt),
        }
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        let notched = self.notch.apply(input);
        notched + self.first_order_lowpass.apply(input - notched)
    }

    pub fn reset(&mut self) {
        self.notch.reset();
        self.first_order_lowpass.reset();
    }
}

pub struct SecondOrderPredictiveNotchFilter {
    notch: NotchFilter,
    second_order_lowpass: SecondOrderLowpassFilter,
}

impl SecondOrderPredictiveNotchFilter {
    pub fn new(notch_cutoff: f32, notch_q: f32, lowpass_cutoff: f32, lowpass_q: f32, dt: f32) -> Self {
        Self {
            notch: NotchFilter::new(notch_q, notch_cutoff, dt),
            second_order_lowpass: SecondOrderLowpassFilter::new(lowpass_q, lowpass_cutoff, dt),
        }
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        let notched = self.notch.apply(input);
        notched + self.second_order_lowpass.apply(input - notched)
    }

    pub fn reset(&mut self) {
        self.notch.reset();
        self.second_order_lowpass.reset();
    }
}

pub struct PredictivePt1Filter {
    main_lpf: Pt1Filter,
    leftover_lpf: Pt1Filter,
}

impl PredictivePt1Filter {
    pub fn new(main_cutoff: f32, leftover_cutoff: f32, dt: f32) -> Self {
        Self {
            main_lpf: Pt1Filter::new(main_cutoff, dt),
            leftover_lpf: Pt1Filter::new(leftover_cutoff, dt),
        }
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        let notched = self.main_lpf.apply(input);
        notched + self.leftover_lpf.apply(input - notched)
    }

    pub fn reset(&mut self) {
        self.main_lpf.reset();
        self.leftover_lpf.reset();
    }
}

pub struct PredictiveFirstOrderFilter {
    main_lpf: FirstOrderLowpassFilter,
    leftover_lpf: FirstOrderLowpassFilter,
}

impl PredictiveFirstOrderFilter {
    pub fn new(main_cutoff: f32, leftover_cutoff: f32, dt: f32) -> Self {
        Self {
            main_lpf: FirstOrderLowpassFilter::new(main_cutoff, dt),
            leftover_lpf: FirstOrderLowpassFilter::new(leftover_cutoff, dt),
        }
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        let notched = self.main_lpf.apply(input);
        notched + self.leftover_lpf.apply(input - notched)
    }

    pub fn reset(&mut self) {
        self.main_lpf.reset();
        self.leftover_lpf.reset();
    }
}

pub struct PredictiveSecondOrderFilter {
    main_lpf: SecondOrderLowpassFilter,
    leftover_lpf: SecondOrderLowpassFilter,
}

impl PredictiveSecondOrderFilter {
    pub fn new(main_cutoff: f32, main_q: f32, leftover_cutoff: f32, leftover_q: f32, dt: f32) -> Self {
        Self {
            main_lpf: SecondOrderLowpassFilter::new(main_q, main_cutoff, dt),
            leftover_lpf: SecondOrderLowpassFilter::new(leftover_q, leftover_cutoff, dt),
        }
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        let notched = self.main_lpf.apply(input);
        notched + self.leftover_lpf.apply(input - notched)
    }

    pub fn reset(&mut self) {
        self.main_lpf.reset();
        self.leftover_lpf.reset();
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

    #[cfg(test)]
    mod pt1_predictive_notch_tests {
        use super::*;

        #[test]
        fn pt1_predictive_notch_apply() {
            // given
            let mut pt1_predictive_notch = Pt1PredictiveNotchFilter::new(MAIN_CUTOFF, MAIN_Q, LEFTOVER_CUTOFF, DT);

            // then
            assert_eq!(pt1_predictive_notch.apply(INPUT), 948.0456);
        }
    }

    #[cfg(test)]
    mod first_order_predictive_notch_tests {
        use super::*;

        #[test]
        fn first_order_predictive_notch_apply() {
            // given
            let mut first_order_predictive_notch = FirstOrderPredictiveNotchFilter::new(MAIN_CUTOFF, MAIN_Q, LEFTOVER_CUTOFF, DT);

            // then
            assert_eq!(first_order_predictive_notch.apply(INPUT), 947.73755);
        }
    }

    #[cfg(test)]
    mod second_order_predictive_notch_tests {
        use super::*;

        #[test]
        fn second_order_predictive_notch_apply() {
            // given
            let mut second_order_predictive_notch = SecondOrderPredictiveNotchFilter::new(MAIN_CUTOFF, MAIN_Q, LEFTOVER_CUTOFF, LEFTOVER_Q, DT);

            // then
            assert_eq!(second_order_predictive_notch.apply(INPUT), 947.4315);
        }
    }

    #[cfg(test)]
    mod predictive_pt1_tests {
        use super::*;

        #[test]
        fn predictive_pt1_apply() {
            // given
            let mut predictive_pt1 = PredictivePt1Filter::new(MAIN_CUTOFF, LEFTOVER_CUTOFF, DT);

            // then
            assert_eq!(predictive_pt1.apply(INPUT), 86.328735);
        }
    }

    #[cfg(test)]
    mod predictive_first_order_tests {
        use super::*;

        #[test]
        fn predictive_first_order_apply() {
            // given
            let mut predictive_first_order = PredictiveFirstOrderFilter::new(MAIN_CUTOFF, LEFTOVER_CUTOFF, DT);

            // then
            assert_eq!(predictive_first_order.apply(INPUT), 43.43939);
        }
    }

    #[cfg(test)]
    mod predictive_second_order_tests {
        use super::*;

        #[test]
        fn predictive_second_order_apply() {
            // given
            let mut predictive_second_order = PredictiveSecondOrderFilter::new(MAIN_CUTOFF, MAIN_Q, LEFTOVER_CUTOFF, LEFTOVER_Q, DT);

            // then
            assert_eq!(predictive_second_order.apply(INPUT), 1.4945825);
        }
    }
}