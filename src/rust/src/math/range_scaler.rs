use core::ops::Range;

#[derive(Copy, Clone)]
#[repr(C)]
pub struct RangeScalar {
    scale_factor: f32,
    offset: f32,
}

impl RangeScalar {
    pub fn new(input: &Range<f32>, desired: &Range<f32>) -> Self {
        let range_desired: f32 = desired.end - desired.start;
        let range_input: f32 = input.end - input.start;

        let scale_factor = range_desired / range_input;
        let offset = desired.start - input.start * scale_factor;

        Self {
            scale_factor,
            offset
        }
    }

    pub fn apply(&self, input: f32) -> f32 {
        self.scale_factor * input + self.offset
    }
}

#[cfg(test)]
mod range_scalar_tests {
    use super::*;

    #[test]
    fn range_scalar() {
        // given
        let input: f32 = 5.0;
        let input_start: f32 = -5.0;
        let input_end: f32 = 10.0;
        let desired_end: f32 = 100.0;
        let desired_start: f32 = 10.0;

        let range_scalar = RangeScalar::new(&Range { start: input_start, end: input_end }, &Range { start: desired_start, end: desired_end});

        // when
        let output = range_scalar.apply(input);

        // then
        assert_eq!(output, 70.0);
    }
}