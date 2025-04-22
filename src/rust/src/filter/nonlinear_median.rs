#[derive(Copy, Clone)]
#[repr(C)]
pub struct NonLinearMedian {
    min: f32,
    max: f32,
    previous_output: f32,
}

impl NonLinearMedian {
    pub fn new(initial_value: f32) -> Self {
        Self {
            min: initial_value,
            max: initial_value,
            previous_output: initial_value,
        }
    }

    pub fn reset(&mut self, new_value: f32) {
        self.max = new_value;
        self.min = new_value;
        self.previous_output = new_value;
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        let output: f32;
        if input > self.max {
            output = self.max;
        } else if input < self.min {
            output = self.min;
        } else {
            output = input;
        }

        if input > self.previous_output {
            self.max = input;
            self.min = self.previous_output;
        } else {
            self.max = self.previous_output;
            self.min = input;
        }
        self.previous_output = output;

        output
    }
}

#[no_mangle] pub extern "C" fn nonlinear_median_init(filter: *mut NonLinearMedian) {
    unsafe {
        *filter = NonLinearMedian::new(0.0);
    }
}

#[link_section = ".tcm_code"]
#[no_mangle] pub extern "C" fn nonlinear_median_apply(filter: *mut NonLinearMedian, input: f32) -> f32 {
    unsafe {
        (*filter).apply(input)
    }
}

#[cfg(test)]
mod nonlinear_median_tests {
    use super::*;

    #[test]
    fn outlier_suppression() {
        // given
        let values: [f32; 8] = [5.0, 5.0, 10.0, 5.0, 5.0, 0.0, 5.0, 5.0];
        let expected_outputs: [f32; 8] = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0];
        let mut nmf = NonLinearMedian::new(5.0);
        let mut outputs: [f32; 8] = [0.0; 8];

        // when
        for (i, value) in values.iter().enumerate() {
            outputs[i] = nmf.apply(*value);
        }

        // then
        assert_eq!(outputs, expected_outputs);
    }

    #[test]
    fn below_min() {
        // given
        let mut nmf = NonLinearMedian::new(10.0);
        let output: f32;

        // when
        nmf.apply(7.0);
        nmf.apply(6.0);
        nmf.apply(5.0);
        output = nmf.apply(0.0);

        // then
        assert_eq!(output, 5.0);
    }

    #[test]
    fn above_max() {
        // given
        let mut nmf = NonLinearMedian::new(0.0);
        let output: f32;

        // when
        nmf.apply(5.0);
        nmf.apply(6.0);
        nmf.apply(7.0);
        output = nmf.apply(10.0);

        // then
        assert_eq!(output, 7.0);
    }

    #[test]
    fn first_input() {
        // given
        let mut nmf = NonLinearMedian::new(50.0);
        let output: f32;

        // when
        output = nmf.apply(10.0);

        // then
        assert_eq!(output, 50.0);
    }
}