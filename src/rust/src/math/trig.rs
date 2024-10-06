use core::f32::consts::PI;

pub const DEGREE_TO_RADIAN: f32 = PI / 180.0;
pub const RADIAN_TO_DEGREE: f32 = 180.0 / PI;

pub trait Trig {
    fn sinf(&self) -> f32;
    fn cosf(&self) -> f32;
    fn sinf_unchecked(&self) -> f32;
    fn cosf_unchecked(&self) -> f32;
}

// will make the input be between -PI/2 and PI/2 on the unit circle
fn unit_circle_constrain(rad: f32) -> f32 {
    let mut x: f32 = rad;

    // make the input be between -PI..PI
    while x > PI {
        x -= 2.0 * PI;
    }
    while x < -PI {
        x += 2.0 * PI;
    }

    // make the input be between -PI/2..PI/2
    if x > (0.5 * PI) {
        x = PI - x;
    } else if x < -(0.5 * PI) {
        x = -PI - x;
    }

    x
}

impl Trig for f32 {
    // See for sin anc cos approximations: https://gist.github.com/publik-void/067f7f2fef32dbe5c27d6e215f824c91

    fn sinf(&self) -> f32 {
        let x = unit_circle_constrain(*self);

        x.sinf_unchecked()
    }

    fn cosf(&self) -> f32 {
        (self + (0.5 * PI)).sinf()
    }

    /// input must be between -PI/2..PI/2
    /// use whenever possible
    fn sinf_unchecked(&self) -> f32 {
        const C1: f32 = 0.999_999_05;
        const C3: f32 = -0.166_655_54;
        const C5: f32 = 0.008_311_899;
        const C7: f32 = -0.000_184_881_4;

        let x1 = *self;
        let x2 = x1 * x1;

        x1 * (C1 + x2 * (C3 + x2 * (C5 + C7 * x2)))
    }

    /// input must be between -PI/2..PI/2
    /// use whenever possible
    fn cosf_unchecked(&self) -> f32 {
        const C0: f32 = 0.999_999_94;
        const C2: f32 = -0.499_999_05;
        const C4: f32 = 0.041_663_583;
        const C6: f32 = -0.001_385_370_4;
        const C8: f32 = 0.000_023_153_932;

        let x1 = *self;
        let x2 = x1 * x1;

        C0 + x2 * (C2 + x2 * (C4 + x2 * (C6 + C8 * x2)))
    }
}

#[cfg(test)]
mod trig_tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn sin_unchecked() {
        // given
        let pi_2 = PI / 2.0;
        let pi_3 = PI / 3.0;
        let pi_4 = -PI / 4.0;
        let zero = 0.0;

        // then
        assert_abs_diff_eq!(pi_2.sinf_unchecked(), pi_2.sin(), epsilon = 0.000001);
        assert_abs_diff_eq!(pi_3.sinf_unchecked(), pi_3.sin(), epsilon = 0.000001);
        assert_abs_diff_eq!(pi_4.sinf_unchecked(), pi_4.sin(), epsilon = 0.000001);
        assert_abs_diff_eq!(zero.sinf_unchecked(), zero.sin(), epsilon = f32::EPSILON);
    }

    #[test]
    fn cos_unchecked() {
        // given
        let pi_2 = PI / 2.0;
        let pi_3 = PI / 3.0;
        let pi_4 = -PI / 4.0;
        let zero = 0.0;

        // then
        assert_abs_diff_eq!(pi_2.cosf_unchecked(), pi_2.cos(), epsilon = f32::EPSILON);
        assert_abs_diff_eq!(pi_3.cosf_unchecked(), pi_3.cos(), epsilon = f32::EPSILON);
        assert_abs_diff_eq!(pi_4.cosf_unchecked(), pi_4.cos(), epsilon = f32::EPSILON);
        assert_abs_diff_eq!(zero.cosf_unchecked(), zero.cos(), epsilon = f32::EPSILON);
    }

    #[test]
    fn sin() {
        // given
        let pi_2 = 5.0 * PI / 2.0;
        let pi_3 = -5.0 * PI / 3.0;
        let pi_4 = -PI / 4.0;
        let zero = 0.0;

        // then
        assert_abs_diff_eq!(pi_2.sinf(), pi_2.sin(), epsilon = 0.000001);
        assert_abs_diff_eq!(pi_3.sinf(), pi_3.sin(), epsilon = 0.000001);
        assert_abs_diff_eq!(pi_4.sinf(), pi_4.sin(), epsilon = 0.000001);
        assert_abs_diff_eq!(zero.sinf(), zero.sin(), epsilon = 0.000001);
    }

    #[test]
    fn cos() {
        // given
        let pi_2 = 5.0 * PI / 2.0;
        let pi_3 = -5.0 * PI / 3.0;
        let pi_4 = -PI / 4.0;
        let zero = 0.0;

        // then
        assert_abs_diff_eq!(pi_2.cosf(), pi_2.cos(), epsilon = 0.000001);
        assert_abs_diff_eq!(pi_3.cosf(), pi_3.cos(), epsilon = 0.000001);
        assert_abs_diff_eq!(pi_4.cosf(), pi_4.cos(), epsilon = 0.000001);
        assert_abs_diff_eq!(zero.cosf(), zero.cos(), epsilon = 0.000001);
    }
}
