pub trait Sign<T> {
    fn sign(&self) -> T;
}

impl Sign<f32> for f32 {

    /// Safety: zero is handled as follows
    /// Positive zero ( 0.0) returns 1.0
    /// Negative zero (-0.0) returns 1.0
    #[inline(always)]
    fn sign(&self) -> f32 {
        if *self > 0.0 {
            1.0
        } else if *self < 0.0 {
            -1.0
        } else {
            0.0
        }
    }
}

#[cfg(test)]
mod sign_tests {
    use super::*;

    #[test]
    fn sign_positive() {
        // expect
        assert_eq!(100.0.sign(), 1.0);
    }

    #[test]
    fn sign_negative() {
        // expect
        assert_eq!((-100.0).sign(), -1.0);
    }

    #[test]
    fn sign_positive_zero() {
        // expect
        assert_eq!(0.0.sign(), 0.0);
    }

    #[test]
    fn sign_negative_zero() {
        // expect
        assert_eq!((-0.0).sign(), 0.0);
    }
}