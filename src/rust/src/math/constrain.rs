#[inline(always)]
pub fn constrain(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

#[inline(always)]
pub fn bound(value: f32, bound: f32) -> f32 {
    constrain(value, -bound, bound)
}

#[cfg(test)]
mod constrainf_tests {
    use super::*;

    #[test]
    fn constrain_max() {
        assert_eq!(constrain(10.0, 0.0, 1.0), 1.0);
    }

    #[test]
    fn constrain_min() {
        assert_eq!(constrain(-10.0, -1.0, 0.0), -1.0);
    }

    #[test]
    fn no_constrain() {
        assert_eq!(constrain(0.0, -1.0, 1.0), 0.0);
    }

    #[test]
    fn bound_max() {
        assert_eq!(bound(10.0, 1.0), 1.0);
    }

    #[test]
    fn bound_min() {
        assert_eq!(bound(-10.0, 10.0), -10.0);
    }

    #[test]
    fn no_bound() {
        assert_eq!(bound(0.0, 10.0), 0.0);
    }

}