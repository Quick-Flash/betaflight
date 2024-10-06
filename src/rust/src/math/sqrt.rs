#[inline(always)]
fn sqrt_f32(f: f32) -> f32 {
    unsafe { core::intrinsics::sqrtf32(f) }
}

pub trait Sqrtf<T> {
    fn sqrtf(&self) -> T;
}

impl Sqrtf<f32> for f32 {
    #[inline(always)]
    fn sqrtf(&self) -> f32 {
        sqrt_f32(*self)
    }
}

#[cfg(test)]
mod sqrt_tests {
    use super::*;

    #[test]
    pub fn sqrt_16() {
        // expect
        assert_eq!(16.0.sqrtf(), 4.0);
    }

    #[test]
    pub fn sqrt_negative() {
        // expect
        assert!((-1.0).sqrtf().is_nan());
    }
}
