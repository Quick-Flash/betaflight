#[inline(always)]
fn powi_f32(input: f32, power: i32) -> f32 {
    unsafe { core::intrinsics::powif32(input, power) }
}

pub trait PowiF<T> {
    fn powif(&self, power: i32) -> T;
}

impl PowiF<f32> for f32 {
    #[inline(always)]
    fn powif(&self, power: i32) -> f32 {
        powi_f32(*self, power)
    }
}

#[cfg(test)]
mod powi_tests {
    use super::*;

    #[test]
    pub fn powi_positive() {
        // expect
        assert_eq!(16.0.powif(2), 256.0);
    }

    #[test]
    pub fn powi_negative_power() {
        // expect
        assert_eq!(4.0.powif(-2), 0.0625);
    }


    #[test]
    pub fn powi_negative() {
        // expect
        assert_eq!((-4.0).powif(3), -64.0);
    }
}
