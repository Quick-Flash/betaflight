// TODO linearly go from very low min throttle to a higher one as we leave soft arm

#[repr(C)]
pub struct SoftArm {
    pub exceeded_throttle_threshold: bool,
}

impl SoftArm {
    pub fn new() -> Self {
        Self {
            exceeded_throttle_threshold: false,
        }
    }

    pub fn reset(&mut self) {
        self.exceeded_throttle_threshold = false;
    }

    pub fn in_soft_arm(&self) -> bool {
        !self.exceeded_throttle_threshold
    }

    // TODO look at using this to attenuate the mixer range, requires updating the mixer, but that's needed and will be a mess...
    pub fn attenuate(&mut self, throttle_threshold: f32, throttle: f32) -> f32 {
        if throttle < throttle_threshold {
            throttle / throttle_threshold
        } else {
            self.exceeded_throttle_threshold = true;
            1.0
        }
    }
}

#[no_mangle]
pub extern "C" fn soft_arm_init(soft_arm: *mut SoftArm) {
    unsafe {
        *soft_arm = SoftArm::new();
    }
}

#[no_mangle]
pub extern "C" fn in_soft_arm(soft_arm: SoftArm) -> bool {
    soft_arm.in_soft_arm()
}

#[no_mangle]
pub extern "C" fn soft_arm_reset(soft_arm: *mut SoftArm) {
    unsafe {
        (*soft_arm).reset();
    }
}

#[link_section = ".tcm_code"]
#[inline]
#[no_mangle] pub extern "C" fn soft_arm_attenuate(
    soft_arm: *mut SoftArm,
    throttle_threshold: f32,
    throttle: f32,
) -> f32
{
    unsafe {
        (*soft_arm).attenuate(throttle_threshold, throttle)
    }
}

#[cfg(test)]
mod soft_arm_tests {
    use super::*;

    #[test]
    fn soft_arm_initial_state() {
        // given
        let soft_arm = SoftArm::new();

        // then
        assert!(!soft_arm.exceeded_throttle_threshold); // Should be false by default
    }

    #[test]
    fn soft_arm_reset() {
        // given
        let mut soft_arm = SoftArm::new();
        soft_arm.exceeded_throttle_threshold = true; // Set the state to true

        // when
        soft_arm.reset();

        // then
        assert!(!soft_arm.exceeded_throttle_threshold); // Should reset to false
    }

    #[test]
    fn soft_arm_in_soft_arm() {
        // given
        let soft_arm = SoftArm::new();

        // then
        assert!(soft_arm.in_soft_arm()); // Should return true when not exceeded
    }

    #[test]
    fn soft_arm_in_soft_arm_exceeded() {
        // given
        let mut soft_arm = SoftArm::new();
        soft_arm.exceeded_throttle_threshold = true; // Exceeded state

        // then
        assert!(!soft_arm.in_soft_arm()); // Should return false when exceeded
    }

    #[test]
    fn soft_arm_attenuate_below_threshold() {
        // given
        let mut soft_arm = SoftArm::new();
        let throttle_threshold = 0.5;
        let throttle = 0.3; // Below the threshold

        // when
        let result = soft_arm.attenuate(throttle_threshold, throttle);

        // then
        assert_eq!(result, 0.6); // Should return (throttle / threshold) = 0.3 / 0.5
        assert!(!soft_arm.exceeded_throttle_threshold); // Should not exceed
    }

    #[test]
    fn soft_arm_attenuate_above_threshold() {
        // given
        let mut soft_arm = SoftArm::new();
        let throttle_threshold = 0.5;
        let throttle = 0.7; // Above the threshold

        // when
        let result = soft_arm.attenuate(throttle_threshold, throttle);

        // then
        assert_eq!(result, 1.0); // Should return 1.0
        assert!(soft_arm.exceeded_throttle_threshold); // Should exceed
    }
}
