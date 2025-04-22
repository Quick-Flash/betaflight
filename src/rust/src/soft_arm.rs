#[repr(C)]
pub struct SoftArm {
    pub exceeded_throttle_threshold: bool,
    pub soft_arm_percent_inv: f32,
}

impl SoftArm {
    pub fn new() -> Self {
        Self {
            exceeded_throttle_threshold: false,
            soft_arm_percent_inv: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.exceeded_throttle_threshold = false;
    }

    pub fn in_soft_arm(&self) -> bool {
        !self.exceeded_throttle_threshold
    }
    
    pub fn soft_arm_percent(&self) -> f32 {
        self.soft_arm_percent_inv
    }

    pub fn attenuate(&mut self, throttle_threshold: f32, throttle: f32) -> f32 {
        if throttle < throttle_threshold {
            let percent = throttle / throttle_threshold;
            self.soft_arm_percent_inv = percent;
            
            percent
        } else {
            self.exceeded_throttle_threshold = true;
            self.soft_arm_percent_inv = 1.0;
            
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
pub extern "C" fn soft_arm_percent(soft_arm: SoftArm) -> f32 {
    soft_arm.soft_arm_percent()
}

#[no_mangle]
pub extern "C" fn soft_arm_reset(soft_arm: *mut SoftArm) {
    unsafe {
        (*soft_arm).reset();
    }
}

#[link_section = ".tcm_code"]
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
