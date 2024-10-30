use crate::filter::gyro_filter::{GyroFilters, GyroLowpassVariants};
use crate::two_tap_switch::TwoTapSwitchStage::*;

#[repr(C)]
#[derive(Debug, PartialEq, Eq)]
pub enum TwoTapSwitchStage {
    Idle,
    FirstStage,
    SecondStage,
    Ready,
    Timeout,
}

#[repr(C)]
pub struct TwoTapSwitch {
    start_time_us: u32,
    stage: TwoTapSwitchStage,
}

impl TwoTapSwitch {
    pub fn new() -> Self {
        Self {
            start_time_us: 0,
            stage: Idle,
        }
    }

    fn reset_if_timeout(&mut self, timestamp_us: u32, switch_high: bool) {
        if (timestamp_us - self.start_time_us) > 500_000 { // half a second
            if switch_high {
                self.stage = Timeout;
            } else {
                self.stage = Idle;
            }
        }
    }

    pub fn process(&mut self, timestamp_us: u32, switch_high: bool) -> bool {
        match &self.stage {
            Idle => {
                if switch_high {
                    self.stage = FirstStage;
                    self.start_time_us = timestamp_us;
                }
            }
            FirstStage => {
                if !switch_high {
                    self.stage = SecondStage;
                }
                self.reset_if_timeout(timestamp_us, switch_high);
            }
            SecondStage => {
                if switch_high {
                    self.stage = Ready;
                }
                self.reset_if_timeout(timestamp_us, switch_high);
            }
            Ready => {
                if !switch_high {
                    self.stage = Idle;
                }
            }
            Timeout => {
                if !switch_high {
                    self.stage = Idle; // Reset to Off when going low
                }
                // If still high, remain in Timeout state
            }
        }

        self.stage == Ready
    }
}

#[no_mangle] pub extern "C" fn two_tap_init(two_tap: *mut TwoTapSwitch)
{
    unsafe {
        *two_tap = TwoTapSwitch::new();
    }
}

#[inline]
#[no_mangle] pub extern "C" fn two_tap_apply(
    two_tap: *mut TwoTapSwitch,
    timestamp: u32,
    switch_high: bool,
) -> bool
{
    unsafe {
        (*two_tap).process(timestamp, switch_high)
    }
}

#[cfg(test)]
mod two_tap_switch_tests {
    use super::*;
    const SWITCH_HIGH: bool = true;
    const SWITCH_LOW: bool = false;
    const INITIAL_TIMESTAMP_US: u32 = 100_000;

    #[test]
    fn switch_initial_state() {
        // given
        let switch = TwoTapSwitch::new();

        // then
        assert_eq!(switch.stage, Idle);
    }

    #[test]
    fn switch_first_stage_activation() {
        // given
        let mut switch = TwoTapSwitch::new();
        let timestamp = INITIAL_TIMESTAMP_US;

        // when
        let result = switch.process(timestamp, SWITCH_HIGH);

        // then
        assert!(!result);
        assert_eq!(switch.stage, FirstStage);
    }

    #[test]
    fn switch_second_stage_activation() {
        // given
        let mut switch = TwoTapSwitch::new();
        let timestamp = INITIAL_TIMESTAMP_US;

        // Activate first stage
        switch.process(timestamp, SWITCH_HIGH);

        // when
        let result = switch.process(timestamp + 100_000, SWITCH_LOW); // Switch low to move to second stage

        // then
        assert!(!result); // Not ready yet
        assert_eq!(switch.stage, SecondStage);
    }

    #[test]
    fn switch_ready_state_activation() {
        // given
        let mut switch = TwoTapSwitch::new();
        let timestamp = INITIAL_TIMESTAMP_US;

        // Activate first and second stages
        switch.process(timestamp, SWITCH_HIGH); // First stage
        switch.process(timestamp + 100_000, SWITCH_LOW); // Second stage
        let result = switch.process(timestamp + 200_000, SWITCH_HIGH); // Move to ready stage

        // then
        assert!(result); // Now ready
        assert_eq!(switch.stage, Ready);
    }

    #[test]
    fn switch_timeout_resets() {
        // given
        let mut switch = TwoTapSwitch::new();
        let timestamp = INITIAL_TIMESTAMP_US;

        // Activate first stage
        switch.process(timestamp, SWITCH_HIGH);

        // Simulate timeout
        let result = switch.process(timestamp + 600_000, SWITCH_LOW); // 600ms later, should reset to Off

        // then
        assert!(!result); // Not ready yet
        assert_eq!(switch.stage, Idle);
    }

    #[test]
    fn switch_ready_reset() {
        // given
        let mut switch = TwoTapSwitch::new();
        let timestamp = INITIAL_TIMESTAMP_US;

        // Activate to Ready state
        switch.process(timestamp, SWITCH_HIGH); // First stage
        switch.process(timestamp + 100_000, SWITCH_LOW); // Second stage
        switch.process(timestamp + 200_000, SWITCH_HIGH); // Ready state

        // when
        let result = switch.process(timestamp + 300_000, SWITCH_LOW); // Switch low

        // then
        assert!(!result); // Not ready anymore
        assert_eq!(switch.stage, Idle);
    }

    #[test]
    fn switch_first_stage_timeout() {
        // given
        let mut switch = TwoTapSwitch::new();
        let timestamp = INITIAL_TIMESTAMP_US;

        // Enter FirstStage
        switch.process(timestamp, SWITCH_HIGH);

        // when
        let result = switch.process(timestamp + 600_000, SWITCH_HIGH); // Remain high and timeout

        // then
        assert!(!result); // Not ready
        assert_eq!(switch.stage, Timeout);
    }

    #[test]
    fn switch_second_stage_timeout() {
        // given
        let mut switch = TwoTapSwitch::new();
        let timestamp = INITIAL_TIMESTAMP_US;

        // Enter FirstStage
        switch.process(timestamp, SWITCH_HIGH);
        // enter second stage
        switch.process(timestamp + 100_000, SWITCH_LOW);

        // when
        let result = switch.process(timestamp + 600_000, SWITCH_LOW);

        // then
        assert!(!result); // Not ready
        assert_eq!(switch.stage, Idle);
    }

    #[test]
    fn switch_timeout_reset() {
        // given
        let mut switch = TwoTapSwitch::new();
        let timestamp = INITIAL_TIMESTAMP_US;

        // Enter FirstStage
        switch.process(timestamp, SWITCH_HIGH);

        // Simulate timeout
        switch.process(timestamp + 600_000, SWITCH_HIGH); // Transition to Timeout

        // when
        let result = switch.process(timestamp + 700_000, SWITCH_LOW);

        // then
        assert!(!result); // Not ready
        assert_eq!(switch.stage, Idle);
    }
}
