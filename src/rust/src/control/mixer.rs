use core::ops::Range;
use crate::control::cg_learner::CGLearner;
use crate::filter::ptn::Pt1Filter;
use crate::math::constrain::constrain;
use crate::math::range_scaler::RangeScalar;
use crate::math::sqrt::Sqrtf;
use crate::math::sign::Sign;

pub const NUM_MOTORS: usize = 4;

#[repr(C)]
pub struct Mixer {
    /// settings
    pub thrust_linear_scalar: RangeScalar,
    pub filter_k_scaler: RangeScalar,

    /// runtime
    pub motor_gains: MotorGains,
    pub throttle_linearization_filter: Pt1Filter,
    pub linearization_filter: [Pt1Filter; NUM_MOTORS],
    pub motor_filter: [Pt1Filter; NUM_MOTORS],
    pub cg_learner: CGLearner,
}

impl Mixer {
    pub fn new(
        motor_gains: MotorGains,
        thrust_linear_low: f32,
        thrust_linear_high: f32,
        linearization_cut: f32,
        motor_cut_low: f32,
        motor_cut_high: f32,
        cg_learning_time: f32,
        dt: f32
    ) -> Self {
        let unit_range = Range {start: 0.0, end: 1.0};
        let mut k_low = 1.0;
        let mut k_high = 1.0;

        if motor_cut_low > 5.0 {
            k_low = Pt1Filter::pt1_k(motor_cut_low, dt);
        }

        if motor_cut_high > 5.0 {
            k_high = Pt1Filter::pt1_k(motor_cut_high, dt);
        }

        let mut linearization_pt1 = Pt1Filter::new(linearization_cut, dt);
        if linearization_cut < 5.0 {
            linearization_pt1.k = 1.0;
        }

        // todo make this work for roll offsets as well, just more matrix math...
        // for now one motor is enough to determine this if we only have a pitch offset
        let y_offset = (1.0 - motor_gains.gains[0].throttle) / motor_gains.gains[0].pitch.sign();

        Self {
            motor_gains,
            thrust_linear_scalar: RangeScalar::new(&unit_range, &Range {start: thrust_linear_low, end: thrust_linear_high}),
            filter_k_scaler: RangeScalar::new(&unit_range, &Range {start: k_low, end: k_high}),
            throttle_linearization_filter: linearization_pt1,
            linearization_filter: [linearization_pt1; NUM_MOTORS],
            motor_filter: [Pt1Filter::new(motor_cut_low, dt); NUM_MOTORS],
            cg_learner: CGLearner::new(0.0, y_offset, cg_learning_time, dt),
        }
    }

    pub fn update_non_motor_gains(
        &mut self,
        thrust_linear_low: f32,
        thrust_linear_high: f32,
        linearization_cut: f32,
        motor_cut_low: f32,
        motor_cut_high: f32,
        cg_learning_time: f32,
        dt: f32
    ) {
        let unit_range = Range {start: 0.0, end: 1.0};
        let mut k_low = 1.0;
        let mut k_high = 1.0;

        if motor_cut_low > 5.0 {
            k_low = Pt1Filter::pt1_k(motor_cut_low, dt);
        }

        if motor_cut_high > 5.0 {
            k_high = Pt1Filter::pt1_k(motor_cut_high, dt);
        }

        let mut linearization_pt1 = Pt1Filter::new(linearization_cut, dt);
        if linearization_cut < 5.0 {
            linearization_pt1.k = 1.0;
        }

        self.thrust_linear_scalar = RangeScalar::new(&unit_range, &Range {start: thrust_linear_low, end: thrust_linear_high});
        self.filter_k_scaler = RangeScalar::new(&unit_range, &Range {start: k_low, end: k_high});
        self.throttle_linearization_filter = linearization_pt1;
        self.linearization_filter = [linearization_pt1; NUM_MOTORS];
        self.motor_filter = [Pt1Filter::new(motor_cut_low, dt); NUM_MOTORS];
        self.cg_learner.update_learning_time(cg_learning_time, dt);
    }

    /// converts normalized thrust values to a normalized motor output
    fn thrust_to_motor(thrust: f32, scalar: &RangeScalar) -> f32 {
        let compensation = scalar.apply(thrust);
        if compensation != 0.0 && thrust > 0.0 {
            (compensation - 1.0 + ((1.0 - compensation) * (1.0 - compensation) + 4.0 * compensation * thrust).sqrtf()) / (2.0 * compensation)
        } else {
            thrust
        }
    }

    /// converts normalized motor numbers to a normalized thrust
    fn motor_to_thrust(motor: f32, scalar: &RangeScalar) -> f32 {
        let compensation = scalar.apply(motor);
        if compensation != 0.0 && motor > 0.0 {
            (1.0 - compensation) * motor + compensation * motor * motor
        } else {
            motor
        }
    }

    fn motor_rpy_mix(&self, rate_controls: &RateControls) -> [f32; NUM_MOTORS] {
        let mut mix = [0.0; NUM_MOTORS];
        for (i, gains) in self.motor_gains.gains.iter().enumerate() {
            mix[i] += rate_controls.pidsum_roll * gains.roll;
            mix[i] += rate_controls.pidsum_pitch * gains.pitch;
            mix[i] += rate_controls.pidsum_yaw * gains.yaw;
        }

        mix
    }

    fn mix_and_constrain(&mut self, rpy_mixed: [f32; NUM_MOTORS], throttle: f32, motor_delta_max: f32) -> ([f32; NUM_MOTORS], f32) {
        //
        // First pass of allowed throttle range
        // If scaling is needed this will be overwritten
        //

        let mut throttle_range = Range { start: f32::MIN, end: f32::MAX };

        for (gains, rpy_mix) in self.motor_gains.gains.iter().zip(rpy_mixed) {
            let throttle_mix_inv = 1.0 / gains.throttle;

            let min_needed_throttle  = -rpy_mix * throttle_mix_inv; // this motor will be 0.0 at this throttle
            let max_allowed_throttle  = (1.0 - rpy_mix) * throttle_mix_inv; // this motor will be 1.0 at this throttle

            if min_needed_throttle > throttle_range.start {
                throttle_range.start = min_needed_throttle;
            }

            if max_allowed_throttle < throttle_range.end {
                throttle_range.end = max_allowed_throttle;
            }
        }

        let mut motors = rpy_mixed;


        //
        // If the throttle_range.start is greater than the throttle_range.end scaling is needed
        // throttle_range.start * gains.throttle will be above 1.0
        // Find the max motor after adding the throttle_rang.start (first try minimum throttle)
        // Using this max motor value we calculate the motor scaling and minimum throttle
        // This minimum throttle will ensure that no motors are below 0.0 after scaling
        //

        // if true motors will need scaling and constraining
        let (motor_scale, min_throttle) = if throttle_range.start > throttle_range.end {
            // highest motor should be above 1.0, but this guards against floating point math issues allowing throttle_range.start to be greater than throttle_range.end when both are similar values
            let mut highest_motor = 1.0;

            for (motor, gains) in motors.iter().zip(self.motor_gains.gains.iter()) {
                // add throttle
                let above_zero_motors = motor + throttle_range.start * gains.throttle;

                // find max motor
                if above_zero_motors > highest_motor {
                    highest_motor = above_zero_motors;
                }
            }

            let max_motor_inv = 1.0 / highest_motor;
            let current_throttle = throttle_range.start * max_motor_inv;
            let motor_scale = motor_delta_max * max_motor_inv;

            (motor_scale, current_throttle)
        } else {
            (motor_delta_max, throttle_range.start)
        };

        //
        // Scale all motors and use this scale value for future calculations
        // Scaled motors will all be within a range of motor_delta_max
        // Find the maximum throttle to not put each motor above 1.0
        // The minimum maximum throttle is used for all motors
        //

        let mut highest_allowed_throttle = f32::MAX;
        for (motor, gains) in motors.iter_mut().zip(self.motor_gains.gains.iter()) {
            // scales motors to range of 1
            *motor *= motor_scale;

            let throttle_mix = gains.throttle;
            let max_allowed_throttle  = (1.0 - *motor) / throttle_mix; // this motor will be 1.0 at this throttle
            if max_allowed_throttle < highest_allowed_throttle {
                highest_allowed_throttle = max_allowed_throttle;
            }
        }

        //
        // Apply thrust linear compensation to throttle (convert from motor value to thrust value)
        // Constrain the desired throttle to a value that makes the motor range at most 0.0 to 1.0
        // Add the throttle to the motor
        //

        let throttle_to_thrust = Self::motor_to_thrust(throttle, &self.thrust_linear_scalar);
        let thrust_change = throttle_to_thrust - throttle;
        let filtered_change = self.throttle_linearization_filter.apply(thrust_change);
        let throttle_thrust = throttle + filtered_change;
        let constrained_throttle = constrain(
            throttle_thrust,
            min_throttle * motor_delta_max, highest_allowed_throttle
        );

        for (motor, gains) in motors.iter_mut().zip(self.motor_gains.gains.iter()) {
            *motor += constrained_throttle * gains.throttle;
        }

        //
        // Apply thrust linear compensation to motors (convert from thrust to motor value)
        // Filter the change that this thrust linear compensation creates
        //

        for (motor, mut filter) in motors.iter_mut().zip(self.linearization_filter) {
            let thrust = *motor;
            // convert from a thrust mixer view to a motor mixer view
            let thrust_linearized = Self::thrust_to_motor(thrust, &self.thrust_linear_scalar);
            let linearization_change = thrust_linearized - thrust;
            let filtered_change = filter.apply(linearization_change);
            *motor = thrust + filtered_change;
        }

        (motors, constrained_throttle)
    }

    pub fn mix_motors(&mut self, rate_controls: &RateControls, throttle: f32, motor_delta_max: f32) -> ([f32; NUM_MOTORS], f32) {
        let rpy_mixed = self.motor_rpy_mix(rate_controls);
        let (mut motors, thrust) = self.mix_and_constrain(rpy_mixed, throttle, motor_delta_max);

        for (motor, motor_filter) in motors.iter_mut().zip(self.motor_filter.iter_mut()) {
            motor_filter.k = self.filter_k_scaler.apply(*motor);
            *motor = motor_filter.apply(*motor);

            // ensure all motors are in a 0.0 to 1.0 range (floating point errors can make this false)
            // *motor = constrain(*motor, 0.0, 1.0);
        }

        (motors, thrust)
    }

    // only works under the assumption that all motors produce the same thrust total
    pub fn update_cg_compensation(&mut self, steady_state_roll: f32, steady_state_pitch: f32, thrust: f32) -> f32 {
        let learning_k = self.cg_learner.learn_cg_offset(steady_state_roll, steady_state_pitch, thrust);
        let x_cg_offset = self.cg_learner.x.state;
        let y_cg_offset = self.cg_learner.y.state;

        for gains in self.motor_gains.gains.iter_mut() {
            let mut throttle = 1.0 + x_cg_offset * gains.roll.sign();
            throttle -= y_cg_offset * gains.pitch.sign();
            gains.throttle = throttle;
        }

        // if you have poor CG your thrust will begin to clip at the top
        // TODO optionally allow not letting it clip your rc throttle

        learning_k
    }
}

#[no_mangle] pub extern "C" fn mixer_init(
    mixer: *mut Mixer,
    gains: MotorGains,
    thrust_linear_low: f32,
    thrust_linear_high: f32,
    linearization_cut: f32,
    motor_cut_low: f32,
    motor_cut_high: f32,
    cg_learning_time: f32,
    dt: f32
) {
    unsafe {
        *mixer = Mixer::new(gains, thrust_linear_low, thrust_linear_high, linearization_cut, motor_cut_low, motor_cut_high, cg_learning_time, dt);
    }
}

#[no_mangle] pub extern "C" fn mixer_update_non_motor_gains(
    mixer: *mut Mixer,
    thrust_linear_low: f32,
    thrust_linear_high: f32,
    linearization_cut: f32,
    motor_cut_low: f32,
    motor_cut_high: f32,
    cg_learning_time: f32,
    dt: f32
) {
    unsafe {
        (*mixer).update_non_motor_gains(thrust_linear_low, thrust_linear_high, linearization_cut, motor_cut_low, motor_cut_high, cg_learning_time, dt);
    }
}

#[link_section = ".tcm_code"]
#[inline]
#[no_mangle] pub extern "C" fn mix_motors(mixer: *mut Mixer, motors: *mut [f32; NUM_MOTORS], rate_controls: &RateControls, throttle: f32, motor_delta_max: f32) -> f32 {
    unsafe {
        let thrust;
        (*motors, thrust) = (*mixer).mix_motors(rate_controls, throttle, motor_delta_max);
        thrust
    }
}

#[link_section = ".tcm_code"]
#[inline]
#[no_mangle] pub extern "C" fn update_cg_compensation(mixer: *mut Mixer, steady_state_roll: f32, steady_state_pitch: f32, thrust: f32,) -> f32 {
    unsafe {
        (*mixer).update_cg_compensation(steady_state_roll, steady_state_pitch, thrust)
    }
}

#[repr(C)]
pub struct MotorModel {
    // relative coordinates from the center of mass (don't worry if it's not perfect, just do your best)
    // coordinates follow a top-down view
    pub x: f32,
    pub y: f32,

    // torque relative to its thrust
    pub torque_ratio: f32,

    // relative thrust compared to other motors
    pub thrust: f32,
}

impl MotorModel {
    pub fn new(x: f32, y: f32, torque_ratio: f32, thrust: f32) -> Self {
        Self {
            x,
            y,
            torque_ratio,
            thrust,
        }
    }
}

#[cfg_attr(test, derive(PartialEq, Debug))]
#[repr(C)]
pub struct MotorGains {
    gains: [RPYTGain; NUM_MOTORS],
}

impl MotorGains {
    const fn empty() -> Self {
        Self {
            gains: [RPYTGain { roll: 0.0, pitch: 0.0, yaw: 0.0, throttle: 0.0 }; NUM_MOTORS],
        }
    }

    pub const fn new(gains: [RPYTGain; NUM_MOTORS]) -> Self {
        Self {
            gains
        }
    }

    pub const fn new_with_model(model: &[MotorModel; NUM_MOTORS]) -> Self {
        let mut motor_gains = Self::empty();
        motor_gains.update_model(model);

        motor_gains
    }

    pub const fn update_model(&mut self, model: &[MotorModel; NUM_MOTORS]) {
        // only works for 4 motors at the moment, need to get a better inverse function
        let m0 = &model[0];
        let m1 = &model[1];
        let m2 = &model[2];
        let m3 = &model[3];

        // x and y are reversed as they create torque in directions opposite to their sign
        let m = [
            -m0.x * m0.thrust, -m0.y * m0.thrust, m0.torque_ratio * m0.thrust, m0.thrust,
            -m1.x * m1.thrust, -m1.y * m1.thrust, m1.torque_ratio * m0.thrust, m1.thrust,
            -m2.x * m2.thrust, -m2.y * m2.thrust, m2.torque_ratio * m0.thrust, m2.thrust,
            -m3.x * m3.thrust, -m3.y * m3.thrust, m3.torque_ratio * m0.thrust, m3.thrust,
        ];

        let mut inverse = [[0.0; 4];4];
        inverse[0][0] =
            m[5]  * m[10] * m[15] -
            m[5]  * m[11] * m[14] -
            m[9]  * m[6]  * m[15] +
            m[9]  * m[7]  * m[14] +
            m[13] * m[6]  * m[11] -
            m[13] * m[7]  * m[10];

        inverse[1][0] = 0.0 -
            m[4]  * m[10] * m[15] +
            m[4]  * m[11] * m[14] +
            m[8]  * m[6]  * m[15] -
            m[8]  * m[7]  * m[14] -
            m[12] * m[6]  * m[11] +
            m[12] * m[7]  * m[10];

        inverse[2][0] =
            m[4]  * m[9] * m[15] -
            m[4]  * m[11] * m[13] -
            m[8]  * m[5] * m[15] +
            m[8]  * m[7] * m[13] +
            m[12] * m[5] * m[11] -
            m[12] * m[7] * m[9];

        inverse[3][0] = 0.0 -
            m[4]  * m[9] * m[14] +
            m[4]  * m[10] * m[13] +
            m[8]  * m[5] * m[14] -
            m[8]  * m[6] * m[13] -
            m[12] * m[5] * m[10] +
            m[12] * m[6] * m[9];

        inverse[0][1] = 0.0 -
            m[1]  * m[10] * m[15] +
            m[1]  * m[11] * m[14] +
            m[9]  * m[2] * m[15] -
            m[9]  * m[3] * m[14] -
            m[13] * m[2] * m[11] +
            m[13] * m[3] * m[10];

        inverse[1][1] =
            m[0]  * m[10] * m[15] -
            m[0]  * m[11] * m[14] -
            m[8]  * m[2] * m[15] +
            m[8]  * m[3] * m[14] +
            m[12] * m[2] * m[11] -
            m[12] * m[3] * m[10];

        inverse[2][1] = 0.0 -
            m[0]  * m[9] * m[15] +
            m[0]  * m[11] * m[13] +
            m[8]  * m[1] * m[15] -
            m[8]  * m[3] * m[13] -
            m[12] * m[1] * m[11] +
            m[12] * m[3] * m[9];

        inverse[3][1] =
            m[0]  * m[9] * m[14] -
            m[0]  * m[10] * m[13] -
            m[8]  * m[1] * m[14] +
            m[8]  * m[2] * m[13] +
            m[12] * m[1] * m[10] -
            m[12] * m[2] * m[9];

        inverse[0][2] =
            m[1]  * m[6] * m[15] -
            m[1]  * m[7] * m[14] -
            m[5]  * m[2] * m[15] +
            m[5]  * m[3] * m[14] +
            m[13] * m[2] * m[7] -
            m[13] * m[3] * m[6];

        inverse[1][2] = 0.0 -
            m[0]  * m[6] * m[15] +
            m[0]  * m[7] * m[14] +
            m[4]  * m[2] * m[15] -
            m[4]  * m[3] * m[14] -
            m[12] * m[2] * m[7] +
            m[12] * m[3] * m[6];

        inverse[2][2] =
            m[0]  * m[5] * m[15] -
            m[0]  * m[7] * m[13] -
            m[4]  * m[1] * m[15] +
            m[4]  * m[3] * m[13] +
            m[12] * m[1] * m[7] -
            m[12] * m[3] * m[5];

        inverse[3][2] = 0.0 -
            m[0]  * m[5] * m[14] +
            m[0]  * m[6] * m[13] +
            m[4]  * m[1] * m[14] -
            m[4]  * m[2] * m[13] -
            m[12] * m[1] * m[6] +
            m[12] * m[2] * m[5];

        inverse[0][3] = 0.0 -
            m[1] * m[6] * m[11] +
            m[1] * m[7] * m[10] +
            m[5] * m[2] * m[11] -
            m[5] * m[3] * m[10] -
            m[9] * m[2] * m[7] +
            m[9] * m[3] * m[6];

        inverse[1][3] =
            m[0] * m[6] * m[11] -
            m[0] * m[7] * m[10] -
            m[4] * m[2] * m[11] +
            m[4] * m[3] * m[10] +
            m[8] * m[2] * m[7] -
            m[8] * m[3] * m[6];

        inverse[2][3] = 0.0 -
            m[0] * m[5] * m[11] +
            m[0] * m[7] * m[9] +
            m[4] * m[1] * m[11] -
            m[4] * m[3] * m[9] -
            m[8] * m[1] * m[7] +
            m[8] * m[3] * m[5];

        inverse[3][3] =
            m[0] * m[5] * m[10] -
            m[0] * m[6] * m[9] -
            m[4] * m[1] * m[10] +
            m[4] * m[2] * m[9] +
            m[8] * m[1] * m[6] -
            m[8] * m[2] * m[5];

        // multiply det by 4 (since we have 4 motors)
        let det = m[0] * inverse[0][0] + m[1] * inverse[1][0] + m[2] * inverse[2][0] + m[3] * inverse[3][0];

        let recip_for_det_for_4_motors = 4.0 / det;

        inverse[0][0] *= recip_for_det_for_4_motors;
        inverse[0][1] *= recip_for_det_for_4_motors;
        inverse[0][2] *= recip_for_det_for_4_motors;
        inverse[0][3] *= recip_for_det_for_4_motors;
        inverse[1][0] *= recip_for_det_for_4_motors;
        inverse[1][1] *= recip_for_det_for_4_motors;
        inverse[1][2] *= recip_for_det_for_4_motors;
        inverse[1][3] *= recip_for_det_for_4_motors;
        inverse[2][0] *= recip_for_det_for_4_motors;
        inverse[2][1] *= recip_for_det_for_4_motors;
        inverse[2][2] *= recip_for_det_for_4_motors;
        inverse[2][3] *= recip_for_det_for_4_motors;
        inverse[3][0] *= recip_for_det_for_4_motors;
        inverse[3][1] *= recip_for_det_for_4_motors;
        inverse[3][2] *= recip_for_det_for_4_motors;
        inverse[3][3] *= recip_for_det_for_4_motors;

        self.gains = [
            RPYTGain {
                roll: inverse[0][0],
                pitch: inverse[1][0],
                yaw: inverse[2][0],
                throttle: inverse[3][0]
            },
            RPYTGain {
                roll: inverse[0][1],
                pitch: inverse[1][1],
                yaw: inverse[2][1],
                throttle: inverse[3][1]
            },
            RPYTGain {
                roll: inverse[0][2],
                pitch: inverse[1][2],
                yaw: inverse[2][2],
                throttle: inverse[3][2]
            },
            RPYTGain {
                roll: inverse[0][3],
                pitch: inverse[1][3],
                yaw: inverse[2][3],
                throttle: inverse[3][3]
            },
        ];
    }
}

#[no_mangle] pub extern "C" fn motor_gains_init(gains: *mut MotorGains, model: &[MotorModel; NUM_MOTORS]) {
    unsafe {
        *gains = MotorGains::new_with_model(model);
    }
}

#[cfg_attr(test, derive(PartialEq, Debug))]
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RPYTGain {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub throttle: f32,
}

impl RPYTGain {
    pub fn new(
        roll: f32,
        pitch: f32,
        yaw: f32,
        throttle: f32,
    ) -> Self {
        Self {
            roll,
            pitch,
            yaw,
            throttle,
        }
    }
}

#[cfg_attr(test, derive(PartialEq, Debug))]
#[repr(C)]
pub struct RateControls {
    pub pidsum_roll: f32,
    pub pidsum_pitch: f32,
    pub pidsum_yaw: f32
}

impl RateControls {
    pub fn new(
        pidsum_roll: f32,
        pidsum_pitch: f32,
        pidsum_yaw: f32,
    ) -> Self {
        Self {
            pidsum_roll,
            pidsum_pitch,
            pidsum_yaw,
        }
    }
}

#[cfg(test)]
mod mixer_tests {
    use super::*;

    const X_QUAD_GAINS: MotorGains = MotorGains::new([
        RPYTGain { throttle: 1.0, roll: -1.0, pitch:  1.0, yaw: -1.0 }, // BR
        RPYTGain { throttle: 1.0, roll: -1.0, pitch: -1.0, yaw:  1.0 }, // FR
        RPYTGain { throttle: 1.0, roll:  1.0, pitch:  1.0, yaw:  1.0 }, // BL
        RPYTGain { throttle: 1.0, roll:  1.0, pitch: -1.0, yaw: -1.0 }, // FL
    ]);

    const X_QUAD_MODELED_GAINS: MotorGains = MotorGains::new_with_model(&[
        MotorModel { x:  1.0, y: -1.0, torque_ratio: -1.0, thrust: 1.0 }, // BR
        MotorModel { x:  1.0, y:  1.0, torque_ratio:  1.0, thrust: 1.0 }, // FR
        MotorModel { x: -1.0, y: -1.0, torque_ratio:  1.0, thrust: 1.0 }, // BL
        MotorModel { x: -1.0, y:  1.0, torque_ratio: -1.0, thrust: 1.0 }, // FL
    ]);

    const STRETCH_X_QUAD_MODELED_GAINS: MotorGains = MotorGains::new_with_model(&[
        MotorModel { x:  0.875, y: -1.0, torque_ratio: -1.0, thrust: 1.0 }, // BR
        MotorModel { x:  0.875, y:  1.0, torque_ratio:  1.0, thrust: 1.0 }, // FR
        MotorModel { x: -0.875, y: -1.0, torque_ratio:  1.0, thrust: 1.0 }, // BL
        MotorModel { x: -0.875, y:  1.0, torque_ratio: -1.0, thrust: 1.0 }, // FL
    ]);

    const CG_OFF_STRETCH_X_QUAD_MODELED_GAINS: MotorGains = MotorGains::new_with_model(&[
        MotorModel { x:  0.875, y: -1.0, torque_ratio: -1.0, thrust: 1.0 }, // BR
        MotorModel { x:  0.875, y:  0.9, torque_ratio:  1.0, thrust: 1.0 }, // FR
        MotorModel { x: -0.875, y: -1.0, torque_ratio:  1.0, thrust: 1.0 }, // BL
        MotorModel { x: -0.875, y:  0.9, torque_ratio: -1.0, thrust: 1.0 }, // FL
    ]);

    const LOOPRATE: f32 = 8000.0;
    const DT: f32 = 1.0 / LOOPRATE;

    #[test]
    fn roll_test() {
        // given
        let mut mixer = Mixer::new(X_QUAD_GAINS, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, DT);
        let roll_control = RateControls {
            pidsum_roll: 0.5,
            pidsum_pitch: 0.0,
            pidsum_yaw: 0.0,
        };

        let expected_motors = [0.0, 0.0, 1.0, 1.0];

        // then
        let (motors, thrust) = mixer.mix_motors(&roll_control, 0.0, 1.0);


        // then
        assert_eq!(motors, expected_motors);
    }

    #[test]
    fn pitch_test() {
        // given
        let mut mixer = Mixer::new(X_QUAD_GAINS, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, DT);
        let pitch_control = RateControls {
            pidsum_roll: 0.0,
            pidsum_pitch: 0.5,
            pidsum_yaw: 0.0,
        };

        let expected_motors = [1.0, 0.0, 1.0, 0.0];

        // then
        let (motors, thrust) = mixer.mix_motors(&pitch_control, 0.0, 1.0);


        // then
        assert_eq!(motors, expected_motors);
    }

    #[test]
    fn yaw_test() {
        // given
        let mut mixer = Mixer::new(X_QUAD_GAINS, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, DT);
        let yaw_control = RateControls {
            pidsum_roll: 0.0,
            pidsum_pitch: 0.0,
            pidsum_yaw: 0.5,
        };

        let expected_motors = [0.0, 1.0, 1.0, 0.0];

        // then
        let (motors, thrust) = mixer.mix_motors(&yaw_control, 0.0, 1.0);


        // then
        assert_eq!(motors, expected_motors);
    }

    #[test]
    fn rpy_test() {
        // given
        let mut mixer = Mixer::new(X_QUAD_GAINS, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, DT);
        let yaw_control = RateControls {
            pidsum_roll: 1.0,
            pidsum_pitch: 1.0,
            pidsum_yaw: 1.0,
        };

        let expected_motors = [0.0, 0.0, 1.0, 0.0];

        // then
        let (motors, thrust) = mixer.mix_motors(&yaw_control, 0.0, 1.0);


        // then
        assert_eq!(motors, expected_motors);
    }

    #[test]
    fn rpy_mixed_test() {
        // given
        let mut mixer = Mixer::new(X_QUAD_GAINS, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, DT);
        let yaw_control = RateControls {
            pidsum_roll: -0.05,
            pidsum_pitch: 0.05,
            pidsum_yaw: -0.05,
        };

        let expected_motors = [0.65, 0.45, 0.45, 0.45];

        // then
        let (motors, thrust) = mixer.mix_motors(&yaw_control, 0.5, 1.0);


        // then
        assert_eq!(motors, expected_motors);
    }

    #[test]
    fn rpy_mixed_thrust_linear_test() {
        // given
        let mut mixer = Mixer::new(X_QUAD_GAINS, 0.6, 0.3, 0.0, 0.0, 0.0, 0.0, DT);
        let yaw_control = RateControls {
            pidsum_roll: -0.05,
            pidsum_pitch: 0.05,
            pidsum_yaw: -0.05,
        };

        let expected_motors = [0.63874197, 0.4614461, 0.4614461, 0.4614461];

        // then
        let (motors, thrust) = mixer.mix_motors(&yaw_control, 0.5, 1.0);


        // then
        assert_eq!(motors, expected_motors);
    }

    #[test]
    fn rpy_mixed_thrust_linear_filtered_test() {
        // given
        let mut mixer = Mixer::new(X_QUAD_GAINS, 0.6, 0.3, 25.0, 500.0, 1000.0, 0.0, DT);
        let yaw_control = RateControls {
            pidsum_roll: -0.05,
            pidsum_pitch: 0.05,
            pidsum_yaw: -0.05,
        };

        let expected_motors = [0.29524347, 0.18620808, 0.18620808, 0.18620808];

        // then
        let (motors, thrust) = mixer.mix_motors(&yaw_control, 0.5, 1.0);


        // then
        assert_eq!(motors, expected_motors);
    }

    #[test]
    fn model_test() {
        // given
        let modeled_motor_gains = X_QUAD_MODELED_GAINS;
        let expected_motor_gains = X_QUAD_GAINS;

        // expect
        assert_eq!(modeled_motor_gains, expected_motor_gains);
    }

    #[test]
    fn model_stretch_test() {
        // given
        let modeled_motor_gains = STRETCH_X_QUAD_MODELED_GAINS;
        let expected_motor_gains = MotorGains::new([
            RPYTGain { throttle: 1.0, roll: -1.1428572, pitch:  1.0, yaw: -1.0 }, // BR
            RPYTGain { throttle: 1.0, roll: -1.1428572, pitch: -1.0, yaw:  1.0 }, // FR
            RPYTGain { throttle: 1.0, roll:  1.1428572, pitch:  1.0, yaw:  1.0 }, // BL
            RPYTGain { throttle: 1.0, roll:  1.1428572, pitch: -1.0, yaw: -1.0 }, // FL
        ]);

        // expect
        assert_eq!(modeled_motor_gains, expected_motor_gains);
    }

    #[test]
    fn model_cg_off_stretch_test() {
        // given
        let modeled_motor_gains = CG_OFF_STRETCH_X_QUAD_MODELED_GAINS;
        let expected_motor_gains = MotorGains::new([
            RPYTGain { throttle: 0.94736844, roll: -1.1428571, pitch:  1.0526316, yaw: -1.0 }, // BR
            RPYTGain { throttle: 1.0526316, roll: -1.1428573, pitch: -1.0526316, yaw:  1.0 }, // FR
            RPYTGain { throttle: 0.9473684, roll:  1.1428573, pitch:  1.0526316, yaw:  1.0 }, // BL
            RPYTGain { throttle: 1.0526316, roll:  1.1428572, pitch: -1.0526316, yaw: -1.0 }, // FL
        ]);

        // expect
        assert_eq!(modeled_motor_gains, expected_motor_gains);
    }
}
