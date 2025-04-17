use core::ops::Range;
use crate::c_interop::DebugType::DEBUG_CG_COMPENSATION;
use crate::c_interop::set_debug_float;
use crate::filter::ptn::Pt1Filter;
use crate::math::constrain::constrain;
use crate::math::range_scaler::RangeScalar;

const THRUST_LEARNING_LOWER_BOUND: f32 = 0.1;
const THRUST_LEARNING_UPPER_BOUND: f32 = 0.5;
const ROTATION_LEARNING_LOWER_BOUND: f32 = 50.0;
const ROTATION_LEARNING_UPPER_BOUND: f32 = 550.0;

#[repr(C)]
pub struct CGLearner {
    // percentage in the x and y direction that the cg is to the mixer center
    pub x: Pt1Filter,
    pub y: Pt1Filter,
    thrust_scalar: RangeScalar,
    rotation_scalar: RangeScalar,
}

impl CGLearner {
    pub fn new(starting_x: f32, starting_y: f32, learning_time: f32, dt: f32) -> Self {
        let mut k = 0.0;
        if learning_time != 0.0 {
            k = Pt1Filter::time_constant_k(learning_time, dt);
        }

        let mut x = Pt1Filter::new(learning_time, dt);
        let mut y = Pt1Filter::new(learning_time, dt);

        x.set(starting_x);
        y.set(starting_y);

        Self {
            x,
            y,
            thrust_scalar: RangeScalar::new(
                &Range { start: THRUST_LEARNING_LOWER_BOUND, end: THRUST_LEARNING_UPPER_BOUND },
                &Range { start: 0.0, end: k }
            ),
            rotation_scalar: RangeScalar::new(
                &Range { start: ROTATION_LEARNING_LOWER_BOUND, end: ROTATION_LEARNING_UPPER_BOUND },
                &Range { start: 1.0, end: 0.0 }
            ),
        }
    }

    pub fn update_learning_time(&mut self, learning_time: f32, dt: f32) {
        let mut k = 0.0;
        if learning_time != 0.0 {
            k = Pt1Filter::time_constant_k(learning_time, dt);
        }

        self.thrust_scalar = RangeScalar::new(
            &Range { start: THRUST_LEARNING_LOWER_BOUND, end: THRUST_LEARNING_UPPER_BOUND },
            &Range { start: 0.0, end: k }
        );
        self.rotation_scalar = RangeScalar::new(
            &Range { start: ROTATION_LEARNING_LOWER_BOUND, end: ROTATION_LEARNING_UPPER_BOUND },
            &Range { start: 1.0, end: 0.0 }
        );
    }

    pub fn learn_cg_offset(&mut self, steady_state_roll: f32, steady_state_pitch: f32, mixer_thrust: f32, rotation_mag: f32) -> f32 {
        if mixer_thrust > 0.01 { // prevent divide by 0, can't learn at very low thrust anyhow
            let x = self.x.state - steady_state_roll / mixer_thrust;
            let y = self.y.state + steady_state_pitch / mixer_thrust;

            set_debug_float(DEBUG_CG_COMPENSATION, 6, x * 1000.0);
            set_debug_float(DEBUG_CG_COMPENSATION, 7, y * 1000.0);

            let mut learning_k = self.thrust_scalar.apply(constrain(mixer_thrust, THRUST_LEARNING_LOWER_BOUND, THRUST_LEARNING_UPPER_BOUND));
            learning_k *= self.rotation_scalar.apply(constrain(rotation_mag, ROTATION_LEARNING_LOWER_BOUND, ROTATION_LEARNING_UPPER_BOUND));
            self.x.k = learning_k;
            self.y.k = learning_k;

            self.x.apply_constrained(x, 0.25);
            self.y.apply_constrained(y, 0.25);

            learning_k
        } else {
            0.0
        }
    }
}

#[cfg(test)]
mod mixer_tests {
    use super::*;

    const LOOPRATE: f32 = 8000.0;
    const DT: f32 = 1.0 / LOOPRATE;

    #[test]
    fn low_thrust() {
        // given
        let mut cg_learner = CGLearner::new(0.0, 0.0, 2.0, DT);

        // when
        let learning_k = cg_learner.learn_cg_offset(0.2, 0.2, 0.0, 0.0);

        // then
        assert_eq!(learning_k, 0.0);
        assert_eq!(cg_learner.x.state, 0.0);
        assert_eq!(cg_learner.y.state, 0.0);
    }

    #[test]
    fn quarter_thrust() {
        // given
        let mut cg_learner = CGLearner::new(0.0, 0.0, 2.0, DT);

        // when
        let learning_k = cg_learner.learn_cg_offset(0.2, 0.2, 0.25, 0.0);

        // then
        assert_eq!(learning_k, 2.3436034e-5);
        assert_eq!(cg_learner.x.state, 1.8748828e-5);
        assert_eq!(cg_learner.y.state, -1.8748828e-5);
    }

    #[test]
    fn half_thrust() {
        // given
        let mut cg_learner = CGLearner::new(0.0, 0.0, 2.0, DT);

        // when
        let learning_k = cg_learner.learn_cg_offset(0.2, 0.2, 0.5, 0.0);

        // then
        assert_eq!(learning_k, 6.2496096e-5);
        assert_eq!(cg_learner.x.state, 2.4998439e-5);
        assert_eq!(cg_learner.y.state, -2.4998439e-5);
    }

    #[test]
    fn low_rotation() {
        // given
        let mut cg_learner = CGLearner::new(0.0, 0.0, 2.0, DT);

        // when
        let learning_k = cg_learner.learn_cg_offset(0.2, 0.2, 0.5, 50.0);

        // then
        assert_eq!(learning_k, 6.2496096e-5);
        assert_eq!(cg_learner.x.state, 2.4998439e-5);
        assert_eq!(cg_learner.y.state, -2.4998439e-5);
    }

    #[test]
    fn mid_rotation() {
        // given
        let mut cg_learner = CGLearner::new(0.0, 0.0, 2.0, DT);

        // when
        let learning_k = cg_learner.learn_cg_offset(0.2, 0.2, 0.5, 350.0);

        // then
        assert_eq!(learning_k, 2.4998437e-5);
        assert_eq!(cg_learner.x.state, 9.999375e-6);
        assert_eq!(cg_learner.y.state, -9.999375e-6);
    }

    #[test]
    fn fast_rotation() {
        // given
        let mut cg_learner = CGLearner::new(0.0, 0.0, 2.0, DT);

        // when
        let learning_k = cg_learner.learn_cg_offset(0.2, 0.2, 0.5, 750.0);

        // then
        assert_eq!(learning_k, 0.0);
        assert_eq!(cg_learner.x.state, 0.0);
        assert_eq!(cg_learner.y.state, 0.0);
    }

    #[test]
    fn two_seconds_learning() {
        // given
        let mut cg_learner = CGLearner::new(0.0, 0.0, 2.0, DT);

        // when
        let mut initial_steady_roll = 0.1;
        let mut initial_steady_pitch = 0.1;
        let learning_k = cg_learner.learn_cg_offset(initial_steady_roll, initial_steady_pitch, 0.5, 100.0);
        for _ in 0..LOOPRATE as usize * 2 {
            initial_steady_roll -= initial_steady_roll * learning_k;
            initial_steady_pitch -= initial_steady_pitch * learning_k;
            cg_learner.learn_cg_offset(initial_steady_roll, initial_steady_pitch, 0.5, 100.0);
        }

        // then
        assert_eq!(learning_k, 5.624649e-5);
        assert_eq!(cg_learner.x.state, 0.118688084);
        assert_eq!(cg_learner.y.state, -0.118688084);
    }
}