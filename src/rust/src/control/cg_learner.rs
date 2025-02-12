use core::ops::Range;
use crate::c_interop::DebugType::DEBUG_CG_COMPENSATION;
use crate::c_interop::set_debug_float;
use crate::filter::ptn::Pt1Filter;
use crate::math::constrain::constrain;
use crate::math::range_scaler::RangeScalar;

#[repr(C)]
pub struct CGLearner {
    // percentage in the x and y direction that the cg is to the mixer center
    pub x: Pt1Filter,
    pub y: Pt1Filter,
    learning_time: RangeScalar,
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
            learning_time: RangeScalar::new(
                &Range { start: 0.05, end: 0.5 },
                &Range { start: 0.0, end: k }
            ),
        }
    }

    pub fn update_learning_time(&mut self, learning_time: f32, dt: f32) {
        let mut k = 0.0;
        if learning_time != 0.0 {
            k = Pt1Filter::time_constant_k(learning_time, dt);
        }

        self.learning_time = RangeScalar::new(
            &Range { start: 0.05, end: 0.5 },
            &Range { start: 0.0, end: k }
        );
    }

    pub fn learn_cg_offset(&mut self, steady_state_roll: f32, steady_state_pitch: f32, mixer_thrust: f32) -> f32 {
        if mixer_thrust > 0.01 { // prevent divide by 0, can't learn at very low thrust anyhow
            let x = self.x.state - steady_state_roll / mixer_thrust;
            let y = self.y.state + steady_state_pitch / mixer_thrust;

            set_debug_float(DEBUG_CG_COMPENSATION, 6, x * 1000.0);
            set_debug_float(DEBUG_CG_COMPENSATION, 7, y * 1000.0);

            let learning_k = self.learning_time.apply(constrain(mixer_thrust, 0.05, 0.5));
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
        let learning_k = cg_learner.learn_cg_offset(0.2, 0.2, 0.0);

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
        let learning_k = cg_learner.learn_cg_offset(0.2, 0.2, 0.25);

        // then
        assert_eq!(learning_k, 2.7776045e-5);
        assert_eq!(cg_learner.x.state, -2.2220836e-5);
        assert_eq!(cg_learner.y.state, 2.2220836e-5);
    }

    #[test]
    fn half_thrust() {
        // given
        let mut cg_learner = CGLearner::new(0.0, 0.0, 2.0, DT);

        // when
        let learning_k = cg_learner.learn_cg_offset(0.2, 0.2, 0.5);

        // then
        assert_eq!(learning_k, 6.2496096e-5);
        assert_eq!(cg_learner.x.state, -2.4998439e-5);
        assert_eq!(cg_learner.y.state, 2.4998439e-5);
    }

    #[test]
    fn two_seconds_learning() {
        // given
        let mut cg_learner = CGLearner::new(0.0, 0.0, 2.0, DT);

        // when
        let mut initial_steady_roll = 0.1;
        let mut initial_steady_pitch = 0.1;
        let learning_k = cg_learner.learn_cg_offset(initial_steady_roll, initial_steady_pitch, 0.5);
        for i in 0..LOOPRATE as usize * 2 {
            initial_steady_roll -= initial_steady_roll * learning_k;
            initial_steady_pitch -= initial_steady_pitch * learning_k;
            cg_learner.learn_cg_offset(initial_steady_roll, initial_steady_pitch, 0.5);
        }

        // then
        assert_eq!(learning_k, 6.2496096e-5);
        assert_eq!(cg_learner.x.state, -0.1264264);
        assert_eq!(cg_learner.y.state, 0.1264264);
    }
}