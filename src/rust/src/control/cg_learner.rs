use core::ops::Range;
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
        if learning_time == 0.0 {
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
        if learning_time == 0.0 {
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