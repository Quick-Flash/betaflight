use core::ops::Range;
use crate::c_interop::DebugType::DEBUG_COLLISION_DETECTION;
use crate::c_interop::set_debug_float;
use crate::math::constrain::constrain;
use crate::math::range_scaler::RangeScalar;
use crate::math::sqrt::Sqrtf;

#[repr(C)]
pub struct CollisionDetection {
    // settings
    jerk_range_scaler: RangeScalar,

    // runtime
    previous_accel: [f32; 3],
    pub crash_motor_delta: f32,
    // may need an additional filter here
}

impl CollisionDetection {
    pub fn new(start_jerk: f32, end_jerk: f32) -> Self {
        Self {
            jerk_range_scaler: RangeScalar::new(&Range { start: start_jerk, end: end_jerk }, &Range { start: 1.0, end: 0.0 }),
            previous_accel: [0.0; 3],
            crash_motor_delta: 1.0,
        }
    }

    fn mag_accel_derivative(&mut self, accel: [f32; 3], looprate: f32) -> f32 {
        let mut derivative_squared = 0.0;
        for (previous, current) in self.previous_accel.iter_mut().zip(accel) {
            let derivative = *previous - current;
            derivative_squared += derivative * derivative;
            *previous = current;
        }
        derivative_squared.sqrtf() * looprate
    }

    fn update_motor_delta(&mut self, jerk: f32) {
        self.crash_motor_delta = constrain(self.jerk_range_scaler.apply(jerk), 0.0, 1.0);
    }

    pub fn run(&mut self, accel: [f32; 3], looprate: f32) {
        let jerk = self.mag_accel_derivative(accel, looprate);
        self.update_motor_delta(jerk);

        set_debug_float(DEBUG_COLLISION_DETECTION, 0, jerk);
        set_debug_float(DEBUG_COLLISION_DETECTION, 1, self.crash_motor_delta * 1000.0);
    }
}

#[no_mangle] pub extern "C" fn collision_detection_init(collision_detection: &mut CollisionDetection, jerk_start: f32, jerk_end: f32) {
    *collision_detection = CollisionDetection::new(jerk_start, jerk_end);
}

#[no_mangle] pub extern "C" fn run_collision_detection(collision_detection: &mut CollisionDetection, accel: *mut [f32; 3], looprate: f32) {
    unsafe {
        (*collision_detection).run(*accel, looprate)
    }
}
