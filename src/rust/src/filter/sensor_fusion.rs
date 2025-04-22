use crate::filter::ptn::Pt1Filter;
use crate::math::abs::Absf;
use crate::c_interop::*;
use crate::c_interop::DebugType::DEBUG_DUAL_GYRO_DIFF;
// just support dual gyros for the moment
// takes the absolute value of the derivative and filters is
// this gives a rough measure of how much noise each gyro has
// thus when fusing we can do a better job of picking which gyro to use


#[repr(C)]
struct SensorNoiseEstimation {
    previous_sensor: [f32; 3],
    filter: [Pt1Filter; 3],
}

#[repr(C)]
pub struct SensorFusion {
    noise_estimate_1: SensorNoiseEstimation,
    noise_estimate_2: SensorNoiseEstimation,
}

impl SensorNoiseEstimation {
    fn new(noise_estimation_cutoff: f32, dt: f32) -> Self {
        Self {
            previous_sensor: [0.0; 3],
            filter: [Pt1Filter::new(noise_estimation_cutoff, dt); 3],
        }
    }

    #[inline(never)]
    fn estimate_noise(&mut self, sensor_input: [f32; 3]) -> [f32; 3] {
        let mut noise = [0.0; 3];

        for axis in 0..3 {
            let derivative = (sensor_input[axis] - self.previous_sensor[axis]).absf();
            noise[axis] = self.filter[axis].apply(derivative);
        }

        self.previous_sensor = sensor_input;

        noise
    }
}

impl SensorFusion {
    pub fn new(noise_estimation_cutoff: f32, dt: f32) -> Self {
        Self {
            noise_estimate_1: SensorNoiseEstimation::new(noise_estimation_cutoff, dt),
            noise_estimate_2: SensorNoiseEstimation::new(noise_estimation_cutoff, dt),
        }
    }

    pub fn fuse_sensors(&mut self, sensor1: [f32; 3], sensor2: [f32; 3]) -> [f32; 3] {
        let noise1 = self.noise_estimate_1.estimate_noise(sensor1);
        let noise2 = self.noise_estimate_2.estimate_noise(sensor2);

        let mut fused_output = [0.0; 3];
        for axis in 0..3 {
            let noise1_squared = noise1[axis] * noise1[axis];
            let noise2_squared = noise2[axis] * noise2[axis];
            let denominator = noise1_squared + noise2_squared;

            // Handle divide by 0 bugs
            fused_output[axis] = if denominator != 0.0 {
                (noise2_squared * sensor1[axis] + noise1_squared * sensor2[axis]) / denominator
            } else {
                // Fallback: filter each evenly
                0.5 * (sensor1[axis] + sensor2[axis])
            };
        }

        fused_output
    }

    pub fn fuse_sensors_debug(&mut self, sensor1: [f32; 3], sensor2: [f32; 3]) -> [f32; 3] {
        let noise1 = self.noise_estimate_1.estimate_noise(sensor1);
        let noise2 = self.noise_estimate_2.estimate_noise(sensor2);

        Self::sensor_fusion_debug(sensor1, sensor2, noise1, noise2);

        let mut fused_output = [0.0; 3];
        for axis in 0..3 {
            let noise1_squared = noise1[axis] * noise1[axis];
            let noise2_squared = noise2[axis] * noise2[axis];
            let denominator = noise1_squared + noise2_squared;

            // Handle divide by 0 bugs
            fused_output[axis] = if denominator != 0.0 {
                (noise2_squared * sensor1[axis] + noise1_squared * sensor2[axis]) / denominator
            } else {
                // Fallback: filter each evenly
                0.5 * (sensor1[axis] + sensor2[axis])
            };
        }

        fused_output
    }

    #[inline(never)]
    pub fn sensor_fusion_debug(sensor1: [f32; 3], sensor2: [f32; 3], noise1: [f32; 3], noise2: [f32; 3]) {
        set_debug_float(DEBUG_DUAL_GYRO_DIFF, 0, sensor1[0]);
        set_debug_float(DEBUG_DUAL_GYRO_DIFF, 1, noise1[0] * 100.0);
        set_debug_float(DEBUG_DUAL_GYRO_DIFF, 2, sensor2[0]);
        set_debug_float(DEBUG_DUAL_GYRO_DIFF, 3, noise2[0] * 100.0);
        set_debug_float(DEBUG_DUAL_GYRO_DIFF, 4, sensor1[1]);
        set_debug_float(DEBUG_DUAL_GYRO_DIFF, 5, noise1[1] * 100.0);
        set_debug_float(DEBUG_DUAL_GYRO_DIFF, 6, sensor2[1]);
        set_debug_float(DEBUG_DUAL_GYRO_DIFF, 7, noise2[1] * 100.0);
    }
}

// C stuff
#[no_mangle] pub extern "C" fn sensor_fusion_new(fusion: *mut SensorFusion, noise_estimation_cutoff: f32, dt: f32) {
    unsafe {
        (*fusion) = SensorFusion::new(noise_estimation_cutoff, dt);
    }
}

#[link_section = ".tcm_code"]
#[no_mangle] pub extern "C" fn fuse_sensors_debug(fusion: *mut SensorFusion, gyro_output: *mut [f32; 3], sensor1: *const [f32; 3], sensor2: *const [f32; 3]) {
    unsafe {
        *gyro_output = (*fusion).fuse_sensors_debug(*sensor1, *sensor2);
    }
}

#[cfg(test)]
mod sensor_fusion_tests {
    use super::*;
    const LOOPRATE: f32 = 8000.0;
    const DT: f32 = 1.0 / LOOPRATE;

    #[test]
    fn sensor_noise_test_positive() {
        // given
        let mut noise = SensorNoiseEstimation::new(15.0, DT);
        let input = [
            [5.0, 5.0, 1.0],
            [10.0, 15.0, 1.0],
            [15.0, 30.0, 1.0],
            [20.0, 50.0, 1.0],
            [25.0, 75.0, 1.0],
            [30.0, 105.0, 1.0],
        ];

        // when
        let mut output = [[0.0; 3]; 6];
        for i in 0..6 {
            output[i] = noise.estimate_noise(input[i]);
        }

        // then
        assert_eq!(output, [
            [0.058577254, 0.058577254, 0.011715451],
            [0.11646825, 0.1750455, 0.011578199],
            [0.17368102, 0.3487265, 0.011442556],
            [0.23022352, 0.57895005, 0.0113085015],
            [0.2861036, 0.86505365, 0.011176017],
            [0.34132904, 1.2063828, 0.011045085]
        ]);
    }

    #[test]
    fn sensor_noise_test_negative() {
        // given
        let mut noise = SensorNoiseEstimation::new(15.0, DT);
        let input = [
            [-5.0, -5.0, -1.0],
            [-10.0, -15.0, -1.0],
            [-15.0, -30.0, -1.0],
            [-20.0, -50.0, -1.0],
            [-25.0, -75.0, -1.0],
            [-30.0, -105.0, -1.0],
        ];

        // when
        let mut output = [[0.0; 3]; 6];
        for i in 0..6 {
            output[i] = noise.estimate_noise(input[i]);
        }

        // then
        assert_eq!(output, [
            [0.058577254, 0.058577254, 0.011715451],
            [0.11646825, 0.1750455, 0.011578199],
            [0.17368102, 0.3487265, 0.011442556],
            [0.23022352, 0.57895005, 0.0113085015],
            [0.2861036, 0.86505365, 0.011176017],
            [0.34132904, 1.2063828, 0.011045085]
        ]);
    }

    #[test]
    fn sensor_noise_test_sign_switching() {
        // given
        let mut noise = SensorNoiseEstimation::new(15.0, DT);
        let input = [
            [-5.0, -5.0, -1.0],
            [0.0, 5.0, -1.0],
            [5.0, -10.0, -1.0],
            [0.0, 10.0, -1.0],
            [5.0, -15.0, -1.0],
            [0.0, 15.0, -1.0],
        ];

        // when
        let mut output = [[0.0; 3]; 6];
        for i in 0..6 {
            output[i] = noise.estimate_noise(input[i]);
        }

        // then
        assert_eq!(output, [
            [0.058577254, 0.058577254, 0.011715451],
            [0.11646825, 0.1750455, 0.011578199],
            [0.17368102, 0.3487265, 0.011442556],
            [0.23022352, 0.57895005, 0.0113085015],
            [0.2861036, 0.86505365, 0.011176017],
            [0.34132904, 1.2063828, 0.011045085]
        ]);
    }

    #[test]
    fn fuse_two_identical_sensors() {
        // given
        let mut fusion = SensorFusion::new(15.0, DT);
        let sensor1 = [
            [-5.0, -5.0, -1.0],
            [0.0, 5.0, -1.0],
            [5.0, -10.0, -1.0],
            [0.0, 10.0, -1.0],
            [5.0, -15.0, -1.0],
            [0.0, 15.0, -1.0],
        ];

        let sensor2 = [
            [-5.0, -5.0, -1.0],
            [0.0, 5.0, -1.0],
            [5.0, -10.0, -1.0],
            [0.0, 10.0, -1.0],
            [5.0, -15.0, -1.0],
            [0.0, 15.0, -1.0],
        ];

        // when
        let mut output = [[0.0; 3]; 6];
        for i in 0..6 {
            output[i] = fusion.fuse_sensors(sensor1[i], sensor2[i]);
        }

        // then
        assert_eq!(output, [
            [-5.0, -5.0, -1.0],
            [0.0, 5.0, -1.0],
            [5.0, -10.0, -1.0],
            [0.0, 10.0, -1.0],
            [5.0, -15.0, -1.0],
            [0.0, 15.000001, -1.0],
        ]);
    }

    #[test]
    fn fuse_two_sensors() {
        // given
        let mut fusion = SensorFusion::new(15.0, DT);
        let sensor1 = [
            [-5.0, -5.0, -1.0],
            [0.0, 5.0, -1.0],
            [5.0, -10.0, -1.0],
            [0.0, 10.0, -1.0],
            [5.0, -15.0, -1.0],
            [0.0, 15.0, -1.0],
        ];

        let sensor2 = [
            [-5.0, 5.0, -2.0],
            [0.0, 5.0, -2.0],
            [5.0, 10.0, -2.0],
            [0.0, 10.0, -2.0],
            [5.0, 15.0, -2.0],
            [0.0, 15.0, -2.0],
        ];

        // when
        let mut output = [[0.0; 3]; 6];
        for i in 0..6 {
            output[i] = fusion.fuse_sensors(sensor1[i], sensor2[i]);
        }

        // then
        assert_eq!(output, [
            [-5.0, 0.0, -1.2],
            [0.0, 5.0, -1.2],
            [5.0, 8.013986, -1.2],
            [0.0, 10.0, -1.1999999],
            [5.0, 13.863292, -1.2],
            [0.0, 15.000001, -1.1999999],
        ]);
    }
}