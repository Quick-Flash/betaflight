use crate::filter::ptn::Pt1Filter;
use crate::math::abs::Absf;

// just support dual gyros for the moment
// takes the absolute value of the derivative and filters is
// this gives a rough measure of how much noise each gyro has
// thus when fusing we can do a better job of picking which gyro to use


struct SensorNoiseEstimation {
    previous_sensor: [f32; 3],
    filter: [Pt1Filter; 3],
}

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

    fn estimate_noise(&mut self, sensor_input: [f32; 3], looprate: f32) -> [f32; 3] {
        let mut noise = [0.0; 3];

        for axis in 0..3 {
            let derivative = (sensor_input[axis] - self.previous_sensor[axis]).absf() * looprate;
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

    pub fn fuse_sensors(&mut self, sensor1: [f32; 3], sensor2: [f32; 3], looprate: f32) -> [f32; 3] {
        let noise1 = self.noise_estimate_1.estimate_noise(sensor1, looprate);
        let noise2 = self.noise_estimate_2.estimate_noise(sensor2, looprate);

        let mut fused_output = [0.0; 3];
        for axis in 0..3 {
            let noise1_squared = noise1[axis] * noise1[axis];
            let noise2_squared = noise2[axis] * noise2[axis];
            fused_output[axis] = (noise2_squared * sensor1[axis] + noise1_squared * sensor2[axis]) / (noise1_squared + noise2_squared);
        }

        fused_output
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
            output[i] = noise.estimate_noise(input[i], LOOPRATE);
        }

        // then
        assert_eq!(output, [
            [468.61804, 468.61804, 93.72361],
            [931.74603, 1400.364, 92.625595],
            [1389.4482, 2789.8123, 91.54044],
            [1841.7883, 4631.6006, 90.468],
            [2288.829, 6920.4297, 89.40813],
            [2730.6326, 9651.0625, 88.36067]
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
            output[i] = noise.estimate_noise(input[i], LOOPRATE);
        }

        // then
        assert_eq!(output, [
            [468.61804, 468.61804, 93.72361],
            [931.74603, 1400.364, 92.625595],
            [1389.4482, 2789.8123, 91.54044],
            [1841.7883, 4631.6006, 90.468],
            [2288.829, 6920.4297, 89.40813],
            [2730.6326, 9651.0625, 88.36067]
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
            output[i] = noise.estimate_noise(input[i], LOOPRATE);
        }

        // then
        assert_eq!(output, [
            [468.61804, 468.61804, 93.72361],
            [931.74603, 1400.364, 92.625595],
            [1389.4482, 2789.8123, 91.54044],
            [1841.7883, 4631.6006, 90.468],
            [2288.829, 6920.4297, 89.40813],
            [2730.6326, 9651.0625, 88.36067]
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
            output[i] = fusion.fuse_sensors(sensor1[i], sensor2[i], LOOPRATE);
        }

        // then
        assert_eq!(output, [
            [-5.0, -5.0, -1.0],
            [0.0, 5.0, -1.0],
            [5.0, -10.0, -1.0],
            [0.0, 10.0, -1.0],
            [5.0, -15.0, -1.0],
            [0.0, 15.0, -1.0],
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
            output[i] = fusion.fuse_sensors(sensor1[i], sensor2[i], LOOPRATE);
        }

        // then
        assert_eq!(output, [
            [-5.0, 0.0, -1.1999999],
            [0.0, 4.9999995, -1.2],
            [5.0, 8.013988, -1.2],
            [0.0, 10.0, -1.2],
            [5.0, 13.863292, -1.1999999],
            [0.0, 15.0, -1.1999999],
        ]);
    }
}