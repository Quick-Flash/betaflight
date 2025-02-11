use crate::math::constrain::constrain;
use crate::filter::auto_notch::{NUM_PEAKS, AXIS_COUNT, SdftPeakTracking};
use crate::filter::biquad::{NotchFilter, PeakFilter};
use crate::filter::ptn::Pt1Filter;

pub struct AutoNotch {
    peak_tracking: SdftPeakTracking,
    // TODO add Peak filters
    // TODO reduce peak filters filtering when they are doing very little
    peak: [[PeakFilter; NUM_PEAKS]; AXIS_COUNT],
    peak_width: f32,
    peak_width_doubling_freq: f32,
    mag_filter: [[Pt1Filter; NUM_PEAKS]; AXIS_COUNT],
    full_filter_mag: f32,
    no_filter_mag: f32,
}

impl AutoNotch {
    pub fn new(frequency: f32, min_hz: u16, max_hz: u16, notch_width: f32, notch_width_doubling_freq: f32) -> Self {
        let dt = 1.0 / frequency;
        let mut mag_filter = [[Pt1Filter::new_time_constant(0.05, dt * 6.0); NUM_PEAKS]; AXIS_COUNT];
        for axis in 0..AXIS_COUNT {
            for peak in 0..NUM_PEAKS {
                mag_filter[axis][peak].set(0.5);
            }
        }

        Self {
            peak_tracking: SdftPeakTracking::new(frequency, min_hz, max_hz),
            peak: [[PeakFilter::new(1.0, 1.0, 300.0, dt); NUM_PEAKS]; AXIS_COUNT],
            peak_width: notch_width,
            peak_width_doubling_freq: notch_width_doubling_freq,
            mag_filter,
            full_filter_mag: 2500.0,
            no_filter_mag: 350.0,
        }
    }

    pub fn reset(&mut self) {
        self.peak_tracking.reset();
        for peak in 0..NUM_PEAKS {
            for axis in 0..AXIS_COUNT {
                self.peak[axis][peak].reset();
                self.mag_filter[axis][peak].set(0.5);
            }
        }
    }

    pub fn apply(&mut self, dt: f32, input: [f32; AXIS_COUNT]) -> [f32; AXIS_COUNT] {
        let notch_update = self.peak_tracking.update(input);

        if let Some(notch_values) = notch_update {
            let ((frequencies, magnitude), axis) = notch_values;
            self.update_notches(axis, frequencies, magnitude, dt);
        }

        let mut filtered_gyro = input;

        for axis in 0..AXIS_COUNT {
            for notch in 0..NUM_PEAKS {
                filtered_gyro[axis] = self.peak[axis][notch].apply(filtered_gyro[axis]);
            }
        }

        filtered_gyro
    }

    fn update_notches(&mut self, axis: usize, frequencies: [f32; NUM_PEAKS], magnitude: [f32; NUM_PEAKS], dt: f32) {
        for (notch_num, (cutoff, mag)) in frequencies.iter().zip(magnitude).enumerate() {
            let notch_width_multiplier = cutoff / self.peak_width_doubling_freq;
            let end_freq = cutoff + self.peak_width + self.peak_width * notch_width_multiplier;

            let mut peak_mag = constrain((-mag + self.no_filter_mag) / (self.full_filter_mag - self.no_filter_mag) + 1.0, 0.0, 1.0);
            peak_mag = self.mag_filter[axis][notch_num].apply(peak_mag);

            let notch_q = NotchFilter::q_from_center_and_end_freq(*cutoff, end_freq);
            self.peak[axis][notch_num].update_cutoff(peak_mag, notch_q, *cutoff, dt);
        }
    }
}

#[cfg(test)]
mod test {
    extern crate alloc;

    use super::*;
    use alloc::vec::Vec;
    use std::println;
    use core::f32::consts::PI;
    use std::any::Any;
    use approx::assert_abs_diff_eq;
    use crate::math::trig::Trig;
    use crate::filter::auto_notch::NUM_BINS;

    #[test]
    fn four_different_peaks() {
        // given
        let frequency = 8000.0;
        let dt = 1.0 / frequency;
        let notch_width= 15.0;
        let notch_doubling_freq = 300.0;
        let min_hz: u16 = 65;
        let max_hz: u16 = 600;

        let mut auto_notch = AutoNotch::new(frequency, min_hz, max_hz, notch_width, notch_doubling_freq);

        println!("min_bin {}, max_bin {}", auto_notch.peak_tracking.sdft.starting_bin, auto_notch.peak_tracking.sdft.ending_bin);

        // and
        let amplitude1: f32 = 3.0;
        let amplitude2: f32 = 5.0;
        let amplitude3: f32 = 7.0;
        let amplitude4: f32 = 10.0;

        let freq1 = 65.0;
        let freq2 = 310.0;
        let freq3 = 405.0;
        let freq4 = 600.0;

        let signal_time_divider1 = frequency / freq1;
        let signal_time_divider2 = frequency / freq2;
        let signal_time_divider3 = frequency / freq3;
        let signal_time_divider4 = frequency / freq4;

        let mut signal1: f32;
        let mut signal2: f32;
        let mut signal3: f32;
        let mut signal4: f32;

        // and
        let signal_floor = 10.0;

        // and
        let mut results: Vec<[f32; AXIS_COUNT]> = Vec::new();

        // when
        let mut output;
        // 0.1875 seconds worth running
        for i in 0..4000 {
            signal1 = ((i as f32 / signal_time_divider1) * 2.0 * PI).sinf() * amplitude1;
            signal2 = (((i + 0) as f32 / signal_time_divider2) * 2.0 * PI).sinf() * amplitude2;
            signal3 = (((i + 0) as f32 / signal_time_divider3) * 2.0 * PI).sinf() * amplitude3;
            signal4 = (((i + 0) as f32 / signal_time_divider4) * 2.0 * PI).sinf() * amplitude4;

            let signal = signal_floor + signal1 + signal2 + signal3 + signal4;
            let gyro_signal = [signal; AXIS_COUNT];

            output = auto_notch.apply(dt, gyro_signal);

            let peaks = auto_notch.peak_tracking.peaks[0];
            if (i % 6) == 0 {
                for peak in peaks.iter() {
                    println!("peaks: {:?}", peak.frequency);
                }
                println!();
            }

            results.push(output);
        }

        let magnitude_squared = auto_notch.peak_tracking.sdft.bin_magnitude_squared(0);
        for i in 0..NUM_BINS {
            let frequency = auto_notch.peak_tracking.sdft.bin_width * i as f32;
            println!("bin: {}, freq: {}, value: {}", i, frequency, magnitude_squared[i]);
        }

        // then
        // see how accurate the filters have been after 0.15 seconds
        for result in results[3800..].iter() {
            // then
            assert_abs_diff_eq!(result[0], signal_floor, epsilon = 0.06);
            assert_abs_diff_eq!(result[1], signal_floor, epsilon = 0.06);
            assert_abs_diff_eq!(result[2], signal_floor, epsilon = 0.06);
        }
    }
}