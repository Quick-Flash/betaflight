use core::cmp;
use core::f32::consts::PI;
use microfft::real::rfft_64;
use crate::math::circular_buffer::CircularBuffer;
use crate::math::trig::Trig;
use crate::filter::biquad::SecondOrderHighpassFilter;

/// NUM_SAMPLES must match the size you have set up with microfft
pub const NUM_SAMPLES: usize = 64;
pub const NUM_BINS: usize = NUM_SAMPLES / 2;
pub const AXIS_COUNT: usize = 3;

pub struct Sdft {
    // constants
    hanning: [f32; NUM_SAMPLES], // hanning constants to window the samples
    pub starting_bin: usize,     // bins before this are ignored
    pub ending_bin: usize,       // bins after this are ignored
    down_samples: usize,         // how many loops we run before we add a new sample
    pub bin_width: f32,          // how wide each bin is in hz

    // mutable members
    samples: [CircularBuffer<NUM_SAMPLES>; AXIS_COUNT],
    loop_counter: usize,         // downsample counter
    highpass_filter: [SecondOrderHighpassFilter; AXIS_COUNT],
}

/// max_hz must be lower than looprate / 2
impl Sdft {
    pub fn new(looprate: f32, min_hz: u16, max_hz: u16) -> Self {
        let mut down_samples = (looprate / (2 * max_hz) as f32) as usize;
        let mut sdft_looprate = looprate / down_samples as f32;
        let mut bin_width = sdft_looprate / (2.0 * NUM_BINS as f32);

        // this ensures that we can reliably measure to max_hz
        if (sdft_looprate / 2.0) - (2.0 * bin_width) < max_hz as f32 {
            down_samples = cmp::max(1, down_samples - 1);
            sdft_looprate = looprate / down_samples as f32;
            bin_width = sdft_looprate / (2.0 * NUM_BINS as f32);
        }

        let starting_bin = cmp::max(3, (min_hz as f32 / bin_width + 0.5) as usize) - 1;
        let ending_bin = cmp::min(NUM_BINS - 2, (max_hz as f32 / bin_width + 0.5) as usize) + 1;

        let c = 2.0 * PI / (NUM_SAMPLES as f32 - 1.0);
        let mut hanning = [0.0; NUM_SAMPLES];
        for (i, hanning) in hanning.iter_mut().enumerate() {
            *hanning = 0.5 - (i as f32 * c).cosf() * 0.5;
        }

        Self {
            hanning,
            starting_bin,
            ending_bin,
            down_samples,
            bin_width,
            samples: [CircularBuffer::<NUM_SAMPLES>::new(NUM_SAMPLES); AXIS_COUNT],
            loop_counter: 0,
            highpass_filter: [SecondOrderHighpassFilter::new(0.7, 20.0, 1.0 / looprate); AXIS_COUNT],
        }
    }

    pub fn reset(&mut self) {
        for axis in 0..AXIS_COUNT {
            self.samples[axis].reset();
            self.highpass_filter[axis].reset();
            self.loop_counter = 0;
        }
    }

    pub fn update_bins(&mut self, input: [f32; AXIS_COUNT]) {
        self.loop_counter += 1;

        let mut filtered_input = [0.0; AXIS_COUNT];
        for axis in 0..AXIS_COUNT {
            filtered_input[axis] = self.highpass_filter[axis].apply(input[axis]);
        }

        if self.loop_counter >= self.down_samples {
            for axis in 0..AXIS_COUNT {
                self.samples[axis].append(filtered_input[axis]);
            }
            self.loop_counter = 0;
        }
    }

    pub fn bin_magnitude_squared(&mut self, axis: usize) -> [f32; NUM_BINS] {
        // If this is true then the unsafe code will be safe
        debug_assert!(self.ending_bin < NUM_BINS);
        debug_assert!(axis < AXIS_COUNT);

        let mut hanned_samples = unsafe { self.samples.get_unchecked(axis).multiply_by_array(&self.hanning) };

        let fft = rfft_64(&mut hanned_samples);
        let mut amplitude = [0.0; NUM_BINS];
        for i in 0..NUM_BINS {
            amplitude[i] = fft[i].norm_sqr(); // TODO see if we do need to take the square root of this, I'm guessing no :)
        }

        amplitude
    }
}

#[cfg(test)]
mod sdft_tests {
    use super::*;
    use std::println;
    use approx::assert_abs_diff_eq;

    #[test]
    fn two_hundred_hz_signal() {
        // given
        let frequency = 8000.0;
        let mut sdft = Sdft::new(frequency, 80, 650);
        let amplitude: f32 = 10.0;

        // when
        println!("start_bin: {}, end_bin: {}", sdft.starting_bin, sdft.ending_bin);

        let signal_time_divider = frequency / 200.0;
        let signal_time_divider2 = frequency / 10.0;
        for i in 0..900 {
            let signal = ((i as f32 / signal_time_divider) * 2.0 * PI).sinf() * amplitude + 200.0 * ((i as f32 / signal_time_divider2) * 2.0 * PI).sinf();
            sdft.update_bins([signal, signal, signal]);
        }
        let magnitude_squared: [f32; NUM_BINS];
        magnitude_squared = sdft.bin_magnitude_squared(0);

        // then
        let mut max_amplitude: f32 = 0.0;
        let mut max_bin: usize = 0;
        for bin in sdft.starting_bin..sdft.ending_bin {
            println!("bin: {}, magnitude_squared: {}", bin, magnitude_squared[bin]);
            if magnitude_squared[bin] > max_amplitude {
                max_amplitude = magnitude_squared[bin];
                max_bin = bin;
            }
        }
        println!("max_bin: {}, max_amplitude: {}", max_bin, max_amplitude);
        println!("resolution_hz: {}", sdft.bin_width);
        let freq = max_bin as f32 * sdft.bin_width;
        println!("freq: {}", freq);

        assert_abs_diff_eq!(freq, 200.0, epsilon = 0.00015);
    }
}
