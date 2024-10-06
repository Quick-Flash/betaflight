use crate::filter::auto_notch::{NUM_BINS, Sdft, AXIS_COUNT};
use crate::filter::ptn::Pt1Filter;
use crate::math::abs::Absf;

pub const NUM_PEAKS: usize = 4;

#[derive(Copy, Clone)]
pub struct Peak {
    pub frequency: f32,
    pub mag: f32,
    pub bin: Option<usize>,
}

impl Default for Peak {
    fn default() -> Self {
        Self {
            frequency: 300.0, // we don't want to initialize filters at 0hz
            mag: 0.0,
            bin: None,
        }
    }
}

impl Peak {
    fn reset(&mut self) {
        self.mag = 0.0;
        self.bin = None;
    }
}

pub struct SdftPeakTracking {
    pub sdft: Sdft,
    pub peaks: [[Peak; NUM_PEAKS]; AXIS_COUNT],
    bin_magnitudes: [[f32; NUM_BINS]; AXIS_COUNT],
    pt1_k: f32,
    step: usize,
}

impl SdftPeakTracking {
    pub fn new(frequency: f32, min_hz: u16, max_hz: u16) -> Self {
        let sdft = Sdft::new(frequency, min_hz, max_hz);

        Self {
            sdft,
            peaks: [[Peak::default(); NUM_PEAKS]; AXIS_COUNT],
            bin_magnitudes: [[0.0; NUM_BINS]; AXIS_COUNT],
            // set up the center hz filter to run at a potentially different rate than the sdft side of things
            pt1_k: Pt1Filter::pt1_k(10.0, 6.0 / frequency),
            step: 0,
        }
    }

    pub fn reset(&mut self) {
        self.sdft.reset();
        self.bin_magnitudes = [[0.0; NUM_BINS]; AXIS_COUNT];
        for axis in 0..AXIS_COUNT {
            for peak in self.peaks[axis].iter_mut() {
                peak.reset();
                peak.frequency = 300.0;
            }
        }
        self.step = 0;
    }

    fn peak_frequency_array(&self, axis: usize) -> ([f32; NUM_PEAKS], [f32; NUM_PEAKS]) {
        let mut peak_frequency = [0.0; NUM_PEAKS];
        let mut peak_magnitude = [0.0; NUM_PEAKS];

        for (i, peak) in self.peaks[axis].iter().enumerate() {
            peak_frequency[i] = peak.frequency;
            peak_magnitude[i] = peak.mag;
        }

        return (peak_frequency, peak_magnitude)
    }

    fn peak_estimate(y0: f32, y1: f32, y2: f32, max_bin: usize) -> f32 {
        let denominator: f32 = y0 + y1 + y2;
        if denominator != 0.0 {
            return max_bin as f32 + (y2 - y0) / denominator;
        } else {
            return max_bin as f32;
        }
    }

    pub fn find_peak_bins(&mut self, axis: usize) {
        let mut current_bin: usize = self.sdft.starting_bin + 1;

        for peak in self.peaks[axis].iter_mut() {
            peak.reset();
        }

        // reset the peaks
        // Safety: bounds checking always OK due to initialization
        unsafe {
            while current_bin < self.sdft.ending_bin {
                let current_bin_magnitude = *self.bin_magnitudes[axis].get_unchecked(current_bin);

                if (
                    current_bin_magnitude <= *self.bin_magnitudes[axis].get_unchecked(current_bin - 1))
                    || (current_bin_magnitude <= *self.bin_magnitudes[axis].get_unchecked(current_bin + 1)
                ) {
                    current_bin += 1;
                    continue
                }

                // sorts the peaks in order of amplitude
                for peak in 0..NUM_PEAKS {
                    if current_bin_magnitude <= self.peaks[axis].get_unchecked(peak).mag {
                        continue
                    }

                    for p in (peak + 1..NUM_PEAKS).rev() {
                        let previous_peak = self.peaks[axis].get_unchecked(p - 1);
                        // let previous_frequency = previous_peak.frequency;
                        let previous_mag = previous_peak.mag;
                        let previous_bin = previous_peak.bin;

                        let current_peak = self.peaks[axis].get_unchecked_mut(p);
                        // current_peak.frequency = previous_frequency;
                        current_peak.mag = previous_mag;
                        current_peak.bin = previous_bin;
                    }
                    let current_peak = self.peaks[axis].get_unchecked_mut(peak);
                    current_peak.mag = current_bin_magnitude;
                    current_peak.bin = Some(current_bin);
                    break
                }
                // peak found, next bin cannot be peak
                current_bin += 2;
            }
        }
    }


    pub fn estimate_and_sort_peaks(&mut self, axis: usize) {
        let mut peaks_detected = 0;
        let mut new_peaks = [0.0; NUM_PEAKS];

        for peak in self.peaks[axis].iter_mut() {
            // don't update the peak estimate if it isn't a peak
            if let Some(peak_bin) = peak.bin {
                // If this is true then the unsafe code will be safe
                debug_assert!(peak_bin >= 2);
                debug_assert!(peak_bin + 2 < NUM_BINS);
                debug_assert!(axis < AXIS_COUNT);

                let mut magnitude = [0.0; AXIS_COUNT];

                unsafe {
                    magnitude[0] = *self.bin_magnitudes.get_unchecked(axis).get_unchecked(peak_bin - 1);
                    magnitude[1] = *self.bin_magnitudes.get_unchecked(axis).get_unchecked(peak_bin);
                    magnitude[2] = *self.bin_magnitudes.get_unchecked(axis).get_unchecked(peak_bin + 1);
                }

                peak.mag = magnitude[0] + magnitude[1] + magnitude[2];

                let estimated_peak = SdftPeakTracking::peak_estimate(
                    magnitude[0],
                    magnitude[1],
                    magnitude[2],
                    peak_bin
                ) * self.sdft.bin_width;

                new_peaks[peaks_detected] = estimated_peak;
                peaks_detected += 1;
            } else {
                // the rest are not a peak and can be skipped
                break;
            }
        }

        let mut sorted_notches = [false; NUM_PEAKS];
        let mut sorted_peaks = [false; NUM_PEAKS];
        let mut peak_distance = [[0.0; NUM_PEAKS]; NUM_PEAKS];

        let mut min_distance_away = f32::MAX;
        let mut nn_result = None;

        for peak in 0..peaks_detected {
            if let Some(_) = self.peaks[axis][peak].bin {
                let peak_frequency = new_peaks[peak];
                for notch in 0..NUM_PEAKS {
                    let notch_frequency = self.peaks[axis][notch].frequency;
                    let distance_away = (notch_frequency - peak_frequency).absf();
                    peak_distance[peak][notch] = distance_away;
                    if distance_away < min_distance_away {
                        min_distance_away = distance_away;
                        nn_result = Some((peak_frequency, notch, peak));
                    }
                }
            }
        }

        if let Some((prefiltered_peak, closest_notch_num, closest_peak_num)) = nn_result {
            sorted_peaks[closest_peak_num] = true;
            sorted_notches[closest_notch_num] = true;
            self.peaks[axis][closest_peak_num].frequency += self.pt1_k * (prefiltered_peak - self.peaks[axis][closest_notch_num].frequency);
        }

        for _ in 0..peaks_detected {
            min_distance_away = f32::MAX;
            nn_result = None;

            for peak in 0..peaks_detected {
                if sorted_peaks[peak] == false {
                    for notch in 0..NUM_PEAKS {
                        if sorted_notches[notch] == false {
                            let distance = peak_distance[peak][notch];
                            if distance < min_distance_away {
                                min_distance_away = distance;

                                nn_result = Some((new_peaks[peak], notch, peak));
                            }
                        }
                    }
                }
            }

            if let Some((prefiltered_peak, closest_notch_num, closest_peak_num)) = nn_result {
                sorted_peaks[closest_peak_num] = true;
                sorted_notches[closest_notch_num] = true;
                self.peaks[axis][closest_peak_num].frequency += self.pt1_k * (prefiltered_peak - self.peaks[axis][closest_notch_num].frequency);
            }
        }
    }

    pub fn update(&mut self, input: [f32; AXIS_COUNT]) -> Option<(([f32; NUM_PEAKS], [f32; NUM_PEAKS]), usize)> {
        self.sdft.update_bins(input);

        let mut notch_cutoff = None;

        match self.step {
            0 => {
                self.bin_magnitudes[0] = self.sdft.bin_magnitude_squared(0);
                self.find_peak_bins(1);
                // axis 2 do nothing

                self.step = 1;
            }
            1 => {
                self.find_peak_bins(0);
                // axis 1 do nothing
                self.estimate_and_sort_peaks(2);

                self.step = 2;
            }
            2 => {
                // axis 0 do nothing
                self.estimate_and_sort_peaks(1);
                notch_cutoff = Some((self.peak_frequency_array(2), 2));

                self.step = 3;
            }
            3 => {
                self.estimate_and_sort_peaks(0);
                notch_cutoff = Some((self.peak_frequency_array(1), 1));
                // axis 2 do nothing

                self.step = 4;
            }
            4 => {
                notch_cutoff = Some((self.peak_frequency_array(0), 0));
                // axis 1 do nothing
                self.bin_magnitudes[2] = self.sdft.bin_magnitude_squared(2);

                self.step = 5;
            }
            5 => {
                // axis 0 do nothing
                self.bin_magnitudes[1] = self.sdft.bin_magnitude_squared(1);
                self.find_peak_bins(2);

                self.step = 0;
            }
            _ => {}
        }

        notch_cutoff
    }
}
