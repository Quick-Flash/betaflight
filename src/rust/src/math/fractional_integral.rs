use libm::{sqrt, tgamma};

fn fractional_integral_y_equal_1(time_seconds: f64, order: f64, gamma_multiplier: f64) -> f64 {
    gamma_multiplier * time_seconds.powf(order)
}

fn gamma_multiplier(order: f64) -> f64 {
    1.0 / tgamma(order + 1.0)
}

fn geometric_sum_r(r: f64, n: usize) -> f64 {
    if r == 1.0 {
        n as f64
    } else {
        r * (1.0 - r.powi(n as i32)) / (1.0 - r)
    }
}

#[derive(Clone, Debug)]
pub struct FractionalIntegral {
    leaky_integral: Vec<f64>,
    previous_weighted: f64,
}
impl FractionalIntegral {
    pub fn new(iir_size: usize) -> Self {
        Self {
            leaky_integral: vec![0.0; iir_size],
            previous_weighted: 0.0,
        }
    }

    pub fn apply(
        &mut self,
        table: &FractionalIntegralTable,
        input: f64,
    ) -> f64 {
        let fir_integral = input * table.weighting[0];
        let cascaded_iir = self.leaky_integral_array(self.previous_weighted, &table.leak, &table.removal);
        self.previous_weighted = input * table.weighting[1];

        fir_integral + cascaded_iir
    }

    fn leaky_integral_array(&mut self, input: f64, leak: &[f64], removal: &[f64]) -> f64 {
        let size = leak.len();
        let mut next_input = input;
        let mut output = 0.0;
        for i in 0..size {
            if i < size - 1 {
                self.leaky_integral[i] += next_input;
                self.leaky_integral[i] *= leak[i];
                next_input = self.leaky_integral[i] * removal[i];
                self.leaky_integral[i] -= next_input;
            } else {
                self.leaky_integral[i] += next_input;
                self.leaky_integral[i] *= leak[i];
            }
            output += self.leaky_integral[i];
        }
        output
    }

    pub fn reset(&mut self) {
        self.leaky_integral = vec![0.0; self.leaky_integral.len()];
        self.previous_weighted = 0.0;
    }
}

#[derive(Clone, Debug)]
pub struct FractionalIntegralTable {
    pub leak: Vec<f64>,
    pub removal: Vec<f64>,

    // these are only used to make it easier to tell tables apart
    pub looprate: f64,
    pub order: f64,
    pub weighting: [f64; 2],
}

impl FractionalIntegralTable {
    ///
    /// fir_size is the FIR filter order
    /// iir_size is how many cascaded IIR filters are cascaded
    /// looprate is how many times a second you are going to run the integral at
    /// order is the fractional integral order, valid values are 0.0<1.0
    /// iir_sample_estimate is how many samples the first IIR filter will try to estimate over
    /// iir_growth is a multiplier of many samples each new IIR filter will try to estimate
    ///

    pub fn empty() -> Self {
        Self {
            leak: vec![0.0],
            removal: vec![0.0],
            looprate: 0.0,
            order: 0.0,
            weighting: [0.0; 2],
        }
    }

    pub fn new(
        iir_size: usize,
        looprate: f64,
        order: f64,
        iir_sample_estimate: usize,
        iir_growth: f64,
        magic_number: f64,
    ) -> Self {
        let mut tables = Self {
            leak: Vec::with_capacity(iir_size),
            removal: Vec::with_capacity(if iir_size > 0 { iir_size - 1 } else { 0 }),
            looprate,
            order,
            weighting: [0.0; 2],
        };
        let num_fir_weights = 2 + iir_size;

        let mut sample_count = 2;
        let mut current_sample_estimate = iir_sample_estimate;

        let mut iir_samples = Vec::with_capacity(iir_size);

        iir_samples.push(sample_count);

        for _ in 0..iir_size {
            sample_count += current_sample_estimate;
            iir_samples.push(sample_count);
            current_sample_estimate = (current_sample_estimate as f64 * iir_growth) as usize;
        }

        let fir_weights = Self::generate_fractional_fir_weights(
            looprate,
            order,
            num_fir_weights + 1,
            0,
            &iir_samples,
        );

        tables.weighting[0] = fir_weights[0];
        tables.weighting[1] = fir_weights[1];

        let mut current_sample_estimate = iir_sample_estimate;
        for i in 0..iir_size {
            let starting_weight = fir_weights[0 + i + 1];
            let end_weight = fir_weights[0 + i + 2];

            // this is how much the leaky integral needs to leak
            let decay_percent = end_weight / starting_weight;
            let mut leak = decay_percent.powf(1.0 / (current_sample_estimate as f64));
            // this line is pretty janky, but could probably lead to better results
            // look into changing this based on current_sample_estimate
            // perhaps have the code modify this to get the best variance
            // perhaps weight the variance more heavily at lower values
            // perhaps scale the variance as a percent difference
            leak -= (1.0 - leak) * magic_number;

            tables.leak.push(leak);

            if i < iir_size - 1 {
                let special = (1.0 / (1.0 - leak)) - 1.0;
                let leaky_total = special * (1.0 - leak.powi(current_sample_estimate as i32));

                // let mut leaky_total = 0.0;
                // for _ in 0..current_sample_estimate {
                //     leaky_total += 1.0;
                //     leaky_total *= leak;
                // }
                // this is how much we need to remove from the leaky integral and into the next integral
                // this represents what percentage of the input is left current_sample_estimate cycles
                tables.removal.push(decay_percent / leaky_total);
            }

            current_sample_estimate = (current_sample_estimate as f64 * iir_growth) as usize;
        }
        tables
    }

    // uses the fractional integrals of the function 'y = 1.0' to calculate the FIR weights
    fn generate_fractional_fir_weights(
        looprate: f64,
        order: f64,
        size: usize,
        fir_samples: usize,
        iir_samples: &[usize],
    ) -> Vec<f64> {
        let mut fir_weights = Vec::with_capacity(size);

        // gamma_multiplier = 1/gamma(order+1)
        // fir[n] = gamma_multiplier * (((n+1)*dt)^order - (n^order))

        // precalculate this to speed up calculations
        let gamma_multiplier = gamma_multiplier(order);

        for i in 0..fir_samples + 1 {
            fir_weights.push(fractional_integral_y_equal_1((i + 1) as f64 / looprate, order, gamma_multiplier)
                - fractional_integral_y_equal_1(i as f64 / looprate, order, gamma_multiplier));
        }
        for sample in iir_samples.iter() {
            fir_weights.push(fractional_integral_y_equal_1((*sample + 1) as f64 / looprate, order, gamma_multiplier)
                - fractional_integral_y_equal_1(*sample as f64 / looprate, order, gamma_multiplier));
        }

        fir_weights
    }
}

#[cfg(test)]
mod fractional_integral_tests {
    use super::*;

    #[test]
    pub fn first_10_samples() {
        let looprate = 8000.0;
        let order = 0.5;
        let gamma_mul = gamma_multiplier(order);
        let fractional_table = FractionalIntegralTable::new(15, looprate, order, 5, 10.0, 0.0);

        let mut integral = FractionalIntegral::new(15);

        for i in 1..10 {
            println!("{}, {}", fractional_integral_y_equal_1(i as f64 / looprate, order, gamma_mul), integral.apply(&fractional_table, 1.0));
        }
        // expect
        assert!(true);
    }
}
