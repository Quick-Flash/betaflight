use core::f32::consts::PI;
use crate::math::sqrt::Sqrtf;
use crate::math::trig::Trig;

#[derive(Copy, Clone)]
#[repr(C)]
pub struct Pt1Filter {
    pub state: f32,
    pub k: f32,
}

impl Pt1Filter {
    /// Only use if you will be updating filter cutoff on the fly and need reduced cpu load
    pub fn new_approx(cutoff: f32, dt: f32) -> Self {
        let mut pt1 = Self { state: 0.0, k: 0.0 };
        pt1.update_approx(cutoff, dt);

        pt1
    }

    /// Only use if you will be updating filter cutoff on the fly and need reduced cpu load
    pub fn update_approx(&mut self, cutoff: f32, dt: f32) {
        self.k = dt / ((1.0 / (2.0 * PI * cutoff)) + dt);
    }

    /// Use if you are setting the filter time constant
    pub fn new_time_constant(time_seconds: f32, dt: f32) -> Self {
        Self {
            state: 0.0,
            k: Self::time_constant_k(time_seconds, dt),
        }
    }

    /// Use if you are updating the filter time constant
    pub fn update_time_constant(&mut self, time_seconds: f32, dt: f32) {
        self.k = Self::time_constant_k(time_seconds, dt);
    }

    /// cutoff can never be set higher than nyquist, aka looprate / 2.0
    pub fn new(cutoff: f32, dt: f32) -> Self {
        Self {
            state: 0.0,
            k: Self::pt1_k(cutoff, dt),
        }
    }

    /// cutoff can never be set higher than nyquist, aka looprate / 2.0
    pub fn update(&mut self, cutoff: f32, dt: f32) {
        self.k = Self::pt1_k(cutoff, dt);
    }

    pub fn pt1_k(cutoff: f32, dt: f32) -> f32 {
        let omega = 2.0 * PI * cutoff * dt;
        let cos = 1.0 - omega.cosf_unchecked();
        (cos * (cos + 2.0)).sqrtf() - cos
    }

    pub fn time_constant_k(time_seconds: f32, dt: f32) -> f32 {
        dt / (time_seconds + dt)
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        self.state += self.k * (input - self.state);

        self.state
    }

    pub fn apply_highpass(&mut self, input: f32) -> f32 {
        input - self.apply(input)
    }

    pub fn reset(&mut self) {
        self.state = 0.0;
    }

    pub fn set(&mut self, value: f32) {
        self.state = value;
    }

    pub fn last_output(& self) -> f32 {
        self.state
    }
}

// C stuff
#[link_section = ".tcm_code"]
#[no_mangle] pub extern "C" fn pt1FilterGain(f_cut: f32, dt: f32) -> f32
{
    Pt1Filter::pt1_k(f_cut, dt)
}

// Calculates filter gain based on delay (time constant of filter) - time it takes for filter response to reach 63.2% of a step input.
#[no_mangle] pub extern "C" fn pt1FilterGainFromDelay(delay: f32, dt: f32) -> f32
{
    Pt1Filter::time_constant_k(delay, dt)
}

#[no_mangle] pub extern "C" fn pt1FilterInit(filter: *mut Pt1Filter, k: f32)
{
    unsafe {
        (*filter).state = 0.0;
        (*filter).k = k;
    }
}

#[link_section = ".tcm_code"]
#[inline]
#[no_mangle] pub extern "C" fn pt1FilterUpdateCutoff(filter: *mut Pt1Filter, k: f32)
{
    unsafe {
    (*filter).k = k;
    }
}

#[link_section = ".tcm_code"]
#[inline]
#[no_mangle] pub extern "C" fn pt1FilterApply(filter: *mut Pt1Filter, input: f32) -> f32
{
    unsafe {
        (*filter).apply(input)
    }
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct Pt2Filter {
    state: f32,
    pt1: Pt1Filter,
}

impl Pt2Filter {
    /// order_scale = 1 / (sqrt(2^(1/Order)) -1))

    /// Only use if you will be updating filter cutoff on the fly and need reduced cpu load
    pub fn new_approx(cutoff: f32, dt: f32) -> Self {
        Self {
            state: 0.0,
            pt1: Pt1Filter::new_approx(cutoff * 1.553_774, dt),
        }
    }

    /// Only use if you  need reduced cpu load as it is less accurate
    pub fn update_approx(&mut self, cutoff: f32, dt: f32) {
        self.pt1.update_approx(cutoff * 1.553_774, dt);
    }

    /// Use if you are setting the filter time constant
    pub fn new_time_constant(time_seconds: f32, dt: f32) -> Self {
        Self {
            state: 0.0,
            pt1: Pt1Filter::new_time_constant(time_seconds, dt),
        }
    }

    /// Use if you are updating the filter time constant
    pub fn update_time_constant(&mut self, time_seconds: f32, dt: f32) {
        self.pt1.update_time_constant(time_seconds, dt);
    }

    /// cutoff can never be set higher than nyquist, aka looprate / 2.0
    pub fn new(cutoff: f32, dt: f32) -> Self {
        Self {
            state: 0.0,
            pt1: Pt1Filter::new(cutoff * 1.553_774, dt),
        }
    }

    pub fn update(&mut self, cutoff: f32, dt: f32) {
        self.pt1.update(cutoff * 1.553_774, dt);
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        self.state += self.pt1.k * (input - self.state);
        self.pt1.apply(self.state)
    }

    pub fn apply_highpass(&mut self, input: f32) -> f32 {
        input - self.apply(input)
    }

    pub fn reset(&mut self) {
        self.state = 0.0;
        self.pt1.reset();
    }

    pub fn set(&mut self, value: f32) {
        self.state = value;
        self.pt1.set(value);
    }

    pub fn k(&self) -> f32 {
        self.pt1.k
    }

    pub fn last_output(& self) -> f32 {
        self.pt1.last_output()
    }
}

// C stuff
#[link_section = ".tcm_code"]
#[no_mangle] pub extern "C" fn pt2FilterGain(f_cut: f32, dt: f32) -> f32
{
    Pt1Filter::pt1_k(f_cut * 1.553_774, dt)
}

#[no_mangle] pub extern "C" fn pt2FilterInit(filter: *mut Pt2Filter, k: f32)
{
    unsafe {
        (*filter).state = 0.0;
        (*filter).pt1.state = 0.0;
        (*filter).pt1.k = k;
    }
}

#[link_section = ".tcm_code"]
#[inline]
#[no_mangle] pub extern "C" fn pt2FilterUpdateCutoff(filter: *mut Pt2Filter, k: f32)
{
    unsafe {
        (*filter).pt1.k = k;
    }
}

#[link_section = ".tcm_code"]
#[inline]
#[no_mangle] pub extern "C" fn pt2FilterApply(filter: *mut Pt2Filter, input: f32) -> f32
{
    unsafe {
        (*filter).apply(input)
    }
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct Pt3Filter {
    state: f32,
    pt2: Pt2Filter,
}

impl Pt3Filter {
    /// order_scale = 1 / (sqrt(2^(1/Order)) -1))

    /// Only use if you will be updating filter cutoff on the fly and need reduced cpu load
    pub fn new_approx(cutoff: f32, dt: f32) -> Self {
        Self {
            state: 0.0,
            pt2: Pt2Filter {
                state: 0.0,
                pt1: Pt1Filter::new_approx(cutoff * 1.961_459_2, dt),
            },
        }
    }

    /// cutoff can never be set higher than nyquist, aka looprate / 2.0
    pub fn update_approx(&mut self, cutoff: f32, dt: f32) {
        self.pt2.pt1.update_approx(cutoff * 1.961_459_2, dt);
    }

    /// Use if you are setting the filter time constant
    pub fn new_time_constant(time_seconds: f32, dt: f32) -> Self {
        Self {
            state: 0.0,
            pt2: Pt2Filter::new_time_constant(time_seconds, dt),
        }
    }

    /// Use if you are updating the filter time constant
    pub fn update_time_constant(&mut self, time_seconds: f32, dt: f32) {
        self.pt2.update_time_constant(time_seconds, dt);
    }

    /// cutoff can never be set higher than nyquist, aka looprate / 2.0
    pub fn new(cutoff: f32, dt: f32) -> Self {
        Self {
            state: 0.0,
            pt2: Pt2Filter {
                state: 0.0,
                pt1: Pt1Filter::new(cutoff * 1.961_459_2, dt),
            },
        }
    }

    /// Only use if you  need reduced cpu load as it is less accurate
    pub fn update(&mut self, cutoff: f32, dt: f32) {
        self.pt2.pt1.update(cutoff * 1.961_459_2, dt);
    }

    pub fn apply(&mut self, input: f32) -> f32 {
        self.state += self.pt2.pt1.k * (input - self.state);
        self.pt2.apply(self.state)
    }

    pub fn apply_highpass(&mut self, input: f32) -> f32 {
        input - self.apply(input)
    }

    pub fn reset(&mut self) {
        self.state = 0.0;
        self.pt2.reset();
    }

    pub fn set(&mut self, value: f32) {
        self.state = value;
        self.pt2.set(value);
    }

    pub fn k(&self) -> f32 {
        self.pt2.k()
    }

    pub fn last_output(& self) -> f32 {
        self.pt2.last_output()
    }
}

// C stuff
#[link_section = ".tcm_code"]
#[no_mangle] pub extern "C" fn pt3FilterGain(f_cut: f32, dt: f32) -> f32
{
    Pt1Filter::pt1_k(f_cut * 1.961_459_2, dt)
}

#[no_mangle] pub extern "C" fn pt3FilterInit(filter: *mut Pt3Filter, k: f32)
{
    unsafe {
        (*filter).state = 0.0;
        (*filter).pt2.state = 0.0;
        (*filter).pt2.pt1.state = 0.0;
        (*filter).pt2.pt1.k = k;
    }
}

#[link_section = ".tcm_code"]
#[inline]
#[no_mangle] pub extern "C" fn pt3FilterUpdateCutoff(filter: *mut Pt3Filter, k: f32)
{
    unsafe {
        (*filter).pt2.pt1.k = k;
    }
}

#[link_section = ".tcm_code"]
#[inline]
#[no_mangle] pub extern "C" fn pt3FilterApply(filter: *mut Pt3Filter, input: f32) -> f32
{
    unsafe {
        (*filter).apply(input)
    }
}

#[cfg(test)]
mod ptn_tests {
    use super::*;

    const LOOPRATE: f32 = 8000.0;
    const DT: f32 = 1.0 / LOOPRATE;

    const CUTOFF: f32 = 100.0;
    const TIME_CONSTANT: f32 = 1.0;
    const INPUT: f32 = 1.0;

    #[cfg(test)]
    mod pt1_tests {
        use super::*;

        #[test]
        fn pt1_k() {
            // when
            let pt1 = Pt1Filter::new(CUTOFF, DT);

            // then
            assert_eq!(pt1.k, 0.07549777);
        }

        #[test]
        fn pt1_approx_k() {
            // when
            let pt1 = Pt1Filter::new_approx(CUTOFF, DT);

            // then
            assert_eq!(pt1.k, 0.072820514);
        }

        #[test]
        fn pt1_time_constant_k() {
            // when
            let pt1 = Pt1Filter::new_time_constant(TIME_CONSTANT, DT);

            // then
            assert_eq!(pt1.k, 0.00012498438);
        }

        #[test]
        fn pt1_apply() {
            // when
            let mut pt1 = Pt1Filter::new(CUTOFF, DT);

            // then
            assert_eq!(pt1.apply(INPUT), pt1.k);
        }

        #[test]
        fn pt1_apply_highpass() {
            // when
            let mut pt1 = Pt1Filter::new(CUTOFF, DT);

            // then
            assert_eq!(pt1.apply_highpass(INPUT), INPUT - pt1.k);
        }
    }

    #[cfg(test)]
    mod pt2_tests {
        use super::*;

        #[test]
        fn pt2_k() {
            // when
            let pt2 = Pt2Filter::new(CUTOFF, DT);

            // then
            assert_eq!(pt2.k(), 0.11474762);
        }

        #[test]
        fn pt2_approx_k() {
            // when
            let pt2 = Pt2Filter::new_approx(CUTOFF, DT);

            // then
            assert_eq!(pt2.k(), 0.10876072);
        }

        #[test]
        fn pt2_time_constant_k() {
            // when
            let pt2 = Pt2Filter::new_time_constant(TIME_CONSTANT, DT);

            // then
            assert_eq!(pt2.pt1.k, 0.00012498438);
        }

        #[test]
        fn pt2_apply() {
            // when
            let mut pt2 = Pt2Filter::new(CUTOFF, DT);

            // then
            assert_eq!(pt2.apply(INPUT), pt2.k().powf(2.0));
        }

        #[test]
        fn pt2_apply_highpass() {
            // when
            let mut pt2 = Pt2Filter::new(CUTOFF, DT);

            // then
            assert_eq!(pt2.apply_highpass(INPUT), INPUT - pt2.k().powf(2.0));
        }
    }

    #[cfg(test)]
    mod pt3_tests {
        use super::*;

        #[test]
        fn pt3_k() {
            // when
            let pt3 = Pt3Filter::new(CUTOFF, DT);

            // then
            assert_eq!(pt3.k(), 0.14251305);
        }

        #[test]
        fn pt3_approx_k() {
            // when
            let pt3 = Pt3Filter::new_approx(CUTOFF, DT);

            // then
            assert_eq!(pt3.k(), 0.1334884);
        }

        #[test]
        fn pt3_time_constant_k() {
            // when
            let pt3 = Pt3Filter::new_time_constant(TIME_CONSTANT, DT);

            // then
            assert_eq!(pt3.k(), 0.00012498438);
        }

        #[test]
        fn pt3_apply() {
            // when
            let mut pt3 = Pt3Filter::new(CUTOFF, DT);

            // then
            assert_eq!(pt3.apply(INPUT), pt3.k().powf(3.0));
        }

        #[test]
        fn pt3_apply_highpass() {
            // when
            let mut pt3 = Pt3Filter::new(CUTOFF, DT);

            // then
            assert_eq!(pt3.apply_highpass(INPUT), INPUT - pt3.k().powf(3.0));
        }
    }
}