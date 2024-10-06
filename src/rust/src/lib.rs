#![no_std]
#![no_main]
#![feature(core_intrinsics)]

pub mod auto_notch;
pub mod biquad;
pub mod ptn;
pub mod predictive;
pub mod sqrt;
pub mod trig;
pub mod abs;
pub mod circular_buffer;
pub mod powi;
pub mod constrain;

// dev profile: easier to debug panics; can put a breakpoint on `rust_begin_unwind`
#[cfg(all(debug_assertions, not(test)))]
use panic_halt as _;

// release profile: minimize the binary size of the application
#[cfg(all(not(debug_assertions), not(test)))]
use panic_abort as _;

pub struct FilterT;

#[no_mangle] pub extern "C" fn nullFilterApply(_filter: *mut FilterT, input: f32) -> f32 {
    input
}