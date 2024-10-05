#![no_std]

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