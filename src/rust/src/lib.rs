#![cfg_attr(not(any(test, feature = "testing")), no_std)]
#![cfg_attr(not(test), no_main)]
#![feature(core_intrinsics)]

pub mod filter;
pub mod math;
pub mod c_interop;
pub mod two_tap_switch;

// dev profile: easier to debug panics; can put a breakpoint on `rust_begin_unwind`
#[cfg(all(debug_assertions, not(any(test, feature = "testing"))))]
use panic_halt as _;

// release profile: minimize the binary size of the application
#[cfg(all(not(debug_assertions), not(test)))]
use panic_abort as _;

pub struct FilterT;

#[no_mangle] pub extern "C" fn nullFilterApply(_filter: *mut FilterT, input: f32) -> f32 {
    input
}