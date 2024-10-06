#[inline(always)]
fn abs_f32(f: f32) -> f32 {
    unsafe { core::intrinsics::fabsf32(f) }
}

pub trait Absf<T> {
    fn absf(&self) -> T;
}

impl Absf<f32> for f32 {
    #[inline(always)]
    fn absf(&self) -> f32 {
        abs_f32(*self)
    }
}