pub struct Crossfade {
    weight: f32,
}

impl Crossfade {
    pub fn new(weight: f32) -> Self {
        Self {
            weight
        }
    }

    pub fn apply(&self, pre: f32, post: f32) -> f32 {
        pre * (1.0 - self.weight) + post * self.weight
    }
}

#[cfg(test)]
mod crossfade_tests {
    use super::*;

    #[test]
    fn crossfade_test_1() {
        // given
        let crossfade = Crossfade::new(0.5);

        // then
        assert_eq!(crossfade.apply(10.0, 5.0), 7.5);
    }

    #[test]
    fn crossfade_test_2() {
        // given
        let crossfade = Crossfade::new(0.25);

        // then
        assert_eq!(crossfade.apply(10.0, 5.0), 8.75);
    }

    #[test]
    fn crossfade_test_3() {
        // given
        let crossfade = Crossfade::new(0.75);

        // then
        assert_eq!(crossfade.apply(10.0, 5.0), 6.25);
    }
}