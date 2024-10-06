#[derive(Copy, Clone)]
pub struct CircularBuffer<const LENGTH: usize> {
    pub tail_index: usize,
    pub values: [f32; LENGTH],
    pub size: usize,
}

impl<const LENGTH: usize> Default for CircularBuffer<LENGTH> {
    fn default() -> Self {
        Self {
            tail_index: 0,
            values: [0.0; LENGTH],
            size: LENGTH,
        }
    }
}

impl<const LENGTH: usize> CircularBuffer<LENGTH> {
    pub fn new(size: usize) -> Self {
        debug_assert!(size <= LENGTH);
        Self {
            tail_index: 0,
            values: [0.0; LENGTH],
            size
        }
    }

    pub fn append(&mut self, input: f32) -> f32 {
        unsafe {
            let oldest = *self.values.get_unchecked(self.tail_index);

            *self.values.get_unchecked_mut(self.tail_index) = input;
            self.tail_index += 1;

            if self.tail_index >= self.size {
                self.tail_index = 0;
            }
            oldest
        }
    }

    pub fn get_element(&self, element: usize) -> f32 {
        let mut index = self.tail_index as i32 - 1 - element as i32;
        if index < 0 {
            index = self.size as i32 + index;
        }
        unsafe {
            *self.values.get_unchecked(index as usize)
        }
    }

    pub fn get_all_elements(&self) -> [f32; LENGTH] {
        let mut output = [0.0; LENGTH];
        let mut temp_pointer = self.tail_index;

        for i in (0..self.size).rev() {
            unsafe {
                output[i] = *self.values.get_unchecked(temp_pointer);
            }
            temp_pointer += 1;

            if temp_pointer >= self.size {
                temp_pointer = 0;
            }
        }

        output
    }

    pub fn fir_filter(&self, fir_array: &[f32]) -> f32 {
        let mut output = 0.0;
        let mut temp_pointer = self.tail_index;

        for i in (0..self.size).rev() {
            unsafe {
                output += self.values.get_unchecked(temp_pointer) * fir_array.get_unchecked(i);
            }
            temp_pointer += 1;

            if temp_pointer >= self.size {
                temp_pointer = 0;
            }
        }
        output
    }

    pub fn multiply_by_array(&self, array: &[f32; LENGTH]) -> [f32; LENGTH] {
        let mut output = [0.0; LENGTH];
        let mut temp_pointer = self.tail_index;

        for i in (0..self.size).rev() {
            output[i] = self.values[temp_pointer] * array[i];
            temp_pointer += 1;
            if temp_pointer >= self.size {
                temp_pointer = 0;
            }
        }
        output
    }

    pub fn append_and_find_maximum(&mut self, input: f32) -> f32 {
        let oldest = self.append(input);
        let mut maximum = oldest.max(unsafe { *self.values.get_unchecked(0) });
        for i in 1..self.size {
            unsafe {
                maximum = maximum.max(*self.values.get_unchecked(i));
            }
        }
        maximum
    }

    pub fn append_and_find_minimum(&mut self, input: f32) -> f32 {
        let oldest = self.append(input);
        let mut minimum = oldest.min(unsafe { *self.values.get_unchecked(0) });
        for i in 1..self.size {
            unsafe {
                minimum = minimum.min(*self.values.get_unchecked(i));
            }
        }
        minimum
    }

    pub fn reset(&mut self) {
        self.tail_index = 0;
        self.values = [0.0; LENGTH];
    }
}