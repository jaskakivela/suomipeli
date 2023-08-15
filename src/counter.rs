// counter.rs

pub struct Counter {
    counter: u32,
    enable: bool,
}

impl Counter {
    pub fn new() -> Self {
        Counter {
            counter: 0_u32,
            enable: true,
        }
    }
    pub fn get(&self) -> u32 {
        self.counter
    }
    pub fn reset(&mut self) {
        self.counter = 0_u32;
    }
    pub fn increment(&mut self) {
        self.counter += 1_u32;
    }
    pub fn enable(&mut self) {
        self.enable = true;
    }
    pub fn disable(&mut self) {
        self.enable = false;
    }
    pub fn set_enable(&mut self, state: bool) {
        self.enable = state;
    }
    pub fn enabled(&self) -> bool {
        self.enable
    }
}

impl Default for Counter {
    fn default() -> Self {
        Self::new()
    }
}

// EOF
