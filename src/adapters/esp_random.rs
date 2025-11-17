use crate::ports::RandomPort;

/// ESP32 hardware random number generator adapter
pub struct EspRandom;

impl EspRandom {
    pub fn new() -> Self {
        Self
    }
}

impl RandomPort for EspRandom {
    fn next_bool(&mut self) -> bool {
        // Use ESP32 hardware RNG
        // We'll implement this properly in the main application
        // where we have access to the hardware RNG
        true // Placeholder - will be replaced
    }
}

impl Default for EspRandom {
    fn default() -> Self {
        Self::new()
    }
}
