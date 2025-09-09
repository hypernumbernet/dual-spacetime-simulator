#[derive(Default)]
pub struct UiState {
    pub particle_count: u32,
    pub gravity: f32,
}

#[derive(Default)]
pub struct SimulationState {
    // Placeholder: Add particle data here
}

#[derive(Clone)]
pub struct AppConfig {
    pub acquire_timeout_ms: u64,
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            acquire_timeout_ms: 10, // Default to 10ms as per original value
        }
    }
}