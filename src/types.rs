#[derive(Clone)]
pub struct UiState {
    pub input_panel_width: f32,
    pub particle_count: u32,
    pub max_particle_count: u32,
    pub gravity: f32,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            input_panel_width: 200.0,
            particle_count: 1000,
            max_particle_count: 20000,
            gravity: 9.81,
        }
    }
}
/*
#[derive(Default)]
pub struct SimulationState {
    // Placeholder: Add particle data here
}

#[derive(Clone)]
pub struct AppConfig {
}
*/