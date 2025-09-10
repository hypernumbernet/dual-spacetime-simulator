#[derive(Default)]
pub struct UiState {
    pub particle_count: u32,
    pub gravity: f32,
    //pub initial_particle_count: usize,
    //pub max_particle_count: usize,
}

/*#[derive(Default)]
pub struct SimulationState {
    // Placeholder: Add particle data here
}

#[derive(Clone)]
pub struct AppConfig {
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            initial_particle_count: 1000,
            max_particle_count: 20000,
        }
    }
}*/