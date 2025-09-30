use rand::Rng;

#[derive(Clone)]
pub struct SimulationState {
    pub particles: Vec<Particle>,
}

#[derive(Clone, Copy)]
pub struct Particle {
    pub position: [f32; 3],
}

impl SimulationState {
    pub fn new(particle_count: u32) -> Self {
        let mut rng = rand::thread_rng();
        let particles = (0..particle_count)
            .map(|_| Particle {
                position: [
                    rng.gen_range(-1.0..1.0),
                    rng.gen_range(-1.0..1.0),
                    rng.gen_range(-1.0..1.0),
                ],
            })
            .collect();
        Self { particles }
    }
}

impl Default for SimulationState {
    fn default() -> Self {
        Self::new(0)
    }
}

//pub fn update_simulation(_simulation_state: &mut SimulationState, _ui_state: &UiState) {
    // Placeholder: Implement particle simulation logic here
//}
