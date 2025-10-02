use rand::Rng;

#[derive(Clone)]
pub struct SimulationState {
    pub particles: Vec<Particle>,
    pub time: f64,
}

#[derive(Clone, Copy)]
pub struct Particle {
    pub position: [f32; 3],
    pub speed: [f32; 3],
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
                speed: [
                    rng.gen_range(-0.01..0.01),
                    rng.gen_range(-0.01..0.01),
                    rng.gen_range(-0.01..0.01),
                ],
            })
            .collect();
        Self {
            particles,
            time: 0.0,
        }
    }

    pub fn advance_time(&mut self, delta_seconds: f64) {
        self.time += delta_seconds;
        let dt_f32 = delta_seconds as f32;
        for particle in &mut self.particles {
            for i in 0..3 {
                particle.position[i] += particle.speed[i] * dt_f32;
            }
        }
    }
}

impl Default for SimulationState {
    fn default() -> Self {
        Self::new(0)
    }
}
