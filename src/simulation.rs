use num_cpus;
use rand::Rng;
use rayon::prelude::*;

pub struct SimulationState {
    pub particles: Vec<Particle>,
    pub time: f64,
    pub thread_pool: Option<rayon::ThreadPool>,
}

#[derive(Clone, Copy)]
pub struct Particle {
    pub position: [f32; 3],
    pub velocity: [f32; 3],
    pub mass: f32,
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
                velocity: [
                    rng.gen_range(-0.01..0.01),
                    rng.gen_range(-0.01..0.01),
                    rng.gen_range(-0.01..0.01),
                ],
                mass: rng.gen_range(0.5..2.0),
            })
            .collect();
        let thread_pool = rayon::ThreadPoolBuilder::new()
            .num_threads(num_cpus::get())
            .build()
            .unwrap();
        Self {
            particles,
            time: 0.0,
            thread_pool: Some(thread_pool),
        }
    }

    pub fn reset(&mut self, particle_count: u32) {
        let mut rng = rand::thread_rng();
        self.particles = (0..particle_count)
            .map(|_| Particle {
                position: [
                    rng.gen_range(-1.0..1.0),
                    rng.gen_range(-1.0..1.0),
                    rng.gen_range(-1.0..1.0),
                ],
                velocity: [
                    rng.gen_range(-0.01..0.01),
                    rng.gen_range(-0.01..0.01),
                    rng.gen_range(-0.01..0.01),
                ],
                mass: rng.gen_range(0.5..2.0),
            })
            .collect();
        self.time = 0.0;
    }

    pub fn update_velocities_with_gravity(&mut self, delta_seconds: f64) {
        const G: f32 = 0.000001;
        let dt = delta_seconds as f32;
        let positions: Vec<[f32; 3]> = self.particles.iter().map(|p| p.position).collect();
        let masses: Vec<f32> = self.particles.iter().map(|p| p.mass).collect();
        if let Some(pool) = &self.thread_pool {
            pool.install(|| {
                self.particles
                    .par_iter_mut()
                    .enumerate()
                    .for_each(|(i, particle)| {
                        let mut acceleration = [0.0, 0.0, 0.0];
                        for (j, (&pos, &mass_j)) in positions.iter().zip(masses.iter()).enumerate()
                        {
                            if i == j {
                                continue;
                            }
                            let dx = pos[0] - particle.position[0];
                            let dy = pos[1] - particle.position[1];
                            let dz = pos[2] - particle.position[2];
                            let r_squared = dx * dx + dy * dy + dz * dz;
                            if r_squared > 0.0 {
                                let r = r_squared.sqrt();
                                let force_magnitude = G * mass_j / r_squared;
                                let accel_magnitude = force_magnitude / particle.mass;
                                acceleration[0] += accel_magnitude * dx / r;
                                acceleration[1] += accel_magnitude * dy / r;
                                acceleration[2] += accel_magnitude * dz / r;
                            }
                        }
                        for k in 0..3 {
                            particle.velocity[k] += acceleration[k] * dt;
                        }
                    });
            });
        }
    }

    pub fn advance_time(&mut self, delta_seconds: f64) {
        self.time += delta_seconds;
        let dt_f32 = delta_seconds as f32;
        if let Some(pool) = &self.thread_pool {
            pool.install(|| {
                self.particles.par_iter_mut().for_each(|particle| {
                    for i in 0..3 {
                        particle.position[i] += particle.velocity[i] * dt_f32;
                    }
                });
            });
        }
    }
}

impl Default for SimulationState {
    fn default() -> Self {
        Self {
            particles: vec![],
            time: 0.0,
            thread_pool: None,
        }
    }
}
