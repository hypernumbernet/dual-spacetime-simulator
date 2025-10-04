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
            })
            .collect();
        dbg!(num_cpus::get());
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
