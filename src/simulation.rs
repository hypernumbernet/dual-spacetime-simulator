use glam::Vec3;
use num_cpus;
use rand::Rng;
use rayon::prelude::*;

pub const AU: f64 = 149_597_870_700.0; // Astronomical Unit in meters
pub const _LIGHT_SPEED: f64 = 299_792_458.0; // Speed of light in meters per second
pub const G: f32 = 6.6743e-11; // Gravitational constant in m^3 kg^-1 s^-2

pub struct SimulationState {
    pub particles: Vec<Particle>,
    pub time: f32,
    pub thread_pool: Option<rayon::ThreadPool>,
    pub scale: f32, // Scale factor (meters per simulation unit)
    pub dt: f32, // Duration per frame in seconds
}

#[derive(Clone, Copy)]
pub struct Particle {
    pub position: Vec3,
    pub velocity: Vec3,
    pub mass: f32,
}

fn init_particles(particle_count: u32) -> (Vec<Particle>, f32, f32) {
    let scale = 1e10f32;
    let correct_m = 1.0f32 / scale; // Scale-corrected length
    let correct_kg = correct_m * correct_m * correct_m; // Scale-corrected mass
    let speed_max = 1e-6f32;
    let mut rng = rand::thread_rng();
    (
        (0..particle_count)
            .map(|_| Particle {
                position: Vec3::new(
                    rng.gen_range(-1.0f32..1.0f32),
                    rng.gen_range(-1.0f32..1.0f32),
                    rng.gen_range(-1.0f32..1.0f32),
                ),
                velocity: Vec3::new(
                    rng.gen_range(-speed_max..speed_max),
                    rng.gen_range(-speed_max..speed_max),
                    rng.gen_range(-speed_max..speed_max),
                ),
                mass: rng.gen_range(1e31f32 * correct_kg..1e33f32 * correct_kg),
            })
            .collect(),
        scale,
        10.5f32,
    )
}

impl SimulationState {
    pub fn new(particle_count: u32) -> Self {
        let thread_pool = rayon::ThreadPoolBuilder::new()
            .num_threads(num_cpus::get())
            .build()
            .unwrap();
        let (particles, scale, dt) = init_particles(particle_count);
        Self {
            particles,
            time: 0.0f32,
            thread_pool: Some(thread_pool),
            scale,
            dt
        }
    }

    pub fn reset(&mut self, particle_count: u32) {
        (self.particles, self.scale, self.dt) = init_particles(particle_count);
        self.time = 0.0f32;
    }

    pub fn update_velocities_with_gravity(&mut self, delta_seconds: f32, gravity_threshold: f32) {
        let positions: Vec<Vec3> = self.particles.iter().map(|p| p.position).collect();
        let masses: Vec<f32> = self.particles.iter().map(|p| p.mass).collect();
        if let Some(pool) = &self.thread_pool {
            pool.install(|| {
                self.particles
                    .par_iter_mut()
                    .enumerate()
                    .for_each(|(i, particle)| {
                        let mut acceleration = Vec3::ZERO;
                        for (j, (&pos_j, &mass_j)) in positions.iter().zip(masses.iter()).enumerate()
                        {
                            if i == j {
                                continue;
                            }
                            let diff = pos_j - particle.position;
                            let r_squared = diff.length_squared();
                            if r_squared > 0.0 && (gravity_threshold <= 0.0 || r_squared <= gravity_threshold) {
                                let force_magnitude = G * mass_j / r_squared;
                                let accel_magnitude = force_magnitude / particle.mass;
                                acceleration += accel_magnitude * diff.normalize();
                            }
                        }
                        particle.velocity += acceleration * delta_seconds;
                    });
            });
        }
    }

    pub fn advance_time(&mut self, delta_seconds: f32) {
        self.time += delta_seconds;
        if let Some(pool) = &self.thread_pool {
            pool.install(|| {
                self.particles.par_iter_mut().for_each(|particle| {
                    particle.position += particle.velocity * delta_seconds;
                });
            });
        }
    }
}

impl Default for SimulationState {
    fn default() -> Self {
        Self {
            particles: vec![],
            time: 0.0f32,
            thread_pool: None,
            scale: 1.0f32,
            dt: 1.0f32,
        }
    }
}
