use num_cpus;
use rand::Rng;
use rayon::prelude::*;

pub const AU: f64 = 149_597_870_700.0; // Astronomical Unit in meters
pub const _LIGHT_SPEED: f64 = 299_792_458.0; // Speed of light in meters per second
pub const G: f64 = 6.6743e-11; // Gravitational constant in m^3 kg^-1 s^-2

pub struct SimulationState {
    pub particles: Vec<Particle>,
    pub time: f64,
    pub thread_pool: Option<rayon::ThreadPool>,
    pub scale: f64, // Scale factor (meters per simulation unit)
    pub dt: f64, // Duration per frame in seconds
}

#[derive(Clone, Copy)]
pub struct Particle {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub mass: f64,
}

fn init_particles(particle_count: u32) -> (Vec<Particle>, f64, f64) {
    let scale = 1e10;
    let correct_m = 1.0 / scale; // Scale-corrected length
    let correct_kg = correct_m * correct_m * correct_m; // Scale-corrected mass
    let speed_max = 1e-6;
    let mut rng = rand::thread_rng();
    (
        (0..particle_count)
            .map(|_| Particle {
                position: [
                    rng.gen_range(-1.0..1.0),
                    rng.gen_range(-1.0..1.0),
                    rng.gen_range(-1.0..1.0),
                ],
                velocity: [
                    rng.gen_range(-speed_max..speed_max),
                    rng.gen_range(-speed_max..speed_max),
                    rng.gen_range(-speed_max..speed_max),
                ],
                mass: rng.gen_range(1e31 * correct_kg..1e33 * correct_kg),
            })
            .collect(),
        scale,
        10.5,
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
            time: 0.0,
            thread_pool: Some(thread_pool),
            scale,
            dt
        }
    }

    pub fn reset(&mut self, particle_count: u32) {
        (self.particles, self.scale, self.dt) = init_particles(particle_count);
        self.time = 0.0;
    }

    pub fn update_velocities_with_gravity(&mut self, delta_seconds: f64) {
        let positions: Vec<[f64; 3]> = self.particles.iter().map(|p| p.position).collect();
        let masses: Vec<f64> = self.particles.iter().map(|p| p.mass).collect();
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
                            particle.velocity[k] += acceleration[k] * delta_seconds;
                        }
                    });
            });
        }
    }

    pub fn advance_time(&mut self, delta_seconds: f64) {
        self.time += delta_seconds;
        if let Some(pool) = &self.thread_pool {
            pool.install(|| {
                self.particles.par_iter_mut().for_each(|particle| {
                    for i in 0..3 {
                        particle.position[i] += particle.velocity[i] * delta_seconds;
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
            scale: 1.0,
            dt: 1.0,
        }
    }
}
