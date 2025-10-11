use glam::DVec3;
use rand::Rng;
use rayon::prelude::*;

pub const AU: f64 = 149_597_870_700.0; // Astronomical Unit in meters
pub const _LIGHT_SPEED: f64 = 299_792_458.0; // Speed of light in meters per second
pub const G: f64 = 6.6743e-11; // Gravitational constant in m^3 kg^-1 s^-2

pub struct SimulationState {
    pub particles: Vec<Particle>,
    pub scale: f64, // Scale factor (meters per simulation unit)
    pub dt: f64,    // Duration per frame in seconds
}

#[derive(Clone, Copy)]
pub struct Particle {
    pub position: DVec3,
    pub velocity: DVec3,
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
                position: DVec3::new(
                    rng.gen_range(-1.0..1.0),
                    rng.gen_range(-1.0..1.0),
                    rng.gen_range(-1.0..1.0),
                ),
                velocity: DVec3::new(
                    rng.gen_range(-speed_max..speed_max),
                    rng.gen_range(-speed_max..speed_max),
                    rng.gen_range(-speed_max..speed_max),
                ),
                mass: rng.gen_range(1e31 * correct_kg..1e33 * correct_kg),
            })
            .collect(),
        scale,
        10.5,
    )
}

impl SimulationState {
    pub fn new(particle_count: u32) -> Self {
        let (particles, scale, dt) = init_particles(particle_count);
        Self {
            particles,
            scale,
            dt,
        }
    }

    pub fn reset(&mut self, particle_count: u32) {
        (self.particles, self.scale, self.dt) = init_particles(particle_count);
    }

    pub fn update_velocities_with_gravity(&mut self, delta_seconds: f64) {
        let positions: Vec<DVec3> = self.particles.iter().map(|p| p.position).collect();
        let masses: Vec<f64> = self.particles.iter().map(|p| p.mass).collect();
        self.particles
            .par_iter_mut()
            .enumerate()
            .for_each(|(i, particle)| {
                let mut acceleration = DVec3::ZERO;
                for (j, (&pos_j, &mass_j)) in positions.iter().zip(masses.iter()).enumerate() {
                    if i == j {
                        continue;
                    }
                    let diff = pos_j - particle.position;
                    let r_squared = diff.length_squared();
                    if r_squared > 0.0 {
                        let force_magnitude = G * mass_j / r_squared;
                        let accel_magnitude = force_magnitude / particle.mass;
                        acceleration += accel_magnitude * diff.normalize();
                    }
                }
                particle.velocity += acceleration * delta_seconds;
            });
    }

    pub fn advance_time(&mut self, delta_seconds: f64) {
        self.particles.par_iter_mut().for_each(|particle| {
            particle.position += particle.velocity * delta_seconds;
        });
    }
}

impl Default for SimulationState {
    fn default() -> Self {
        Self {
            particles: vec![],
            scale: 1.0,
            dt: 1.0,
        }
    }
}
