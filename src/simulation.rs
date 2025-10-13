use glam::DVec3;
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
    pub color: [f32; 4],
}

impl SimulationState {
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
                        let accel_magnitude = G * mass_j / r_squared;
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
