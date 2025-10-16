use glam::DVec3;
use rayon::prelude::*;

pub const AU: f64 = 149_597_870_700.0; // Astronomical Unit in meters
pub const _LIGHT_SPEED: f64 = 299_792_458.0; // Speed of light in meters per second
pub const G: f64 = 6.6743e-11; // Gravitational constant in m^3 kg^-1 s^-2

pub trait SimulationEngine {
    fn update_velocities_with_gravity(&mut self, delta_seconds: f64);
    fn advance_time(&mut self, delta_seconds: f64);
}

pub struct SimulationNormal {
    pub particles: Vec<Particle>,
    pub scale: f64, // Scale factor (meters per simulation unit)
    pub dt: f64,    // Duration per frame in seconds
}

pub struct SimulationSpecial {
    pub particles: Vec<Particle>,
    pub scale: f64, // Scale factor (meters per simulation unit)
    pub dt: f64,    // Duration per frame in seconds
}

pub enum SimulationState {
    Normal(SimulationNormal),
    Special(SimulationSpecial),
}

#[derive(Clone, Copy)]
pub struct Particle {
    pub position: DVec3,
    pub velocity: DVec3,
    pub mass: f64,
    pub color: [f32; 4],
}

impl SimulationEngine for SimulationNormal {
    fn update_velocities_with_gravity(&mut self, delta_seconds: f64) {
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

    fn advance_time(&mut self, delta_seconds: f64) {
        self.particles.par_iter_mut().for_each(|particle| {
            particle.position += particle.velocity * delta_seconds;
        });
    }
}

impl SimulationEngine for SimulationSpecial {
    fn update_velocities_with_gravity(&mut self, delta_seconds: f64) {
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

    fn advance_time(&mut self, delta_seconds: f64) {
        self.particles.par_iter_mut().for_each(|particle| {
            particle.position += particle.velocity * delta_seconds;
        });
    }
}

impl SimulationEngine for SimulationState {
    fn update_velocities_with_gravity(&mut self, delta_seconds: f64) {
        match self {
            SimulationState::Normal(s) => s.update_velocities_with_gravity(delta_seconds),
            SimulationState::Special(s) => s.update_velocities_with_gravity(delta_seconds),
        }
    }

    fn advance_time(&mut self, delta_seconds: f64) {
        match self {
            SimulationState::Normal(s) => s.advance_time(delta_seconds),
            SimulationState::Special(s) => s.advance_time(delta_seconds),
        }
    }
}

impl Default for SimulationNormal {
    fn default() -> Self {
        Self {
            particles: vec![],
            scale: 1.0,
            dt: 1.0,
        }
    }
}

impl Default for SimulationSpecial {
    fn default() -> Self {
        Self {
            particles: vec![],
            scale: 1.0,
            dt: 1.0,
        }
    }
}

impl SimulationState {
    pub fn particles(&self) -> &Vec<Particle> {
        match self {
            SimulationState::Normal(s) => &s.particles,
            SimulationState::Special(s) => &s.particles,
        }
    }
}

impl Default for SimulationState {
    fn default() -> Self {
        Self::Normal(SimulationNormal::default())
    }
}
