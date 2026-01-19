use glam::DVec3;
use rayon::prelude::*;

use crate::math::spacetime::Spacetime;

pub const AU: f64 = 149_597_870_700.0; // Astronomical Unit in meters
pub const LIGHT_SPEED: f64 = 299_792_458.0; // Speed of light in meters per second
pub const LIGHT_SPEED_SQUARED: f64 = LIGHT_SPEED * LIGHT_SPEED;
pub const G: f64 = 6.6743e-11; // Gravitational constant in m^3 kg^-1 s^-2
pub const EPSILON: f64 = 1e-10;

pub trait SimulationEngine {
    fn update_velocities(&mut self, delta_seconds: f64);
    fn advance_time(&mut self, delta_seconds: f64);
}

pub struct SimulationNormal {
    pub particles: Vec<Particle>,
}

pub struct SimulationSpeedOfLightLimit {
    pub particles: Vec<Particle>,
}

pub struct SimulationLorentzTransformation {
    pub particles: Vec<Particle>,
    pub scale: f64,
}

pub enum SimulationState {
    Normal(SimulationNormal),
    SpeedOfLightLimit(SimulationSpeedOfLightLimit),
    LorentzTransformation(SimulationLorentzTransformation),
}

#[derive(Clone, Copy)]
pub struct Particle {
    pub position: DVec3,
    pub velocity: DVec3,
    pub mass: f64,
    pub color: [f32; 4],
}

impl SimulationEngine for SimulationNormal {
    fn update_velocities(&mut self, delta_seconds: f64) {
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
                    if r_squared < EPSILON {
                        continue;
                    }
                    let accel_magnitude = G * mass_j / r_squared;
                    acceleration += accel_magnitude * diff.normalize();
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

impl SimulationEngine for SimulationSpeedOfLightLimit {
    fn update_velocities(&mut self, delta_seconds: f64) {
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
                    if r_squared < EPSILON {
                        continue;
                    }
                    let accel_magnitude = G * mass_j / r_squared;
                    acceleration += accel_magnitude * diff.normalize();
                }
                particle.velocity += acceleration * delta_seconds;
            });
    }

    fn advance_time(&mut self, delta_seconds: f64) {
        self.particles.par_iter_mut().for_each(|particle| {
            let speed_squared = particle.velocity.length_squared();
            let gamma_inv = (1.0 - speed_squared / LIGHT_SPEED_SQUARED).sqrt();
            particle.position += particle.velocity * gamma_inv * delta_seconds;
        });
    }
}

impl SimulationEngine for SimulationLorentzTransformation {
    // fn update_velocities(&mut self, delta_seconds: f64) {
    //     let positions: Vec<DVec3> = self.particles.iter().map(|p| p.position).collect();
    //     let masses: Vec<f64> = self.particles.iter().map(|p| p.mass).collect();
    //     let ls = self.scale / LIGHT_SPEED;
    //     let time_g = G * delta_seconds;
    //     self.particles
    //         .par_iter_mut()
    //         .enumerate()
    //         .for_each(|(i, particle)| {
    //             let mut acceleration = DVec3::ZERO;
    //             for (j, (&pos_j, &mass_j)) in positions.iter().zip(masses.iter()).enumerate() {
    //                 if i == j {
    //                     continue;
    //                 }
    //                 let diff = pos_j - particle.position;
    //                 let r = diff.length();
    //                 if r < EPSILON {
    //                     continue;
    //                 }
    //                 let inv_r = 1.0 / r;
    //                 let accel_magnitude = time_g * mass_j * inv_r * inv_r;
    //                 let momentum_delta = diff * accel_magnitude;
    //                 let angle = Spacetime::versor_angle(momentum_delta, ls);
    //                 acceleration += angle;
    //             }
    //             particle.velocity += acceleration;
    //         });
    // }
    fn update_velocities(&mut self, delta_seconds: f64) {
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
                    if r_squared < EPSILON {
                        continue;
                    }
                    let accel_magnitude = G * mass_j / r_squared;
                    acceleration += accel_magnitude * diff.normalize();
                }
                particle.velocity += acceleration * delta_seconds;
            });
    }

    fn advance_time(&mut self, delta_seconds: f64) {
        let ls = self.scale / LIGHT_SPEED;
        let ct = LIGHT_SPEED * delta_seconds / self.scale;
        self.particles.par_iter_mut().for_each(|particle| {
            let mut st = Spacetime::from_w(ct);
            st.lorentz_transformation_va(particle.velocity * ls);
            let tau = ct / st.w();
            particle.position += DVec3::new(st.x() * tau, st.y() * tau, st.z() * tau);
        });
    }
}

impl SimulationEngine for SimulationState {
    fn update_velocities(&mut self, delta_seconds: f64) {
        match self {
            SimulationState::Normal(s) => s.update_velocities(delta_seconds),
            SimulationState::SpeedOfLightLimit(s) => s.update_velocities(delta_seconds),
            SimulationState::LorentzTransformation(s) => s.update_velocities(delta_seconds),
        }
    }

    fn advance_time(&mut self, delta_seconds: f64) {
        match self {
            SimulationState::Normal(s) => s.advance_time(delta_seconds),
            SimulationState::SpeedOfLightLimit(s) => s.advance_time(delta_seconds),
            SimulationState::LorentzTransformation(s) => s.advance_time(delta_seconds),
        }
    }
}

impl Default for SimulationNormal {
    fn default() -> Self {
        Self { particles: vec![] }
    }
}

impl Default for SimulationSpeedOfLightLimit {
    fn default() -> Self {
        Self { particles: vec![] }
    }
}

impl Default for SimulationLorentzTransformation {
    fn default() -> Self {
        Self {
            particles: vec![],
            scale: 1e10,
        }
    }
}

impl SimulationState {
    pub fn particles(&self) -> &Vec<Particle> {
        match self {
            SimulationState::Normal(s) => &s.particles,
            SimulationState::SpeedOfLightLimit(s) => &s.particles,
            SimulationState::LorentzTransformation(s) => &s.particles,
        }
    }
}

impl Default for SimulationState {
    fn default() -> Self {
        Self::Normal(SimulationNormal::default())
    }
}
