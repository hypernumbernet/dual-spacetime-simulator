use glam::DVec3;
use rayon::prelude::*;
use std::sync::{Arc, RwLock};

use crate::initial_condition::InitialCondition;
use crate::math::spacetime::{Spacetime, rapidity_from_momentum};
use crate::ui_state::SimulationType;

pub const AU: f64 = 149_597_870_700.0; // Astronomical Unit in meters
pub const LIGHT_SPEED: f64 = 299_792_458.0; // Speed of light in meters per second
pub const LIGHT_SPEED_SQUARED: f64 = LIGHT_SPEED * LIGHT_SPEED;
pub const G: f64 = 6.6743e-11; // Gravitational constant in m^3 kg^-1 s^-2
pub const EPSILON: f64 = 1e-10;

pub trait SimulationEngine {
    /// Updates particle velocities for one simulation step duration.
    fn update_velocities(&mut self, delta_seconds: f64);
    /// Advances particle positions or states for one simulation step duration.
    fn advance_time(&mut self, delta_seconds: f64);
}

pub struct SimulationNormal {
    pub particles: Vec<Particle>,
}

pub struct SimulationSpeedOfLightLimit {
    pub particles: Vec<Particle>,
    pub scale: f64,
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
    /// Applies Newtonian gravity to update velocities for all particles.
    fn update_velocities(&mut self, delta_seconds: f64) {
        let positions: Vec<DVec3> = self.particles.iter().map(|p| p.position).collect();
        let masses: Vec<f64> = self.particles.iter().map(|p| p.mass).collect();
        let time_g = G * delta_seconds;
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
                    let accel_magnitude = time_g * mass_j / r_squared;
                    acceleration += accel_magnitude * diff.normalize();
                }
                particle.velocity += acceleration;
            });
    }

    /// Advances positions using current velocities under classical kinematics.
    fn advance_time(&mut self, delta_seconds: f64) {
        self.particles.par_iter_mut().for_each(|particle| {
            particle.position += particle.velocity * delta_seconds;
        });
    }
}

impl SimulationEngine for SimulationSpeedOfLightLimit {
    /// Applies Newtonian gravity to update velocities before relativistic position correction.
    fn update_velocities(&mut self, delta_seconds: f64) {
        let positions: Vec<DVec3> = self.particles.iter().map(|p| p.position).collect();
        let masses: Vec<f64> = self.particles.iter().map(|p| p.mass).collect();
        let time_g = G * delta_seconds;
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
                    let accel_magnitude = time_g * mass_j / r_squared;
                    acceleration += accel_magnitude * diff.normalize();
                }
                particle.velocity += acceleration;
            });
    }

    /// Advances positions with a gamma-based speed limit correction.
    fn advance_time(&mut self, delta_seconds: f64) {
        let lss = LIGHT_SPEED_SQUARED / (self.scale * self.scale);
        self.particles.par_iter_mut().for_each(|particle| {
            let speed_squared = particle.velocity.length_squared();
            let gamma_inv = (1.0 - speed_squared / lss).sqrt();
            particle.position += particle.velocity * gamma_inv * delta_seconds;
        });
    }
}

impl SimulationEngine for SimulationLorentzTransformation {
    /// Updates rapidity-like velocities from momentum-based relativistic interactions.
    fn update_velocities(&mut self, delta_seconds: f64) {
        let positions: Vec<DVec3> = self.particles.iter().map(|p| p.position).collect();
        let masses: Vec<f64> = self.particles.iter().map(|p| p.mass).collect();
        let time_g = G * delta_seconds;
        let ls = LIGHT_SPEED / self.scale;
        self.particles
            .par_iter_mut()
            .enumerate()
            .for_each(|(i, particle)| {
                let mass_i = particle.mass;
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
                    let force = time_g * mass_i * mass_j / r_squared;
                    let rapidity = rapidity_from_momentum(force * diff.normalize(), mass_i, ls);
                    acceleration += rapidity;
                }
                particle.velocity += acceleration;
            });
    }

    /// Advances positions by applying Lorentz transformation to proper-time increments.
    fn advance_time(&mut self, delta_seconds: f64) {
        let ct = delta_seconds * LIGHT_SPEED / self.scale;
        self.particles.par_iter_mut().for_each(|particle| {
            let mut st = Spacetime::from_t(ct);
            st.lorentz_transformation_rapidity(particle.velocity);
            let tau = ct / st.t;
            particle.position += DVec3::new(st.x * tau, st.y * tau, st.z * tau);
        });
    }
}

impl SimulationEngine for SimulationState {
    /// Delegates velocity updates to the active simulation variant.
    fn update_velocities(&mut self, delta_seconds: f64) {
        match self {
            SimulationState::Normal(s) => s.update_velocities(delta_seconds),
            SimulationState::SpeedOfLightLimit(s) => s.update_velocities(delta_seconds),
            SimulationState::LorentzTransformation(s) => s.update_velocities(delta_seconds),
        }
    }

    /// Delegates time advancement to the active simulation variant.
    fn advance_time(&mut self, delta_seconds: f64) {
        match self {
            SimulationState::Normal(s) => s.advance_time(delta_seconds),
            SimulationState::SpeedOfLightLimit(s) => s.advance_time(delta_seconds),
            SimulationState::LorentzTransformation(s) => s.advance_time(delta_seconds),
        }
    }
}

impl Default for SimulationNormal {
    /// Creates an empty Newtonian simulation state.
    fn default() -> Self {
        Self { particles: vec![] }
    }
}

impl Default for SimulationSpeedOfLightLimit {
    /// Creates an empty speed-limited simulation state with default world scale.
    fn default() -> Self {
        Self {
            particles: vec![],
            scale: 1e10,
        }
    }
}

impl Default for SimulationLorentzTransformation {
    /// Creates an empty Lorentz-transformation simulation state with default scale.
    fn default() -> Self {
        Self {
            particles: vec![],
            scale: 1e10,
        }
    }
}

impl SimulationState {
    /// Returns an immutable reference to particles in the active simulation variant.
    pub fn particles(&self) -> &Vec<Particle> {
        match self {
            SimulationState::Normal(s) => &s.particles,
            SimulationState::SpeedOfLightLimit(s) => &s.particles,
            SimulationState::LorentzTransformation(s) => &s.particles,
        }
    }
}

impl Default for SimulationState {
    /// Creates a default simulation state using the normal Newtonian variant.
    fn default() -> Self {
        Self::Normal(SimulationNormal::default())
    }
}

pub struct SimulationManager {
    pub state: Arc<RwLock<SimulationState>>,
}

impl SimulationManager {
    /// Creates a simulation manager with an initially empty default state.
    pub fn new() -> Self {
        Self {
            state: Arc::new(RwLock::new(SimulationState::default())),
        }
    }

    /// Builds a simulation state from initial conditions and selected simulation model.
    pub fn create_simulation(
        initial_condition: InitialCondition,
        simulation_type: SimulationType,
        particle_count: u32,
        scale: f64,
    ) -> SimulationState {
        let normal = initial_condition.generate_particles(particle_count);
        match simulation_type {
            SimulationType::Normal => SimulationState::Normal(SimulationNormal {
                particles: normal.particles,
            }),
            SimulationType::SpeedOfLightLimit => {
                SimulationState::SpeedOfLightLimit(SimulationSpeedOfLightLimit {
                    particles: normal.particles,
                    scale,
                })
            }
            SimulationType::LorentzTransformation => {
                SimulationState::LorentzTransformation(SimulationLorentzTransformation {
                    particles: Self::convert_to_lorentz(normal.particles, scale),
                    scale,
                })
            }
        }
    }

    /// Converts particle velocities into rapidity representation for Lorentz mode.
    pub fn convert_to_lorentz(particles: Vec<Particle>, scale: f64) -> Vec<Particle> {
        let ls = scale / LIGHT_SPEED;
        particles
            .into_iter()
            .map(|p| Particle {
                position: p.position,
                velocity: crate::math::spacetime::rapidity_vector(p.velocity, ls),
                mass: p.mass,
                color: p.color,
            })
            .collect()
    }

    /// Replaces current simulation state with a freshly generated one.
    pub fn reset(
        &self,
        initial_condition: InitialCondition,
        simulation_type: SimulationType,
        particle_count: u32,
        scale: f64,
    ) {
        let new_state =
            Self::create_simulation(initial_condition, simulation_type, particle_count, scale);
        let mut state_guard = self.state.write().unwrap();
        *state_guard = new_state;
    }

    /// Advances the active simulation by one frame and updates velocities.
    pub fn advance(&self, time_per_frame: f64) {
        let mut sim = self.state.write().unwrap();
        sim.advance_time(time_per_frame);
        sim.update_velocities(time_per_frame);
    }

    /// Returns a cloned particle list from the current simulation state.
    pub fn particles(&self) -> Vec<Particle> {
        let state = self.state.read().unwrap();
        state.particles().clone()
    }
}

impl Default for SimulationManager {
    /// Creates a default simulation manager instance.
    fn default() -> Self {
        Self::new()
    }
}
