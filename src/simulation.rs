use glam::DVec3;
use rayon::prelude::*;
use std::sync::{Arc, RwLock};

use crate::initial_condition::InitialCondition;
use crate::math::spacetime::{Spacetime, rapidity_from_momentum};
use crate::ui_state::{AppMode, SimulationType};

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
        Self {
            particles: vec![],
            scale: 1e10,
        }
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

pub struct SimulationManager {
    pub state: Arc<RwLock<SimulationState>>,
}

impl SimulationManager {
    pub fn new() -> Self {
        Self {
            state: Arc::new(RwLock::new(SimulationState::default())),
        }
    }

    /// SimulationTypeとInitialConditionからSimulationStateを作成
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

    /// 速度をrapidityに変換（Lorentzモード用）
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

    /// リセット処理（UIから呼ばれる）
    pub fn reset(
        &self,
        initial_condition: InitialCondition,
        simulation_type: SimulationType,
        particle_count: u32,
        scale: f64,
    ) {
        let new_state = Self::create_simulation(
            initial_condition,
            simulation_type,
            particle_count,
            scale,
        );
        let mut state_guard = self.state.write().unwrap();
        *state_guard = new_state;
    }

    /// モード切替（AppMode変更時。Graph3D時はシミュレーションを停止準備）
    pub fn switch_mode(&self, mode: AppMode) {
        if mode == AppMode::Graph3D {
            // Graph3Dモードではシミュレーションを一時停止（将来拡張用placeholder）
            // ここでGraph専用の状態に切り替える余地を残す
            let _state = self.state.write().unwrap();
            // GraphStateなどに変換する拡張ポイント
        }
        // Simulationモードでは何もしない（既存状態を維持）
    }

    /// 1フレーム分の物理更新（advance_time + update_velocities）
    pub fn advance(&self, time_per_frame: f64) {
        let mut sim = self.state.write().unwrap();
        sim.advance_time(time_per_frame);
        sim.update_velocities(time_per_frame);
    }

    /// particles()の委譲（RwLock経由で安全に取得）
    pub fn particles(&self) -> Vec<Particle> {
        let state = self.state.read().unwrap();
        state.particles().clone()
    }
}

impl Default for SimulationManager {
    fn default() -> Self {
        Self::new()
    }
}
