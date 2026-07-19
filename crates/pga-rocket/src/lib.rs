//! PGA-based legged rocket launch/landing simulator.
//!
//! Pure simulation modules (`euclidean_pga`, `sim`, `control`, `mesh`) are free of
//! Vulkan/window I/O so unit tests exercise the real physics and input mapping.

pub mod control;
pub mod euclidean_pga;
pub mod explosion;
pub mod landing;
pub mod mesh;
pub mod sim;
pub mod target_landing;

pub use control::{
    ControlMapper, KeySnapshot, THROTTLE_LATCH_RAMP_S, ThrottleLatch, map_keys,
};
pub use landing::LandingAutopilot;
pub use target_landing::{
    inside_target_pad, TargetLandingAutopilot, TargetPhase, CLIMB_ALT_M, TARGET_PAD_HALF_M,
};
pub use sim::{
    BodyWrench, ContactKind, ContactProbe, ControlCommand, RocketParams, RocketState, ThrusterSample,
    body_wrench_at, cross, engine_wrench, gimbal_rotor, ground_contact_probes, propulsive_wrench,
    rcs_wrench, roll_thrusters, rotate_vector_by_rotor, step_rocket, thrust_dir_body,
};
