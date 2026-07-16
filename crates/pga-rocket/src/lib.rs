//! PGA-based legged rocket launch/landing simulator.
//!
//! Pure simulation modules (`euclidean_pga`, `sim`, `control`, `mesh`) are free of
//! Vulkan/window I/O so unit tests exercise the real physics and input mapping.

pub mod control;
pub mod euclidean_pga;
pub mod mesh;
pub mod sim;

pub use control::{ControlMapper, KeySnapshot, map_keys};
pub use sim::{ControlCommand, RocketParams, RocketState, step_rocket};
