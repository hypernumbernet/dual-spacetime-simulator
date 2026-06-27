//! Shared Vulkan foundation for the workspace's renderers: instance/device/swapchain
//! setup, GPU buffer/image allocation helpers, shader module creation, orbit camera,
//! and keyboard camera controls.

pub mod base;
pub mod buffer;
pub mod camera;
pub mod input;
pub mod shader;

#[cfg(feature = "egui")]
pub mod spacecraft_markers;

pub use base::{MAX_FRAMES_IN_FLIGHT, VulkanBase};
pub use buffer::{
    AllocatedBuffer, AllocatedImage, create_buffer_with_data, create_depth_image,
    select_depth_format,
};
pub use camera::*;
#[cfg(feature = "egui")]
pub use spacecraft_markers::{
    draw_spacecraft_steer_marker, draw_spacecraft_yaw_steer_marker,
};
pub use input::InputState;
pub use shader::create_shader_module;
