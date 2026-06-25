//! Shared Vulkan foundation for the workspace's renderers: instance/device/swapchain
//! setup, GPU buffer/image allocation helpers, shader module creation, orbit camera,
//! and keyboard camera controls.

pub mod base;
pub mod buffer;
pub mod camera;
pub mod input;
pub mod shader;

pub use base::{MAX_FRAMES_IN_FLIGHT, VulkanBase};
pub use buffer::{
    AllocatedBuffer, AllocatedImage, create_buffer_with_data, create_depth_image,
    select_depth_format,
};
pub use camera::{
    OrbitCamera, KEYBOARD_ORBIT_YAW_SPEED, KEYBOARD_PAN_SPEED, WHEEL_FORWARD_SPEED,
    apply_orbit_keyboard, apply_wheel_forward, tick_orbit_camera,
};
pub use input::InputState;
pub use shader::create_shader_module;
