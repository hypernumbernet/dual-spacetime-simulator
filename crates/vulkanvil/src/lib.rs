//! Shared Vulkan foundation for the workspace's renderers: instance/device/swapchain
//! setup, GPU buffer/image allocation helpers, and shader module creation.

pub mod base;
pub mod buffer;
pub mod shader;

pub use base::{VulkanBase, MAX_FRAMES_IN_FLIGHT};
pub use buffer::{
    create_buffer_with_data, create_depth_image, select_depth_format, AllocatedBuffer,
    AllocatedImage,
};
pub use shader::create_shader_module;
