use std::sync::Arc;

use vulkano::{
    command_buffer::allocator::{
        StandardCommandBufferAllocator, StandardCommandBufferAllocatorCreateInfo,
    },
    descriptor_set::allocator::StandardDescriptorSetAllocator,
    device::Device,
    memory::allocator::StandardMemoryAllocator,
};

pub struct Allocators {
    pub memory: Arc<StandardMemoryAllocator>,
    pub descriptor_set: Arc<StandardDescriptorSetAllocator>,
    pub command_buffer: Arc<StandardCommandBufferAllocator>,
}

impl Allocators {
    pub fn new_default(device: &Arc<Device>) -> Self {
        Self {
            memory: Arc::new(StandardMemoryAllocator::new_default(device.clone())),
            descriptor_set: StandardDescriptorSetAllocator::new(device.clone(), Default::default())
                .into(),
            command_buffer: StandardCommandBufferAllocator::new(
                device.clone(),
                StandardCommandBufferAllocatorCreateInfo {
                    secondary_buffer_count: 32,
                    ..Default::default()
                },
            )
            .into(),
        }
    }
}
