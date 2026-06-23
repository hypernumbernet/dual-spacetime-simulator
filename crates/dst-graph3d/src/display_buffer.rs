//! Display-only particle SSBO management for the 3D graph viewer.
//!
//! Provides a GPU storage buffer for visual particle points (positions + colors)
//! without any of the compute simulation logic from the parent simulator.

use ash::vk;
use gpu_allocator::vulkan::Allocator;
use std::sync::{Arc, Mutex};
use vulkanvil::AllocatedBuffer;

/// GPU particle layout: four 16-byte vec4 slots (64 bytes total).
/// Must match GLSL `Particle` in vertex shaders exactly under std430.
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuParticle {
    pub position: [f32; 4],
    pub velocity: [f32; 4],
    pub attrs: [f32; 4],
    pub color: [f32; 4],
}

const _: () = assert!(std::mem::size_of::<GpuParticle>() == 64);

impl GpuParticle {
    pub fn from_display(position: [f32; 3], color: [f32; 4]) -> Self {
        Self {
            position: [position[0], position[1], position[2], 0.0],
            velocity: [0.0; 4],
            attrs: [0.0; 4],
            color,
        }
    }
}

pub struct DisplayParticleBuffer {
    device: ash::Device,
    allocator: Arc<Mutex<Allocator>>,
    particle_buffer: AllocatedBuffer,
    descriptor_pool: vk::DescriptorPool,
    descriptor_set: vk::DescriptorSet,
    particle_count: u32,
    buffer_capacity: usize,
}

impl DisplayParticleBuffer {
    /// Creates display-only particle resources backed by a CPU-visible SSBO.
    pub fn new(
        device: ash::Device,
        allocator: Arc<Mutex<Allocator>>,
        descriptor_set_layout: vk::DescriptorSetLayout,
    ) -> Self {
        let descriptor_pool = create_descriptor_pool(&device);
        let buffer_capacity = 1;
        let particle_buffer = create_particle_storage_buffer(
            &device,
            &allocator,
            buffer_capacity,
            "display_particle_ssbo",
        );
        let descriptor_set =
            allocate_descriptor_set(&device, descriptor_pool, descriptor_set_layout);
        update_particle_descriptor(
            &device,
            descriptor_set,
            particle_buffer.buffer,
            particle_buffer_size(buffer_capacity),
        );

        Self {
            device,
            allocator,
            particle_buffer,
            descriptor_pool,
            descriptor_set,
            particle_count: 0,
            buffer_capacity,
        }
    }

    pub fn particle_count(&self) -> u32 {
        self.particle_count
    }

    pub fn descriptor_set(&self) -> vk::DescriptorSet {
        self.descriptor_set
    }

    /// Uploads display-only point data (positions + colors) into the mapped SSBO.
    pub fn upload_display_points(&mut self, positions: &[[f32; 3]], colors: &[[f32; 4]]) {
        self.particle_count = positions.len() as u32;
        if positions.is_empty() {
            return;
        }
        self.ensure_buffer_capacity(positions.len());
        self.write_display_points(positions, colors);
    }

    fn ensure_buffer_capacity(&mut self, count: usize) {
        if count <= self.buffer_capacity {
            return;
        }
        let capacity = count.max(1);
        let alloc = Arc::clone(&self.allocator);
        let old = std::mem::replace(
            &mut self.particle_buffer,
            create_particle_storage_buffer(
                &self.device,
                &alloc,
                capacity,
                "display_particle_ssbo",
            ),
        );
        old.destroy(&self.device, &alloc);
        self.buffer_capacity = capacity;
        update_particle_descriptor(
            &self.device,
            self.descriptor_set,
            self.particle_buffer.buffer,
            particle_buffer_size(capacity),
        );
    }

    fn write_display_points(&self, positions: &[[f32; 3]], colors: &[[f32; 4]]) {
        let Some(dst) = mapped_particle_slice_mut(&self.particle_buffer, positions.len()) else {
            return;
        };
        for (slot, (position, color)) in dst.iter_mut().zip(positions.iter().zip(colors)) {
            *slot = GpuParticle::from_display(*position, *color);
        }
    }
}

impl Drop for DisplayParticleBuffer {
    fn drop(&mut self) {
        unsafe {
            self.device
                .destroy_descriptor_pool(self.descriptor_pool, None);
        }
        let alloc = Arc::clone(&self.allocator);
        let mut empty = AllocatedBuffer {
            buffer: vk::Buffer::null(),
            allocation: None,
        };
        std::mem::swap(&mut self.particle_buffer, &mut empty);
        if empty.buffer != vk::Buffer::null() {
            empty.destroy(&self.device, &alloc);
        }
    }
}

fn particle_buffer_size(count: usize) -> u64 {
    (std::mem::size_of::<GpuParticle>() * count.max(1)) as u64
}

fn create_particle_storage_buffer(
    device: &ash::Device,
    allocator: &Arc<Mutex<Allocator>>,
    capacity: usize,
    name: &str,
) -> AllocatedBuffer {
    AllocatedBuffer::new(
        device,
        allocator,
        particle_buffer_size(capacity),
        vk::BufferUsageFlags::STORAGE_BUFFER,
        gpu_allocator::MemoryLocation::CpuToGpu,
        name,
    )
}

fn mapped_particle_slice_mut(buffer: &AllocatedBuffer, count: usize) -> Option<&mut [GpuParticle]> {
    if count == 0 {
        return Some(&mut []);
    }
    let alloc = buffer.allocation.as_ref()?;
    let mapped = alloc.mapped_ptr()?;
    Some(unsafe { std::slice::from_raw_parts_mut(mapped.as_ptr() as *mut GpuParticle, count) })
}

/// Creates the descriptor set layout for binding the display particle SSBO to the vertex shader.
pub fn create_particle_descriptor_set_layout(device: &ash::Device) -> vk::DescriptorSetLayout {
    let binding = vk::DescriptorSetLayoutBinding::default()
        .binding(0)
        .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
        .descriptor_count(1)
        .stage_flags(vk::ShaderStageFlags::VERTEX);
    let bindings = [binding];
    let ci = vk::DescriptorSetLayoutCreateInfo::default().bindings(&bindings);
    unsafe { device.create_descriptor_set_layout(&ci, None) }.unwrap()
}

fn create_descriptor_pool(device: &ash::Device) -> vk::DescriptorPool {
    let pool_size = vk::DescriptorPoolSize {
        ty: vk::DescriptorType::STORAGE_BUFFER,
        descriptor_count: 1,
    };
    let pool_sizes = [pool_size];
    let ci = vk::DescriptorPoolCreateInfo::default()
        .pool_sizes(&pool_sizes)
        .max_sets(1);
    unsafe { device.create_descriptor_pool(&ci, None) }.unwrap()
}

fn allocate_descriptor_set(
    device: &ash::Device,
    pool: vk::DescriptorPool,
    layout: vk::DescriptorSetLayout,
) -> vk::DescriptorSet {
    let layouts = [layout];
    let ci = vk::DescriptorSetAllocateInfo::default()
        .descriptor_pool(pool)
        .set_layouts(&layouts);
    unsafe { device.allocate_descriptor_sets(&ci).unwrap()[0] }
}

fn update_particle_descriptor(
    device: &ash::Device,
    descriptor_set: vk::DescriptorSet,
    buffer: vk::Buffer,
    size: u64,
) {
    let info = vk::DescriptorBufferInfo {
        buffer,
        offset: 0,
        range: size,
    };
    let infos = [info];
    let write = vk::WriteDescriptorSet::default()
        .dst_set(descriptor_set)
        .dst_binding(0)
        .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
        .buffer_info(&infos);
    let writes = [write];
    unsafe {
        device.update_descriptor_sets(&writes, &[]);
    }
}
