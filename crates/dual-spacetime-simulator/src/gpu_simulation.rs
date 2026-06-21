use crate::simulation::{Particle, G, EPSILON};
use ash::vk;
use bytemuck::Zeroable;
use glam::DVec3;
use gpu_allocator::vulkan::Allocator;
use std::sync::{Arc, Mutex};
use vulkanvil::{create_shader_module, AllocatedBuffer};

const WORKGROUP_SIZE: u32 = 64;

/// GPU particle layout: four 16-byte vec4 slots (64 bytes total).
/// Must match GLSL `Particle` in compute and vertex shaders exactly under std430.
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
    pub fn from_cpu(particle: &Particle) -> Self {
        Self {
            position: [
                particle.position.x as f32,
                particle.position.y as f32,
                particle.position.z as f32,
                0.0,
            ],
            velocity: [
                particle.velocity.x as f32,
                particle.velocity.y as f32,
                particle.velocity.z as f32,
                0.0,
            ],
            attrs: [particle.mass as f32, 0.0, 0.0, 0.0],
            color: particle.color,
        }
    }

    pub fn to_cpu(self) -> Particle {
        Particle {
            position: DVec3::new(
                self.position[0] as f64,
                self.position[1] as f64,
                self.position[2] as f64,
            ),
            velocity: DVec3::new(
                self.velocity[0] as f64,
                self.velocity[1] as f64,
                self.velocity[2] as f64,
            ),
            mass: self.attrs[0] as f64,
            color: self.color,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct ComputePushConstants {
    particle_count: u32,
    delta_seconds: f32,
    gravity_dt: f32,
    epsilon: f32,
    phase: u32,
}

pub struct GpuParticleSimulation {
    device: ash::Device,
    allocator: Arc<Mutex<Allocator>>,
    particle_buffer: AllocatedBuffer,
    descriptor_pool: vk::DescriptorPool,
    descriptor_set: vk::DescriptorSet,
    compute_pipeline: vk::Pipeline,
    compute_layout: vk::PipelineLayout,
    particle_count: u32,
}

impl GpuParticleSimulation {
    /// Creates GPU particle simulation resources and uploads initial particle data.
    pub fn new(
        device: ash::Device,
        allocator: Arc<Mutex<Allocator>>,
        descriptor_set_layout: vk::DescriptorSetLayout,
        particles: &[Particle],
    ) -> Self {
        let compute_layout =
            create_compute_pipeline_layout(&device, descriptor_set_layout);
        let compute_pipeline = create_compute_pipeline(&device, compute_layout);
        let descriptor_pool = create_descriptor_pool(&device);
        let particle_count = particles.len() as u32;
        let gpu_particles: Vec<GpuParticle> =
            particles.iter().map(GpuParticle::from_cpu).collect();
        let particle_buffer = create_particle_storage_buffer(
            &device,
            &allocator,
            &gpu_particles,
            "gpu_particle_ssbo",
        );
        let descriptor_set =
            allocate_descriptor_set(&device, descriptor_pool, descriptor_set_layout);
        update_particle_descriptor(
            &device,
            descriptor_set,
            particle_buffer.buffer,
            particle_buffer_size(gpu_particles.len()),
        );

        Self {
            device,
            allocator,
            particle_buffer,
            descriptor_pool,
            descriptor_set,
            compute_pipeline,
            compute_layout,
            particle_count,
        }
    }

    pub fn particle_count(&self) -> u32 {
        self.particle_count
    }

    pub fn descriptor_set(&self) -> vk::DescriptorSet {
        self.descriptor_set
    }

    /// Uploads CPU particle data into the GPU storage buffer.
    pub fn upload_from_cpu(&mut self, particles: &[Particle]) {
        let gpu_particles: Vec<GpuParticle> =
            particles.iter().map(GpuParticle::from_cpu).collect();
        let count = gpu_particles.len().max(1);
        self.particle_count = particles.len() as u32;

        let alloc = Arc::clone(&self.allocator);
        let old = std::mem::replace(
            &mut self.particle_buffer,
            create_particle_storage_buffer(
                &self.device,
                &alloc,
                &gpu_particles,
                "gpu_particle_ssbo",
            ),
        );
        old.destroy(&self.device, &alloc);
        update_particle_descriptor(
            &self.device,
            self.descriptor_set,
            self.particle_buffer.buffer,
            particle_buffer_size(count),
        );
    }

    /// Records compute dispatches that advance one Normal simulation step on the GPU.
    pub fn dispatch(&self, command_buffer: vk::CommandBuffer, delta_seconds: f64) {
        if self.particle_count == 0 {
            return;
        }

        let gravity_dt = (G * delta_seconds) as f32;
        let delta = delta_seconds as f32;
        let workgroups = (self.particle_count + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE;

        unsafe {
            self.device.cmd_bind_pipeline(
                command_buffer,
                vk::PipelineBindPoint::COMPUTE,
                self.compute_pipeline,
            );
            self.device.cmd_bind_descriptor_sets(
                command_buffer,
                vk::PipelineBindPoint::COMPUTE,
                self.compute_layout,
                0,
                &[self.descriptor_set],
                &[],
            );

            let advance_pc = ComputePushConstants {
                particle_count: self.particle_count,
                delta_seconds: delta,
                gravity_dt,
                epsilon: EPSILON as f32,
                phase: 0,
            };
            self.device.cmd_push_constants(
                command_buffer,
                self.compute_layout,
                vk::ShaderStageFlags::COMPUTE,
                0,
                bytemuck::bytes_of(&advance_pc),
            );
            self.device
                .cmd_dispatch(command_buffer, workgroups, 1, 1);

            let barrier = vk::MemoryBarrier::default()
                .src_access_mask(vk::AccessFlags::SHADER_WRITE)
                .dst_access_mask(vk::AccessFlags::SHADER_READ);
            self.device.cmd_pipeline_barrier(
                command_buffer,
                vk::PipelineStageFlags::COMPUTE_SHADER,
                vk::PipelineStageFlags::COMPUTE_SHADER,
                vk::DependencyFlags::empty(),
                &[barrier],
                &[],
                &[],
            );

            let velocity_pc = ComputePushConstants {
                particle_count: self.particle_count,
                delta_seconds: delta,
                gravity_dt,
                epsilon: EPSILON as f32,
                phase: 1,
            };
            self.device.cmd_push_constants(
                command_buffer,
                self.compute_layout,
                vk::ShaderStageFlags::COMPUTE,
                0,
                bytemuck::bytes_of(&velocity_pc),
            );
            self.device
                .cmd_dispatch(command_buffer, workgroups, 1, 1);

            let barrier = vk::MemoryBarrier::default()
                .src_access_mask(vk::AccessFlags::SHADER_WRITE)
                .dst_access_mask(vk::AccessFlags::SHADER_READ);
            self.device.cmd_pipeline_barrier(
                command_buffer,
                vk::PipelineStageFlags::COMPUTE_SHADER,
                vk::PipelineStageFlags::VERTEX_SHADER,
                vk::DependencyFlags::empty(),
                &[barrier],
                &[],
                &[],
            );
        }
    }

    /// Copies GPU particle data back to CPU for snapshot export.
    pub fn readback_to_cpu(&self) -> Vec<Particle> {
        read_mapped_particles(&self.particle_buffer, self.particle_count as usize)
    }
}

impl Drop for GpuParticleSimulation {
    fn drop(&mut self) {
        unsafe {
            self.device.destroy_pipeline(self.compute_pipeline, None);
            self.device
                .destroy_pipeline_layout(self.compute_layout, None);
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
    particles: &[GpuParticle],
    name: &str,
) -> AllocatedBuffer {
    let data = if particles.is_empty() {
        vec![GpuParticle::zeroed()]
    } else {
        particles.to_vec()
    };
    let buf = AllocatedBuffer::new(
        device,
        allocator,
        particle_buffer_size(data.len()),
        vk::BufferUsageFlags::STORAGE_BUFFER | vk::BufferUsageFlags::TRANSFER_SRC,
        gpu_allocator::MemoryLocation::CpuToGpu,
        name,
    );
    write_buffer_data(&buf, &data);
    buf
}

fn write_buffer_data<T: bytemuck::Pod>(buffer: &AllocatedBuffer, data: &[T]) {
    if data.is_empty() {
        return;
    }
    if let Some(ref alloc) = buffer.allocation
        && let Some(mapped) = alloc.mapped_ptr()
    {
        let bytes = bytemuck::cast_slice(data);
        unsafe {
            std::ptr::copy_nonoverlapping(bytes.as_ptr(), mapped.as_ptr() as *mut u8, bytes.len());
        }
    }
}

fn read_mapped_particles(buffer: &AllocatedBuffer, count: usize) -> Vec<Particle> {
    if count == 0 {
        return Vec::new();
    }
    let Some(ref alloc) = buffer.allocation else {
        return Vec::new();
    };
    let Some(mapped) = alloc.mapped_ptr() else {
        return Vec::new();
    };
    let gpu_particles: &[GpuParticle] = unsafe {
        std::slice::from_raw_parts(mapped.as_ptr() as *const GpuParticle, count)
    };
    gpu_particles.iter().copied().map(GpuParticle::to_cpu).collect()
}

/// Creates the shared descriptor set layout for particle SSBO access.
pub fn create_particle_descriptor_set_layout(device: &ash::Device) -> vk::DescriptorSetLayout {
    let binding = vk::DescriptorSetLayoutBinding::default()
        .binding(0)
        .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
        .descriptor_count(1)
        .stage_flags(vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::COMPUTE);
    let bindings = [binding];
    let ci = vk::DescriptorSetLayoutCreateInfo::default().bindings(&bindings);
    unsafe { device.create_descriptor_set_layout(&ci, None) }.unwrap()
}

fn create_compute_pipeline_layout(
    device: &ash::Device,
    descriptor_set_layout: vk::DescriptorSetLayout,
) -> vk::PipelineLayout {
    let push_range = vk::PushConstantRange {
        stage_flags: vk::ShaderStageFlags::COMPUTE,
        offset: 0,
        size: std::mem::size_of::<ComputePushConstants>() as u32,
    };
    let ranges = [push_range];
    let set_layouts = [descriptor_set_layout];
    let ci = vk::PipelineLayoutCreateInfo::default()
        .set_layouts(&set_layouts)
        .push_constant_ranges(&ranges);
    unsafe { device.create_pipeline_layout(&ci, None) }.unwrap()
}

fn create_compute_pipeline(
    device: &ash::Device,
    layout: vk::PipelineLayout,
) -> vk::Pipeline {
    let spv = include_bytes!(concat!(
        env!("OUT_DIR"),
        "/shaders/particles_compute.comp.spv"
    ));
    let module = create_shader_module(device, spv);
    let entry = c"main";
    let stage = vk::PipelineShaderStageCreateInfo::default()
        .stage(vk::ShaderStageFlags::COMPUTE)
        .module(module)
        .name(entry);
    let stages = [stage];
    let ci = vk::ComputePipelineCreateInfo::default()
        .stage(stages[0])
        .layout(layout);
    let pipelines = unsafe { device.create_compute_pipelines(vk::PipelineCache::null(), &[ci], None) }
        .unwrap();
    unsafe {
        device.destroy_shader_module(module, None);
    }
    pipelines[0]
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
