use crate::simulation::{Particle, G, EPSILON, LIGHT_SPEED};
use crate::ui_state::SimulationType;
use ash::vk;
use glam::DVec3;
use gpu_allocator::vulkan::Allocator;
use std::sync::{Arc, Mutex};
use vulkanvil::{create_shader_module, AllocatedBuffer};

const WORKGROUP_SIZE: u32 = 64;
const EPSILON_F32: f32 = EPSILON as f32;

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

    pub fn from_display(position: [f32; 3], color: [f32; 4]) -> Self {
        Self {
            position: [position[0], position[1], position[2], 0.0],
            velocity: [0.0; 4],
            attrs: [0.0; 4],
            color,
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
    light_speed_per_scale: f32,
    sim_type: u32,
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
    buffer_capacity: usize,
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
        let buffer_capacity = particles.len().max(1);
        let particle_buffer = create_particle_storage_buffer(
            &device,
            &allocator,
            buffer_capacity,
            "gpu_particle_ssbo",
        );
        let descriptor_set =
            allocate_descriptor_set(&device, descriptor_pool, descriptor_set_layout);
        update_particle_descriptor(
            &device,
            descriptor_set,
            particle_buffer.buffer,
            particle_buffer_size(buffer_capacity),
        );

        let sim = Self {
            device,
            allocator,
            particle_buffer,
            descriptor_pool,
            descriptor_set,
            compute_pipeline,
            compute_layout,
            particle_count,
            buffer_capacity,
        };
        if !particles.is_empty() {
            sim.write_cpu_particles(particles);
        }
        sim
    }

    pub fn particle_count(&self) -> u32 {
        self.particle_count
    }

    pub fn descriptor_set(&self) -> vk::DescriptorSet {
        self.descriptor_set
    }

    /// Uploads CPU simulation particles into the mapped SSBO.
    pub fn upload_from_cpu(&mut self, particles: &[Particle]) {
        self.particle_count = particles.len() as u32;
        if particles.is_empty() {
            return;
        }
        self.ensure_buffer_capacity(particles.len());
        self.write_cpu_particles(particles);
    }

    /// Uploads display-only point data (Graph3D and similar viewers).
    pub fn upload_display_points(
        &mut self,
        positions: &[[f32; 3]],
        colors: &[[f32; 4]],
    ) {
        self.particle_count = positions.len() as u32;
        if positions.is_empty() {
            return;
        }
        self.ensure_buffer_capacity(positions.len());
        self.write_display_points(positions, colors);
    }

    /// Records compute dispatches that advance `steps` simulation steps on the GPU.
    ///
    /// Each step runs phase 0 (position integration) then phase 1 (velocity update),
    /// keeping the GPU step count in lockstep with the simulation frame counter.
    /// `scale` is only consulted for the relativistic simulation types.
    pub fn dispatch(
        &self,
        command_buffer: vk::CommandBuffer,
        simulation_type: SimulationType,
        delta_seconds: f64,
        scale: f64,
        steps: u32,
    ) {
        if self.particle_count == 0 || steps == 0 {
            return;
        }

        let step = ComputePushConstants {
            particle_count: self.particle_count,
            delta_seconds: delta_seconds as f32,
            gravity_dt: (G * delta_seconds) as f32,
            epsilon: EPSILON_F32,
            light_speed_per_scale: (LIGHT_SPEED / scale) as f32,
            sim_type: simulation_type.gpu_code(),
            phase: 0,
        };
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

            for i in 0..steps {
                self.dispatch_phase(command_buffer, workgroups, step);
                shader_rw_barrier(
                    &self.device,
                    command_buffer,
                    vk::PipelineStageFlags::COMPUTE_SHADER,
                    vk::PipelineStageFlags::COMPUTE_SHADER,
                );

                self.dispatch_phase(
                    command_buffer,
                    workgroups,
                    ComputePushConstants { phase: 1, ..step },
                );

                // Between steps the next dispatch reads the buffer in COMPUTE again;
                // only the final step hands the data off to the vertex stage.
                let dst_stage = if i + 1 == steps {
                    vk::PipelineStageFlags::VERTEX_SHADER
                } else {
                    vk::PipelineStageFlags::COMPUTE_SHADER
                };
                shader_rw_barrier(
                    &self.device,
                    command_buffer,
                    vk::PipelineStageFlags::COMPUTE_SHADER,
                    dst_stage,
                );
            }
        }
    }

    /// Copies GPU particle data back to CPU for snapshot export.
    pub fn readback_to_cpu(&self) -> Vec<Particle> {
        read_mapped_particles(&self.particle_buffer, self.particle_count as usize)
    }

    fn ensure_buffer_capacity(&mut self, count: usize) {
        if count <= self.buffer_capacity {
            return;
        }
        let capacity = count.max(1);
        let alloc = Arc::clone(&self.allocator);
        let old = std::mem::replace(
            &mut self.particle_buffer,
            create_particle_storage_buffer(&self.device, &alloc, capacity, "gpu_particle_ssbo"),
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

    fn write_cpu_particles(&self, particles: &[Particle]) {
        let Some(dst) = mapped_particle_slice_mut(&self.particle_buffer, particles.len()) else {
            return;
        };
        for (slot, particle) in dst.iter_mut().zip(particles) {
            *slot = GpuParticle::from_cpu(particle);
        }
    }

    fn write_display_points(&self, positions: &[[f32; 3]], colors: &[[f32; 4]]) {
        let Some(dst) = mapped_particle_slice_mut(&self.particle_buffer, positions.len()) else {
            return;
        };
        for (slot, (position, color)) in dst.iter_mut().zip(positions.iter().zip(colors)) {
            *slot = GpuParticle::from_display(*position, *color);
        }
    }

    unsafe fn dispatch_phase(
        &self,
        command_buffer: vk::CommandBuffer,
        workgroups: u32,
        push: ComputePushConstants,
    ) {
        unsafe {
            self.device.cmd_push_constants(
                command_buffer,
                self.compute_layout,
                vk::ShaderStageFlags::COMPUTE,
                0,
                bytemuck::bytes_of(&push),
            );
            self.device
                .cmd_dispatch(command_buffer, workgroups, 1, 1);
        }
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
    capacity: usize,
    name: &str,
) -> AllocatedBuffer {
    AllocatedBuffer::new(
        device,
        allocator,
        particle_buffer_size(capacity),
        vk::BufferUsageFlags::STORAGE_BUFFER | vk::BufferUsageFlags::TRANSFER_SRC,
        gpu_allocator::MemoryLocation::CpuToGpu,
        name,
    )
}

fn mapped_particle_slice_mut(
    buffer: &AllocatedBuffer,
    count: usize,
) -> Option<&mut [GpuParticle]> {
    if count == 0 {
        return Some(&mut []);
    }
    let alloc = buffer.allocation.as_ref()?;
    let mapped = alloc.mapped_ptr()?;
    Some(unsafe {
        std::slice::from_raw_parts_mut(mapped.as_ptr() as *mut GpuParticle, count)
    })
}

fn read_mapped_particles(buffer: &AllocatedBuffer, count: usize) -> Vec<Particle> {
    if count == 0 {
        return Vec::new();
    }
    let Some(alloc) = buffer.allocation.as_ref() else {
        return Vec::new();
    };
    let Some(mapped) = alloc.mapped_ptr() else {
        return Vec::new();
    };
    let gpu_particles: &[GpuParticle] =
        unsafe { std::slice::from_raw_parts(mapped.as_ptr() as *const GpuParticle, count) };
    gpu_particles.iter().copied().map(GpuParticle::to_cpu).collect()
}

unsafe fn shader_rw_barrier(
    device: &ash::Device,
    command_buffer: vk::CommandBuffer,
    src_stage: vk::PipelineStageFlags,
    dst_stage: vk::PipelineStageFlags,
) {
    unsafe {
        let barrier = vk::MemoryBarrier::default()
            .src_access_mask(vk::AccessFlags::SHADER_WRITE)
            .dst_access_mask(vk::AccessFlags::SHADER_READ);
        device.cmd_pipeline_barrier(
            command_buffer,
            src_stage,
            dst_stage,
            vk::DependencyFlags::empty(),
            &[barrier],
            &[],
            &[],
        );
    }
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
    let ci = vk::ComputePipelineCreateInfo::default()
        .stage(stage)
        .layout(layout);
    let pipelines =
        unsafe { device.create_compute_pipelines(vk::PipelineCache::null(), &[ci], None) }
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
