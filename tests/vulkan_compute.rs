//! Headless dispatch of `tree_compute.comp`. Run: `cargo test -- --ignored`

mod common;

use ash::vk;
use ash::Device;
use bytemuck;
use dual_spacetime_simulator::shader_blobs;
use dual_spacetime_simulator::tree::{GpuTreeComputeParams, TreeParams};
use gpu_allocator::vulkan::{AllocationCreateDesc, AllocationScheme, Allocator};
use gpu_allocator::MemoryLocation;
use std::io::Cursor;
use std::ptr;
use std::sync::Mutex;

const MAX_VERTICES: usize = 65536;
const MAX_BRANCHES: u32 = 512;

struct GpuBuffer {
    buffer: vk::Buffer,
    allocation: Option<gpu_allocator::vulkan::Allocation>,
}

impl GpuBuffer {
    fn new(
        device: &Device,
        allocator: &Mutex<Allocator>,
        size: u64,
        usage: vk::BufferUsageFlags,
        location: MemoryLocation,
        name: &'static str,
    ) -> Self {
        let buffer_ci = vk::BufferCreateInfo::default()
            .size(size.max(1))
            .usage(usage)
            .sharing_mode(vk::SharingMode::EXCLUSIVE);
        let buffer = unsafe { device.create_buffer(&buffer_ci, None) }.unwrap();
        let requirements = unsafe { device.get_buffer_memory_requirements(buffer) };
        let allocation = allocator
            .lock()
            .unwrap()
            .allocate(&AllocationCreateDesc {
                name,
                requirements,
                location,
                linear: true,
                allocation_scheme: AllocationScheme::GpuAllocatorManaged,
            })
            .unwrap();
        unsafe {
            device
                .bind_buffer_memory(buffer, allocation.memory(), allocation.offset())
                .unwrap();
        }
        Self {
            buffer,
            allocation: Some(allocation),
        }
    }

    fn destroy(mut self, device: &Device, allocator: &Mutex<Allocator>) {
        unsafe { device.destroy_buffer(self.buffer, None) };
        if let Some(a) = self.allocation.take() {
            allocator.lock().unwrap().free(a).unwrap();
        }
    }
}

fn create_shader_module(device: &Device, spv: &[u8]) -> vk::ShaderModule {
    let code = ash::util::read_spv(&mut Cursor::new(spv)).unwrap();
    let ci = vk::ShaderModuleCreateInfo::default().code(&code);
    unsafe { device.create_shader_module(&ci, None) }.unwrap()
}

fn create_compute_pipeline(
    device: &Device,
) -> (vk::DescriptorSetLayout, vk::PipelineLayout, vk::Pipeline) {
    let bindings = [
        vk::DescriptorSetLayoutBinding::default()
            .binding(0)
            .descriptor_type(vk::DescriptorType::UNIFORM_BUFFER)
            .descriptor_count(1)
            .stage_flags(vk::ShaderStageFlags::COMPUTE),
        vk::DescriptorSetLayoutBinding::default()
            .binding(1)
            .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
            .descriptor_count(1)
            .stage_flags(vk::ShaderStageFlags::COMPUTE),
        vk::DescriptorSetLayoutBinding::default()
            .binding(2)
            .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
            .descriptor_count(1)
            .stage_flags(vk::ShaderStageFlags::COMPUTE),
        vk::DescriptorSetLayoutBinding::default()
            .binding(3)
            .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
            .descriptor_count(1)
            .stage_flags(vk::ShaderStageFlags::COMPUTE),
        vk::DescriptorSetLayoutBinding::default()
            .binding(4)
            .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
            .descriptor_count(1)
            .stage_flags(vk::ShaderStageFlags::COMPUTE),
    ];
    let ds_layout_ci =
        vk::DescriptorSetLayoutCreateInfo::default().bindings(&bindings);
    let ds_layout = unsafe { device.create_descriptor_set_layout(&ds_layout_ci, None) }.unwrap();

    let set_layouts = [ds_layout];
    let pl_ci = vk::PipelineLayoutCreateInfo::default().set_layouts(&set_layouts);
    let pipeline_layout = unsafe { device.create_pipeline_layout(&pl_ci, None) }.unwrap();

    let cs_mod = create_shader_module(device, shader_blobs::TREE_COMPUTE);
    let entry = c"main";
    let stage = vk::PipelineShaderStageCreateInfo::default()
        .stage(vk::ShaderStageFlags::COMPUTE)
        .module(cs_mod)
        .name(entry);
    let ci = vk::ComputePipelineCreateInfo::default()
        .stage(stage)
        .layout(pipeline_layout);

    let pipeline = unsafe {
        device
            .create_compute_pipelines(vk::PipelineCache::null(), &[ci], None)
            .unwrap()[0]
    };
    unsafe { device.destroy_shader_module(cs_mod, None) };

    (ds_layout, pipeline_layout, pipeline)
}

fn create_descriptor_pool(device: &Device) -> vk::DescriptorPool {
    let pool_sizes = [
        vk::DescriptorPoolSize {
            ty: vk::DescriptorType::UNIFORM_BUFFER,
            descriptor_count: 1,
        },
        vk::DescriptorPoolSize {
            ty: vk::DescriptorType::STORAGE_BUFFER,
            descriptor_count: 4,
        },
    ];
    let ci = vk::DescriptorPoolCreateInfo::default()
        .pool_sizes(&pool_sizes)
        .max_sets(1)
        .flags(vk::DescriptorPoolCreateFlags::FREE_DESCRIPTOR_SET);
    unsafe { device.create_descriptor_pool(&ci, None) }.unwrap()
}

fn write_descriptor_set(
    device: &Device,
    pool: vk::DescriptorPool,
    layout: vk::DescriptorSetLayout,
    params: &GpuBuffer,
    positions: &GpuBuffer,
    normals: &GpuBuffer,
    colors: &GpuBuffer,
    counter: &GpuBuffer,
) -> vk::DescriptorSet {
    let layouts = [layout];
    let alloc_ci = vk::DescriptorSetAllocateInfo::default()
        .descriptor_pool(pool)
        .set_layouts(&layouts);
    let sets = unsafe { device.allocate_descriptor_sets(&alloc_ci) }.unwrap();
    let set = sets[0];

    let params_info = vk::DescriptorBufferInfo::default()
        .buffer(params.buffer)
        .offset(0)
        .range(std::mem::size_of::<GpuTreeComputeParams>() as u64);
    let pos_info = vk::DescriptorBufferInfo::default()
        .buffer(positions.buffer)
        .offset(0)
        .range(vk::WHOLE_SIZE);
    let norm_info = vk::DescriptorBufferInfo::default()
        .buffer(normals.buffer)
        .offset(0)
        .range(vk::WHOLE_SIZE);
    let col_info = vk::DescriptorBufferInfo::default()
        .buffer(colors.buffer)
        .offset(0)
        .range(vk::WHOLE_SIZE);
    let counter_info = vk::DescriptorBufferInfo::default()
        .buffer(counter.buffer)
        .offset(0)
        .range(4);

    let params_infos = [params_info];
    let pos_infos = [pos_info];
    let norm_infos = [norm_info];
    let col_infos = [col_info];
    let counter_infos = [counter_info];

    let writes = [
        vk::WriteDescriptorSet::default()
            .dst_set(set)
            .dst_binding(0)
            .descriptor_type(vk::DescriptorType::UNIFORM_BUFFER)
            .buffer_info(&params_infos),
        vk::WriteDescriptorSet::default()
            .dst_set(set)
            .dst_binding(1)
            .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
            .buffer_info(&pos_infos),
        vk::WriteDescriptorSet::default()
            .dst_set(set)
            .dst_binding(2)
            .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
            .buffer_info(&norm_infos),
        vk::WriteDescriptorSet::default()
            .dst_set(set)
            .dst_binding(3)
            .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
            .buffer_info(&col_infos),
        vk::WriteDescriptorSet::default()
            .dst_set(set)
            .dst_binding(4)
            .descriptor_type(vk::DescriptorType::STORAGE_BUFFER)
            .buffer_info(&counter_infos),
    ];
    unsafe { device.update_descriptor_sets(&writes, &[]) };
    set
}

#[test]
#[ignore = "requires Vulkan device"]
fn tree_compute_shader_writes_vertices() {
    let v = common::try_create_headless_vulkan().expect("vulkan");
    let alloc = v.allocator.as_ref().unwrap();

    let compute_params = GpuTreeComputeParams::from(TreeParams::default());
    let params_buf = GpuBuffer::new(
        &v.device,
        alloc,
        std::mem::size_of::<GpuTreeComputeParams>() as u64,
        vk::BufferUsageFlags::UNIFORM_BUFFER,
        MemoryLocation::CpuToGpu,
        "ub_params",
    );
    if let Some(ref a) = params_buf.allocation {
        if let Some(mapped) = a.mapped_ptr() {
            unsafe {
                ptr::copy_nonoverlapping(
                    bytemuck::bytes_of(&compute_params).as_ptr(),
                    mapped.as_ptr() as *mut u8,
                    std::mem::size_of::<GpuTreeComputeParams>(),
                );
            }
        }
    }

    let pos_size = (MAX_VERTICES * 16) as u64;
    let positions_buf = GpuBuffer::new(
        &v.device,
        alloc,
        pos_size,
        vk::BufferUsageFlags::STORAGE_BUFFER,
        MemoryLocation::GpuToCpu,
        "ssbo_pos",
    );
    let normals_buf = GpuBuffer::new(
        &v.device,
        alloc,
        pos_size,
        vk::BufferUsageFlags::STORAGE_BUFFER,
        MemoryLocation::GpuToCpu,
        "ssbo_norm",
    );
    let colors_buf = GpuBuffer::new(
        &v.device,
        alloc,
        pos_size,
        vk::BufferUsageFlags::STORAGE_BUFFER,
        MemoryLocation::GpuToCpu,
        "ssbo_col",
    );
    let counter_buf = GpuBuffer::new(
        &v.device,
        alloc,
        4,
        vk::BufferUsageFlags::STORAGE_BUFFER,
        MemoryLocation::CpuToGpu,
        "ssbo_counter",
    );
    if let Some(ref a) = counter_buf.allocation {
        if let Some(mapped) = a.mapped_ptr() {
            unsafe {
                ptr::write_bytes(mapped.as_ptr() as *mut u8, 0, 4);
            }
        }
    }

    let (ds_layout, pipeline_layout, pipeline) = create_compute_pipeline(&v.device);
    let desc_pool = create_descriptor_pool(&v.device);
    let desc_set = write_descriptor_set(
        &v.device,
        desc_pool,
        ds_layout,
        &params_buf,
        &positions_buf,
        &normals_buf,
        &colors_buf,
        &counter_buf,
    );

    let alloc_ci = vk::CommandBufferAllocateInfo::default()
        .command_pool(v.command_pool)
        .level(vk::CommandBufferLevel::PRIMARY)
        .command_buffer_count(1);
    let cb = unsafe { v.device.allocate_command_buffers(&alloc_ci) }.unwrap()[0];
    let begin_ci =
        vk::CommandBufferBeginInfo::default().flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT);
    unsafe {
        v.device.begin_command_buffer(cb, &begin_ci).unwrap();
        v.device
            .cmd_bind_pipeline(cb, vk::PipelineBindPoint::COMPUTE, pipeline);
        v.device.cmd_bind_descriptor_sets(
            cb,
            vk::PipelineBindPoint::COMPUTE,
            pipeline_layout,
            0,
            &[desc_set],
            &[],
        );
        let workgroup_count = (MAX_BRANCHES + 63) / 64;
        v.device.cmd_dispatch(cb, workgroup_count, 1, 1);
        v.device.end_command_buffer(cb).unwrap();
    }

    let fence_ci = vk::FenceCreateInfo::default();
    let fence = unsafe { v.device.create_fence(&fence_ci, None) }.unwrap();
    let command_buffers = [cb];
    let submit = vk::SubmitInfo::default().command_buffers(&command_buffers);
    unsafe {
        v.device
            .queue_submit(v.graphics_queue, &[submit], fence)
            .unwrap();
        v.device
            .wait_for_fences(&[fence], true, u64::MAX)
            .unwrap();
        v.device.destroy_fence(fence, None);
        v.device
            .free_command_buffers(v.command_pool, &[cb]);
    }

    let vertex_count = counter_buf
        .allocation
        .as_ref()
        .and_then(|a| a.mapped_ptr())
        .map(|mapped| unsafe { *(mapped.as_ptr() as *const u32) })
        .unwrap_or(0);
    assert!(
        vertex_count >= 3,
        "expected some triangle vertices, got {vertex_count}"
    );

    unsafe {
        v.device.destroy_pipeline(pipeline, None);
        v.device.destroy_pipeline_layout(pipeline_layout, None);
        v.device.destroy_descriptor_set_layout(ds_layout, None);
        v.device
            .reset_descriptor_pool(desc_pool, vk::DescriptorPoolResetFlags::empty())
            .unwrap();
        v.device.destroy_descriptor_pool(desc_pool, None);
    }

    params_buf.destroy(&v.device, alloc);
    positions_buf.destroy(&v.device, alloc);
    normals_buf.destroy(&v.device, alloc);
    colors_buf.destroy(&v.device, alloc);
    counter_buf.destroy(&v.device, alloc);
}
