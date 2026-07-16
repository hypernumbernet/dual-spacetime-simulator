//! Procedural grass tile (minecraft-clone style) and Vulkan upload.

use ash::vk;
use gpu_allocator::MemoryLocation;
use gpu_allocator::vulkan::Allocator;
use std::sync::Mutex;
use vulkanvil::{AllocatedBuffer, AllocatedImage, VulkanBase};

/// Grass tile resolution (pixels). Matches minecraft-clone block tiles.
pub const GRASS_TILE_PX: u32 = 16;

pub struct Texture {
    pub image: AllocatedImage,
    pub sampler: vk::Sampler,
}

impl Texture {
    pub fn destroy(&mut self, device: &ash::Device, allocator: &Mutex<Allocator>) {
        unsafe { device.destroy_sampler(self.sampler, None) };
        self.image.destroy(device, allocator);
    }
}

fn h(x: u32, y: u32, salt: u32) -> u32 {
    let mut v = x
        .wrapping_mul(0x1F1F_1F1F)
        .wrapping_add(y.wrapping_mul(0x8DA6_B343))
        .wrapping_add(salt.wrapping_mul(0x2545_F491));
    v ^= v >> 13;
    v = v.wrapping_mul(0x5BD1_E995);
    v ^= v >> 15;
    v
}

fn jitter(x: u32, y: u32, salt: u32, amt: f32) -> f32 {
    ((h(x, y, salt) & 0xFF) as f32 / 255.0 - 0.5) * 2.0 * amt
}

/// Generates a 16×16 RGBA8 grass-top tile (same palette/noise as minecraft-clone).
pub fn generate_grass_pixels() -> Vec<u8> {
    let grass = [0.30, 0.52, 0.22];
    let mut px = vec![0u8; (GRASS_TILE_PX * GRASS_TILE_PX * 4) as usize];
    for y in 0..GRASS_TILE_PX {
        for x in 0..GRASS_TILE_PX {
            let d = jitter(x, y, 1, 0.10);
            // Sparse darker blades for a bit more structure.
            let blade = if (h(x, y, 3) & 15) == 0 { -0.08 } else { 0.0 };
            let rgb = [
                (grass[0] + d + blade).clamp(0.0, 1.0),
                (grass[1] + d + blade).clamp(0.0, 1.0),
                (grass[2] + d + blade * 0.5).clamp(0.0, 1.0),
            ];
            let i = ((y * GRASS_TILE_PX + x) * 4) as usize;
            px[i] = (rgb[0] * 255.0) as u8;
            px[i + 1] = (rgb[1] * 255.0) as u8;
            px[i + 2] = (rgb[2] * 255.0) as u8;
            px[i + 3] = 255;
        }
    }
    px
}

/// Uploads the grass tile with a REPEAT + NEAREST sampler (pixel-art tiling).
pub fn create_grass_texture(vb: &VulkanBase, allocator: &Mutex<Allocator>) -> Texture {
    let pixels = generate_grass_pixels();
    let width = GRASS_TILE_PX;
    let height = GRASS_TILE_PX;
    let device = &vb.device;

    let staging = AllocatedBuffer::new(
        device,
        allocator,
        pixels.len() as u64,
        vk::BufferUsageFlags::TRANSFER_SRC,
        MemoryLocation::CpuToGpu,
        "grass-staging",
    );
    if let Some(ref alloc) = staging.allocation
        && let Some(mapped) = alloc.mapped_ptr()
    {
        unsafe {
            std::ptr::copy_nonoverlapping(
                pixels.as_ptr(),
                mapped.as_ptr() as *mut u8,
                pixels.len(),
            );
        }
    }

    let mut image = AllocatedImage::new(
        device,
        allocator,
        width,
        height,
        vk::Format::R8G8B8A8_UNORM,
        vk::ImageUsageFlags::TRANSFER_DST | vk::ImageUsageFlags::SAMPLED,
        vk::ImageAspectFlags::COLOR,
        "grass",
    );

    let alloc_ci = vk::CommandBufferAllocateInfo::default()
        .command_pool(vb.command_pool)
        .level(vk::CommandBufferLevel::PRIMARY)
        .command_buffer_count(1);
    let cb = unsafe { device.allocate_command_buffers(&alloc_ci) }.unwrap()[0];
    let begin =
        vk::CommandBufferBeginInfo::default().flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT);
    unsafe { device.begin_command_buffer(cb, &begin) }.unwrap();

    let full_range = vk::ImageSubresourceRange {
        aspect_mask: vk::ImageAspectFlags::COLOR,
        base_mip_level: 0,
        level_count: 1,
        base_array_layer: 0,
        layer_count: 1,
    };

    let to_transfer = vk::ImageMemoryBarrier::default()
        .old_layout(vk::ImageLayout::UNDEFINED)
        .new_layout(vk::ImageLayout::TRANSFER_DST_OPTIMAL)
        .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
        .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
        .image(image.image)
        .subresource_range(full_range)
        .src_access_mask(vk::AccessFlags::empty())
        .dst_access_mask(vk::AccessFlags::TRANSFER_WRITE);
    unsafe {
        device.cmd_pipeline_barrier(
            cb,
            vk::PipelineStageFlags::TOP_OF_PIPE,
            vk::PipelineStageFlags::TRANSFER,
            vk::DependencyFlags::empty(),
            &[],
            &[],
            &[to_transfer],
        );
    }

    let region = vk::BufferImageCopy {
        buffer_offset: 0,
        buffer_row_length: 0,
        buffer_image_height: 0,
        image_subresource: vk::ImageSubresourceLayers {
            aspect_mask: vk::ImageAspectFlags::COLOR,
            mip_level: 0,
            base_array_layer: 0,
            layer_count: 1,
        },
        image_offset: vk::Offset3D { x: 0, y: 0, z: 0 },
        image_extent: vk::Extent3D {
            width,
            height,
            depth: 1,
        },
    };
    unsafe {
        device.cmd_copy_buffer_to_image(
            cb,
            staging.buffer,
            image.image,
            vk::ImageLayout::TRANSFER_DST_OPTIMAL,
            &[region],
        );
    }

    let to_shader = vk::ImageMemoryBarrier::default()
        .old_layout(vk::ImageLayout::TRANSFER_DST_OPTIMAL)
        .new_layout(vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL)
        .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
        .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
        .image(image.image)
        .subresource_range(full_range)
        .src_access_mask(vk::AccessFlags::TRANSFER_WRITE)
        .dst_access_mask(vk::AccessFlags::SHADER_READ);
    unsafe {
        device.cmd_pipeline_barrier(
            cb,
            vk::PipelineStageFlags::TRANSFER,
            vk::PipelineStageFlags::FRAGMENT_SHADER,
            vk::DependencyFlags::empty(),
            &[],
            &[],
            &[to_shader],
        );
    }

    unsafe { device.end_command_buffer(cb) }.unwrap();
    let fence = unsafe { device.create_fence(&vk::FenceCreateInfo::default(), None) }.unwrap();
    let cbs = [cb];
    let submit = vk::SubmitInfo::default().command_buffers(&cbs);
    unsafe {
        device
            .queue_submit(vb.graphics_queue, &[submit], fence)
            .unwrap();
        device.wait_for_fences(&[fence], true, u64::MAX).unwrap();
        device.destroy_fence(fence, None);
        device.free_command_buffers(vb.command_pool, &cbs);
    }
    staging.destroy(device, allocator);

    // NEAREST + REPEAT: minecraft-style pixel grass that tiles forever.
    let sampler_ci = vk::SamplerCreateInfo::default()
        .mag_filter(vk::Filter::NEAREST)
        .min_filter(vk::Filter::NEAREST)
        .mipmap_mode(vk::SamplerMipmapMode::NEAREST)
        .address_mode_u(vk::SamplerAddressMode::REPEAT)
        .address_mode_v(vk::SamplerAddressMode::REPEAT)
        .address_mode_w(vk::SamplerAddressMode::REPEAT)
        .anisotropy_enable(false)
        .min_lod(0.0)
        .max_lod(0.0);
    let sampler = unsafe { device.create_sampler(&sampler_ci, None) }.unwrap();

    let _ = &mut image;
    Texture { image, sampler }
}
