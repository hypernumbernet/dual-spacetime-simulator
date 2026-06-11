//! Procedural 64x64 RGBA texture atlas (8 tiles of 16px) and its Vulkan upload.

use ash::vk;
use gpu_allocator::vulkan::Allocator;
use gpu_allocator::MemoryLocation;
use std::sync::Mutex;
use vulkanvil::{AllocatedBuffer, AllocatedImage, VulkanBase};

pub const ATLAS_PX: u32 = 64;
const TILE: u32 = 16;
const TILES_PER_ROW: u32 = 4;

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

/// Per-pixel hash for deterministic texture noise.
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

/// Brightness jitter in [-amt, amt] from a per-pixel hash.
fn jitter(x: u32, y: u32, salt: u32, amt: f32) -> f32 {
    ((h(x, y, salt) & 0xFF) as f32 / 255.0 - 0.5) * 2.0 * amt
}

fn put(px: &mut [u8], x: u32, y: u32, rgb: [f32; 3]) {
    let i = ((y * ATLAS_PX + x) * 4) as usize;
    px[i] = (rgb[0].clamp(0.0, 1.0) * 255.0) as u8;
    px[i + 1] = (rgb[1].clamp(0.0, 1.0) * 255.0) as u8;
    px[i + 2] = (rgb[2].clamp(0.0, 1.0) * 255.0) as u8;
    px[i + 3] = 255;
}

/// Generates the 64x64 RGBA8 atlas pixels (row-major, 4 bytes/pixel).
pub fn generate_atlas_pixels() -> Vec<u8> {
    let mut px = vec![0u8; (ATLAS_PX * ATLAS_PX * 4) as usize];

    // base colors
    let grass = [0.30, 0.52, 0.22];
    let dirt = [0.47, 0.33, 0.22];
    let stone = [0.49, 0.49, 0.49];
    let sand = [0.85, 0.80, 0.62];
    let gravel = [0.45, 0.42, 0.40];
    let snow = [0.93, 0.95, 0.99];
    let bark = [0.40, 0.28, 0.16];
    let wood = [0.60, 0.45, 0.28];
    let leaf = [0.21, 0.40, 0.17];
    let water = [0.16, 0.34, 0.52];

    for tile in 0u32..11 {
        let ox = (tile % TILES_PER_ROW) * TILE;
        let oy = (tile / TILES_PER_ROW) * TILE;
        for ty in 0..TILE {
            for tx in 0..TILE {
                let x = ox + tx;
                let y = oy + ty;
                let color = match tile {
                    0 => add(grass, jitter(x, y, 1, 0.10)), // grass_top
                    1 => {
                        // grass_side: dirt with ragged green top band
                        let band = 4 + ((h(x, 0, 7) & 1) as u32);
                        if ty < band {
                            add(grass, jitter(x, y, 1, 0.10))
                        } else {
                            add(dirt, jitter(x, y, 2, 0.10))
                        }
                    }
                    2 => add(dirt, jitter(x, y, 2, 0.12)), // dirt
                    3 => add(stone, jitter(x / 2, y / 2, 3, 0.10)), // stone (chunky)
                    4 => add(sand, jitter(x, y, 4, 0.06)), // sand
                    5 => {
                        // bark: vertical stripes
                        let s = (h(tx, 0, 9) & 1) as f32 * 0.08 - 0.04;
                        add(bark, s + jitter(x, y, 5, 0.05))
                    }
                    6 => {
                        // wood rings
                        let cx = tx as i32 - 8;
                        let cy = ty as i32 - 8;
                        let r = cx.abs().max(cy.abs()) % 3;
                        add(wood, r as f32 * 0.06 - 0.06)
                    }
                    7 => {
                        // leaves: green with dark speckles
                        if (h(x, y, 11) & 7) == 0 {
                            add(leaf, -0.18)
                        } else {
                            add(leaf, jitter(x, y, 11, 0.10))
                        }
                    }
                    8 => add(snow, jitter(x, y, 13, 0.05)), // snow
                    9 => add(water, jitter(x, y, 17, 0.06)), // water surface tint
                    _ => {
                        // gravel: chunky pebbles
                        let g = jitter(x / 2, y / 2, 19, 0.14) + jitter(x, y, 23, 0.05);
                        add(gravel, g)
                    }
                };
                put(&mut px, x, y, color);
            }
        }
    }
    px
}

fn add(c: [f32; 3], d: f32) -> [f32; 3] {
    [c[0] + d, c[1] + d, c[2] + d]
}

/// Generates the atlas and uploads it into a sampled GPU image (NEAREST sampler).
pub fn create_atlas_texture(vb: &VulkanBase, allocator: &Mutex<Allocator>) -> Texture {
    create_texture_rgba(vb, allocator, &generate_atlas_pixels(), ATLAS_PX, ATLAS_PX, "atlas")
}

/// Uploads RGBA8 pixels into a sampled GPU image via a one-time transfer (NEAREST sampler).
pub fn create_texture_rgba(
    vb: &VulkanBase,
    allocator: &Mutex<Allocator>,
    pixels: &[u8],
    width: u32,
    height: u32,
    name: &str,
) -> Texture {
    let device = &vb.device;

    // Staging buffer.
    let staging = AllocatedBuffer::new(
        device,
        allocator,
        pixels.len() as u64,
        vk::BufferUsageFlags::TRANSFER_SRC,
        MemoryLocation::CpuToGpu,
        name,
    );
    if let Some(ref alloc) = staging.allocation {
        if let Some(mapped) = alloc.mapped_ptr() {
            unsafe {
                std::ptr::copy_nonoverlapping(
                    pixels.as_ptr(),
                    mapped.as_ptr() as *mut u8,
                    pixels.len(),
                );
            }
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
        name,
    );

    // One-time command buffer for transfer + layout transitions.
    let alloc_ci = vk::CommandBufferAllocateInfo::default()
        .command_pool(vb.command_pool)
        .level(vk::CommandBufferLevel::PRIMARY)
        .command_buffer_count(1);
    let cb = unsafe { device.allocate_command_buffers(&alloc_ci) }.unwrap()[0];

    let begin = vk::CommandBufferBeginInfo::default()
        .flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT);
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

    let fence_ci = vk::FenceCreateInfo::default();
    let fence = unsafe { device.create_fence(&fence_ci, None) }.unwrap();
    let cbs = [cb];
    let submit = vk::SubmitInfo::default().command_buffers(&cbs);
    unsafe {
        device.queue_submit(vb.graphics_queue, &[submit], fence).unwrap();
        device.wait_for_fences(&[fence], true, u64::MAX).unwrap();
        device.destroy_fence(fence, None);
        device.free_command_buffers(vb.command_pool, &cbs);
    }
    staging.destroy(device, allocator);

    // Sampler: NEAREST for the pixel-art look.
    let sampler_ci = vk::SamplerCreateInfo::default()
        .mag_filter(vk::Filter::NEAREST)
        .min_filter(vk::Filter::NEAREST)
        .mipmap_mode(vk::SamplerMipmapMode::NEAREST)
        .address_mode_u(vk::SamplerAddressMode::CLAMP_TO_EDGE)
        .address_mode_v(vk::SamplerAddressMode::CLAMP_TO_EDGE)
        .address_mode_w(vk::SamplerAddressMode::CLAMP_TO_EDGE)
        .anisotropy_enable(false)
        .min_lod(0.0)
        .max_lod(0.0);
    let sampler = unsafe { device.create_sampler(&sampler_ci, None) }.unwrap();

    let _ = &mut image; // keep mutable binding consistent with destroy()
    Texture { image, sampler }
}
