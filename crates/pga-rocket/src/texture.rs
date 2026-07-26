//! Open-world ground albedo (meadow / lunar regolith / paved) with mipmaps.
//!
//! Procedural generators are pure CPU (unit-tested without Vulkan). Upload uses
//! LINEAR filtering, a full mip chain, and anisotropy when the device allows.

use ash::vk;
use gpu_allocator::MemoryLocation;
use gpu_allocator::vulkan::Allocator;
use std::sync::Mutex;
use vulkanvil::{AllocatedBuffer, AllocatedImage, VulkanBase};

/// Ground albedo tile resolution (power of two). Far larger than the old 16×16
/// Minecraft stamp so LINEAR + mips read as continuous open-world terrain.
pub const GROUND_TILE_PX: u32 = 256;

/// Back-compat alias; prefer [`GROUND_TILE_PX`].
pub const GRASS_TILE_PX: u32 = GROUND_TILE_PX;

pub struct Texture {
    pub image: AllocatedImage,
    pub sampler: vk::Sampler,
    /// Number of mip levels uploaded (including base). Exposed for structural checks.
    pub mip_levels: u32,
}

impl Texture {
    pub fn destroy(&mut self, device: &ash::Device, allocator: &Mutex<Allocator>) {
        unsafe { device.destroy_sampler(self.sampler, None) };
        self.image.destroy(device, allocator);
    }
}

// --- Hash / tileable noise (pure, no Vulkan) ---------------------------------

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

/// Unit hash in [0, 1) for integer lattice cell.
fn hash01(ix: i32, iy: i32, salt: u32) -> f32 {
    let x = ix as u32;
    let y = iy as u32;
    (h(x, y, salt) & 0xFFFF) as f32 / 65536.0
}

fn smoothstep_quintic(t: f32) -> f32 {
    t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
}

/// Tileable value noise: period is the lattice wrap in "cells".
fn vnoise_tile(px: f32, py: f32, period: i32, salt: u32) -> f32 {
    let period = period.max(1);
    let x0 = px.floor() as i32;
    let y0 = py.floor() as i32;
    let fx = px - x0 as f32;
    let fy = py - y0 as f32;
    let u = smoothstep_quintic(fx);
    let v = smoothstep_quintic(fy);
    let wrap = |i: i32| {
        let m = i.rem_euclid(period);
        m
    };
    let a = hash01(wrap(x0), wrap(y0), salt);
    let b = hash01(wrap(x0 + 1), wrap(y0), salt);
    let c = hash01(wrap(x0), wrap(y0 + 1), salt);
    let d = hash01(wrap(x0 + 1), wrap(y0 + 1), salt);
    a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v
}

/// Multi-octave tileable fBm in roughly [0, 1].
///
/// `base_cells` = integer lattice periods across the texture (must be ≥ 1).
/// Pixel coords should be passed as `x * base_cells / size` so one full period
/// spans the tile; each octave doubles frequency with a matching wrap period.
fn fbm_tile(px: f32, py: f32, base_cells: i32, salt: u32, octaves: u32) -> f32 {
    let mut amp = 0.5;
    let mut sum = 0.0;
    let mut norm = 0.0;
    let mut cells = base_cells.max(1);
    let mut sx = px;
    let mut sy = py;
    for o in 0..octaves {
        let s = salt.wrapping_add(o.wrapping_mul(97));
        sum += amp * vnoise_tile(sx, sy, cells, s);
        norm += amp;
        amp *= 0.5;
        // Double frequency; wrap period doubles so the tile still seams.
        sx *= 2.0;
        sy *= 2.0;
        cells = (cells * 2).max(1);
        // Cap so we do not exceed ~size lattice samples for fine grit.
        if cells > base_cells.max(1) * 32 {
            break;
        }
    }
    if norm > 0.0 {
        sum / norm
    } else {
        0.0
    }
}

fn lerp3(a: [f32; 3], b: [f32; 3], t: f32) -> [f32; 3] {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]
}

fn clamp01(c: [f32; 3]) -> [f32; 3] {
    [
        c[0].clamp(0.0, 1.0),
        c[1].clamp(0.0, 1.0),
        c[2].clamp(0.0, 1.0),
    ]
}

fn write_px(px: &mut [u8], width: u32, x: u32, y: u32, rgb: [f32; 3]) {
    let i = ((y * width + x) * 4) as usize;
    px[i] = (rgb[0] * 255.0) as u8;
    px[i + 1] = (rgb[1] * 255.0) as u8;
    px[i + 2] = (rgb[2] * 255.0) as u8;
    px[i + 3] = 255;
}

// --- Generators --------------------------------------------------------------

/// Open-world meadow albedo: multi-scale greens, soft dirt patches, fine grit.
/// Seamless at tile edges (tileable noise with period matching the pixel grid cells).
pub fn generate_grass_pixels() -> Vec<u8> {
    generate_grass_pixels_size(GROUND_TILE_PX)
}

/// Same as [`generate_grass_pixels`] with explicit resolution (power-of-two recommended).
pub fn generate_grass_pixels_size(size: u32) -> Vec<u8> {
    let size = size.max(4);
    // Integer cell counts so every octave seams at the tile edge.
    let broad_cells = 4i32;
    let mid_cells = 12i32;
    let fine_cells = 48i32;
    let mut px = vec![0u8; (size * size * 4) as usize];
    let lush = [0.22, 0.48, 0.18];
    let dry = [0.38, 0.42, 0.22];
    let dirt = [0.36, 0.28, 0.18];
    let sf = size as f32;
    for y in 0..size {
        for x in 0..size {
            let nx = x as f32 / sf;
            let ny = y as f32 / sf;
            // Broad meadow patches (4 periods across tile).
            let meadow = fbm_tile(nx * broad_cells as f32, ny * broad_cells as f32, broad_cells, 11, 4);
            // Medium clump variation.
            let clump = fbm_tile(nx * mid_cells as f32, ny * mid_cells as f32, mid_cells, 23, 3);
            // Fine blade / grit.
            let grit = fbm_tile(nx * fine_cells as f32, ny * fine_cells as f32, fine_cells, 41, 2);
            // Dirt patch mask (sparse).
            let dirt_n = fbm_tile(
                nx * broad_cells as f32 + 2.0,
                ny * broad_cells as f32 + 1.0,
                broad_cells,
                59,
                3,
            );
            let dirt_m = ((dirt_n - 0.62) * 4.0).clamp(0.0, 1.0);

            let mut col = lerp3(dry, lush, meadow * 0.65 + clump * 0.35);
            col = lerp3(col, dirt, dirt_m * 0.85);
            // Micro variation.
            let g = (grit - 0.5) * 0.12;
            col = [col[0] + g, col[1] + g * 1.05, col[2] + g * 0.7];
            // Slight yellow-green bias in bright clumps.
            col[1] = (col[1] + clump * 0.04).min(1.0);
            write_px(&mut px, size, x, y, clamp01(col));
        }
    }
    px
}

/// Continuous lunar regolith: gray dust, multi-scale grit, sparse crater flecks.
pub fn generate_moon_pixels() -> Vec<u8> {
    generate_moon_pixels_size(GROUND_TILE_PX)
}

pub fn generate_moon_pixels_size(size: u32) -> Vec<u8> {
    let size = size.max(4);
    let broad_cells = 3i32;
    let mid_cells = 10i32;
    let fine_cells = 40i32;
    let mut px = vec![0u8; (size * size * 4) as usize];
    let base = [0.52, 0.51, 0.48];
    let dark = [0.28, 0.27, 0.25];
    let ejecta = [0.68, 0.66, 0.60];
    let sf = size as f32;
    for y in 0..size {
        for x in 0..size {
            let nx = x as f32 / sf;
            let ny = y as f32 / sf;
            let broad = fbm_tile(nx * broad_cells as f32, ny * broad_cells as f32, broad_cells, 71, 4);
            let mid = fbm_tile(nx * mid_cells as f32, ny * mid_cells as f32, mid_cells, 83, 3);
            let grit = fbm_tile(nx * fine_cells as f32, ny * fine_cells as f32, fine_cells, 97, 2);
            // Soft crater bowls via low-frequency threshold.
            let crater_field = fbm_tile(
                nx * broad_cells as f32 + 1.5,
                ny * broad_cells as f32 + 0.5,
                broad_cells,
                101,
                3,
            );
            let crater = ((0.78 - crater_field) * 3.5).clamp(0.0, 1.0);
            // Occasional bright ejecta (inverse of deep crater).
            let bright = ((crater_field - 0.72) * 2.5).clamp(0.0, 1.0) * (1.0 - crater);

            let mut col = lerp3(base, dark, broad * 0.35 + mid * 0.25 + crater * 0.55);
            col = lerp3(col, ejecta, bright * 0.35);
            let g = (grit - 0.5) * 0.10;
            col = [col[0] + g, col[1] + g * 0.98, col[2] + g * 0.92];
            // Cool gray bias slightly.
            col[2] = (col[2] * 0.98 + 0.01).min(1.0);
            write_px(&mut px, size, x, y, clamp01(col));
        }
    }
    px
}

/// Soft cast-concrete / paved tile: low-frequency panels + grit (no hard 1px mortar grid).
pub fn generate_paved_pixels() -> Vec<u8> {
    generate_paved_pixels_size(GROUND_TILE_PX)
}

pub fn generate_paved_pixels_size(size: u32) -> Vec<u8> {
    let size = size.max(4);
    let mid_cells = 8i32;
    let fine_cells = 32i32;
    let mut px = vec![0u8; (size * size * 4) as usize];
    let base = [0.50, 0.50, 0.48];
    let dark = [0.40, 0.40, 0.38];
    let mortar = [0.36, 0.36, 0.34];
    let sf = size as f32;
    // Soft panel seams every ~1/4 of the tile (integer so edges match).
    let seam_period = (size / 4).max(8);
    for y in 0..size {
        for x in 0..size {
            let nx = x as f32 / sf;
            let ny = y as f32 / sf;
            let n = fbm_tile(nx * mid_cells as f32, ny * mid_cells as f32, mid_cells, 7, 3);
            let grit = fbm_tile(nx * fine_cells as f32, ny * fine_cells as f32, fine_cells, 13, 2);
            // Soft mortar lines (distance to nearest seam, feathered).
            let sx = (x % seam_period) as f32;
            let sy = (y % seam_period) as f32;
            let seam_x = (1.0 - (sx.min(seam_period as f32 - sx) / 3.0).clamp(0.0, 1.0)).powf(2.0);
            let seam_y = (1.0 - (sy.min(seam_period as f32 - sy) / 3.0).clamp(0.0, 1.0)).powf(2.0);
            let seam = seam_x.max(seam_y) * 0.55;

            let mut col = lerp3(base, dark, n * 0.45);
            col = lerp3(col, mortar, seam);
            let g = (grit - 0.5) * 0.07;
            col = [col[0] + g, col[1] + g, col[2] + g * 0.9];
            write_px(&mut px, size, x, y, clamp01(col));
        }
    }
    px
}

// --- Mip chain (pure) --------------------------------------------------------

/// Number of mip levels for a square power-of-two (or any) size, including base.
pub fn mip_level_count(width: u32, height: u32) -> u32 {
    let m = width.max(height).max(1);
    (u32::BITS - m.leading_zeros()).max(1)
}

/// Box-filter downsample by 2 in each axis (last odd edge averaged with itself).
pub fn downsample_rgba8(src: &[u8], width: u32, height: u32) -> (u32, u32, Vec<u8>) {
    let nw = (width / 2).max(1);
    let nh = (height / 2).max(1);
    let mut out = vec![0u8; (nw * nh * 4) as usize];
    for y in 0..nh {
        for x in 0..nw {
            let x0 = x * 2;
            let y0 = y * 2;
            let x1 = (x0 + 1).min(width - 1);
            let y1 = (y0 + 1).min(height - 1);
            let mut acc = [0u32; 4];
            for (sx, sy) in [(x0, y0), (x1, y0), (x0, y1), (x1, y1)] {
                let i = ((sy * width + sx) * 4) as usize;
                acc[0] += src[i] as u32;
                acc[1] += src[i + 1] as u32;
                acc[2] += src[i + 2] as u32;
                acc[3] += src[i + 3] as u32;
            }
            let o = ((y * nw + x) * 4) as usize;
            out[o] = (acc[0] / 4) as u8;
            out[o + 1] = (acc[1] / 4) as u8;
            out[o + 2] = (acc[2] / 4) as u8;
            out[o + 3] = (acc[3] / 4) as u8;
        }
    }
    (nw, nh, out)
}

/// Full mip chain: level 0 is `base` at `(width, height)`, then box-filtered down to 1×1.
pub fn generate_mip_chain(base: &[u8], width: u32, height: u32) -> Vec<(u32, u32, Vec<u8>)> {
    assert_eq!(base.len(), (width * height * 4) as usize);
    let mut levels = Vec::new();
    levels.push((width, height, base.to_vec()));
    let mut w = width;
    let mut h = height;
    while w > 1 || h > 1 {
        let prev = &levels.last().unwrap().2;
        let (nw, nh, next) = downsample_rgba8(prev, w, h);
        levels.push((nw, nh, next));
        w = nw;
        h = nh;
    }
    levels
}

/// Mean RGB of an RGBA8 buffer as f32 in [0, 1] — used by tests for gamut checks.
pub fn mean_rgb(pixels: &[u8]) -> [f32; 3] {
    assert!(pixels.len() >= 4 && pixels.len() % 4 == 0);
    let n = (pixels.len() / 4) as f32;
    let mut acc = [0.0f32; 3];
    for chunk in pixels.chunks_exact(4) {
        acc[0] += chunk[0] as f32;
        acc[1] += chunk[1] as f32;
        acc[2] += chunk[2] as f32;
    }
    [acc[0] / (n * 255.0), acc[1] / (n * 255.0), acc[2] / (n * 255.0)]
}

// --- Vulkan upload -----------------------------------------------------------

/// Uploads grass with LINEAR + mip chain (+ anisotropy when available).
pub fn create_grass_texture(vb: &VulkanBase, allocator: &Mutex<Allocator>) -> Texture {
    upload_rgba8_mipped(vb, allocator, &generate_grass_pixels(), GROUND_TILE_PX, "grass")
}

/// Uploads paved concrete with LINEAR + mip chain.
pub fn create_paved_texture(vb: &VulkanBase, allocator: &Mutex<Allocator>) -> Texture {
    upload_rgba8_mipped(vb, allocator, &generate_paved_pixels(), GROUND_TILE_PX, "paved")
}

/// Uploads lunar regolith with LINEAR + mip chain.
pub fn create_moon_texture(vb: &VulkanBase, allocator: &Mutex<Allocator>) -> Texture {
    upload_rgba8_mipped(vb, allocator, &generate_moon_pixels(), GROUND_TILE_PX, "moon")
}

fn upload_rgba8_mipped(
    vb: &VulkanBase,
    allocator: &Mutex<Allocator>,
    pixels: &[u8],
    size: u32,
    label: &str,
) -> Texture {
    let width = size;
    let height = size;
    let mips = generate_mip_chain(pixels, width, height);
    let mip_levels = mips.len() as u32;
    let device = &vb.device;

    // Pack all mip levels into one staging buffer.
    let total_bytes: usize = mips.iter().map(|(_, _, p)| p.len()).sum();
    let staging = AllocatedBuffer::new(
        device,
        allocator,
        total_bytes as u64,
        vk::BufferUsageFlags::TRANSFER_SRC,
        MemoryLocation::CpuToGpu,
        &format!("{label}-staging"),
    );
    if let Some(ref alloc) = staging.allocation
        && let Some(mapped) = alloc.mapped_ptr()
    {
        let mut offset = 0usize;
        unsafe {
            let dst = mapped.as_ptr() as *mut u8;
            for (_, _, level) in &mips {
                std::ptr::copy_nonoverlapping(level.as_ptr(), dst.add(offset), level.len());
                offset += level.len();
            }
        }
    }

    let mut image = AllocatedImage::new_with_mips(
        device,
        allocator,
        width,
        height,
        mip_levels,
        vk::Format::R8G8B8A8_UNORM,
        vk::ImageUsageFlags::TRANSFER_DST | vk::ImageUsageFlags::SAMPLED,
        vk::ImageAspectFlags::COLOR,
        label,
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
        level_count: mip_levels,
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

    let mut buffer_offset = 0u64;
    let mut regions = Vec::with_capacity(mips.len());
    for (level, (w, h, data)) in mips.iter().enumerate() {
        regions.push(vk::BufferImageCopy {
            buffer_offset,
            buffer_row_length: 0,
            buffer_image_height: 0,
            image_subresource: vk::ImageSubresourceLayers {
                aspect_mask: vk::ImageAspectFlags::COLOR,
                mip_level: level as u32,
                base_array_layer: 0,
                layer_count: 1,
            },
            image_offset: vk::Offset3D { x: 0, y: 0, z: 0 },
            image_extent: vk::Extent3D {
                width: *w,
                height: *h,
                depth: 1,
            },
        });
        buffer_offset += data.len() as u64;
    }
    unsafe {
        device.cmd_copy_buffer_to_image(
            cb,
            staging.buffer,
            image.image,
            vk::ImageLayout::TRANSFER_DST_OPTIMAL,
            &regions,
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

    // LINEAR + full mip chain; anisotropy when the device advertised it.
    let props = unsafe {
        vb.instance
            .get_physical_device_properties(vb.physical_device)
    };
    let max_aniso = props.limits.max_sampler_anisotropy;
    let features = unsafe {
        vb.instance
            .get_physical_device_features(vb.physical_device)
    };
    let aniso_enable = features.sampler_anisotropy == vk::TRUE && max_aniso >= 1.0;
    let aniso = if aniso_enable {
        max_aniso.min(8.0)
    } else {
        1.0
    };

    let sampler_ci = vk::SamplerCreateInfo::default()
        .mag_filter(vk::Filter::LINEAR)
        .min_filter(vk::Filter::LINEAR)
        .mipmap_mode(vk::SamplerMipmapMode::LINEAR)
        .address_mode_u(vk::SamplerAddressMode::REPEAT)
        .address_mode_v(vk::SamplerAddressMode::REPEAT)
        .address_mode_w(vk::SamplerAddressMode::REPEAT)
        .anisotropy_enable(aniso_enable)
        .max_anisotropy(aniso)
        .min_lod(0.0)
        .max_lod(mip_levels as f32);
    let sampler = unsafe { device.create_sampler(&sampler_ci, None) }.unwrap();

    let _ = &mut image;
    Texture {
        image,
        sampler,
        mip_levels,
    }
}
