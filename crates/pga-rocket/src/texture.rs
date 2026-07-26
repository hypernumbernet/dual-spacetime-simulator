//! Open-world ground albedo (meadow / lunar regolith / paved) with mipmaps.
//!
//! Procedural generators are pure CPU (unit-tested without Vulkan). Upload uses
//! LINEAR filtering, a full mip chain, and anisotropy when the device allows.
//! Grass + paved + moon share one queue submit at startup.

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

#[inline]
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
#[inline]
fn hash01(ix: u32, iy: u32, salt: u32) -> f32 {
    (h(ix, iy, salt) & 0xFFFF) as f32 * (1.0 / 65536.0)
}

#[inline]
fn smoothstep_cubic(t: f32) -> f32 {
    // Hermite cubic — cheaper than quintic; seamless tiling does not need C2.
    t * t * (3.0 - 2.0 * t)
}

/// Tileable value noise. `period` must be a power of two (≥ 1) so wrap is a mask.
#[inline]
fn vnoise_tile_pow2(px: f32, py: f32, period: u32, salt: u32) -> f32 {
    debug_assert!(period.is_power_of_two() && period >= 1);
    let mask = period - 1;
    let x0 = px.floor() as i32;
    let y0 = py.floor() as i32;
    let fx = px - x0 as f32;
    let fy = py - y0 as f32;
    let u = smoothstep_cubic(fx);
    let v = smoothstep_cubic(fy);
    let x0u = (x0 as u32) & mask;
    let y0u = (y0 as u32) & mask;
    let x1u = (x0u + 1) & mask;
    let y1u = (y0u + 1) & mask;
    let a = hash01(x0u, y0u, salt);
    let b = hash01(x1u, y0u, salt);
    let c = hash01(x0u, y1u, salt);
    let d = hash01(x1u, y1u, salt);
    a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v
}

/// Multi-octave tileable fBm in roughly [0, 1]. `base_cells` must be power of two.
fn fbm_tile(px: f32, py: f32, base_cells: u32, salt: u32, octaves: u32) -> f32 {
    debug_assert!(base_cells.is_power_of_two() && base_cells >= 1);
    let mut amp = 0.5;
    let mut sum = 0.0;
    let mut norm = 0.0;
    let mut cells = base_cells;
    let mut sx = px;
    let mut sy = py;
    for o in 0..octaves {
        let s = salt.wrapping_add(o.wrapping_mul(97));
        sum += amp * vnoise_tile_pow2(sx, sy, cells, s);
        norm += amp;
        amp *= 0.5;
        sx *= 2.0;
        sy *= 2.0;
        let next = cells.saturating_mul(2);
        if next == 0 || next > base_cells.saturating_mul(16) {
            break;
        }
        cells = next;
    }
    if norm > 0.0 {
        sum / norm
    } else {
        0.0
    }
}

#[inline]
fn lerp3(a: [f32; 3], b: [f32; 3], t: f32) -> [f32; 3] {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]
}

#[inline]
fn clamp01(c: [f32; 3]) -> [f32; 3] {
    [
        c[0].clamp(0.0, 1.0),
        c[1].clamp(0.0, 1.0),
        c[2].clamp(0.0, 1.0),
    ]
}

#[inline]
fn write_px(px: &mut [u8], width: u32, x: u32, y: u32, rgb: [f32; 3]) {
    let i = ((y * width + x) * 4) as usize;
    px[i] = (rgb[0] * 255.0) as u8;
    px[i + 1] = (rgb[1] * 255.0) as u8;
    px[i + 2] = (rgb[2] * 255.0) as u8;
    px[i + 3] = 255;
}

// --- Generators --------------------------------------------------------------

/// Open-world meadow albedo: multi-scale greens, soft dirt patches, fine grit.
pub fn generate_grass_pixels() -> Vec<u8> {
    generate_grass_pixels_size(GROUND_TILE_PX)
}

/// Same as [`generate_grass_pixels`] with explicit resolution (power-of-two recommended).
pub fn generate_grass_pixels_size(size: u32) -> Vec<u8> {
    let size = size.max(4);
    // Power-of-two cell counts → fast bit-mask wrap in vnoise.
    let broad_cells = 4u32;
    let mid_cells = 8u32;
    let fine_cells = 32u32;
    let mut px = vec![0u8; (size * size * 4) as usize];
    let lush = [0.22, 0.48, 0.18];
    let dry = [0.38, 0.42, 0.22];
    let dirt = [0.36, 0.28, 0.18];
    let inv = 1.0 / size as f32;
    // Fewer octaves: hardware mips already blur fine structure.
    for y in 0..size {
        let ny = y as f32 * inv;
        for x in 0..size {
            let nx = x as f32 * inv;
            let meadow = fbm_tile(nx * broad_cells as f32, ny * broad_cells as f32, broad_cells, 11, 2);
            let clump = fbm_tile(nx * mid_cells as f32, ny * mid_cells as f32, mid_cells, 23, 2);
            let grit = vnoise_tile_pow2(nx * fine_cells as f32, ny * fine_cells as f32, fine_cells, 41);
            let dirt_n = fbm_tile(
                nx * broad_cells as f32 + 2.0,
                ny * broad_cells as f32 + 1.0,
                broad_cells,
                59,
                2,
            );
            let dirt_m = ((dirt_n - 0.62) * 4.0).clamp(0.0, 1.0);

            let mut col = lerp3(dry, lush, meadow * 0.65 + clump * 0.35);
            col = lerp3(col, dirt, dirt_m * 0.85);
            let g = (grit - 0.5) * 0.12;
            col = [col[0] + g, col[1] + g * 1.05, col[2] + g * 0.7];
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
    let broad_cells = 4u32;
    let mid_cells = 8u32;
    let fine_cells = 32u32;
    let mut px = vec![0u8; (size * size * 4) as usize];
    let base = [0.52, 0.51, 0.48];
    let dark = [0.28, 0.27, 0.25];
    let ejecta = [0.68, 0.66, 0.60];
    let inv = 1.0 / size as f32;
    for y in 0..size {
        let ny = y as f32 * inv;
        for x in 0..size {
            let nx = x as f32 * inv;
            let broad = fbm_tile(nx * broad_cells as f32, ny * broad_cells as f32, broad_cells, 71, 2);
            let mid = fbm_tile(nx * mid_cells as f32, ny * mid_cells as f32, mid_cells, 83, 2);
            let grit = vnoise_tile_pow2(nx * fine_cells as f32, ny * fine_cells as f32, fine_cells, 97);
            let crater_field = fbm_tile(
                nx * broad_cells as f32 + 1.5,
                ny * broad_cells as f32 + 0.5,
                broad_cells,
                101,
                2,
            );
            let crater = ((0.78 - crater_field) * 3.5).clamp(0.0, 1.0);
            let bright = ((crater_field - 0.72) * 2.5).clamp(0.0, 1.0) * (1.0 - crater);

            let mut col = lerp3(base, dark, broad * 0.35 + mid * 0.25 + crater * 0.55);
            col = lerp3(col, ejecta, bright * 0.35);
            let g = (grit - 0.5) * 0.10;
            col = [col[0] + g, col[1] + g * 0.98, col[2] + g * 0.92];
            col[2] = (col[2] * 0.98 + 0.01).min(1.0);
            write_px(&mut px, size, x, y, clamp01(col));
        }
    }
    px
}

/// Soft cast-concrete / paved tile: low-frequency panels + grit.
pub fn generate_paved_pixels() -> Vec<u8> {
    generate_paved_pixels_size(GROUND_TILE_PX)
}

pub fn generate_paved_pixels_size(size: u32) -> Vec<u8> {
    let size = size.max(4);
    let mid_cells = 8u32;
    let fine_cells = 32u32;
    let mut px = vec![0u8; (size * size * 4) as usize];
    let base = [0.50, 0.50, 0.48];
    let dark = [0.40, 0.40, 0.38];
    let mortar = [0.36, 0.36, 0.34];
    let inv = 1.0 / size as f32;
    let seam_period = (size / 4).max(8).next_power_of_two().min(size.max(8));
    let seam_f = seam_period as f32;
    let inv3 = 1.0 / 3.0;
    for y in 0..size {
        let ny = y as f32 * inv;
        let sy = (y % seam_period) as f32;
        let d_y = sy.min(seam_f - sy);
        let t_y = 1.0 - (d_y * inv3).clamp(0.0, 1.0);
        let seam_y = t_y * t_y;
        for x in 0..size {
            let nx = x as f32 * inv;
            let n = fbm_tile(nx * mid_cells as f32, ny * mid_cells as f32, mid_cells, 7, 2);
            let grit = vnoise_tile_pow2(nx * fine_cells as f32, ny * fine_cells as f32, fine_cells, 13);
            let sx = (x % seam_period) as f32;
            let d_x = sx.min(seam_f - sx);
            let t_x = 1.0 - (d_x * inv3).clamp(0.0, 1.0);
            let seam = (t_x * t_x).max(seam_y) * 0.55;

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
    let w = width as usize;
    for y in 0..nh as usize {
        let y0 = y * 2;
        let y1 = (y0 + 1).min(height as usize - 1);
        let row0 = y0 * w * 4;
        let row1 = y1 * w * 4;
        for x in 0..nw as usize {
            let x0 = x * 2;
            let x1 = (x0 + 1).min(width as usize - 1);
            let i00 = row0 + x0 * 4;
            let i10 = row0 + x1 * 4;
            let i01 = row1 + x0 * 4;
            let i11 = row1 + x1 * 4;
            let o = (y * nw as usize + x) * 4;
            out[o] = ((src[i00] as u32 + src[i10] as u32 + src[i01] as u32 + src[i11] as u32) / 4) as u8;
            out[o + 1] = ((src[i00 + 1] as u32
                + src[i10 + 1] as u32
                + src[i01 + 1] as u32
                + src[i11 + 1] as u32)
                / 4) as u8;
            out[o + 2] = ((src[i00 + 2] as u32
                + src[i10 + 2] as u32
                + src[i01 + 2] as u32
                + src[i11 + 2] as u32)
                / 4) as u8;
            out[o + 3] = ((src[i00 + 3] as u32
                + src[i10 + 3] as u32
                + src[i01 + 3] as u32
                + src[i11 + 3] as u32)
                / 4) as u8;
        }
    }
    (nw, nh, out)
}

/// Full mip chain: level 0 takes ownership of `base`, then box-filters down to 1×1.
pub fn generate_mip_chain(base: Vec<u8>, width: u32, height: u32) -> Vec<(u32, u32, Vec<u8>)> {
    assert_eq!(base.len(), (width * height * 4) as usize);
    let mut levels = Vec::with_capacity(mip_level_count(width, height) as usize);
    levels.push((width, height, base));
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

/// Create grass, paved, and moon textures with one queue submit (startup path).
pub fn create_ground_textures(
    vb: &VulkanBase,
    allocator: &Mutex<Allocator>,
) -> (Texture, Texture, Texture) {
    let aniso = sampler_anisotropy(vb);
    let labels = ["grass", "paved", "moon"];
    let bases = [
        generate_grass_pixels(),
        generate_paved_pixels(),
        generate_moon_pixels(),
    ];

    // Build mip pyramids and pack into one staging blob.
    let mut pyramids: Vec<Vec<(u32, u32, Vec<u8>)>> = Vec::with_capacity(3);
    let mut base_offsets = [0u64; 3];
    let mut staging_blob = Vec::new();
    for (i, base) in bases.into_iter().enumerate() {
        base_offsets[i] = staging_blob.len() as u64;
        let mips = generate_mip_chain(base, GROUND_TILE_PX, GROUND_TILE_PX);
        for (_, _, level) in &mips {
            staging_blob.extend_from_slice(level);
        }
        pyramids.push(mips);
    }

    let device = &vb.device;
    let staging = AllocatedBuffer::new(
        device,
        allocator,
        staging_blob.len().max(1) as u64,
        vk::BufferUsageFlags::TRANSFER_SRC,
        MemoryLocation::CpuToGpu,
        "ground-staging-all",
    );
    if let Some(ref alloc) = staging.allocation
        && let Some(mapped) = alloc.mapped_ptr()
    {
        unsafe {
            std::ptr::copy_nonoverlapping(
                staging_blob.as_ptr(),
                mapped.as_ptr() as *mut u8,
                staging_blob.len(),
            );
        }
    }

    let mut images: Vec<AllocatedImage> = Vec::with_capacity(3);
    for (i, label) in labels.iter().enumerate() {
        let mip_levels = pyramids[i].len() as u32;
        images.push(AllocatedImage::new_with_mips(
            device,
            allocator,
            GROUND_TILE_PX,
            GROUND_TILE_PX,
            mip_levels,
            vk::Format::R8G8B8A8_UNORM,
            vk::ImageUsageFlags::TRANSFER_DST | vk::ImageUsageFlags::SAMPLED,
            vk::ImageAspectFlags::COLOR,
            label,
        ));
    }

    let alloc_ci = vk::CommandBufferAllocateInfo::default()
        .command_pool(vb.command_pool)
        .level(vk::CommandBufferLevel::PRIMARY)
        .command_buffer_count(1);
    let cb = unsafe { device.allocate_command_buffers(&alloc_ci) }.unwrap()[0];
    let begin =
        vk::CommandBufferBeginInfo::default().flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT);
    unsafe { device.begin_command_buffer(cb, &begin) }.unwrap();

    for i in 0..3 {
        let mip_levels = pyramids[i].len() as u32;
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
            .image(images[i].image)
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

        let mut buffer_offset = base_offsets[i];
        let mut regions = Vec::with_capacity(pyramids[i].len());
        for (level, (w, h, data)) in pyramids[i].iter().enumerate() {
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
                images[i].image,
                vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                &regions,
            );
        }

        let to_shader = vk::ImageMemoryBarrier::default()
            .old_layout(vk::ImageLayout::TRANSFER_DST_OPTIMAL)
            .new_layout(vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL)
            .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
            .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
            .image(images[i].image)
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

    let mut textures = images.into_iter().enumerate().map(|(i, image)| {
        let mip_levels = pyramids[i].len() as u32;
        Texture {
            image,
            sampler: create_ground_sampler(device, mip_levels, aniso),
            mip_levels,
        }
    });
    let grass = textures.next().unwrap();
    let paved = textures.next().unwrap();
    let moon = textures.next().unwrap();
    (grass, paved, moon)
}

/// Uploads a single albedo (prefer [`create_ground_textures`] for the full set).
pub fn create_grass_texture(vb: &VulkanBase, allocator: &Mutex<Allocator>) -> Texture {
    upload_one(vb, allocator, generate_grass_pixels(), "grass")
}

/// Uploads paved alone (prefer [`create_ground_textures`] for the full set).
pub fn create_paved_texture(vb: &VulkanBase, allocator: &Mutex<Allocator>) -> Texture {
    upload_one(vb, allocator, generate_paved_pixels(), "paved")
}

/// Uploads moon alone (prefer [`create_ground_textures`] for the full set).
pub fn create_moon_texture(vb: &VulkanBase, allocator: &Mutex<Allocator>) -> Texture {
    upload_one(vb, allocator, generate_moon_pixels(), "moon")
}

/// Single-texture upload path (one submit). Used by individual helpers.
fn upload_one(
    vb: &VulkanBase,
    allocator: &Mutex<Allocator>,
    base: Vec<u8>,
    label: &str,
) -> Texture {
    let aniso = sampler_anisotropy(vb);
    let device = &vb.device;
    let mips = generate_mip_chain(base, GROUND_TILE_PX, GROUND_TILE_PX);
    let mip_levels = mips.len() as u32;

    let mut blob = Vec::new();
    for (_, _, level) in &mips {
        blob.extend_from_slice(level);
    }
    let staging = AllocatedBuffer::new(
        device,
        allocator,
        blob.len().max(1) as u64,
        vk::BufferUsageFlags::TRANSFER_SRC,
        MemoryLocation::CpuToGpu,
        &format!("{label}-staging"),
    );
    if let Some(ref alloc) = staging.allocation
        && let Some(mapped) = alloc.mapped_ptr()
    {
        unsafe {
            std::ptr::copy_nonoverlapping(blob.as_ptr(), mapped.as_ptr() as *mut u8, blob.len());
        }
    }

    let mut image = AllocatedImage::new_with_mips(
        device,
        allocator,
        GROUND_TILE_PX,
        GROUND_TILE_PX,
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
        device.end_command_buffer(cb).unwrap();
    }
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

    let sampler = create_ground_sampler(device, mip_levels, aniso);
    let _ = &mut image;
    Texture {
        image,
        sampler,
        mip_levels,
    }
}

fn sampler_anisotropy(vb: &VulkanBase) -> (bool, f32) {
    let props = unsafe {
        vb.instance
            .get_physical_device_properties(vb.physical_device)
    };
    let max_aniso = props.limits.max_sampler_anisotropy;
    let features = unsafe {
        vb.instance
            .get_physical_device_features(vb.physical_device)
    };
    let enable = features.sampler_anisotropy == vk::TRUE && max_aniso >= 1.0;
    let aniso = if enable { max_aniso.min(8.0) } else { 1.0 };
    (enable, aniso)
}

fn create_ground_sampler(device: &ash::Device, mip_levels: u32, aniso: (bool, f32)) -> vk::Sampler {
    let (aniso_enable, max_aniso) = aniso;
    let sampler_ci = vk::SamplerCreateInfo::default()
        .mag_filter(vk::Filter::LINEAR)
        .min_filter(vk::Filter::LINEAR)
        .mipmap_mode(vk::SamplerMipmapMode::LINEAR)
        .address_mode_u(vk::SamplerAddressMode::REPEAT)
        .address_mode_v(vk::SamplerAddressMode::REPEAT)
        .address_mode_w(vk::SamplerAddressMode::REPEAT)
        .anisotropy_enable(aniso_enable)
        .max_anisotropy(max_aniso)
        .min_lod(0.0)
        .max_lod(mip_levels as f32);
    unsafe { device.create_sampler(&sampler_ci, None) }.unwrap()
}
