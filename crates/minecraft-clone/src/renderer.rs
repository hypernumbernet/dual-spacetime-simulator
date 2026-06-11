//! Voxel renderer: render pass (color + depth), depth resources, framebuffers, an opaque
//! terrain pipeline, a translucent water pipeline, a gradient sky pipeline, the atlas
//! descriptor set, per-chunk GPU buffers, and deferred buffer garbage collection.

use crate::hud::{self, TextVertex};
use crate::mesher::{ChunkMeshData, VoxelVertex};
use crate::texture::{create_atlas_texture, create_texture_rgba, Texture};
use crate::worldgen::SEA_LEVEL;
use ash::vk;
use glam::{IVec2, Mat4, Vec3, Vec4, Vec4Swizzles};
use gpu_allocator::vulkan::Allocator;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use vulkanvil::{
    create_buffer_with_data, create_shader_module, AllocatedBuffer, AllocatedImage, VulkanBase,
    MAX_FRAMES_IN_FLIGHT,
};

/// SPIR-V compiled by build.rs.
const VOXEL_VERT: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/shaders/voxel.vert.spv"));
const VOXEL_FRAG: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/shaders/voxel.frag.spv"));
const WATER_VERT: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/shaders/water.vert.spv"));
const WATER_FRAG: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/shaders/water.frag.spv"));
const SKY_VERT: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/shaders/sky.vert.spv"));
const SKY_FRAG: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/shaders/sky.frag.spv"));
const TEXT_VERT: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/shaders/text.vert.spv"));
const TEXT_FRAG: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/shaders/text.frag.spv"));

/// Fog color / horizon tint that distant terrain fades into.
pub const HORIZON_COLOR: [f32; 4] = [0.74, 0.84, 0.94, 1.0];
/// Murky short-range fog used while the camera is submerged.
const UNDERWATER_COLOR: [f32; 4] = [0.04, 0.20, 0.29, 1.0];
const UNDERWATER_FOG_START: f32 = 3.0;
const UNDERWATER_FOG_END: f32 = 42.0;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct PushConstants {
    pub view_proj: [[f32; 4]; 4],
    pub camera_pos: [f32; 4],
    pub fog_color: [f32; 4],
    pub fog_params: [f32; 4],
}

impl PushConstants {
    /// `time` animates waves/caustics; `underwater` switches fog to the murky in-water
    /// preset and flags the shaders (carried in camera_pos.w). `render_distance` (in
    /// chunks) drives the fog so pop-in stays hidden when the ring is resized at runtime.
    pub fn new(
        view_proj: [[f32; 4]; 4],
        camera_pos: [f32; 3],
        time: f32,
        underwater: bool,
        render_distance: i32,
    ) -> Self {
        let flag = if underwater { 1.0 } else { 0.0 };
        let fog_end = (render_distance * 16 - 8) as f32;
        let (color, start, end) = if underwater {
            (UNDERWATER_COLOR, UNDERWATER_FOG_START, UNDERWATER_FOG_END)
        } else {
            (HORIZON_COLOR, fog_end * 0.72, fog_end)
        };
        Self {
            view_proj,
            camera_pos: [camera_pos[0], camera_pos[1], camera_pos[2], flag],
            fog_color: color,
            // w carries the sea level so shader depth effects track WORLD_SCALE.
            fog_params: [start, end, time, SEA_LEVEL as f32],
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct SkyPushConstants {
    pub cam_right: [f32; 4],
    pub cam_up: [f32; 4],
    pub cam_fwd: [f32; 4],
    pub sun_dir: [f32; 4],
    pub params: [f32; 4], // x = tan(fov/2), y = aspect
}

/// Per-draw push constant: the chunk's world origin, written at offset 112 right after
/// the shared 112-byte PushConstants block (112 + 16 = 128 = the guaranteed minimum
/// maxPushConstantsSize).
const ORIGIN_OFFSET: u32 = std::mem::size_of::<PushConstants>() as u32;
const PUSH_TOTAL: u32 = ORIGIN_OFFSET + 16;

struct ChunkMesh {
    vertex: AllocatedBuffer,
    index: AllocatedBuffer,
    index_count: u32,
    y_min: f32,
    y_max: f32,
}

pub struct Renderer {
    device: ash::Device,
    allocator: Arc<Mutex<Allocator>>,
    depth_format: vk::Format,
    render_pass: vk::RenderPass,
    depth_image: AllocatedImage,
    framebuffers: Vec<vk::Framebuffer>,
    atlas: Texture,
    desc_set_layout: vk::DescriptorSetLayout,
    desc_pool: vk::DescriptorPool,
    desc_set: vk::DescriptorSet,
    pipeline_layout: vk::PipelineLayout,
    opaque_pipeline: vk::Pipeline,
    water_pipeline: vk::Pipeline,
    sky_pipeline_layout: vk::PipelineLayout,
    sky_pipeline: vk::Pipeline,
    font: Texture,
    text_desc_set: vk::DescriptorSet,
    text_pipeline_layout: vk::PipelineLayout,
    text_pipeline: vk::Pipeline,
    hud_buffer: Option<(AllocatedBuffer, u32)>,
    opaque_meshes: HashMap<IVec2, ChunkMesh>,
    water_meshes: HashMap<IVec2, ChunkMesh>,
    retired: Vec<(u64, AllocatedBuffer)>,
    frame_counter: u64,
}

/// GPU-side counters for the developer HUD.
pub struct RenderStats {
    pub chunk_meshes: usize,
    pub mesh_bytes: u64,
}

impl Renderer {
    pub fn new(vb: &VulkanBase) -> Self {
        let device = vb.device.clone();
        let allocator = vb.allocator.as_ref().unwrap().clone();

        let depth_format = select_depth_format(&vb.instance, vb.physical_device);
        let render_pass = create_render_pass(&device, vb.swapchain_format, depth_format);
        let depth_image = create_depth_image(&device, &allocator, depth_format, vb.swapchain_extent);
        let framebuffers = create_framebuffers(
            &device,
            render_pass,
            &vb.swapchain_image_views,
            depth_image.view,
            vb.swapchain_extent,
        );

        let atlas = create_atlas_texture(vb, &allocator);

        // Descriptor set layout / pool / set for the combined image sampler.
        let binding = vk::DescriptorSetLayoutBinding::default()
            .binding(0)
            .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
            .descriptor_count(1)
            .stage_flags(vk::ShaderStageFlags::FRAGMENT);
        let bindings = [binding];
        let layout_ci = vk::DescriptorSetLayoutCreateInfo::default().bindings(&bindings);
        let desc_set_layout = unsafe { device.create_descriptor_set_layout(&layout_ci, None) }.unwrap();

        // The HUD font texture shares the sampler-only set layout (set 0 = atlas,
        // set 1 = font, both allocated from the same pool).
        let font = create_texture_rgba(
            vb,
            &allocator,
            &hud::generate_font_pixels(),
            hud::FONT_W,
            hud::FONT_H,
            "font",
        );

        let pool_size = vk::DescriptorPoolSize {
            ty: vk::DescriptorType::COMBINED_IMAGE_SAMPLER,
            descriptor_count: 2,
        };
        let pool_sizes = [pool_size];
        let pool_ci = vk::DescriptorPoolCreateInfo::default()
            .pool_sizes(&pool_sizes)
            .max_sets(2);
        let desc_pool = unsafe { device.create_descriptor_pool(&pool_ci, None) }.unwrap();

        let set_layouts = [desc_set_layout, desc_set_layout];
        let alloc_ci = vk::DescriptorSetAllocateInfo::default()
            .descriptor_pool(desc_pool)
            .set_layouts(&set_layouts);
        let sets = unsafe { device.allocate_descriptor_sets(&alloc_ci) }.unwrap();
        let (desc_set, text_desc_set) = (sets[0], sets[1]);

        let atlas_info = vk::DescriptorImageInfo {
            sampler: atlas.sampler,
            image_view: atlas.image.view,
            image_layout: vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
        };
        let atlas_infos = [atlas_info];
        let font_info = vk::DescriptorImageInfo {
            sampler: font.sampler,
            image_view: font.image.view,
            image_layout: vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
        };
        let font_infos = [font_info];
        let writes = [
            vk::WriteDescriptorSet::default()
                .dst_set(desc_set)
                .dst_binding(0)
                .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                .image_info(&atlas_infos),
            vk::WriteDescriptorSet::default()
                .dst_set(text_desc_set)
                .dst_binding(0)
                .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
                .image_info(&font_infos),
        ];
        unsafe { device.update_descriptor_sets(&writes, &[]) };

        // Shared layout for the opaque + water pipelines (frame constants + per-draw
        // chunk origin).
        let push_range = vk::PushConstantRange {
            stage_flags: vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT,
            offset: 0,
            size: PUSH_TOTAL,
        };
        let ranges = [push_range];
        let pl_layouts = [desc_set_layout];
        let pl_ci = vk::PipelineLayoutCreateInfo::default()
            .set_layouts(&pl_layouts)
            .push_constant_ranges(&ranges);
        let pipeline_layout = unsafe { device.create_pipeline_layout(&pl_ci, None) }.unwrap();

        let opaque_pipeline = create_voxel_pipeline(
            &device,
            render_pass,
            pipeline_layout,
            VOXEL_VERT,
            VOXEL_FRAG,
            false,
        );
        let water_pipeline = create_voxel_pipeline(
            &device,
            render_pass,
            pipeline_layout,
            WATER_VERT,
            WATER_FRAG,
            true,
        );

        // Sky pipeline: own layout (no descriptors), its own push constants.
        let sky_range = vk::PushConstantRange {
            stage_flags: vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT,
            offset: 0,
            size: std::mem::size_of::<SkyPushConstants>() as u32,
        };
        let sky_ranges = [sky_range];
        let sky_pl_ci = vk::PipelineLayoutCreateInfo::default().push_constant_ranges(&sky_ranges);
        let sky_pipeline_layout = unsafe { device.create_pipeline_layout(&sky_pl_ci, None) }.unwrap();
        let sky_pipeline = create_sky_pipeline(&device, render_pass, sky_pipeline_layout);

        // HUD text pipeline: font sampler set + screen size push constant.
        let text_range = vk::PushConstantRange {
            stage_flags: vk::ShaderStageFlags::VERTEX,
            offset: 0,
            size: 8, // vec2 screen size
        };
        let text_ranges = [text_range];
        let text_layouts = [desc_set_layout];
        let text_pl_ci = vk::PipelineLayoutCreateInfo::default()
            .set_layouts(&text_layouts)
            .push_constant_ranges(&text_ranges);
        let text_pipeline_layout =
            unsafe { device.create_pipeline_layout(&text_pl_ci, None) }.unwrap();
        let text_pipeline = create_text_pipeline(&device, render_pass, text_pipeline_layout);

        Self {
            device,
            allocator,
            depth_format,
            render_pass,
            depth_image,
            framebuffers,
            atlas,
            desc_set_layout,
            desc_pool,
            desc_set,
            pipeline_layout,
            opaque_pipeline,
            water_pipeline,
            sky_pipeline_layout,
            sky_pipeline,
            font,
            text_desc_set,
            text_pipeline_layout,
            text_pipeline,
            hud_buffer: None,
            opaque_meshes: HashMap::new(),
            water_meshes: HashMap::new(),
            retired: Vec::new(),
            frame_counter: 0,
        }
    }

    /// Replaces the HUD text mesh (empty text hides the HUD). The old vertex buffer
    /// goes through the retired list so in-flight frames can still read it.
    pub fn update_hud(&mut self, text: &str) {
        if let Some((buf, _)) = self.hud_buffer.take() {
            self.retired.push((self.frame_counter, buf));
        }
        if text.is_empty() {
            return;
        }
        let verts = hud::build_text_vertices(text);
        if verts.is_empty() {
            return;
        }
        let (buf, count) = create_buffer_with_data(
            &self.device,
            &self.allocator,
            &verts,
            vk::BufferUsageFlags::VERTEX_BUFFER,
            "hud-text",
        );
        self.hud_buffer = Some((buf, count));
    }

    /// Sums the GPU bytes held by per-chunk vertex/index buffers.
    pub fn stats(&self) -> RenderStats {
        let mut chunk_meshes = 0;
        let mut mesh_bytes = 0u64;
        for map in [&self.opaque_meshes, &self.water_meshes] {
            chunk_meshes += map.len();
            for mesh in map.values() {
                if let Some(a) = &mesh.vertex.allocation {
                    mesh_bytes += a.size();
                }
                if let Some(a) = &mesh.index.allocation {
                    mesh_bytes += a.size();
                }
            }
        }
        RenderStats {
            chunk_meshes,
            mesh_bytes,
        }
    }

    /// Recreates depth + framebuffers after a swapchain resize. Caller must have already
    /// called `vb.recreate_swapchain` (which waits for device idle).
    pub fn recreate_size_dependent(&mut self, vb: &VulkanBase) {
        for &fb in &self.framebuffers {
            unsafe { self.device.destroy_framebuffer(fb, None) };
        }
        self.framebuffers.clear();
        self.depth_image.destroy(&self.device, &self.allocator);

        self.depth_image = create_depth_image(
            &self.device,
            &self.allocator,
            self.depth_format,
            vb.swapchain_extent,
        );
        self.framebuffers = create_framebuffers(
            &self.device,
            self.render_pass,
            &vb.swapchain_image_views,
            self.depth_image.view,
            vb.swapchain_extent,
        );
    }

    /// Uploads a chunk's opaque + water meshes into per-chunk buffers. Empty meshes drop out.
    pub fn upload_chunk_mesh(&mut self, coord: IVec2, m: &ChunkMeshData) {
        self.replace_mesh(coord, &m.opaque_verts, &m.opaque_indices, m.y_min, m.y_max, false);
        self.replace_mesh(coord, &m.water_verts, &m.water_indices, m.y_min, m.y_max, true);
    }

    fn replace_mesh(
        &mut self,
        coord: IVec2,
        verts: &[VoxelVertex],
        indices: &[u32],
        y_min: f32,
        y_max: f32,
        water: bool,
    ) {
        let map = if water {
            &mut self.water_meshes
        } else {
            &mut self.opaque_meshes
        };
        if let Some(old) = map.remove(&coord) {
            self.retired.push((self.frame_counter, old.vertex));
            self.retired.push((self.frame_counter, old.index));
        }
        if indices.is_empty() {
            return;
        }
        let (vertex, _) = create_buffer_with_data(
            &self.device,
            &self.allocator,
            verts,
            vk::BufferUsageFlags::VERTEX_BUFFER,
            "chunk-vertex",
        );
        let (index, index_count) = create_buffer_with_data(
            &self.device,
            &self.allocator,
            indices,
            vk::BufferUsageFlags::INDEX_BUFFER,
            "chunk-index",
        );
        let mesh = ChunkMesh {
            vertex,
            index,
            index_count,
            y_min,
            y_max,
        };
        if water {
            self.water_meshes.insert(coord, mesh);
        } else {
            self.opaque_meshes.insert(coord, mesh);
        }
    }

    /// Defers a chunk's GPU buffers for freeing once no in-flight frame references them.
    pub fn remove_chunk_mesh(&mut self, coord: IVec2) {
        for map in [&mut self.opaque_meshes, &mut self.water_meshes] {
            if let Some(mesh) = map.remove(&coord) {
                self.retired.push((self.frame_counter, mesh.vertex));
                self.retired.push((self.frame_counter, mesh.index));
            }
        }
    }

    /// Frees retired buffers older than MAX_FRAMES_IN_FLIGHT frames. Call after wait_for_fence.
    pub fn collect_garbage(&mut self) {
        let threshold = MAX_FRAMES_IN_FLIGHT as u64;
        let mut still_pending = Vec::new();
        for (retired_at, buf) in self.retired.drain(..) {
            if self.frame_counter.saturating_sub(retired_at) >= threshold {
                buf.destroy(&self.device, &self.allocator);
            } else {
                still_pending.push((retired_at, buf));
            }
        }
        self.retired = still_pending;
    }

    /// Records the frame: sky, then opaque terrain, then translucent water.
    pub fn record(
        &self,
        cb: vk::CommandBuffer,
        image_index: usize,
        extent: vk::Extent2D,
        pc: &PushConstants,
        sky: &SkyPushConstants,
    ) {
        let clear_values = [
            vk::ClearValue {
                color: vk::ClearColorValue {
                    float32: HORIZON_COLOR,
                },
            },
            vk::ClearValue {
                depth_stencil: vk::ClearDepthStencilValue {
                    depth: 1.0,
                    stencil: 0,
                },
            },
        ];
        let begin = vk::RenderPassBeginInfo::default()
            .render_pass(self.render_pass)
            .framebuffer(self.framebuffers[image_index])
            .render_area(vk::Rect2D {
                offset: vk::Offset2D::default(),
                extent,
            })
            .clear_values(&clear_values);

        unsafe {
            self.device
                .cmd_begin_render_pass(cb, &begin, vk::SubpassContents::INLINE);

            let viewport = vk::Viewport {
                x: 0.0,
                y: 0.0,
                width: extent.width as f32,
                height: extent.height as f32,
                min_depth: 0.0,
                max_depth: 1.0,
            };
            self.device.cmd_set_viewport(cb, 0, &[viewport]);
            self.device.cmd_set_scissor(
                cb,
                0,
                &[vk::Rect2D {
                    offset: vk::Offset2D::default(),
                    extent,
                }],
            );

            // Sky background (fullscreen triangle, no depth).
            self.device
                .cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, self.sky_pipeline);
            self.device.cmd_push_constants(
                cb,
                self.sky_pipeline_layout,
                vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT,
                0,
                bytemuck::bytes_of(sky),
            );
            self.device.cmd_draw(cb, 3, 1, 0, 0);

            // Frustum planes for CPU-side chunk culling (skips most draws).
            let planes = frustum_planes(&Mat4::from_cols_array_2d(&pc.view_proj));

            // Opaque terrain.
            self.device
                .cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, self.opaque_pipeline);
            self.device.cmd_bind_descriptor_sets(
                cb,
                vk::PipelineBindPoint::GRAPHICS,
                self.pipeline_layout,
                0,
                &[self.desc_set],
                &[],
            );
            self.device.cmd_push_constants(
                cb,
                self.pipeline_layout,
                vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT,
                0,
                bytemuck::bytes_of(pc),
            );
            for (coord, mesh) in &self.opaque_meshes {
                self.draw_mesh_culled(cb, *coord, mesh, &planes);
            }

            // Translucent water (shares layout; descriptor + push constants stay bound).
            self.device
                .cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, self.water_pipeline);
            for (coord, mesh) in &self.water_meshes {
                self.draw_mesh_culled(cb, *coord, mesh, &planes);
            }

            // Developer HUD text overlay (no depth, screen-space pixels).
            if let Some((buf, count)) = &self.hud_buffer {
                self.device.cmd_bind_pipeline(
                    cb,
                    vk::PipelineBindPoint::GRAPHICS,
                    self.text_pipeline,
                );
                self.device.cmd_bind_descriptor_sets(
                    cb,
                    vk::PipelineBindPoint::GRAPHICS,
                    self.text_pipeline_layout,
                    0,
                    &[self.text_desc_set],
                    &[],
                );
                let screen = [extent.width as f32, extent.height as f32];
                self.device.cmd_push_constants(
                    cb,
                    self.text_pipeline_layout,
                    vk::ShaderStageFlags::VERTEX,
                    0,
                    bytemuck::bytes_of(&screen),
                );
                self.device.cmd_bind_vertex_buffers(cb, 0, &[buf.buffer], &[0]);
                self.device.cmd_draw(cb, *count, 1, 0, 0);
            }

            self.device.cmd_end_render_pass(cb);
        }
    }

    /// Frustum-culls the chunk's AABB, pushes its origin, and draws.
    fn draw_mesh_culled(
        &self,
        cb: vk::CommandBuffer,
        coord: IVec2,
        mesh: &ChunkMesh,
        planes: &[Vec4; 6],
    ) {
        let x0 = (coord.x * 16) as f32;
        let z0 = (coord.y * 16) as f32;
        // Pad Y for the water shader's surface inset/waves.
        let min = Vec3::new(x0, mesh.y_min - 0.3, z0);
        let max = Vec3::new(x0 + 16.0, mesh.y_max + 0.3, z0 + 16.0);
        if planes.iter().any(|p| aabb_outside(*p, min, max)) {
            return;
        }

        let origin = [x0, 0.0f32, z0, 0.0];
        unsafe {
            self.device.cmd_push_constants(
                cb,
                self.pipeline_layout,
                vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT,
                ORIGIN_OFFSET,
                bytemuck::bytes_of(&origin),
            );
        }
        self.draw_mesh(cb, mesh);
    }

    fn draw_mesh(&self, cb: vk::CommandBuffer, mesh: &ChunkMesh) {
        unsafe {
            self.device
                .cmd_bind_vertex_buffers(cb, 0, &[mesh.vertex.buffer], &[0]);
            self.device
                .cmd_bind_index_buffer(cb, mesh.index.buffer, 0, vk::IndexType::UINT32);
            self.device.cmd_draw_indexed(cb, mesh.index_count, 1, 0, 0, 0);
        }
    }

    /// Advances the frame counter (after present).
    pub fn end_frame(&mut self) {
        self.frame_counter += 1;
    }
}

impl Drop for Renderer {
    fn drop(&mut self) {
        // App::drop has already waited for device idle, mirroring the simulator crate.
        unsafe {
            let _ = self.device.device_wait_idle();

            for (_, buf) in self.retired.drain(..) {
                buf.destroy(&self.device, &self.allocator);
            }
            if let Some((buf, _)) = self.hud_buffer.take() {
                buf.destroy(&self.device, &self.allocator);
            }
            for map in [&mut self.opaque_meshes, &mut self.water_meshes] {
                for (_, mesh) in map.drain() {
                    mesh.vertex.destroy(&self.device, &self.allocator);
                    mesh.index.destroy(&self.device, &self.allocator);
                }
            }

            self.device.destroy_pipeline(self.text_pipeline, None);
            self.device
                .destroy_pipeline_layout(self.text_pipeline_layout, None);
            self.font.destroy(&self.device, &self.allocator);
            self.device.destroy_pipeline(self.sky_pipeline, None);
            self.device
                .destroy_pipeline_layout(self.sky_pipeline_layout, None);
            self.device.destroy_pipeline(self.water_pipeline, None);
            self.device.destroy_pipeline(self.opaque_pipeline, None);
            self.device
                .destroy_pipeline_layout(self.pipeline_layout, None);
            self.device.destroy_descriptor_pool(self.desc_pool, None);
            self.device
                .destroy_descriptor_set_layout(self.desc_set_layout, None);
            self.atlas.destroy(&self.device, &self.allocator);
            for &fb in &self.framebuffers {
                self.device.destroy_framebuffer(fb, None);
            }
            self.depth_image.destroy(&self.device, &self.allocator);
            self.device.destroy_render_pass(self.render_pass, None);
        }
    }
}

/// Extracts the 6 frustum planes (Gribb–Hartmann) from a Vulkan [0,1]-depth view-proj.
fn frustum_planes(m: &Mat4) -> [Vec4; 6] {
    let r0 = m.row(0);
    let r1 = m.row(1);
    let r2 = m.row(2);
    let r3 = m.row(3);
    [r3 + r0, r3 - r0, r3 + r1, r3 - r1, r2, r3 - r2]
}

/// True when the AABB lies entirely on the negative side of the plane (outside).
fn aabb_outside(plane: Vec4, min: Vec3, max: Vec3) -> bool {
    let p = Vec3::new(
        if plane.x >= 0.0 { max.x } else { min.x },
        if plane.y >= 0.0 { max.y } else { min.y },
        if plane.z >= 0.0 { max.z } else { min.z },
    );
    plane.xyz().dot(p) + plane.w < 0.0
}

/// Picks the first supported depth format with optimal-tiling depth-stencil support.
fn select_depth_format(instance: &ash::Instance, pd: vk::PhysicalDevice) -> vk::Format {
    for &fmt in &[
        vk::Format::D32_SFLOAT,
        vk::Format::D32_SFLOAT_S8_UINT,
        vk::Format::D24_UNORM_S8_UINT,
    ] {
        let props = unsafe { instance.get_physical_device_format_properties(pd, fmt) };
        if props
            .optimal_tiling_features
            .contains(vk::FormatFeatureFlags::DEPTH_STENCIL_ATTACHMENT)
        {
            return fmt;
        }
    }
    panic!("No supported depth format found");
}

/// Render pass with a swapchain color attachment + a depth attachment.
fn create_render_pass(
    device: &ash::Device,
    color_format: vk::Format,
    depth_format: vk::Format,
) -> vk::RenderPass {
    let color = vk::AttachmentDescription::default()
        .format(color_format)
        .samples(vk::SampleCountFlags::TYPE_1)
        .load_op(vk::AttachmentLoadOp::CLEAR)
        .store_op(vk::AttachmentStoreOp::STORE)
        .stencil_load_op(vk::AttachmentLoadOp::DONT_CARE)
        .stencil_store_op(vk::AttachmentStoreOp::DONT_CARE)
        .initial_layout(vk::ImageLayout::UNDEFINED)
        .final_layout(vk::ImageLayout::PRESENT_SRC_KHR);

    let depth = vk::AttachmentDescription::default()
        .format(depth_format)
        .samples(vk::SampleCountFlags::TYPE_1)
        .load_op(vk::AttachmentLoadOp::CLEAR)
        .store_op(vk::AttachmentStoreOp::DONT_CARE)
        .stencil_load_op(vk::AttachmentLoadOp::DONT_CARE)
        .stencil_store_op(vk::AttachmentStoreOp::DONT_CARE)
        .initial_layout(vk::ImageLayout::UNDEFINED)
        .final_layout(vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL);

    let color_ref = vk::AttachmentReference {
        attachment: 0,
        layout: vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL,
    };
    let depth_ref = vk::AttachmentReference {
        attachment: 1,
        layout: vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
    };
    let color_refs = [color_ref];

    let subpass = vk::SubpassDescription::default()
        .pipeline_bind_point(vk::PipelineBindPoint::GRAPHICS)
        .color_attachments(&color_refs)
        .depth_stencil_attachment(&depth_ref);

    let dependency = vk::SubpassDependency::default()
        .src_subpass(vk::SUBPASS_EXTERNAL)
        .dst_subpass(0)
        .src_stage_mask(
            vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT
                | vk::PipelineStageFlags::EARLY_FRAGMENT_TESTS,
        )
        .src_access_mask(vk::AccessFlags::empty())
        .dst_stage_mask(
            vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT
                | vk::PipelineStageFlags::EARLY_FRAGMENT_TESTS,
        )
        .dst_access_mask(
            vk::AccessFlags::COLOR_ATTACHMENT_WRITE
                | vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_WRITE,
        );

    let attachments = [color, depth];
    let subpasses = [subpass];
    let dependencies = [dependency];
    let ci = vk::RenderPassCreateInfo::default()
        .attachments(&attachments)
        .subpasses(&subpasses)
        .dependencies(&dependencies);
    unsafe { device.create_render_pass(&ci, None) }.unwrap()
}

fn create_depth_image(
    device: &ash::Device,
    allocator: &Mutex<Allocator>,
    format: vk::Format,
    extent: vk::Extent2D,
) -> AllocatedImage {
    AllocatedImage::new(
        device,
        allocator,
        extent.width.max(1),
        extent.height.max(1),
        format,
        vk::ImageUsageFlags::DEPTH_STENCIL_ATTACHMENT,
        vk::ImageAspectFlags::DEPTH,
        "depth-buffer",
    )
}

fn create_framebuffers(
    device: &ash::Device,
    render_pass: vk::RenderPass,
    color_views: &[vk::ImageView],
    depth_view: vk::ImageView,
    extent: vk::Extent2D,
) -> Vec<vk::Framebuffer> {
    color_views
        .iter()
        .map(|&cv| {
            let attachments = [cv, depth_view];
            let ci = vk::FramebufferCreateInfo::default()
                .render_pass(render_pass)
                .attachments(&attachments)
                .width(extent.width)
                .height(extent.height)
                .layers(1);
            unsafe { device.create_framebuffer(&ci, None) }.unwrap()
        })
        .collect()
}

/// Builds a voxel pipeline. `translucent` enables alpha blending with depth-write off
/// and disables culling (the water surface must be visible from below); otherwise
/// opaque with depth write and back-face culling.
fn create_voxel_pipeline(
    device: &ash::Device,
    render_pass: vk::RenderPass,
    layout: vk::PipelineLayout,
    vs_spv: &[u8],
    fs_spv: &[u8],
    translucent: bool,
) -> vk::Pipeline {
    let vs = create_shader_module(device, vs_spv);
    let fs = create_shader_module(device, fs_spv);

    let entry = c"main";
    let stages = [
        vk::PipelineShaderStageCreateInfo::default()
            .stage(vk::ShaderStageFlags::VERTEX)
            .module(vs)
            .name(entry),
        vk::PipelineShaderStageCreateInfo::default()
            .stage(vk::ShaderStageFlags::FRAGMENT)
            .module(fs)
            .name(entry),
    ];

    // Compact 12-byte vertex: u16 local pos + u16 light at offset 0, u16x2 quantized
    // uv at offset 8 (see mesher::VoxelVertex).
    let binding = vk::VertexInputBindingDescription {
        binding: 0,
        stride: std::mem::size_of::<VoxelVertex>() as u32,
        input_rate: vk::VertexInputRate::VERTEX,
    };
    let bindings = [binding];
    let attrs = [
        vk::VertexInputAttributeDescription {
            location: 0,
            binding: 0,
            format: vk::Format::R16G16B16A16_UINT,
            offset: 0,
        },
        vk::VertexInputAttributeDescription {
            location: 1,
            binding: 0,
            format: vk::Format::R16G16_UINT,
            offset: 8,
        },
    ];
    let vertex_input = vk::PipelineVertexInputStateCreateInfo::default()
        .vertex_binding_descriptions(&bindings)
        .vertex_attribute_descriptions(&attrs);

    let input_assembly = vk::PipelineInputAssemblyStateCreateInfo::default()
        .topology(vk::PrimitiveTopology::TRIANGLE_LIST);

    let viewport_state = vk::PipelineViewportStateCreateInfo::default()
        .viewport_count(1)
        .scissor_count(1);

    let rasterizer = vk::PipelineRasterizationStateCreateInfo::default()
        .polygon_mode(vk::PolygonMode::FILL)
        .line_width(1.0)
        .cull_mode(if translucent {
            vk::CullModeFlags::NONE
        } else {
            vk::CullModeFlags::BACK
        })
        .front_face(vk::FrontFace::COUNTER_CLOCKWISE);

    let multisampling = vk::PipelineMultisampleStateCreateInfo::default()
        .rasterization_samples(vk::SampleCountFlags::TYPE_1);

    let depth_stencil = vk::PipelineDepthStencilStateCreateInfo::default()
        .depth_test_enable(true)
        .depth_write_enable(!translucent)
        .depth_compare_op(vk::CompareOp::LESS);

    let blend = if translucent {
        vk::PipelineColorBlendAttachmentState::default()
            .color_write_mask(vk::ColorComponentFlags::RGBA)
            .blend_enable(true)
            .src_color_blend_factor(vk::BlendFactor::SRC_ALPHA)
            .dst_color_blend_factor(vk::BlendFactor::ONE_MINUS_SRC_ALPHA)
            .color_blend_op(vk::BlendOp::ADD)
            .src_alpha_blend_factor(vk::BlendFactor::ONE)
            .dst_alpha_blend_factor(vk::BlendFactor::ZERO)
            .alpha_blend_op(vk::BlendOp::ADD)
    } else {
        vk::PipelineColorBlendAttachmentState::default()
            .color_write_mask(vk::ColorComponentFlags::RGBA)
            .blend_enable(false)
    };
    let blend_attachments = [blend];
    let color_blending =
        vk::PipelineColorBlendStateCreateInfo::default().attachments(&blend_attachments);

    let dynamic_states = [vk::DynamicState::VIEWPORT, vk::DynamicState::SCISSOR];
    let dynamic_state =
        vk::PipelineDynamicStateCreateInfo::default().dynamic_states(&dynamic_states);

    let ci = vk::GraphicsPipelineCreateInfo::default()
        .stages(&stages)
        .vertex_input_state(&vertex_input)
        .input_assembly_state(&input_assembly)
        .viewport_state(&viewport_state)
        .rasterization_state(&rasterizer)
        .multisample_state(&multisampling)
        .depth_stencil_state(&depth_stencil)
        .color_blend_state(&color_blending)
        .dynamic_state(&dynamic_state)
        .layout(layout)
        .render_pass(render_pass)
        .subpass(0);

    let pipeline = unsafe {
        device
            .create_graphics_pipelines(vk::PipelineCache::null(), &[ci], None)
            .unwrap()[0]
    };
    unsafe {
        device.destroy_shader_module(vs, None);
        device.destroy_shader_module(fs, None);
    }
    pipeline
}

/// HUD text pipeline: screen-space quads, alpha blended, no depth test/write.
fn create_text_pipeline(
    device: &ash::Device,
    render_pass: vk::RenderPass,
    layout: vk::PipelineLayout,
) -> vk::Pipeline {
    let vs = create_shader_module(device, TEXT_VERT);
    let fs = create_shader_module(device, TEXT_FRAG);

    let entry = c"main";
    let stages = [
        vk::PipelineShaderStageCreateInfo::default()
            .stage(vk::ShaderStageFlags::VERTEX)
            .module(vs)
            .name(entry),
        vk::PipelineShaderStageCreateInfo::default()
            .stage(vk::ShaderStageFlags::FRAGMENT)
            .module(fs)
            .name(entry),
    ];

    let binding = vk::VertexInputBindingDescription {
        binding: 0,
        stride: std::mem::size_of::<TextVertex>() as u32,
        input_rate: vk::VertexInputRate::VERTEX,
    };
    let bindings = [binding];
    let attrs = [
        vk::VertexInputAttributeDescription {
            location: 0,
            binding: 0,
            format: vk::Format::R32G32_SFLOAT,
            offset: 0,
        },
        vk::VertexInputAttributeDescription {
            location: 1,
            binding: 0,
            format: vk::Format::R32G32_SFLOAT,
            offset: 8,
        },
        vk::VertexInputAttributeDescription {
            location: 2,
            binding: 0,
            format: vk::Format::R32G32B32A32_SFLOAT,
            offset: 16,
        },
    ];
    let vertex_input = vk::PipelineVertexInputStateCreateInfo::default()
        .vertex_binding_descriptions(&bindings)
        .vertex_attribute_descriptions(&attrs);

    let input_assembly = vk::PipelineInputAssemblyStateCreateInfo::default()
        .topology(vk::PrimitiveTopology::TRIANGLE_LIST);
    let viewport_state = vk::PipelineViewportStateCreateInfo::default()
        .viewport_count(1)
        .scissor_count(1);
    let rasterizer = vk::PipelineRasterizationStateCreateInfo::default()
        .polygon_mode(vk::PolygonMode::FILL)
        .line_width(1.0)
        .cull_mode(vk::CullModeFlags::NONE)
        .front_face(vk::FrontFace::COUNTER_CLOCKWISE);
    let multisampling = vk::PipelineMultisampleStateCreateInfo::default()
        .rasterization_samples(vk::SampleCountFlags::TYPE_1);
    let depth_stencil = vk::PipelineDepthStencilStateCreateInfo::default()
        .depth_test_enable(false)
        .depth_write_enable(false);
    let blend = vk::PipelineColorBlendAttachmentState::default()
        .color_write_mask(vk::ColorComponentFlags::RGBA)
        .blend_enable(true)
        .src_color_blend_factor(vk::BlendFactor::SRC_ALPHA)
        .dst_color_blend_factor(vk::BlendFactor::ONE_MINUS_SRC_ALPHA)
        .color_blend_op(vk::BlendOp::ADD)
        .src_alpha_blend_factor(vk::BlendFactor::ONE)
        .dst_alpha_blend_factor(vk::BlendFactor::ZERO)
        .alpha_blend_op(vk::BlendOp::ADD);
    let blend_attachments = [blend];
    let color_blending =
        vk::PipelineColorBlendStateCreateInfo::default().attachments(&blend_attachments);
    let dynamic_states = [vk::DynamicState::VIEWPORT, vk::DynamicState::SCISSOR];
    let dynamic_state =
        vk::PipelineDynamicStateCreateInfo::default().dynamic_states(&dynamic_states);

    let ci = vk::GraphicsPipelineCreateInfo::default()
        .stages(&stages)
        .vertex_input_state(&vertex_input)
        .input_assembly_state(&input_assembly)
        .viewport_state(&viewport_state)
        .rasterization_state(&rasterizer)
        .multisample_state(&multisampling)
        .depth_stencil_state(&depth_stencil)
        .color_blend_state(&color_blending)
        .dynamic_state(&dynamic_state)
        .layout(layout)
        .render_pass(render_pass)
        .subpass(0);

    let pipeline = unsafe {
        device
            .create_graphics_pipelines(vk::PipelineCache::null(), &[ci], None)
            .unwrap()[0]
    };
    unsafe {
        device.destroy_shader_module(vs, None);
        device.destroy_shader_module(fs, None);
    }
    pipeline
}

/// Fullscreen gradient-sky pipeline: no vertex input, no depth test/write, no culling.
fn create_sky_pipeline(
    device: &ash::Device,
    render_pass: vk::RenderPass,
    layout: vk::PipelineLayout,
) -> vk::Pipeline {
    let vs = create_shader_module(device, SKY_VERT);
    let fs = create_shader_module(device, SKY_FRAG);

    let entry = c"main";
    let stages = [
        vk::PipelineShaderStageCreateInfo::default()
            .stage(vk::ShaderStageFlags::VERTEX)
            .module(vs)
            .name(entry),
        vk::PipelineShaderStageCreateInfo::default()
            .stage(vk::ShaderStageFlags::FRAGMENT)
            .module(fs)
            .name(entry),
    ];

    let vertex_input = vk::PipelineVertexInputStateCreateInfo::default();
    let input_assembly = vk::PipelineInputAssemblyStateCreateInfo::default()
        .topology(vk::PrimitiveTopology::TRIANGLE_LIST);
    let viewport_state = vk::PipelineViewportStateCreateInfo::default()
        .viewport_count(1)
        .scissor_count(1);
    let rasterizer = vk::PipelineRasterizationStateCreateInfo::default()
        .polygon_mode(vk::PolygonMode::FILL)
        .line_width(1.0)
        .cull_mode(vk::CullModeFlags::NONE)
        .front_face(vk::FrontFace::COUNTER_CLOCKWISE);
    let multisampling = vk::PipelineMultisampleStateCreateInfo::default()
        .rasterization_samples(vk::SampleCountFlags::TYPE_1);
    let depth_stencil = vk::PipelineDepthStencilStateCreateInfo::default()
        .depth_test_enable(false)
        .depth_write_enable(false);
    let blend = vk::PipelineColorBlendAttachmentState::default()
        .color_write_mask(vk::ColorComponentFlags::RGBA)
        .blend_enable(false);
    let blend_attachments = [blend];
    let color_blending =
        vk::PipelineColorBlendStateCreateInfo::default().attachments(&blend_attachments);
    let dynamic_states = [vk::DynamicState::VIEWPORT, vk::DynamicState::SCISSOR];
    let dynamic_state =
        vk::PipelineDynamicStateCreateInfo::default().dynamic_states(&dynamic_states);

    let ci = vk::GraphicsPipelineCreateInfo::default()
        .stages(&stages)
        .vertex_input_state(&vertex_input)
        .input_assembly_state(&input_assembly)
        .viewport_state(&viewport_state)
        .rasterization_state(&rasterizer)
        .multisample_state(&multisampling)
        .depth_stencil_state(&depth_stencil)
        .color_blend_state(&color_blending)
        .dynamic_state(&dynamic_state)
        .layout(layout)
        .render_pass(render_pass)
        .subpass(0);

    let pipeline = unsafe {
        device
            .create_graphics_pipelines(vk::PipelineCache::null(), &[ci], None)
            .unwrap()[0]
    };
    unsafe {
        device.destroy_shader_module(vs, None);
        device.destroy_shader_module(fs, None);
    }
    pipeline
}
