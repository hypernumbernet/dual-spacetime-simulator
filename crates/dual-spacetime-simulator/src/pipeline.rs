use crate::camera::OrbitCamera;
use crate::integration::Gui;
use crate::ui_state::*;
use ash::vk;
use glam::{Mat4, Vec3};
use gpu_allocator::vulkan::Allocator;
use std::sync::{Arc, Mutex};
use vulkanvil::{create_buffer_with_data, create_shader_module, AllocatedBuffer, VulkanBase};

const MOUSE_LEFT_DRAG_SENS: f32 = 0.003f32;
const MOUSE_RIGHT_DRAG_SENS: f32 = 0.001f32;
const SIZE_RATIO: f32 = 0.06;
const INITIAL_POSITION: Vec3 = Vec3::new(1.6, -1.6, 3.0);
const INITIAL_TARGET: Vec3 = Vec3::new(0.0, 0.0, 0.0);
const AXIS_XZ_GRID_EXTENT: f32 = 2.0;
const AXIS_XZ_GRID_LINE_COUNT: usize = 9;
const ADD_CENTER_MARKER_VERTICES: usize = 36;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct AxesVertex {
    position: [f32; 3],
    color: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct ParticleVertex {
    position: [f32; 3],
    color: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct AxesPushConstants {
    view_proj: [[f32; 4]; 4],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct PushConstants {
    view_proj: [[f32; 4]; 4],
    size_scale: f32,
}

pub struct ParticleRenderPipeline {
    device: ash::Device,
    allocator: Arc<Mutex<Allocator>>,

    render_pass: vk::RenderPass,
    framebuffers: Vec<vk::Framebuffer>,

    pipeline_axes: vk::Pipeline,
    pipeline_particles: vk::Pipeline,
    layout_axes: vk::PipelineLayout,
    layout_particles: vk::PipelineLayout,

    axes_buffer: AllocatedBuffer,
    axes_vertex_count: u32,
    graph_lines_buffer: Option<AllocatedBuffer>,
    graph_lines_vertex_count: u32,
    add_center_marker_buffer: Option<AllocatedBuffer>,
    add_center_marker_vertex_count: u32,
    last_add_center_marker_key: Option<(glam::DVec3, u64)>,
    particle_buffer: AllocatedBuffer,
    particle_draw_vertex_count: u32,
    retired_buffers: Vec<AllocatedBuffer>,

    camera: OrbitCamera,
}

impl ParticleRenderPipeline {
    /// Creates graphics and compute pipelines with all persistent rendering resources.
    pub fn new(base: &VulkanBase) -> Self {
        let device = base.device.clone();
        let allocator = Arc::clone(base.allocator.as_ref().unwrap());

        let render_pass = create_render_pass(&device, base.swapchain_format);
        let framebuffers = create_framebuffers(
            &device,
            render_pass,
            &base.swapchain_image_views,
            base.swapchain_extent,
        );

        let (layout_axes, pipeline_axes) = create_axes_pipeline(&device, render_pass);
        let (layout_particles, pipeline_particles) = create_particles_pipeline(&device, render_pass);

        let (axes_buffer, axes_vertex_count) =
            create_axes_vertices(&device, &allocator);
        let (particle_buffer, particle_vertex_count) =
            create_initial_particles(&device, &allocator);

        let camera = OrbitCamera::new(INITIAL_POSITION, INITIAL_TARGET);

        Self {
            device,
            allocator,
            render_pass,
            framebuffers,
            pipeline_axes,
            pipeline_particles,
            layout_axes,
            layout_particles,
            axes_buffer,
            axes_vertex_count,
            graph_lines_buffer: None,
            graph_lines_vertex_count: 0,
            add_center_marker_buffer: None,
            add_center_marker_vertex_count: 0,
            last_add_center_marker_key: None,
            particle_buffer,
            particle_draw_vertex_count: particle_vertex_count,
            retired_buffers: Vec::new(),
            camera,
        }
    }

    /// Returns shared allocator used for dynamic GPU buffer management.
    fn allocator(&self) -> &Mutex<Allocator> {
        &self.allocator
    }

    /// Returns render pass handle used by graphics pipelines.
    pub fn render_pass(&self) -> vk::RenderPass {
        self.render_pass
    }

    /// Recreates framebuffers to match current swapchain image views.
    pub fn recreate_framebuffers(&mut self, base: &VulkanBase) {
        for fb in self.framebuffers.drain(..) {
            unsafe { self.device.destroy_framebuffer(fb, None) };
        }
        self.framebuffers = create_framebuffers(
            &self.device,
            self.render_pass,
            &base.swapchain_image_views,
            base.swapchain_extent,
        );
    }

    /// Records full frame rendering commands for scene geometry and UI.
    pub fn render(
        &mut self,
        command_buffer: vk::CommandBuffer,
        framebuffer_index: usize,
        extent: vk::Extent2D,
        gui: &mut Gui,
        scale: f64,
        link_point_size_to_scale: bool,
        show_grid: bool,
        app_mode: AppMode,
    ) {
        self.flush_retired_buffers();
        let clear_values = [vk::ClearValue {
            color: vk::ClearColorValue {
                float32: [0.0, 0.0, 0.0, 1.0],
            },
        }];
        let render_pass_info = vk::RenderPassBeginInfo::default()
            .render_pass(self.render_pass)
            .framebuffer(self.framebuffers[framebuffer_index])
            .render_area(vk::Rect2D {
                offset: vk::Offset2D::default(),
                extent,
            })
            .clear_values(&clear_values);

        unsafe {
            self.device.cmd_begin_render_pass(
                command_buffer,
                &render_pass_info,
                vk::SubpassContents::INLINE,
            );
        }

        let viewport = vk::Viewport {
            x: 0.0,
            y: 0.0,
            width: extent.width as f32,
            height: extent.height as f32,
            min_depth: 0.0,
            max_depth: 1.0,
        };
        unsafe {
            self.device
                .cmd_set_viewport(command_buffer, 0, &[viewport]);
            self.device.cmd_set_scissor(
                command_buffer,
                0,
                &[vk::Rect2D {
                    offset: vk::Offset2D::default(),
                    extent,
                }],
            );
        }

        let aspect_ratio = extent.width as f32 / extent.height as f32;

        if show_grid {
            let view_proj = self.compute_mvp_axes(aspect_ratio);
            let pc = AxesPushConstants {
                view_proj: view_proj.to_cols_array_2d(),
            };
            self.draw_axes(command_buffer, &pc);
        }

        let scale_factor = (scale / DEFAULT_SCALE_UI).powi(4) as f32;
        let view_proj = self.compute_mvp_particle(aspect_ratio, scale_factor);
        let point_scale_factor = if link_point_size_to_scale {
            scale_factor
        } else {
            1.0
        };
        let size_scale = extent.height as f32 * SIZE_RATIO * point_scale_factor;
        let pc = PushConstants {
            view_proj: view_proj.to_cols_array_2d(),
            size_scale,
        };

        let line_pc = AxesPushConstants {
            view_proj: view_proj.to_cols_array_2d(),
        };

        if app_mode == AppMode::Graph3D {
            if let Some(ref buf) = self.graph_lines_buffer {
                self.draw_axes_lines(
                    command_buffer,
                    &line_pc,
                    buf.buffer,
                    self.graph_lines_vertex_count,
                );
            }
        }

        self.draw_particles(command_buffer, &pc);

        if app_mode == AppMode::Simulation {
            if let Some(ref buf) = self.add_center_marker_buffer {
                self.draw_axes_lines(
                    command_buffer,
                    &line_pc,
                    buf.buffer,
                    self.add_center_marker_vertex_count,
                );
            }
        }

        gui.draw(command_buffer, extent);

        unsafe {
            self.device.cmd_end_render_pass(command_buffer);
        }
    }

    /// Uploads particle point data and rebuilds the particle vertex buffer.
    pub fn set_particles(&mut self, positions: &[[f32; 3]], colors: &[[f32; 4]]) {
        let mut verts: Vec<ParticleVertex> = positions
            .iter()
            .zip(colors.iter())
            .map(|(p, c)| ParticleVertex {
                position: *p,
                color: *c,
            })
            .collect();
        let draw_n = verts.len() as u32;
        if verts.is_empty() {
            verts.push(ParticleVertex {
                position: [0.0; 3],
                color: [0.0; 4],
            });
        }

        let alloc = Arc::clone(&self.allocator);
        let (new_buf, _) = create_buffer_with_data(
            &self.device,
            &alloc,
            &verts,
            vk::BufferUsageFlags::VERTEX_BUFFER,
            "particle_vertices",
        );
        let old = std::mem::replace(&mut self.particle_buffer, new_buf);
        self.retire_buffer(old);
        self.particle_draw_vertex_count = draw_n;
    }

    /// Uploads graph line vertices and rebuilds the line vertex buffer.
    pub fn set_graph_lines(&mut self, vertices: &[([f32; 3], [f32; 4])]) {
        let verts = axes_vertices_from_tuples(vertices);
        let allocator = Arc::clone(&self.allocator);
        upload_axes_line_buffer(
            &self.device,
            &allocator,
            &mut self.retired_buffers,
            &mut self.graph_lines_buffer,
            &mut self.graph_lines_vertex_count,
            &verts,
            "graph_lines",
        );
    }

    /// Uploads add-center preview cross vertices and rebuilds the marker line buffer.
    pub fn set_add_center_marker(&mut self, vertices: &[([f32; 3], [f32; 4])]) {
        let verts = axes_vertices_from_tuples(vertices);
        let allocator = Arc::clone(&self.allocator);
        upload_axes_line_buffer(
            &self.device,
            &allocator,
            &mut self.retired_buffers,
            &mut self.add_center_marker_buffer,
            &mut self.add_center_marker_vertex_count,
            &verts,
            "add_center_marker",
        );
    }

    /// Rebuilds add-center preview geometry from current UI state when needed.
    pub fn sync_add_center_marker(&mut self, ui_state: &crate::ui_state::UiState) {
        use crate::object_input::ObjectInput;
        use crate::ui_state::AppMode;

        let show_marker = ui_state.app_mode == AppMode::Simulation
            && ui_state.is_object_input_panel_open
            && ui_state.show_add_center_preview;
        if !show_marker {
            self.add_center_marker_vertex_count = 0;
            self.last_add_center_marker_key = None;
            return;
        }

        let marker_key = (ui_state.add_center, ui_state.base_scale.to_bits());
        if self.last_add_center_marker_key == Some(marker_key) {
            return;
        }
        self.last_add_center_marker_key = Some(marker_key);

        let world =
            ObjectInput::add_center_world_position(ui_state.add_center, ui_state.base_scale);
        let center = [world.x as f32, world.y as f32, world.z as f32];
        let half_extent = ObjectInput::add_center_marker_half_extent(ui_state.base_scale);
        let verts = build_add_center_marker_vertices(center, half_extent);
        let allocator = Arc::clone(&self.allocator);
        upload_axes_line_buffer(
            &self.device,
            &allocator,
            &mut self.retired_buffers,
            &mut self.add_center_marker_buffer,
            &mut self.add_center_marker_vertex_count,
            &verts,
            "add_center_marker",
        );
    }

    /// Clears add-center preview geometry and cached sync state.
    pub fn reset_add_center_marker(&mut self) {
        let allocator = Arc::clone(&self.allocator);
        upload_axes_line_buffer(
            &self.device,
            &allocator,
            &mut self.retired_buffers,
            &mut self.add_center_marker_buffer,
            &mut self.add_center_marker_vertex_count,
            &[],
            "add_center_marker",
        );
        self.last_add_center_marker_key = None;
    }

    // --- Camera methods ---

    /// Rotates camera around target using viewport-relative yaw and pitch deltas.
    pub fn revolve_camera(&mut self, delta_yaw: f64, delta_pitch: f64) {
        self.camera.revolve(
            delta_yaw as f32 * MOUSE_LEFT_DRAG_SENS,
            delta_pitch as f32 * MOUSE_LEFT_DRAG_SENS,
        );
    }

    /// Rotates camera view direction in place from cursor deltas.
    pub fn look_around(&mut self, dx: f64, dy: f64) {
        self.camera.look_around(
            dx as f32 * MOUSE_RIGHT_DRAG_SENS,
            dy as f32 * MOUSE_RIGHT_DRAG_SENS,
        );
    }

    /// Applies camera zoom toward or away from target.
    pub fn zoom_camera(&mut self, zoom_factor: f32) {
        self.camera.zoom(zoom_factor);
    }

    /// Rotates camera roll around screen-center-aware gesture input.
    pub fn rotate_camera(
        &mut self,
        x: f64,
        lx: f64,
        y: f64,
        ly: f64,
        center_x: f64,
        center_y: f64,
    ) {
        let prev_angle = (ly - center_y).atan2(lx - center_x);
        let current_angle = (y - center_y).atan2(x - center_x);
        let delta_roll = current_angle - prev_angle;
        self.camera.rotate(delta_roll as f32);
    }

    /// Triggers camera up-vector alignment animation.
    pub fn y_top(&mut self) {
        self.camera.y_top();
    }

    /// Triggers camera target-centering animation toward world origin.
    pub fn center_target_on_origin(&mut self) {
        self.camera.center_target_on_origin();
    }

    /// Advances camera animation state.
    pub fn update_animation(&mut self) {
        self.camera.update_animation();
    }

    /// Enables or disables camera up-lock behavior.
    pub fn set_lock_camera_up(&mut self, lock: bool) {
        self.camera.set_lock_up(lock);
    }

    // --- Draw helpers ---

    /// Records draw commands for axis and grid line geometry.
    fn draw_axes(&self, cb: vk::CommandBuffer, pc: &AxesPushConstants) {
        self.draw_axes_lines(cb, pc, self.axes_buffer.buffer, self.axes_vertex_count);
    }

    fn draw_axes_lines(
        &self,
        cb: vk::CommandBuffer,
        pc: &AxesPushConstants,
        buffer: vk::Buffer,
        vertex_count: u32,
    ) {
        if vertex_count == 0 {
            return;
        }
        unsafe {
            self.device
                .cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, self.pipeline_axes);
            self.device
                .cmd_bind_vertex_buffers(cb, 0, &[buffer], &[0]);
            self.device.cmd_push_constants(
                cb,
                self.layout_axes,
                vk::ShaderStageFlags::VERTEX,
                0,
                bytemuck::bytes_of(pc),
            );
            self.device.cmd_draw(cb, vertex_count, 1, 0, 0);
        }
    }

    /// Records draw commands for particle point geometry.
    fn draw_particles(&self, cb: vk::CommandBuffer, pc: &PushConstants) {
        if self.particle_draw_vertex_count == 0 {
            return;
        }
        unsafe {
            self.device.cmd_bind_pipeline(
                cb,
                vk::PipelineBindPoint::GRAPHICS,
                self.pipeline_particles,
            );
            self.device
                .cmd_bind_vertex_buffers(cb, 0, &[self.particle_buffer.buffer], &[0]);
            self.device.cmd_push_constants(
                cb,
                self.layout_particles,
                vk::ShaderStageFlags::VERTEX,
                0,
                bytemuck::bytes_of(pc),
            );
            self.device
                .cmd_draw(cb, self.particle_draw_vertex_count, 1, 0, 0);
        }
    }

    /// Computes model-view-projection transform for axes and helper geometry.
    fn compute_mvp_axes(&self, aspect_ratio: f32) -> Mat4 {
        let view = Mat4::look_at_rh(self.camera.position, self.camera.target, self.camera.up);
        let proj = Mat4::perspective_rh(std::f32::consts::FRAC_PI_4, aspect_ratio, 0.1, 100.0);
        proj * view
    }

    /// Computes model-view-projection transform for particle-space rendering.
    fn compute_mvp_particle(&self, aspect_ratio: f32, scale_factor: f32) -> Mat4 {
        let view = Mat4::look_at_rh(self.camera.position, self.camera.target, self.camera.up);
        let proj = Mat4::perspective_rh(std::f32::consts::FRAC_PI_4, aspect_ratio, 0.1, 100.0);
        let model = Mat4::from_scale(Vec3::splat(scale_factor));
        proj * view * model
    }

    /// Defers buffer destruction until a synchronized frame boundary.
    fn retire_buffer(&mut self, buffer: AllocatedBuffer) {
        self.retired_buffers.push(buffer);
    }

    /// Releases deferred buffers after `wait_for_fence` has completed for the frame.
    fn flush_retired_buffers(&mut self) {
        if self.retired_buffers.is_empty() {
            return;
        }
        let allocator = Arc::clone(&self.allocator);
        for retired in self.retired_buffers.drain(..) {
            retired.destroy(&self.device, &allocator);
        }
    }
}

impl Drop for ParticleRenderPipeline {
    /// Releases all pipeline-owned Vulkan resources and transient buffers.
    fn drop(&mut self) {
        unsafe {
            if let Err(err) = self.device.device_wait_idle() {
                eprintln!("ParticleRenderPipeline::drop device_wait_idle failed: {err:?}");
            }

            if let Some(buf) = self.graph_lines_buffer.take() {
                buf.destroy(&self.device, self.allocator());
            }
            if let Some(buf) = self.add_center_marker_buffer.take() {
                buf.destroy(&self.device, self.allocator());
            }
            self.flush_retired_buffers();

            if let Some(alloc) = self.particle_buffer.allocation.take() {
                self.allocator().lock().unwrap().free(alloc).unwrap();
            }
            self.device
                .destroy_buffer(self.particle_buffer.buffer, None);

            if let Some(alloc) = self.axes_buffer.allocation.take() {
                self.allocator().lock().unwrap().free(alloc).unwrap();
            }
            self.device.destroy_buffer(self.axes_buffer.buffer, None);

            for fb in &self.framebuffers {
                self.device.destroy_framebuffer(*fb, None);
            }
            self.device.destroy_pipeline(self.pipeline_axes, None);
            self.device.destroy_pipeline(self.pipeline_particles, None);
            self.device.destroy_pipeline_layout(self.layout_axes, None);
            self.device
                .destroy_pipeline_layout(self.layout_particles, None);
            self.device.destroy_render_pass(self.render_pass, None);
        }
    }
}

/// Builds an octahedron marker with colored axis spokes at the target center.
pub fn build_add_center_cross(
    center: [f32; 3],
    half_extent: f32,
) -> [([f32; 3], [f32; 4]); ADD_CENTER_MARKER_VERTICES] {
    let verts = build_add_center_marker_vertices(center, half_extent);
    std::array::from_fn(|i| (verts[i].position, verts[i].color))
}

fn add_center_marker_edge(
    p0: [f32; 3],
    p1: [f32; 3],
    color: [f32; 4],
) -> [AxesVertex; 2] {
    [
        AxesVertex {
            position: p0,
            color,
        },
        AxesVertex {
            position: p1,
            color,
        },
    ]
}

fn build_add_center_marker_vertices(
    center: [f32; 3],
    half_extent: f32,
) -> [AxesVertex; ADD_CENTER_MARKER_VERTICES] {
    let [cx, cy, cz] = center;
    let a = half_extent;
    let x_color = [1.0, 0.2, 0.2, 1.0];
    let y_color = [0.2, 1.0, 0.2, 1.0];
    let z_color = [0.3, 0.5, 1.0, 1.0];
    let white = [1.0, 1.0, 1.0, 1.0];

    let px = [cx + a, cy, cz];
    let nx = [cx - a, cy, cz];
    let py = [cx, cy + a, cz];
    let ny = [cx, cy - a, cz];
    let pz = [cx, cy, cz + a];
    let nz = [cx, cy, cz - a];

    let mut verts = [AxesVertex {
        position: [0.0; 3],
        color: [0.0; 4],
    }; ADD_CENTER_MARKER_VERTICES];
    let mut i = 0;
    let mut push_edge = |p0: [f32; 3], p1: [f32; 3], color: [f32; 4]| {
        let edge = add_center_marker_edge(p0, p1, color);
        verts[i] = edge[0];
        verts[i + 1] = edge[1];
        i += 2;
    };

    push_edge(center, px, x_color);
    push_edge(center, nx, x_color);
    push_edge(center, py, y_color);
    push_edge(center, ny, y_color);
    push_edge(center, pz, z_color);
    push_edge(center, nz, z_color);

    push_edge(px, py, white);
    push_edge(px, ny, white);
    push_edge(px, pz, white);
    push_edge(px, nz, white);
    push_edge(nx, py, white);
    push_edge(nx, ny, white);
    push_edge(nx, pz, white);
    push_edge(nx, nz, white);
    push_edge(py, pz, white);
    push_edge(py, nz, white);
    push_edge(ny, pz, white);
    push_edge(ny, nz, white);

    verts
}

fn upload_axes_line_buffer(
    device: &ash::Device,
    allocator: &Mutex<Allocator>,
    retired_buffers: &mut Vec<AllocatedBuffer>,
    buffer: &mut Option<AllocatedBuffer>,
    vertex_count: &mut u32,
    vertices: &[AxesVertex],
    label: &str,
) {
    if vertices.is_empty() {
        if let Some(old) = buffer.take() {
            retired_buffers.push(old);
        }
        *vertex_count = 0;
        return;
    }

    if let Some(buf) = buffer.as_ref() {
        if write_mapped_axes_vertices(buf, vertices) {
            *vertex_count = vertices.len() as u32;
            return;
        }
        if let Some(old) = buffer.take() {
            retired_buffers.push(old);
        }
    }

    let (buf, count) = create_buffer_with_data(
        device,
        allocator,
        vertices,
        vk::BufferUsageFlags::VERTEX_BUFFER,
        label,
    );
    *buffer = Some(buf);
    *vertex_count = count;
}

fn axes_vertices_from_tuples(vertices: &[([f32; 3], [f32; 4])]) -> Vec<AxesVertex> {
    vertices
        .iter()
        .map(|(position, color)| AxesVertex {
            position: *position,
            color: *color,
        })
        .collect()
}

fn write_mapped_axes_vertices(buffer: &AllocatedBuffer, vertices: &[AxesVertex]) -> bool {
    let required_bytes = std::mem::size_of_val(vertices) as u64;
    let Some(alloc) = buffer.allocation.as_ref() else {
        return false;
    };
    if alloc.size() < required_bytes {
        return false;
    }
    let Some(mapped) = alloc.mapped_ptr() else {
        return false;
    };
    let bytes = bytemuck::cast_slice(vertices);
    unsafe {
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), mapped.as_ptr() as *mut u8, bytes.len());
    }
    true
}

// --- Pipeline creation helpers ---

/// Creates a render pass compatible with swapchain color attachments.
fn create_render_pass(device: &ash::Device, format: vk::Format) -> vk::RenderPass {
    let attachment = vk::AttachmentDescription::default()
        .format(format)
        .samples(vk::SampleCountFlags::TYPE_1)
        .load_op(vk::AttachmentLoadOp::CLEAR)
        .store_op(vk::AttachmentStoreOp::STORE)
        .stencil_load_op(vk::AttachmentLoadOp::DONT_CARE)
        .stencil_store_op(vk::AttachmentStoreOp::DONT_CARE)
        .initial_layout(vk::ImageLayout::UNDEFINED)
        .final_layout(vk::ImageLayout::PRESENT_SRC_KHR);

    let color_ref = vk::AttachmentReference {
        attachment: 0,
        layout: vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL,
    };
    let color_refs = [color_ref];

    let subpass = vk::SubpassDescription::default()
        .pipeline_bind_point(vk::PipelineBindPoint::GRAPHICS)
        .color_attachments(&color_refs);

    let dependency = vk::SubpassDependency::default()
        .src_subpass(vk::SUBPASS_EXTERNAL)
        .dst_subpass(0)
        .src_stage_mask(vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT)
        .src_access_mask(vk::AccessFlags::empty())
        .dst_stage_mask(vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT)
        .dst_access_mask(vk::AccessFlags::COLOR_ATTACHMENT_WRITE);

    let attachments = [attachment];
    let subpasses = [subpass];
    let dependencies = [dependency];

    let ci = vk::RenderPassCreateInfo::default()
        .attachments(&attachments)
        .subpasses(&subpasses)
        .dependencies(&dependencies);

    unsafe { device.create_render_pass(&ci, None) }.unwrap()
}

/// Creates one framebuffer per swapchain image view.
fn create_framebuffers(
    device: &ash::Device,
    render_pass: vk::RenderPass,
    image_views: &[vk::ImageView],
    extent: vk::Extent2D,
) -> Vec<vk::Framebuffer> {
    image_views
        .iter()
        .map(|&iv| {
            let attachments = [iv];
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

/// Creates graphics pipeline layout with push constants and descriptor sets.
fn create_pipeline_layout(
    device: &ash::Device,
    push_constant_size: u32,
    push_stages: vk::ShaderStageFlags,
) -> vk::PipelineLayout {
    let push_range = vk::PushConstantRange {
        stage_flags: push_stages,
        offset: 0,
        size: push_constant_size,
    };
    let ranges = [push_range];
    let ci = vk::PipelineLayoutCreateInfo::default().push_constant_ranges(&ranges);
    unsafe { device.create_pipeline_layout(&ci, None) }.unwrap()
}

/// Builds a graphics pipeline from shaders and fixed-function states.
fn create_graphics_pipeline(
    device: &ash::Device,
    render_pass: vk::RenderPass,
    layout: vk::PipelineLayout,
    vs_spv: &[u8],
    fs_spv: &[u8],
    binding_desc: &[vk::VertexInputBindingDescription],
    attr_desc: &[vk::VertexInputAttributeDescription],
    topology: vk::PrimitiveTopology,
    blend: vk::PipelineColorBlendAttachmentState,
    cull_mode: vk::CullModeFlags,
) -> vk::Pipeline {
    let vs_mod = create_shader_module(device, vs_spv);
    let fs_mod = create_shader_module(device, fs_spv);

    let entry = c"main";
    let stages = [
        vk::PipelineShaderStageCreateInfo::default()
            .stage(vk::ShaderStageFlags::VERTEX)
            .module(vs_mod)
            .name(entry),
        vk::PipelineShaderStageCreateInfo::default()
            .stage(vk::ShaderStageFlags::FRAGMENT)
            .module(fs_mod)
            .name(entry),
    ];

    let vertex_input = vk::PipelineVertexInputStateCreateInfo::default()
        .vertex_binding_descriptions(binding_desc)
        .vertex_attribute_descriptions(attr_desc);

    let input_assembly = vk::PipelineInputAssemblyStateCreateInfo::default()
        .topology(topology)
        .primitive_restart_enable(false);

    let viewport_state = vk::PipelineViewportStateCreateInfo::default()
        .viewport_count(1)
        .scissor_count(1);

    let rasterizer = vk::PipelineRasterizationStateCreateInfo::default()
        .depth_clamp_enable(false)
        .rasterizer_discard_enable(false)
        .polygon_mode(vk::PolygonMode::FILL)
        .line_width(1.0)
        .cull_mode(cull_mode)
        .front_face(vk::FrontFace::COUNTER_CLOCKWISE);

    let multisampling = vk::PipelineMultisampleStateCreateInfo::default()
        .rasterization_samples(vk::SampleCountFlags::TYPE_1);

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
        device.destroy_shader_module(vs_mod, None);
        device.destroy_shader_module(fs_mod, None);
    }

    pipeline
}

/// Returns standard alpha-blend state for opaque/alpha primitives.
fn default_blend() -> vk::PipelineColorBlendAttachmentState {
    vk::PipelineColorBlendAttachmentState::default()
        .color_write_mask(vk::ColorComponentFlags::RGBA)
        .blend_enable(false)
}

/// Returns additive blend state for luminous point rendering.
fn additive_blend() -> vk::PipelineColorBlendAttachmentState {
    vk::PipelineColorBlendAttachmentState::default()
        .color_write_mask(vk::ColorComponentFlags::RGBA)
        .blend_enable(true)
        .src_color_blend_factor(vk::BlendFactor::ONE)
        .dst_color_blend_factor(vk::BlendFactor::ONE)
        .color_blend_op(vk::BlendOp::ADD)
        .src_alpha_blend_factor(vk::BlendFactor::ONE)
        .dst_alpha_blend_factor(vk::BlendFactor::ONE)
        .alpha_blend_op(vk::BlendOp::ADD)
}

/// Defines vertex input bindings and attributes for axis vertices.
fn axes_vertex_desc() -> (
    Vec<vk::VertexInputBindingDescription>,
    Vec<vk::VertexInputAttributeDescription>,
) {
    let binding = vec![vk::VertexInputBindingDescription {
        binding: 0,
        stride: std::mem::size_of::<AxesVertex>() as u32,
        input_rate: vk::VertexInputRate::VERTEX,
    }];
    let attrs = vec![
        vk::VertexInputAttributeDescription {
            location: 0,
            binding: 0,
            format: vk::Format::R32G32B32_SFLOAT,
            offset: 0,
        },
        vk::VertexInputAttributeDescription {
            location: 1,
            binding: 0,
            format: vk::Format::R32G32B32A32_SFLOAT,
            offset: 12,
        },
    ];
    (binding, attrs)
}

/// Defines vertex input bindings and attributes for particle vertices.
fn particle_vertex_desc() -> (
    Vec<vk::VertexInputBindingDescription>,
    Vec<vk::VertexInputAttributeDescription>,
) {
    let binding = vec![vk::VertexInputBindingDescription {
        binding: 0,
        stride: std::mem::size_of::<ParticleVertex>() as u32,
        input_rate: vk::VertexInputRate::VERTEX,
    }];
    let attrs = vec![
        vk::VertexInputAttributeDescription {
            location: 0,
            binding: 0,
            format: vk::Format::R32G32B32_SFLOAT,
            offset: 0,
        },
        vk::VertexInputAttributeDescription {
            location: 1,
            binding: 0,
            format: vk::Format::R32G32B32A32_SFLOAT,
            offset: 12,
        },
    ];
    (binding, attrs)
}

/// Creates graphics pipeline specialized for axis rendering.
fn create_axes_pipeline(
    device: &ash::Device,
    render_pass: vk::RenderPass,
) -> (vk::PipelineLayout, vk::Pipeline) {
    let layout = create_pipeline_layout(
        device,
        std::mem::size_of::<AxesPushConstants>() as u32,
        vk::ShaderStageFlags::VERTEX,
    );
    let (binding, attrs) = axes_vertex_desc();
    let pipeline = create_graphics_pipeline(
        device,
        render_pass,
        layout,
        include_bytes!(concat!(env!("OUT_DIR"), "/shaders/axes_vertex.vert.spv")),
        include_bytes!(concat!(env!("OUT_DIR"), "/shaders/axes_fragment.frag.spv")),
        &binding,
        &attrs,
        vk::PrimitiveTopology::LINE_LIST,
        default_blend(),
        vk::CullModeFlags::NONE,
    );
    (layout, pipeline)
}

/// Creates graphics pipeline specialized for particle rendering.
fn create_particles_pipeline(
    device: &ash::Device,
    render_pass: vk::RenderPass,
) -> (vk::PipelineLayout, vk::Pipeline) {
    let layout = create_pipeline_layout(
        device,
        std::mem::size_of::<PushConstants>() as u32,
        vk::ShaderStageFlags::VERTEX,
    );
    let (binding, attrs) = particle_vertex_desc();
    let pipeline = create_graphics_pipeline(
        device,
        render_pass,
        layout,
        include_bytes!(concat!(
            env!("OUT_DIR"),
            "/shaders/particles_vertex.vert.spv"
        )),
        include_bytes!(concat!(
            env!("OUT_DIR"),
            "/shaders/particles_fragment.frag.spv"
        )),
        &binding,
        &attrs,
        vk::PrimitiveTopology::POINT_LIST,
        additive_blend(),
        vk::CullModeFlags::NONE,
    );
    (layout, pipeline)
}

// --- Initial vertex data ---

/// Creates vertex buffer containing static axis and grid helper geometry.
fn create_axes_vertices(
    device: &ash::Device,
    allocator: &Mutex<Allocator>,
) -> (AllocatedBuffer, u32) {
    let mut vertices: Vec<AxesVertex> = Vec::new();
    let range = AXIS_XZ_GRID_EXTENT;
    let num_lines = AXIS_XZ_GRID_LINE_COUNT;
    let step = (2.0 * range) / ((num_lines - 1) as f32);
    for i in 0..num_lines {
        let pos = -range + i as f32 * step;
        vertices.push(AxesVertex {
            position: [-range, 0.0, pos],
            color: [1.0, 0.0, 0.0, 1.0],
        });
        vertices.push(AxesVertex {
            position: [range, 0.0, pos],
            color: [1.0, 0.0, 0.0, 1.0],
        });
        vertices.push(AxesVertex {
            position: [pos, 0.0, -range],
            color: [0.0, 0.0, 1.0, 1.0],
        });
        vertices.push(AxesVertex {
            position: [pos, 0.0, range],
            color: [0.0, 0.0, 1.0, 1.0],
        });
    }
    vertices.push(AxesVertex {
        position: [0.0, 0.0, 0.0],
        color: [0.0, 1.0, 0.0, 1.0],
    });
    vertices.push(AxesVertex {
        position: [0.0, -1.0, 0.0],
        color: [0.0, 1.0, 0.0, 1.0],
    });
    create_buffer_with_data(
        device,
        allocator,
        &vertices,
        vk::BufferUsageFlags::VERTEX_BUFFER,
        "axes_vertices",
    )
}

/// Creates an initial particle buffer used before simulation data arrives.
fn create_initial_particles(
    device: &ash::Device,
    allocator: &Mutex<Allocator>,
) -> (AllocatedBuffer, u32) {
    let mut particles = Vec::with_capacity(100);
    for _ in 0..100 {
        particles.push(ParticleVertex {
            position: [
                rand::random::<f32>() * 2.0 - 1.0,
                rand::random::<f32>() * 2.0 - 1.0,
                rand::random::<f32>() * 2.0 - 1.0,
            ],
            color: [1.0, 1.0, 1.0, 1.0],
        });
    }
    create_buffer_with_data(
        device,
        allocator,
        &particles,
        vk::BufferUsageFlags::VERTEX_BUFFER,
        "particle_vertices",
    )
}
