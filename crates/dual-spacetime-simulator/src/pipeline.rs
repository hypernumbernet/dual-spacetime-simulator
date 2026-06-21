use crate::camera::OrbitCamera;
use crate::gpu_simulation::{create_particle_descriptor_set_layout, GpuParticleSimulation};
use crate::integration::Gui;
use crate::simulation::Particle;
use crate::ui_state::*;
use ash::vk;
use glam::{Mat4, Vec3};
use gpu_allocator::vulkan::Allocator;
use std::sync::{Arc, Mutex};
use vulkanvil::{
    create_buffer_with_data, create_depth_image, create_shader_module, select_depth_format,
    AllocatedBuffer, AllocatedImage, VulkanBase,
};

const MOUSE_LEFT_DRAG_SENS: f32 = 0.003f32;
const MOUSE_RIGHT_DRAG_SENS: f32 = 0.001f32;
const SIZE_RATIO: f32 = 0.06;
const INITIAL_POSITION: Vec3 = Vec3::new(1.6, -1.6, 3.0);
const INITIAL_TARGET: Vec3 = Vec3::new(0.0, 0.0, 0.0);
const AXIS_XZ_GRID_EXTENT: f32 = 2.0;
const AXIS_XZ_GRID_LINE_COUNT: usize = 9;
const ADD_CENTER_MARKER_EDGE_COUNT: usize = 18;
const ADD_CENTER_MARKER_VERTICES: usize = ADD_CENTER_MARKER_EDGE_COUNT * 2;
const ADD_CENTER_X_COLOR: [f32; 4] = [1.0, 0.2, 0.2, 1.0];
const ADD_CENTER_Y_COLOR: [f32; 4] = [0.2, 1.0, 0.2, 1.0];
const ADD_CENTER_Z_COLOR: [f32; 4] = [0.3, 0.5, 1.0, 1.0];
const ADD_CENTER_WHITE: [f32; 4] = [1.0, 1.0, 1.0, 1.0];
const ADD_CENTER_MARKER_EDGES: [([i8; 3], [i8; 3], [f32; 4]); ADD_CENTER_MARKER_EDGE_COUNT] = [
    ([0, 0, 0], [1, 0, 0], ADD_CENTER_X_COLOR),
    ([0, 0, 0], [-1, 0, 0], ADD_CENTER_X_COLOR),
    ([0, 0, 0], [0, 1, 0], ADD_CENTER_Y_COLOR),
    ([0, 0, 0], [0, -1, 0], ADD_CENTER_Y_COLOR),
    ([0, 0, 0], [0, 0, 1], ADD_CENTER_Z_COLOR),
    ([0, 0, 0], [0, 0, -1], ADD_CENTER_Z_COLOR),
    ([1, 0, 0], [0, 1, 0], ADD_CENTER_WHITE),
    ([1, 0, 0], [0, -1, 0], ADD_CENTER_WHITE),
    ([1, 0, 0], [0, 0, 1], ADD_CENTER_WHITE),
    ([1, 0, 0], [0, 0, -1], ADD_CENTER_WHITE),
    ([-1, 0, 0], [0, 1, 0], ADD_CENTER_WHITE),
    ([-1, 0, 0], [0, -1, 0], ADD_CENTER_WHITE),
    ([-1, 0, 0], [0, 0, 1], ADD_CENTER_WHITE),
    ([-1, 0, 0], [0, 0, -1], ADD_CENTER_WHITE),
    ([0, 1, 0], [0, 0, 1], ADD_CENTER_WHITE),
    ([0, 1, 0], [0, 0, -1], ADD_CENTER_WHITE),
    ([0, -1, 0], [0, 0, 1], ADD_CENTER_WHITE),
    ([0, -1, 0], [0, 0, -1], ADD_CENTER_WHITE),
];

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct AxesVertex {
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
    particle_pipelines: [vk::Pipeline; ParticleDisplayMode::ALL.len()],
    layout_axes: vk::PipelineLayout,
    layout_particles: vk::PipelineLayout,
    depth_format: vk::Format,
    depth_image: AllocatedImage,

    axes_buffer: AllocatedBuffer,
    axes_vertex_count: u32,
    graph_lines_buffer: Option<AllocatedBuffer>,
    graph_lines_vertex_count: u32,
    add_center_marker_buffer: Option<AllocatedBuffer>,
    add_center_marker_vertex_count: u32,
    last_add_center_marker_key: Option<(glam::DVec3, u64)>,
    particle_descriptor_set_layout: vk::DescriptorSetLayout,
    gpu_sim: GpuParticleSimulation,
    use_gpu_sim: bool,
    retired_buffers: Vec<AllocatedBuffer>,

    camera: OrbitCamera,
}

impl ParticleRenderPipeline {
    /// Creates graphics and compute pipelines with all persistent rendering resources.
    pub fn new(base: &VulkanBase) -> Self {
        let device = base.device.clone();
        let allocator = Arc::clone(base.allocator.as_ref().unwrap());

        let depth_format =
            select_depth_format(&base.instance, base.physical_device);
        let render_pass = create_render_pass(&device, base.swapchain_format, depth_format);
        let depth_image = create_depth_image(
            &device,
            &allocator,
            depth_format,
            base.swapchain_extent,
            "particle-depth-buffer",
        );
        let framebuffers = create_framebuffers(
            &device,
            render_pass,
            &base.swapchain_image_views,
            depth_image.view,
            base.swapchain_extent,
        );

        let (layout_axes, pipeline_axes) = create_axes_pipeline(&device, render_pass);
        let particle_descriptor_set_layout =
            create_particle_descriptor_set_layout(&device);
        let (layout_particles, particle_pipelines) = create_particles_pipelines(
            &device,
            render_pass,
            particle_descriptor_set_layout,
        );

        let (axes_buffer, axes_vertex_count) =
            create_axes_vertices(&device, &allocator);
        let gpu_sim = GpuParticleSimulation::new(
            device.clone(),
            Arc::clone(&allocator),
            particle_descriptor_set_layout,
            &[],
        );

        let camera = OrbitCamera::new(INITIAL_POSITION, INITIAL_TARGET);

        Self {
            device,
            allocator,
            render_pass,
            framebuffers,
            pipeline_axes,
            particle_pipelines,
            layout_axes,
            layout_particles,
            depth_format,
            depth_image,
            axes_buffer,
            axes_vertex_count,
            graph_lines_buffer: None,
            graph_lines_vertex_count: 0,
            add_center_marker_buffer: None,
            add_center_marker_vertex_count: 0,
            last_add_center_marker_key: None,
            particle_descriptor_set_layout,
            gpu_sim,
            use_gpu_sim: false,
            retired_buffers: Vec::new(),
            camera,
        }
    }

    /// Enables or disables GPU-driven particle simulation stepping.
    pub fn set_use_gpu_sim(&mut self, use_gpu_sim: bool) {
        self.use_gpu_sim = use_gpu_sim;
    }

    /// Returns whether GPU compute drives particle updates.
    pub fn uses_gpu_sim(&self) -> bool {
        self.use_gpu_sim
    }

    /// Records one GPU simulation step before rendering when GPU mode is active.
    pub fn record_gpu_advance(&self, command_buffer: vk::CommandBuffer, delta_seconds: f64) {
        if self.use_gpu_sim {
            self.gpu_sim.dispatch(command_buffer, delta_seconds);
        }
    }

    /// Uploads simulation particles into the shared GPU storage buffer.
    pub fn upload_particles(&mut self, particles: &[Particle]) {
        self.gpu_sim.upload_from_cpu(particles);
    }

    /// Reads back GPU particle state for snapshot export.
    pub fn readback_particles(&self) -> Vec<Particle> {
        self.gpu_sim.readback_to_cpu()
    }

    /// Returns shared allocator used for dynamic GPU buffer management.
    fn allocator(&self) -> &Mutex<Allocator> {
        &self.allocator
    }

    /// Returns render pass handle used by graphics pipelines.
    pub fn render_pass(&self) -> vk::RenderPass {
        self.render_pass
    }

    /// Recreates depth resources and framebuffers to match current swapchain dimensions.
    pub fn recreate_framebuffers(&mut self, base: &VulkanBase) {
        for fb in self.framebuffers.drain(..) {
            unsafe { self.device.destroy_framebuffer(fb, None) };
        }
        self.depth_image.destroy(&self.device, &self.allocator);
        self.depth_image = create_depth_image(
            &self.device,
            &self.allocator,
            self.depth_format,
            base.swapchain_extent,
            "particle-depth-buffer",
        );
        self.framebuffers = create_framebuffers(
            &self.device,
            self.render_pass,
            &base.swapchain_image_views,
            self.depth_image.view,
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
        particle_display_mode: ParticleDisplayMode,
    ) {
        self.flush_retired_buffers();
        let clear_values = [
            vk::ClearValue {
                color: vk::ClearColorValue {
                    float32: [0.0, 0.0, 0.0, 1.0],
                },
            },
            vk::ClearValue {
                depth_stencil: vk::ClearDepthStencilValue {
                    depth: 1.0,
                    stencil: 0,
                },
            },
        ];
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
        let view_proj_cols = view_proj.to_cols_array_2d();
        let size_scale = compute_particle_size_scale(
            extent.height as f32,
            point_scale_factor,
            particle_display_mode,
        );
        let pc = PushConstants {
            view_proj: view_proj_cols,
            size_scale,
        };

        let line_pc = AxesPushConstants {
            view_proj: view_proj_cols,
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

        self.draw_particles(command_buffer, &pc, particle_display_mode);

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

    /// Uploads particle point data into the GPU storage buffer.
    pub fn set_particles(&mut self, positions: &[[f32; 3]], colors: &[[f32; 4]]) {
        let particles: Vec<Particle> = positions
            .iter()
            .zip(colors.iter())
            .map(|(position, color)| Particle {
                position: glam::DVec3::new(
                    position[0] as f64,
                    position[1] as f64,
                    position[2] as f64,
                ),
                velocity: glam::DVec3::ZERO,
                mass: 0.0,
                color: *color,
            })
            .collect();
        self.upload_particles(&particles);
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

    /// Uploads add-center preview marker vertices and rebuilds the marker line buffer.
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

        let (center, half_extent) =
            ObjectInput::add_center_marker_geometry(ui_state.add_center, ui_state.base_scale);
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
    fn draw_particles(
        &self,
        cb: vk::CommandBuffer,
        pc: &PushConstants,
        particle_display_mode: ParticleDisplayMode,
    ) {
        let draw_count = self.gpu_sim.particle_count();
        if draw_count == 0 {
            return;
        }
        let pipeline = self.particle_pipelines[particle_display_mode.pipeline_index()];
        unsafe {
            if !self.use_gpu_sim {
                let barrier = vk::MemoryBarrier::default()
                    .src_access_mask(vk::AccessFlags::HOST_WRITE)
                    .dst_access_mask(vk::AccessFlags::SHADER_READ);
                self.device.cmd_pipeline_barrier(
                    cb,
                    vk::PipelineStageFlags::HOST,
                    vk::PipelineStageFlags::VERTEX_SHADER,
                    vk::DependencyFlags::empty(),
                    &[barrier],
                    &[],
                    &[],
                );
            }
            self.device
                .cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, pipeline);
            self.device.cmd_bind_descriptor_sets(
                cb,
                vk::PipelineBindPoint::GRAPHICS,
                self.layout_particles,
                0,
                &[self.gpu_sim.descriptor_set()],
                &[],
            );
            self.device.cmd_push_constants(
                cb,
                self.layout_particles,
                vk::ShaderStageFlags::VERTEX,
                0,
                bytemuck::bytes_of(pc),
            );
            self.device.cmd_draw(cb, draw_count, 1, 0, 0);
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

            if let Some(alloc) = self.axes_buffer.allocation.take() {
                self.allocator().lock().unwrap().free(alloc).unwrap();
            }
            self.device.destroy_buffer(self.axes_buffer.buffer, None);

            for fb in &self.framebuffers {
                self.device.destroy_framebuffer(*fb, None);
            }
            self.depth_image.destroy(&self.device, &self.allocator);
            self.device.destroy_pipeline(self.pipeline_axes, None);
            for pipeline in &self.particle_pipelines {
                self.device.destroy_pipeline(*pipeline, None);
            }
            self.device.destroy_pipeline_layout(self.layout_axes, None);
            self.device
                .destroy_pipeline_layout(self.layout_particles, None);
            self.device
                .destroy_descriptor_set_layout(self.particle_descriptor_set_layout, None);
            self.device.destroy_render_pass(self.render_pass, None);
        }
    }
}

/// Builds an octahedron marker with colored axis spokes at the target center.
pub fn build_add_center_marker(
    center: [f32; 3],
    half_extent: f32,
) -> [([f32; 3], [f32; 4]); ADD_CENTER_MARKER_VERTICES] {
    let verts = build_add_center_marker_vertices(center, half_extent);
    std::array::from_fn(|i| (verts[i].position, verts[i].color))
}

fn add_center_marker_tip(center: [f32; 3], half_extent: f32, tip: [i8; 3]) -> [f32; 3] {
    [
        center[0] + half_extent * tip[0] as f32,
        center[1] + half_extent * tip[1] as f32,
        center[2] + half_extent * tip[2] as f32,
    ]
}

fn build_add_center_marker_vertices(
    center: [f32; 3],
    half_extent: f32,
) -> [AxesVertex; ADD_CENTER_MARKER_VERTICES] {
    std::array::from_fn(|i| {
        let edge = &ADD_CENTER_MARKER_EDGES[i / 2];
        let tip = if i.is_multiple_of(2) { edge.0 } else { edge.1 };
        AxesVertex {
            position: add_center_marker_tip(center, half_extent, tip),
            color: edge.2,
        }
    })
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

/// Computes perspective-correct point sprite size for the active display mode.
fn compute_particle_size_scale(
    framebuffer_height: f32,
    point_scale_factor: f32,
    mode: ParticleDisplayMode,
) -> f32 {
    framebuffer_height * SIZE_RATIO * point_scale_factor * mode.size_scale_factor()
}

/// Creates a render pass compatible with swapchain color and depth attachments.
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

/// Creates one framebuffer per swapchain image view.
fn create_framebuffers(
    device: &ash::Device,
    render_pass: vk::RenderPass,
    image_views: &[vk::ImageView],
    depth_view: vk::ImageView,
    extent: vk::Extent2D,
) -> Vec<vk::Framebuffer> {
    image_views
        .iter()
        .map(|&iv| {
            let attachments = [iv, depth_view];
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
    descriptor_set_layout: Option<vk::DescriptorSetLayout>,
) -> vk::PipelineLayout {
    let push_range = vk::PushConstantRange {
        stage_flags: push_stages,
        offset: 0,
        size: push_constant_size,
    };
    let ranges = [push_range];
    let layout = if let Some(descriptor_set_layout) = descriptor_set_layout {
        let set_layouts = [descriptor_set_layout];
        let ci = vk::PipelineLayoutCreateInfo::default()
            .set_layouts(&set_layouts)
            .push_constant_ranges(&ranges);
        unsafe { device.create_pipeline_layout(&ci, None) }.unwrap()
    } else {
        let ci = vk::PipelineLayoutCreateInfo::default().push_constant_ranges(&ranges);
        unsafe { device.create_pipeline_layout(&ci, None) }.unwrap()
    };
    layout
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
    depth_enabled: bool,
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

    let depth_stencil = vk::PipelineDepthStencilStateCreateInfo::default()
        .depth_test_enable(depth_enabled)
        .depth_write_enable(depth_enabled)
        .depth_compare_op(vk::CompareOp::LESS);

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
    (Vec::new(), Vec::new())
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
        None,
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
        false,
    );
    (layout, pipeline)
}

/// Creates one particle pipeline per display mode, sharing layout and vertex shader.
fn create_particles_pipelines(
    device: &ash::Device,
    render_pass: vk::RenderPass,
    descriptor_set_layout: vk::DescriptorSetLayout,
) -> (
    vk::PipelineLayout,
    [vk::Pipeline; ParticleDisplayMode::ALL.len()],
) {
    let layout = create_pipeline_layout(
        device,
        std::mem::size_of::<PushConstants>() as u32,
        vk::ShaderStageFlags::VERTEX,
        Some(descriptor_set_layout),
    );
    let (binding, attrs) = particle_vertex_desc();
    let vs_spv = include_bytes!(concat!(
        env!("OUT_DIR"),
        "/shaders/particles_vertex_ssbo.vert.spv"
    ));
    let mut pipelines = [vk::Pipeline::null(); ParticleDisplayMode::ALL.len()];
    for mode in ParticleDisplayMode::ALL {
        let (fs_spv, blend, depth_enabled) = particle_pipeline_spec(mode);
        pipelines[mode.pipeline_index()] = create_graphics_pipeline(
            device,
            render_pass,
            layout,
            vs_spv,
            fs_spv,
            &binding,
            &attrs,
            vk::PrimitiveTopology::POINT_LIST,
            blend,
            vk::CullModeFlags::NONE,
            depth_enabled,
        );
    }
    (layout, pipelines)
}

/// Returns fragment shader bytes, blend state, and depth usage for a particle mode.
fn particle_pipeline_spec(
    mode: ParticleDisplayMode,
) -> (
    &'static [u8],
    vk::PipelineColorBlendAttachmentState,
    bool,
) {
    match mode {
        ParticleDisplayMode::Glow => (
            include_bytes!(concat!(
                env!("OUT_DIR"),
                "/shaders/particles_fragment.frag.spv"
            )),
            additive_blend(),
            false,
        ),
        ParticleDisplayMode::Sphere => (
            include_bytes!(concat!(
                env!("OUT_DIR"),
                "/shaders/particles_sphere_fragment.frag.spv"
            )),
            default_blend(),
            true,
        ),
    }
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
