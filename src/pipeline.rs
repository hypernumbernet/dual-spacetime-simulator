use crate::camera::OrbitCamera;
use crate::integration::Gui;
use crate::tree::{AXIS_XZ_GRID_EXTENT, AXIS_XZ_GRID_LINE_COUNT, GpuTreeComputeParams};
use crate::ui_state::*;
use crate::vulkan_base::VulkanBase;
use ash::vk;
use glam::{Mat4, Vec3};
use gpu_allocator::vulkan::{Allocation, AllocationCreateDesc, AllocationScheme, Allocator};
use gpu_allocator::MemoryLocation;
use std::sync::{Arc, Mutex};

const MOUSE_LEFT_DRAG_SENS: f32 = 0.003f32;
const MOUSE_RIGHT_DRAG_SENS: f32 = 0.001f32;
const SIZE_RATIO: f32 = 0.06;
const INITIAL_POSITION: Vec3 = Vec3::new(1.6, -1.6, 3.0);
const INITIAL_TARGET: Vec3 = Vec3::new(0.0, 0.0, 0.0);

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
struct TreeVertex {
    position: [f32; 3],
    normal: [f32; 3],
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

struct AllocatedBuffer {
    buffer: vk::Buffer,
    allocation: Option<Allocation>,
}

impl AllocatedBuffer {
    fn new(
        device: &ash::Device,
        allocator: &Mutex<Allocator>,
        size: u64,
        usage: vk::BufferUsageFlags,
        location: MemoryLocation,
        name: &str,
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

    fn destroy(mut self, device: &ash::Device, allocator: &Mutex<Allocator>) {
        unsafe { device.destroy_buffer(self.buffer, None) };
        if let Some(alloc) = self.allocation.take() {
            allocator.lock().unwrap().free(alloc).unwrap();
        }
    }
}

fn create_buffer_with_data<T: bytemuck::Pod>(
    device: &ash::Device,
    allocator: &Mutex<Allocator>,
    data: &[T],
    usage: vk::BufferUsageFlags,
    name: &str,
) -> (AllocatedBuffer, u32) {
    let byte_size = (std::mem::size_of::<T>() * data.len().max(1)) as u64;
    let buf = AllocatedBuffer::new(device, allocator, byte_size, usage, MemoryLocation::CpuToGpu, name);

    if !data.is_empty() {
        if let Some(ref alloc) = buf.allocation {
            if let Some(mapped) = alloc.mapped_ptr() {
                let bytes = bytemuck::cast_slice::<T, u8>(data);
                unsafe {
                    std::ptr::copy_nonoverlapping(
                        bytes.as_ptr(),
                        mapped.as_ptr() as *mut u8,
                        bytes.len(),
                    );
                }
            }
        }
    }
    let count = data.len() as u32;
    (buf, count)
}

fn create_shader_module(device: &ash::Device, spv: &[u8]) -> vk::ShaderModule {
    let code = ash::util::read_spv(&mut std::io::Cursor::new(spv)).unwrap();
    let ci = vk::ShaderModuleCreateInfo::default().code(&code);
    unsafe { device.create_shader_module(&ci, None) }.unwrap()
}

pub struct ParticleRenderPipeline {
    device: ash::Device,
    allocator: Arc<Mutex<Allocator>>,

    render_pass: vk::RenderPass,
    framebuffers: Vec<vk::Framebuffer>,

    pipeline_axes: vk::Pipeline,
    pipeline_particles: vk::Pipeline,
    pipeline_tree: vk::Pipeline,
    layout_axes: vk::PipelineLayout,
    layout_particles: vk::PipelineLayout,
    layout_tree: vk::PipelineLayout,

    compute_pipeline: vk::Pipeline,
    compute_layout: vk::PipelineLayout,
    compute_desc_set_layout: vk::DescriptorSetLayout,
    compute_desc_pool: vk::DescriptorPool,

    axes_buffer: AllocatedBuffer,
    axes_vertex_count: u32,
    graph_lines_buffer: Option<AllocatedBuffer>,
    graph_lines_vertex_count: u32,
    particle_buffer: AllocatedBuffer,
    particle_draw_vertex_count: u32,
    tree_buffer: Option<AllocatedBuffer>,
    tree_draw_vertex_count: u32,

    camera: OrbitCamera,
    queue: vk::Queue,
    #[allow(dead_code)]
    queue_family: u32,
    command_pool: vk::CommandPool,
}

impl ParticleRenderPipeline {
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
        let (layout_tree, pipeline_tree) = create_tree_pipeline(&device, render_pass);
        let (compute_desc_set_layout, compute_layout, compute_pipeline) =
            create_compute_pipeline(&device);
        let compute_desc_pool = create_compute_descriptor_pool(&device);

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
            pipeline_tree,
            layout_axes,
            layout_particles,
            layout_tree,
            compute_pipeline,
            compute_layout,
            compute_desc_set_layout,
            compute_desc_pool,
            axes_buffer,
            axes_vertex_count,
            graph_lines_buffer: None,
            graph_lines_vertex_count: 0,
            particle_buffer,
            particle_draw_vertex_count: particle_vertex_count,
            tree_buffer: None,
            tree_draw_vertex_count: 0,
            camera,
            queue: base.graphics_queue,
            queue_family: base.graphics_queue_family,
            command_pool: base.command_pool,
        }
    }

    fn allocator(&self) -> &Mutex<Allocator> {
        &self.allocator
    }

    pub fn render_pass(&self) -> vk::RenderPass {
        self.render_pass
    }

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
        gpu_tree_render_mode: GpuTreeRenderMode,
    ) {
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

        if app_mode == AppMode::Graph3D {
            if let Some(ref buf) = self.graph_lines_buffer {
                if self.graph_lines_vertex_count > 0 {
                    let line_pc = AxesPushConstants {
                        view_proj: view_proj.to_cols_array_2d(),
                    };
                    self.draw_graph_lines(command_buffer, &line_pc, buf.buffer);
                }
            }
        }

        if app_mode == AppMode::GpuTree {
            match gpu_tree_render_mode {
                GpuTreeRenderMode::Lines => {
                    if let Some(ref buf) = self.graph_lines_buffer {
                        if self.graph_lines_vertex_count > 0 {
                            let line_pc = AxesPushConstants {
                                view_proj: view_proj.to_cols_array_2d(),
                            };
                            self.draw_graph_lines(command_buffer, &line_pc, buf.buffer);
                        }
                    }
                }
                GpuTreeRenderMode::Polygons => {
                    if let Some(ref buf) = self.tree_buffer {
                        if self.tree_draw_vertex_count > 0 {
                            let tree_pc = AxesPushConstants {
                                view_proj: view_proj.to_cols_array_2d(),
                            };
                            self.draw_tree(command_buffer, &tree_pc, buf.buffer);
                        }
                    }
                }
            }
        }

        if matches!(
            app_mode,
            AppMode::Simulation | AppMode::Graph3D | AppMode::GpuTree
        ) {
            self.draw_particles(command_buffer, &pc);
        }

        gui.draw(command_buffer, extent);

        unsafe {
            self.device.cmd_end_render_pass(command_buffer);
        }
    }

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
        old.destroy(&self.device, &alloc);
        self.particle_draw_vertex_count = draw_n;
    }

    pub fn set_graph_lines(&mut self, vertices: &[([f32; 3], [f32; 4])]) {
        if let Some(old) = self.graph_lines_buffer.take() {
            old.destroy(&self.device, self.allocator());
        }
        if vertices.is_empty() {
            self.graph_lines_vertex_count = 0;
            return;
        }
        let verts: Vec<AxesVertex> = vertices
            .iter()
            .map(|(p, c)| AxesVertex {
                position: *p,
                color: *c,
            })
            .collect();
        let (buf, count) = create_buffer_with_data(
            &self.device,
            self.allocator(),
            &verts,
            vk::BufferUsageFlags::VERTEX_BUFFER,
            "graph_lines",
        );
        self.graph_lines_buffer = Some(buf);
        self.graph_lines_vertex_count = count;
    }

    pub fn set_tree_vertices(&mut self, vertices: Vec<([f32; 3], [f32; 3], [f32; 4])>) {
        if let Some(old) = self.tree_buffer.take() {
            old.destroy(&self.device, self.allocator());
        }
        if vertices.is_empty() {
            self.tree_draw_vertex_count = 0;
            return;
        }
        let verts: Vec<TreeVertex> = vertices
            .iter()
            .map(|(p, n, c)| TreeVertex {
                position: *p,
                normal: *n,
                color: *c,
            })
            .collect();
        let (buf, count) = create_buffer_with_data(
            &self.device,
            self.allocator(),
            &verts,
            vk::BufferUsageFlags::VERTEX_BUFFER,
            "tree_vertices",
        );
        self.tree_buffer = Some(buf);
        self.tree_draw_vertex_count = count;
    }

    pub fn compute_tree_vertices(
        &mut self,
        params: crate::tree::TreeParams,
        layout: GpuTreeLayout,
    ) {
        // ForestOnGrid requires multiple trees at grid positions; use CPU for now
        if layout == GpuTreeLayout::ForestOnGrid {
            let cpu_verts =
                crate::tree::Tree::generate_forest_tube_vertices_on_axis_xz_grid(params);
            self.set_tree_vertices(cpu_verts);
            return;
        }

        let compute_params = GpuTreeComputeParams::from(params);

        let params_buf = AllocatedBuffer::new(
            &self.device,
            self.allocator(),
            std::mem::size_of::<GpuTreeComputeParams>() as u64,
            vk::BufferUsageFlags::UNIFORM_BUFFER,
            MemoryLocation::CpuToGpu,
            "compute_params",
        );
        if let Some(ref alloc) = params_buf.allocation {
            if let Some(mapped) = alloc.mapped_ptr() {
                unsafe {
                    std::ptr::copy_nonoverlapping(
                        bytemuck::bytes_of(&compute_params).as_ptr(),
                        mapped.as_ptr() as *mut u8,
                        std::mem::size_of::<GpuTreeComputeParams>(),
                    );
                }
            }
        }

        const MAX_VERTICES: usize = 65536;
        const MAX_BRANCHES: u32 = 512;
        let pos_size = (MAX_VERTICES * 16) as u64;
        let norm_size = (MAX_VERTICES * 16) as u64;
        let col_size = (MAX_VERTICES * 16) as u64;
        let counter_size = 4u64;

        let positions_buf = AllocatedBuffer::new(
            &self.device,
            self.allocator(),
            pos_size,
            vk::BufferUsageFlags::STORAGE_BUFFER,
            MemoryLocation::GpuToCpu,
            "compute_positions",
        );
        let normals_buf = AllocatedBuffer::new(
            &self.device,
            self.allocator(),
            norm_size,
            vk::BufferUsageFlags::STORAGE_BUFFER,
            MemoryLocation::GpuToCpu,
            "compute_normals",
        );
        let colors_buf = AllocatedBuffer::new(
            &self.device,
            self.allocator(),
            col_size,
            vk::BufferUsageFlags::STORAGE_BUFFER,
            MemoryLocation::GpuToCpu,
            "compute_colors",
        );
        let counter_buf = AllocatedBuffer::new(
            &self.device,
            self.allocator(),
            counter_size,
            vk::BufferUsageFlags::STORAGE_BUFFER,
            MemoryLocation::CpuToGpu,
            "compute_counter",
        );
        if let Some(ref alloc) = counter_buf.allocation {
            if let Some(mapped) = alloc.mapped_ptr() {
                unsafe {
                    std::ptr::write_bytes(mapped.as_ptr() as *mut u8, 0, 4);
                }
            }
        }

        let desc_set = allocate_compute_descriptor_set(
            &self.device,
            self.compute_desc_pool,
            self.compute_desc_set_layout,
            &params_buf,
            &positions_buf,
            &normals_buf,
            &colors_buf,
            &counter_buf,
        );

        let alloc_ci = vk::CommandBufferAllocateInfo::default()
            .command_pool(self.command_pool)
            .level(vk::CommandBufferLevel::PRIMARY)
            .command_buffer_count(1);
        let cb = unsafe { self.device.allocate_command_buffers(&alloc_ci) }.unwrap()[0];
        let begin_ci =
            vk::CommandBufferBeginInfo::default().flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT);
        unsafe {
            self.device.begin_command_buffer(cb, &begin_ci).unwrap();
            self.device
                .cmd_bind_pipeline(cb, vk::PipelineBindPoint::COMPUTE, self.compute_pipeline);
            self.device.cmd_bind_descriptor_sets(
                cb,
                vk::PipelineBindPoint::COMPUTE,
                self.compute_layout,
                0,
                &[desc_set],
                &[],
            );
            let workgroup_count = (MAX_BRANCHES + 63) / 64;
            self.device.cmd_dispatch(cb, workgroup_count, 1, 1);
            self.device.end_command_buffer(cb).unwrap();
        }

        let cbs = [cb];
        let submit_info = vk::SubmitInfo::default().command_buffers(&cbs);
        let fence_ci = vk::FenceCreateInfo::default();
        let fence = unsafe { self.device.create_fence(&fence_ci, None) }.unwrap();
        unsafe {
            self.device
                .queue_submit(self.queue, &[submit_info], fence)
                .unwrap();
            self.device
                .wait_for_fences(&[fence], true, u64::MAX)
                .unwrap();
            self.device.destroy_fence(fence, None);
            self.device
                .free_command_buffers(self.command_pool, &[cb]);
        }

        let vertex_count = counter_buf
            .allocation
            .as_ref()
            .and_then(|a| a.mapped_ptr())
            .map(|mapped| unsafe { *(mapped.as_ptr() as *const u32) })
            .unwrap_or(0);
        let vertex_count = (vertex_count as usize).min(MAX_VERTICES) as u32;

        let mut tube_verts = Vec::with_capacity(vertex_count as usize);
        if vertex_count >= 3 {
            let pos_ptr = positions_buf.allocation.as_ref().and_then(|a| a.mapped_ptr());
            let norm_ptr = normals_buf.allocation.as_ref().and_then(|a| a.mapped_ptr());
            let col_ptr = colors_buf.allocation.as_ref().and_then(|a| a.mapped_ptr());

            if let (Some(p), Some(n), Some(c)) = (pos_ptr, norm_ptr, col_ptr) {
                let usable = (vertex_count / 3) * 3;
                for i in 0..usable as usize {
                    let pp = unsafe { &*(p.as_ptr() as *const [f32; 4]).add(i) };
                    let nn = unsafe { &*(n.as_ptr() as *const [f32; 4]).add(i) };
                    let cc = unsafe { &*(c.as_ptr() as *const [f32; 4]).add(i) };
                    tube_verts.push((
                        [pp[0], pp[1], pp[2]],
                        [nn[0], nn[1], nn[2]],
                        *cc,
                    ));
                }
            }
        }

        params_buf.destroy(&self.device, self.allocator());
        positions_buf.destroy(&self.device, self.allocator());
        normals_buf.destroy(&self.device, self.allocator());
        colors_buf.destroy(&self.device, self.allocator());
        counter_buf.destroy(&self.device, self.allocator());
        unsafe {
            self.device
                .reset_descriptor_pool(self.compute_desc_pool, vk::DescriptorPoolResetFlags::empty())
                .unwrap();
        }

        if tube_verts.is_empty() {
            let tree = crate::tree::Tree::generate(params);
            let cpu_verts = tree.generate_tube_vertices_at(Vec3::ZERO);
            self.set_tree_vertices(cpu_verts);
        } else {
            self.set_tree_vertices(tube_verts);
        }
    }

    // --- Camera methods ---

    pub fn revolve_camera(&mut self, delta_yaw: f64, delta_pitch: f64) {
        self.camera.revolve(
            delta_yaw as f32 * MOUSE_LEFT_DRAG_SENS,
            delta_pitch as f32 * MOUSE_LEFT_DRAG_SENS,
        );
    }

    pub fn look_around(&mut self, dx: f64, dy: f64) {
        self.camera.look_around(
            dx as f32 * MOUSE_RIGHT_DRAG_SENS,
            dy as f32 * MOUSE_RIGHT_DRAG_SENS,
        );
    }

    pub fn zoom_camera(&mut self, zoom_factor: f32) {
        self.camera.zoom(zoom_factor);
    }

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

    pub fn y_top(&mut self) {
        self.camera.y_top();
    }

    pub fn center_target_on_origin(&mut self) {
        self.camera.center_target_on_origin();
    }

    pub fn update_animation(&mut self) {
        self.camera.update_animation();
    }

    pub fn set_lock_camera_up(&mut self, lock: bool) {
        self.camera.set_lock_up(lock);
    }

    // --- Draw helpers ---

    fn draw_axes(&self, cb: vk::CommandBuffer, pc: &AxesPushConstants) {
        unsafe {
            self.device
                .cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, self.pipeline_axes);
            self.device
                .cmd_bind_vertex_buffers(cb, 0, &[self.axes_buffer.buffer], &[0]);
            self.device.cmd_push_constants(
                cb,
                self.layout_axes,
                vk::ShaderStageFlags::VERTEX,
                0,
                bytemuck::bytes_of(pc),
            );
            self.device.cmd_draw(cb, self.axes_vertex_count, 1, 0, 0);
        }
    }

    fn draw_graph_lines(&self, cb: vk::CommandBuffer, pc: &AxesPushConstants, buffer: vk::Buffer) {
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
            self.device
                .cmd_draw(cb, self.graph_lines_vertex_count, 1, 0, 0);
        }
    }

    fn draw_tree(&self, cb: vk::CommandBuffer, pc: &AxesPushConstants, buffer: vk::Buffer) {
        unsafe {
            self.device
                .cmd_bind_pipeline(cb, vk::PipelineBindPoint::GRAPHICS, self.pipeline_tree);
            self.device
                .cmd_bind_vertex_buffers(cb, 0, &[buffer], &[0]);
            self.device.cmd_push_constants(
                cb,
                self.layout_tree,
                vk::ShaderStageFlags::VERTEX,
                0,
                bytemuck::bytes_of(pc),
            );
            self.device
                .cmd_draw(cb, self.tree_draw_vertex_count, 1, 0, 0);
        }
    }

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

    fn compute_mvp_axes(&self, aspect_ratio: f32) -> Mat4 {
        let view = Mat4::look_at_rh(self.camera.position, self.camera.target, self.camera.up);
        let proj = Mat4::perspective_rh(std::f32::consts::FRAC_PI_4, aspect_ratio, 0.1, 100.0);
        proj * view
    }

    fn compute_mvp_particle(&self, aspect_ratio: f32, scale_factor: f32) -> Mat4 {
        let view = Mat4::look_at_rh(self.camera.position, self.camera.target, self.camera.up);
        let proj = Mat4::perspective_rh(std::f32::consts::FRAC_PI_4, aspect_ratio, 0.1, 100.0);
        let model = Mat4::from_scale(Vec3::splat(scale_factor));
        proj * view * model
    }
}

impl Drop for ParticleRenderPipeline {
    fn drop(&mut self) {
        unsafe {
            self.device.device_wait_idle().unwrap();

            if let Some(buf) = self.graph_lines_buffer.take() {
                buf.destroy(&self.device, self.allocator());
            }
            if let Some(buf) = self.tree_buffer.take() {
                buf.destroy(&self.device, self.allocator());
            }

            if let Some(alloc) = self.particle_buffer.allocation.take() {
                self.allocator().lock().unwrap().free(alloc).unwrap();
            }
            self.device
                .destroy_buffer(self.particle_buffer.buffer, None);

            if let Some(alloc) = self.axes_buffer.allocation.take() {
                self.allocator().lock().unwrap().free(alloc).unwrap();
            }
            self.device.destroy_buffer(self.axes_buffer.buffer, None);

            self.device
                .destroy_descriptor_pool(self.compute_desc_pool, None);
            self.device
                .destroy_descriptor_set_layout(self.compute_desc_set_layout, None);
            self.device
                .destroy_pipeline(self.compute_pipeline, None);
            self.device
                .destroy_pipeline_layout(self.compute_layout, None);

            for fb in &self.framebuffers {
                self.device.destroy_framebuffer(*fb, None);
            }
            self.device.destroy_pipeline(self.pipeline_axes, None);
            self.device.destroy_pipeline(self.pipeline_particles, None);
            self.device.destroy_pipeline(self.pipeline_tree, None);
            self.device.destroy_pipeline_layout(self.layout_axes, None);
            self.device
                .destroy_pipeline_layout(self.layout_particles, None);
            self.device
                .destroy_pipeline_layout(self.layout_tree, None);
            self.device.destroy_render_pass(self.render_pass, None);
        }
    }
}

// --- Pipeline creation helpers ---

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

fn default_blend() -> vk::PipelineColorBlendAttachmentState {
    vk::PipelineColorBlendAttachmentState::default()
        .color_write_mask(vk::ColorComponentFlags::RGBA)
        .blend_enable(false)
}

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

fn tree_vertex_desc() -> (
    Vec<vk::VertexInputBindingDescription>,
    Vec<vk::VertexInputAttributeDescription>,
) {
    let binding = vec![vk::VertexInputBindingDescription {
        binding: 0,
        stride: std::mem::size_of::<TreeVertex>() as u32,
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
            format: vk::Format::R32G32B32_SFLOAT,
            offset: 12,
        },
        vk::VertexInputAttributeDescription {
            location: 2,
            binding: 0,
            format: vk::Format::R32G32B32A32_SFLOAT,
            offset: 24,
        },
    ];
    (binding, attrs)
}

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

fn create_tree_pipeline(
    device: &ash::Device,
    render_pass: vk::RenderPass,
) -> (vk::PipelineLayout, vk::Pipeline) {
    let layout = create_pipeline_layout(
        device,
        std::mem::size_of::<AxesPushConstants>() as u32,
        vk::ShaderStageFlags::VERTEX,
    );
    let (binding, attrs) = tree_vertex_desc();
    let pipeline = create_graphics_pipeline(
        device,
        render_pass,
        layout,
        include_bytes!(concat!(env!("OUT_DIR"), "/shaders/tree_vertex.vert.spv")),
        include_bytes!(concat!(
            env!("OUT_DIR"),
            "/shaders/tree_fragment.frag.spv"
        )),
        &binding,
        &attrs,
        vk::PrimitiveTopology::TRIANGLE_LIST,
        default_blend(),
        vk::CullModeFlags::BACK,
    );
    (layout, pipeline)
}

fn create_compute_pipeline(
    device: &ash::Device,
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

    let cs_mod = create_shader_module(
        device,
        include_bytes!(concat!(
            env!("OUT_DIR"),
            "/shaders/tree_compute.comp.spv"
        )),
    );
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

fn create_compute_descriptor_pool(device: &ash::Device) -> vk::DescriptorPool {
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

fn allocate_compute_descriptor_set(
    device: &ash::Device,
    pool: vk::DescriptorPool,
    layout: vk::DescriptorSetLayout,
    params_buf: &AllocatedBuffer,
    positions_buf: &AllocatedBuffer,
    normals_buf: &AllocatedBuffer,
    colors_buf: &AllocatedBuffer,
    counter_buf: &AllocatedBuffer,
) -> vk::DescriptorSet {
    let layouts = [layout];
    let alloc_ci = vk::DescriptorSetAllocateInfo::default()
        .descriptor_pool(pool)
        .set_layouts(&layouts);
    let sets = unsafe { device.allocate_descriptor_sets(&alloc_ci) }.unwrap();
    let set = sets[0];

    let params_info = vk::DescriptorBufferInfo::default()
        .buffer(params_buf.buffer)
        .offset(0)
        .range(std::mem::size_of::<GpuTreeComputeParams>() as u64);
    let pos_info = vk::DescriptorBufferInfo::default()
        .buffer(positions_buf.buffer)
        .offset(0)
        .range(vk::WHOLE_SIZE);
    let norm_info = vk::DescriptorBufferInfo::default()
        .buffer(normals_buf.buffer)
        .offset(0)
        .range(vk::WHOLE_SIZE);
    let col_info = vk::DescriptorBufferInfo::default()
        .buffer(colors_buf.buffer)
        .offset(0)
        .range(vk::WHOLE_SIZE);
    let counter_info = vk::DescriptorBufferInfo::default()
        .buffer(counter_buf.buffer)
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

// --- Initial vertex data ---

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
