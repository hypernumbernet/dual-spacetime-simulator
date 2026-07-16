//! Simple colored-mesh Vulkan renderer for ground + rocket.

use crate::mesh::{Vertex, ground_fill_indices, ground_mesh, rocket_mesh};
use crate::sim::RocketState;
use ash::vk;
use glam::{Mat4, Vec3};
use gpu_allocator::vulkan::Allocator;
use std::ffi::CStr;
use std::sync::{Arc, Mutex};
use vulkanvil::{
    AllocatedBuffer, AllocatedImage, MAX_FRAMES_IN_FLIGHT, VulkanBase, create_buffer_with_data,
    create_depth_image, create_shader_module, select_depth_format,
};

const MESH_VERT: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/shaders/mesh.vert.spv"));
const MESH_FRAG: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/shaders/mesh.frag.spv"));

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct PushConstants {
    view_proj: [[f32; 4]; 4],
}

struct GpuMesh {
    vertex: AllocatedBuffer,
    index: AllocatedBuffer,
    index_count: u32,
}

pub struct Renderer {
    device: ash::Device,
    allocator: Arc<Mutex<Allocator>>,
    depth_format: vk::Format,
    render_pass: vk::RenderPass,
    depth_image: AllocatedImage,
    framebuffers: Vec<vk::Framebuffer>,
    pipeline_layout: vk::PipelineLayout,
    tri_pipeline: vk::Pipeline,
    line_pipeline: vk::Pipeline,
    ground_lines: Option<GpuMesh>,
    ground_fill: Option<GpuMesh>,
    rocket: Option<GpuMesh>,
    retired: Vec<(u64, AllocatedBuffer)>,
    frame_counter: u64,
    /// Last HUD string (for unit/structural checks without GPU readback).
    pub last_hud: String,
}

impl Renderer {
    pub fn new(vb: &VulkanBase) -> Self {
        let device = vb.device.clone();
        let allocator = vb.allocator.as_ref().unwrap().clone();

        let depth_format = select_depth_format(&vb.instance, vb.physical_device);
        let render_pass = create_render_pass(&device, vb.swapchain_format, depth_format);
        let depth_image = create_depth_image(
            &device,
            &allocator,
            depth_format,
            vb.swapchain_extent,
            "pga-rocket-depth",
        );
        let framebuffers = create_framebuffers(
            &device,
            render_pass,
            &vb.swapchain_image_views,
            depth_image.view,
            vb.swapchain_extent,
        );

        let push_range = vk::PushConstantRange {
            stage_flags: vk::ShaderStageFlags::VERTEX,
            offset: 0,
            size: std::mem::size_of::<PushConstants>() as u32,
        };
        let ranges = [push_range];
        let pl_ci = vk::PipelineLayoutCreateInfo::default().push_constant_ranges(&ranges);
        let pipeline_layout = unsafe { device.create_pipeline_layout(&pl_ci, None) }.unwrap();

        let tri_pipeline =
            create_mesh_pipeline(&device, render_pass, pipeline_layout, vk::PrimitiveTopology::TRIANGLE_LIST);
        let line_pipeline =
            create_mesh_pipeline(&device, render_pass, pipeline_layout, vk::PrimitiveTopology::LINE_LIST);

        let mut renderer = Self {
            device,
            allocator,
            depth_format,
            render_pass,
            depth_image,
            framebuffers,
            pipeline_layout,
            tri_pipeline,
            line_pipeline,
            ground_lines: None,
            ground_fill: None,
            rocket: None,
            retired: Vec::new(),
            frame_counter: 0,
            last_hud: String::new(),
        };

        // Static ground
        let (gverts, glines) = ground_mesh(400.0, 10.0);
        let fill_idx = ground_fill_indices(gverts.len() as u32);
        renderer.ground_lines = Some(renderer.upload_mesh(&gverts, &glines));
        renderer.ground_fill = Some(renderer.upload_mesh(&gverts, &fill_idx));
        renderer
    }

    fn upload_mesh(&self, verts: &[Vertex], indices: &[u32]) -> GpuMesh {
        let (vertex, _) = create_buffer_with_data(
            &self.device,
            &self.allocator,
            verts,
            vk::BufferUsageFlags::VERTEX_BUFFER,
            "mesh-v",
        );
        let (index, index_count) = create_buffer_with_data(
            &self.device,
            &self.allocator,
            indices,
            vk::BufferUsageFlags::INDEX_BUFFER,
            "mesh-i",
        );
        GpuMesh {
            vertex,
            index,
            index_count,
        }
    }

    fn retire_mesh(&mut self, mesh: GpuMesh) {
        self.retired.push((self.frame_counter, mesh.vertex));
        self.retired.push((self.frame_counter, mesh.index));
    }

    /// Rebuild rocket GPU mesh from current sim state.
    pub fn sync_rocket(&mut self, state: &RocketState) {
        if let Some(old) = self.rocket.take() {
            self.retire_mesh(old);
        }
        let (verts, idx) = rocket_mesh(state);
        if !idx.is_empty() {
            self.rocket = Some(self.upload_mesh(&verts, &idx));
        }
    }

    pub fn set_hud(&mut self, text: String) {
        self.last_hud = text;
    }

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
            "pga-rocket-depth",
        );
        self.framebuffers = create_framebuffers(
            &self.device,
            self.render_pass,
            &vb.swapchain_image_views,
            self.depth_image.view,
            vb.swapchain_extent,
        );
    }

    pub fn collect_garbage(&mut self) {
        let threshold = MAX_FRAMES_IN_FLIGHT as u64 + 1;
        let mut keep = Vec::new();
        for (born, buf) in self.retired.drain(..) {
            if self.frame_counter.saturating_sub(born) > threshold {
                buf.destroy(&self.device, &self.allocator);
            } else {
                keep.push((born, buf));
            }
        }
        self.retired = keep;
    }

    pub fn draw(
        &mut self,
        vb: &mut VulkanBase,
        view_proj: Mat4,
        clear: [f32; 4],
    ) -> Result<(), vk::Result> {
        vb.wait_for_fence();
        self.collect_garbage();
        vb.reset_fence();

        let (image_index, suboptimal) = match vb.acquire_next_image() {
            Ok(v) => v,
            Err(vk::Result::ERROR_OUT_OF_DATE_KHR) => return Err(vk::Result::ERROR_OUT_OF_DATE_KHR),
            Err(e) => return Err(e),
        };
        if suboptimal {
            // Still try to render; caller may recreate.
        }

        let cmd = vb.current_command_buffer();
        unsafe {
            vb.device
                .reset_command_buffer(cmd, vk::CommandBufferResetFlags::empty())
                .unwrap();
            let begin = vk::CommandBufferBeginInfo::default();
            vb.device.begin_command_buffer(cmd, &begin).unwrap();

            let clear_values = [
                vk::ClearValue {
                    color: vk::ClearColorValue { float32: clear },
                },
                vk::ClearValue {
                    depth_stencil: vk::ClearDepthStencilValue {
                        depth: 1.0,
                        stencil: 0,
                    },
                },
            ];
            let rp_info = vk::RenderPassBeginInfo::default()
                .render_pass(self.render_pass)
                .framebuffer(self.framebuffers[image_index as usize])
                .render_area(vk::Rect2D {
                    offset: vk::Offset2D { x: 0, y: 0 },
                    extent: vb.swapchain_extent,
                })
                .clear_values(&clear_values);
            vb.device
                .cmd_begin_render_pass(cmd, &rp_info, vk::SubpassContents::INLINE);

            let extent = vb.swapchain_extent;
            let viewport = vk::Viewport {
                x: 0.0,
                y: 0.0,
                width: extent.width as f32,
                height: extent.height as f32,
                min_depth: 0.0,
                max_depth: 1.0,
            };
            let scissor = vk::Rect2D {
                offset: vk::Offset2D { x: 0, y: 0 },
                extent,
            };
            vb.device.cmd_set_viewport(cmd, 0, &[viewport]);
            vb.device.cmd_set_scissor(cmd, 0, &[scissor]);

            let pc = PushConstants {
                view_proj: view_proj.to_cols_array_2d(),
            };
            let pc_bytes = bytemuck::bytes_of(&pc);

            // Ground fill
            vb.device
                .cmd_bind_pipeline(cmd, vk::PipelineBindPoint::GRAPHICS, self.tri_pipeline);
            vb.device.cmd_push_constants(
                cmd,
                self.pipeline_layout,
                vk::ShaderStageFlags::VERTEX,
                0,
                pc_bytes,
            );
            if let Some(mesh) = &self.ground_fill {
                draw_mesh(&vb.device, cmd, mesh);
            }
            if let Some(mesh) = &self.rocket {
                draw_mesh(&vb.device, cmd, mesh);
            }

            // Ground lines
            vb.device
                .cmd_bind_pipeline(cmd, vk::PipelineBindPoint::GRAPHICS, self.line_pipeline);
            vb.device.cmd_push_constants(
                cmd,
                self.pipeline_layout,
                vk::ShaderStageFlags::VERTEX,
                0,
                pc_bytes,
            );
            if let Some(mesh) = &self.ground_lines {
                draw_mesh(&vb.device, cmd, mesh);
            }

            vb.device.cmd_end_render_pass(cmd);
            vb.device.end_command_buffer(cmd).unwrap();
        }

        match vb.submit_and_present(image_index) {
            Ok(_) => {}
            Err(vk::Result::ERROR_OUT_OF_DATE_KHR) | Err(vk::Result::SUBOPTIMAL_KHR) => {
                return Err(vk::Result::ERROR_OUT_OF_DATE_KHR);
            }
            Err(e) => return Err(e),
        }
        vb.advance_frame();
        self.frame_counter += 1;
        Ok(())
    }
}

impl Drop for Renderer {
    fn drop(&mut self) {
        let _ = unsafe { self.device.device_wait_idle() };
        for (_, buf) in self.retired.drain(..) {
            buf.destroy(&self.device, &self.allocator);
        }
        for mesh in [&mut self.ground_lines, &mut self.ground_fill, &mut self.rocket] {
            if let Some(m) = mesh.take() {
                m.vertex.destroy(&self.device, &self.allocator);
                m.index.destroy(&self.device, &self.allocator);
            }
        }
        unsafe {
            self.device.destroy_pipeline(self.tri_pipeline, None);
            self.device.destroy_pipeline(self.line_pipeline, None);
            self.device.destroy_pipeline_layout(self.pipeline_layout, None);
            for &fb in &self.framebuffers {
                self.device.destroy_framebuffer(fb, None);
            }
            self.depth_image.destroy(&self.device, &self.allocator);
            self.device.destroy_render_pass(self.render_pass, None);
        }
    }
}

fn draw_mesh(device: &ash::Device, cmd: vk::CommandBuffer, mesh: &GpuMesh) {
    unsafe {
        device.cmd_bind_vertex_buffers(cmd, 0, &[mesh.vertex.buffer], &[0]);
        device.cmd_bind_index_buffer(cmd, mesh.index.buffer, 0, vk::IndexType::UINT32);
        device.cmd_draw_indexed(cmd, mesh.index_count, 1, 0, 0, 0);
    }
}

fn create_render_pass(
    device: &ash::Device,
    color_format: vk::Format,
    depth_format: vk::Format,
) -> vk::RenderPass {
    let attachments = [
        vk::AttachmentDescription::default()
            .format(color_format)
            .samples(vk::SampleCountFlags::TYPE_1)
            .load_op(vk::AttachmentLoadOp::CLEAR)
            .store_op(vk::AttachmentStoreOp::STORE)
            .stencil_load_op(vk::AttachmentLoadOp::DONT_CARE)
            .stencil_store_op(vk::AttachmentStoreOp::DONT_CARE)
            .initial_layout(vk::ImageLayout::UNDEFINED)
            .final_layout(vk::ImageLayout::PRESENT_SRC_KHR),
        vk::AttachmentDescription::default()
            .format(depth_format)
            .samples(vk::SampleCountFlags::TYPE_1)
            .load_op(vk::AttachmentLoadOp::CLEAR)
            .store_op(vk::AttachmentStoreOp::DONT_CARE)
            .stencil_load_op(vk::AttachmentLoadOp::DONT_CARE)
            .stencil_store_op(vk::AttachmentStoreOp::DONT_CARE)
            .initial_layout(vk::ImageLayout::UNDEFINED)
            .final_layout(vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL),
    ];
    let color_ref = [vk::AttachmentReference {
        attachment: 0,
        layout: vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL,
    }];
    let depth_ref = vk::AttachmentReference {
        attachment: 1,
        layout: vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
    };
    let subpass = vk::SubpassDescription::default()
        .pipeline_bind_point(vk::PipelineBindPoint::GRAPHICS)
        .color_attachments(&color_ref)
        .depth_stencil_attachment(&depth_ref);
    let dependency = vk::SubpassDependency::default()
        .src_subpass(vk::SUBPASS_EXTERNAL)
        .dst_subpass(0)
        .src_stage_mask(
            vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT
                | vk::PipelineStageFlags::EARLY_FRAGMENT_TESTS,
        )
        .dst_stage_mask(
            vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT
                | vk::PipelineStageFlags::EARLY_FRAGMENT_TESTS,
        )
        .dst_access_mask(
            vk::AccessFlags::COLOR_ATTACHMENT_WRITE
                | vk::AccessFlags::DEPTH_STENCIL_ATTACHMENT_WRITE,
        );
    let subpasses = [subpass];
    let deps = [dependency];
    let ci = vk::RenderPassCreateInfo::default()
        .attachments(&attachments)
        .subpasses(&subpasses)
        .dependencies(&deps);
    unsafe { device.create_render_pass(&ci, None) }.unwrap()
}

fn create_framebuffers(
    device: &ash::Device,
    render_pass: vk::RenderPass,
    views: &[vk::ImageView],
    depth_view: vk::ImageView,
    extent: vk::Extent2D,
) -> Vec<vk::Framebuffer> {
    views
        .iter()
        .map(|&view| {
            let attachments = [view, depth_view];
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

fn create_mesh_pipeline(
    device: &ash::Device,
    render_pass: vk::RenderPass,
    layout: vk::PipelineLayout,
    topology: vk::PrimitiveTopology,
) -> vk::Pipeline {
    let vert = create_shader_module(device, MESH_VERT);
    let frag = create_shader_module(device, MESH_FRAG);
    let entry = CStr::from_bytes_with_nul(b"main\0").unwrap();
    let stages = [
        vk::PipelineShaderStageCreateInfo::default()
            .stage(vk::ShaderStageFlags::VERTEX)
            .module(vert)
            .name(entry),
        vk::PipelineShaderStageCreateInfo::default()
            .stage(vk::ShaderStageFlags::FRAGMENT)
            .module(frag)
            .name(entry),
    ];

    let binding = vk::VertexInputBindingDescription::default()
        .binding(0)
        .stride(std::mem::size_of::<Vertex>() as u32)
        .input_rate(vk::VertexInputRate::VERTEX);
    let attrs = [
        vk::VertexInputAttributeDescription::default()
            .location(0)
            .binding(0)
            .format(vk::Format::R32G32B32_SFLOAT)
            .offset(0),
        vk::VertexInputAttributeDescription::default()
            .location(1)
            .binding(0)
            .format(vk::Format::R32G32B32_SFLOAT)
            .offset(12),
    ];
    let bindings = [binding];
    let vertex_input = vk::PipelineVertexInputStateCreateInfo::default()
        .vertex_binding_descriptions(&bindings)
        .vertex_attribute_descriptions(&attrs);
    let input_assembly = vk::PipelineInputAssemblyStateCreateInfo::default().topology(topology);
    let viewport = vk::PipelineViewportStateCreateInfo::default()
        .viewport_count(1)
        .scissor_count(1);
    let raster = vk::PipelineRasterizationStateCreateInfo::default()
        .polygon_mode(vk::PolygonMode::FILL)
        .cull_mode(vk::CullModeFlags::NONE)
        .front_face(vk::FrontFace::COUNTER_CLOCKWISE)
        .line_width(1.0);
    let multisample =
        vk::PipelineMultisampleStateCreateInfo::default().rasterization_samples(vk::SampleCountFlags::TYPE_1);
    let depth = vk::PipelineDepthStencilStateCreateInfo::default()
        .depth_test_enable(true)
        .depth_write_enable(true)
        .depth_compare_op(vk::CompareOp::LESS);
    let blend_att = [vk::PipelineColorBlendAttachmentState::default()
        .color_write_mask(vk::ColorComponentFlags::RGBA)
        .blend_enable(false)];
    let blend = vk::PipelineColorBlendStateCreateInfo::default().attachments(&blend_att);
    let dynamic_states = [vk::DynamicState::VIEWPORT, vk::DynamicState::SCISSOR];
    let dynamic = vk::PipelineDynamicStateCreateInfo::default().dynamic_states(&dynamic_states);

    let ci = vk::GraphicsPipelineCreateInfo::default()
        .stages(&stages)
        .vertex_input_state(&vertex_input)
        .input_assembly_state(&input_assembly)
        .viewport_state(&viewport)
        .rasterization_state(&raster)
        .multisample_state(&multisample)
        .depth_stencil_state(&depth)
        .color_blend_state(&blend)
        .dynamic_state(&dynamic)
        .layout(layout)
        .render_pass(render_pass)
        .subpass(0);
    let pipelines = unsafe {
        device
            .create_graphics_pipelines(vk::PipelineCache::null(), &[ci], None)
            .unwrap()
    };
    unsafe {
        device.destroy_shader_module(vert, None);
        device.destroy_shader_module(frag, None);
    }
    pipelines[0]
}

/// Build a look-at view-projection matrix following the rocket.
pub fn camera_view_proj(
    target: Vec3,
    yaw: f32,
    pitch: f32,
    distance: f32,
    aspect: f32,
) -> Mat4 {
    let pitch = pitch.clamp(-1.4, 1.4);
    let offset = Vec3::new(
        distance * yaw.cos() * pitch.cos(),
        distance * pitch.sin(),
        distance * yaw.sin() * pitch.cos(),
    );
    let eye = target + offset;
    let view = Mat4::look_at_rh(eye, target, Vec3::Y);
    // Vulkan NDC has +Y down; flip projection Y so world +Y appears up on screen
    // (same convention as minecraft-clone).
    let mut proj = Mat4::perspective_rh(45f32.to_radians(), aspect.max(0.1), 0.5, 2000.0);
    proj.y_axis.y *= -1.0;
    proj * view
}
