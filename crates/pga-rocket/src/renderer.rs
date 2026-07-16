//! Colored rocket mesh + textured grass ground (minecraft-style tiling + fog).

use crate::mesh::{
    GRASS_METERS_PER_TILE, GROUND_FOG_END, GROUND_FOG_START, GROUND_HALF_EXTENT, GroundVertex,
    Vertex, grass_ground_mesh, rocket_mesh,
};
use crate::sim::RocketState;
use crate::texture::{Texture, create_grass_texture};
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
const GROUND_VERT: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/shaders/ground.vert.spv"));
const GROUND_FRAG: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/shaders/ground.frag.spv"));

/// Sky / fog color (matches clear color).
pub const SKY_COLOR: [f32; 4] = [0.45, 0.62, 0.85, 1.0];

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct MeshPush {
    view_proj: [[f32; 4]; 4],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct GroundPush {
    view_proj: [[f32; 4]; 4],
    camera_pos: [f32; 4],
    fog_color: [f32; 4],
    fog_params: [f32; 4],
    ground_origin: [f32; 4],
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
    mesh_layout: vk::PipelineLayout,
    ground_layout: vk::PipelineLayout,
    tri_pipeline: vk::Pipeline,
    ground_pipeline: vk::Pipeline,
    grass: Texture,
    desc_set_layout: vk::DescriptorSetLayout,
    desc_pool: vk::DescriptorPool,
    desc_set: vk::DescriptorSet,
    ground: Option<GpuMesh>,
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

        // Mesh (rocket) layout: view_proj only.
        let mesh_push = vk::PushConstantRange {
            stage_flags: vk::ShaderStageFlags::VERTEX,
            offset: 0,
            size: std::mem::size_of::<MeshPush>() as u32,
        };
        let mesh_ranges = [mesh_push];
        let mesh_pl_ci =
            vk::PipelineLayoutCreateInfo::default().push_constant_ranges(&mesh_ranges);
        let mesh_layout = unsafe { device.create_pipeline_layout(&mesh_pl_ci, None) }.unwrap();
        let tri_pipeline = create_mesh_pipeline(&device, render_pass, mesh_layout);

        // Grass texture + ground pipeline.
        let grass = create_grass_texture(vb, &allocator);

        let binding = vk::DescriptorSetLayoutBinding::default()
            .binding(0)
            .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
            .descriptor_count(1)
            .stage_flags(vk::ShaderStageFlags::FRAGMENT);
        let bindings = [binding];
        let dsl_ci = vk::DescriptorSetLayoutCreateInfo::default().bindings(&bindings);
        let desc_set_layout =
            unsafe { device.create_descriptor_set_layout(&dsl_ci, None) }.unwrap();

        let pool_size = vk::DescriptorPoolSize {
            ty: vk::DescriptorType::COMBINED_IMAGE_SAMPLER,
            descriptor_count: 1,
        };
        let pool_sizes = [pool_size];
        let pool_ci = vk::DescriptorPoolCreateInfo::default()
            .pool_sizes(&pool_sizes)
            .max_sets(1);
        let desc_pool = unsafe { device.create_descriptor_pool(&pool_ci, None) }.unwrap();
        let set_layouts = [desc_set_layout];
        let alloc_ci = vk::DescriptorSetAllocateInfo::default()
            .descriptor_pool(desc_pool)
            .set_layouts(&set_layouts);
        let desc_set = unsafe { device.allocate_descriptor_sets(&alloc_ci) }.unwrap()[0];

        let image_info = [vk::DescriptorImageInfo {
            sampler: grass.sampler,
            image_view: grass.image.view,
            image_layout: vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
        }];
        let write = [vk::WriteDescriptorSet::default()
            .dst_set(desc_set)
            .dst_binding(0)
            .descriptor_type(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
            .image_info(&image_info)];
        unsafe { device.update_descriptor_sets(&write, &[]) };

        let ground_push = vk::PushConstantRange {
            stage_flags: vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT,
            offset: 0,
            size: std::mem::size_of::<GroundPush>() as u32,
        };
        let ground_ranges = [ground_push];
        let ground_set_layouts = [desc_set_layout];
        let ground_pl_ci = vk::PipelineLayoutCreateInfo::default()
            .set_layouts(&ground_set_layouts)
            .push_constant_ranges(&ground_ranges);
        let ground_layout = unsafe { device.create_pipeline_layout(&ground_pl_ci, None) }.unwrap();
        let ground_pipeline = create_ground_pipeline(&device, render_pass, ground_layout);

        let mut renderer = Self {
            device,
            allocator,
            depth_format,
            render_pass,
            depth_image,
            framebuffers,
            mesh_layout,
            ground_layout,
            tri_pipeline,
            ground_pipeline,
            grass,
            desc_set_layout,
            desc_pool,
            desc_set,
            ground: None,
            rocket: None,
            retired: Vec::new(),
            frame_counter: 0,
            last_hud: String::new(),
        };

        // Local grass plane; recentered under the rocket each frame via push constants.
        let (gverts, gidx) = grass_ground_mesh(GROUND_HALF_EXTENT, 32);
        renderer.ground = Some(renderer.upload_ground(&gverts, &gidx));
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

    fn upload_ground(&self, verts: &[GroundVertex], indices: &[u32]) -> GpuMesh {
        let (vertex, _) = create_buffer_with_data(
            &self.device,
            &self.allocator,
            verts,
            vk::BufferUsageFlags::VERTEX_BUFFER,
            "ground-v",
        );
        let (index, index_count) = create_buffer_with_data(
            &self.device,
            &self.allocator,
            indices,
            vk::BufferUsageFlags::INDEX_BUFFER,
            "ground-i",
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
        camera_eye: Vec3,
        ground_center_xz: [f32; 2],
        clear: [f32; 4],
    ) -> Result<(), vk::Result> {
        vb.wait_for_fence();
        self.collect_garbage();
        vb.reset_fence();

        let (image_index, _suboptimal) = match vb.acquire_next_image() {
            Ok(v) => v,
            Err(vk::Result::ERROR_OUT_OF_DATE_KHR) => return Err(vk::Result::ERROR_OUT_OF_DATE_KHR),
            Err(e) => return Err(e),
        };

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

            // --- Grass ground ---
            let gpc = GroundPush {
                view_proj: view_proj.to_cols_array_2d(),
                camera_pos: [camera_eye.x, camera_eye.y, camera_eye.z, 0.0],
                fog_color: [clear[0], clear[1], clear[2], 1.0],
                fog_params: [
                    GROUND_FOG_START,
                    GROUND_FOG_END,
                    GRASS_METERS_PER_TILE,
                    0.0,
                ],
                // Snap to tile grid so UV does not crawl when recentering.
                ground_origin: [
                    ground_center_xz[0],
                    0.0,
                    ground_center_xz[1],
                    0.0,
                ],
            };
            vb.device
                .cmd_bind_pipeline(cmd, vk::PipelineBindPoint::GRAPHICS, self.ground_pipeline);
            vb.device.cmd_bind_descriptor_sets(
                cmd,
                vk::PipelineBindPoint::GRAPHICS,
                self.ground_layout,
                0,
                &[self.desc_set],
                &[],
            );
            vb.device.cmd_push_constants(
                cmd,
                self.ground_layout,
                vk::ShaderStageFlags::VERTEX | vk::ShaderStageFlags::FRAGMENT,
                0,
                bytemuck::bytes_of(&gpc),
            );
            if let Some(mesh) = &self.ground {
                draw_mesh(&vb.device, cmd, mesh);
            }

            // --- Rocket ---
            let mpc = MeshPush {
                view_proj: view_proj.to_cols_array_2d(),
            };
            vb.device
                .cmd_bind_pipeline(cmd, vk::PipelineBindPoint::GRAPHICS, self.tri_pipeline);
            vb.device.cmd_push_constants(
                cmd,
                self.mesh_layout,
                vk::ShaderStageFlags::VERTEX,
                0,
                bytemuck::bytes_of(&mpc),
            );
            if let Some(mesh) = &self.rocket {
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
        for mesh in [&mut self.ground, &mut self.rocket] {
            if let Some(m) = mesh.take() {
                m.vertex.destroy(&self.device, &self.allocator);
                m.index.destroy(&self.device, &self.allocator);
            }
        }
        self.grass.destroy(&self.device, &self.allocator);
        unsafe {
            self.device.destroy_pipeline(self.tri_pipeline, None);
            self.device.destroy_pipeline(self.ground_pipeline, None);
            self.device.destroy_pipeline_layout(self.mesh_layout, None);
            self.device.destroy_pipeline_layout(self.ground_layout, None);
            self.device.destroy_descriptor_pool(self.desc_pool, None);
            self.device.destroy_descriptor_set_layout(self.desc_set_layout, None);
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
) -> vk::Pipeline {
    create_graphics_pipeline(
        device,
        render_pass,
        layout,
        MESH_VERT,
        MESH_FRAG,
        std::mem::size_of::<Vertex>() as u32,
        &[
            (
                0,
                vk::Format::R32G32B32_SFLOAT,
                0,
            ),
            (
                1,
                vk::Format::R32G32B32_SFLOAT,
                12,
            ),
        ],
    )
}

fn create_ground_pipeline(
    device: &ash::Device,
    render_pass: vk::RenderPass,
    layout: vk::PipelineLayout,
) -> vk::Pipeline {
    create_graphics_pipeline(
        device,
        render_pass,
        layout,
        GROUND_VERT,
        GROUND_FRAG,
        std::mem::size_of::<GroundVertex>() as u32,
        &[
            (0, vk::Format::R32G32B32_SFLOAT, 0),
            (1, vk::Format::R32G32_SFLOAT, 12),
        ],
    )
}

fn create_graphics_pipeline(
    device: &ash::Device,
    render_pass: vk::RenderPass,
    layout: vk::PipelineLayout,
    vert_spv: &[u8],
    frag_spv: &[u8],
    stride: u32,
    attrs: &[(u32, vk::Format, u32)],
) -> vk::Pipeline {
    let vert = create_shader_module(device, vert_spv);
    let frag = create_shader_module(device, frag_spv);
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
        .stride(stride)
        .input_rate(vk::VertexInputRate::VERTEX);
    let attr_descs: Vec<_> = attrs
        .iter()
        .map(|&(location, format, offset)| {
            vk::VertexInputAttributeDescription::default()
                .location(location)
                .binding(0)
                .format(format)
                .offset(offset)
        })
        .collect();
    let bindings = [binding];
    let vertex_input = vk::PipelineVertexInputStateCreateInfo::default()
        .vertex_binding_descriptions(&bindings)
        .vertex_attribute_descriptions(&attr_descs);
    let input_assembly = vk::PipelineInputAssemblyStateCreateInfo::default()
        .topology(vk::PrimitiveTopology::TRIANGLE_LIST);
    let viewport = vk::PipelineViewportStateCreateInfo::default()
        .viewport_count(1)
        .scissor_count(1);
    let raster = vk::PipelineRasterizationStateCreateInfo::default()
        .polygon_mode(vk::PolygonMode::FILL)
        .cull_mode(vk::CullModeFlags::NONE)
        .front_face(vk::FrontFace::COUNTER_CLOCKWISE)
        .line_width(1.0);
    let multisample = vk::PipelineMultisampleStateCreateInfo::default()
        .rasterization_samples(vk::SampleCountFlags::TYPE_1);
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

/// Build look-at view-projection and return (view_proj, eye position).
pub fn camera_view_proj(
    target: Vec3,
    yaw: f32,
    pitch: f32,
    distance: f32,
    aspect: f32,
) -> (Mat4, Vec3) {
    let pitch = pitch.clamp(-1.4, 1.4);
    let offset = Vec3::new(
        distance * yaw.cos() * pitch.cos(),
        distance * pitch.sin(),
        distance * yaw.sin() * pitch.cos(),
    );
    let eye = target + offset;
    let view = Mat4::look_at_rh(eye, target, Vec3::Y);
    // Vulkan NDC has +Y down; flip projection Y so world +Y appears up on screen.
    let mut proj = Mat4::perspective_rh(45f32.to_radians(), aspect.max(0.1), 0.5, 4000.0);
    proj.y_axis.y *= -1.0;
    (proj * view, eye)
}
