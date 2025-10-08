use crate::camera::OrbitCamera;
use crate::integration::Gui;
use crate::ui_state::*;
use glam::{Mat4, Vec3};
use std::sync::Arc;
use vulkano::{
    buffer::{Buffer, BufferContents, BufferCreateInfo, BufferUsage, Subbuffer},
    command_buffer::{
        AutoCommandBufferBuilder, CommandBufferInheritanceInfo, CommandBufferUsage,
        RenderPassBeginInfo, SecondaryAutoCommandBuffer, SubpassBeginInfo, SubpassContents,
        allocator::{StandardCommandBufferAllocator, StandardCommandBufferAllocatorCreateInfo},
    },
    device::{Device, Queue},
    format::Format,
    image::{SampleCount, view::ImageView},
    memory::allocator::{AllocationCreateInfo, MemoryTypeFilter, StandardMemoryAllocator},
    pipeline::{
        DynamicState, GraphicsPipeline, Pipeline, PipelineLayout, PipelineShaderStageCreateInfo,
        graphics::{
            GraphicsPipelineCreateInfo,
            color_blend::{AttachmentBlend, BlendFactor, BlendOp},
            color_blend::{ColorBlendAttachmentState, ColorBlendState},
            input_assembly::{InputAssemblyState, PrimitiveTopology},
            multisample::MultisampleState,
            rasterization::RasterizationState,
            vertex_input::{Vertex, VertexDefinition},
            viewport::{Viewport, ViewportState},
        },
        layout::PipelineDescriptorSetLayoutCreateInfo,
    },
    render_pass::{Framebuffer, FramebufferCreateInfo, RenderPass, Subpass},
    sync::GpuFuture,
};

const MOUSE_LEFT_DRAG_SENS: f32 = 0.003f32;
const MOUSE_RIGHT_DRAG_SENS: f32 = 0.001f32;
const SIZE_RATIO: f32 = 0.06;
const INITIAL_POSITION: Vec3 = Vec3::new(1.6, -1.6, 3.0);
const INITIAL_TARGET: Vec3 = Vec3::new(0.0, 0.0, 0.0);

#[repr(C)]
#[derive(BufferContents, Vertex)]
struct AxesVertex {
    #[format(R32G32B32_SFLOAT)]
    position: [f32; 3],
    #[format(R32G32B32A32_SFLOAT)]
    color: [f32; 4],
}

#[repr(C)]
#[derive(BufferContents, Vertex)]
struct ParticleVertex {
    #[format(R32G32B32_SFLOAT)]
    position: [f32; 3],
    #[format(R32G32B32A32_SFLOAT)]
    color: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, BufferContents)]
struct AxesPushConstants {
    view_proj: [[f32; 4]; 4],
}

#[repr(C)]
#[derive(Clone, Copy, BufferContents)]
struct PushConstants {
    view_proj: [[f32; 4]; 4],
    size_scale: f32,
}

mod vs_axes {
    vulkano_shaders::shader! {
        ty: "vertex",
        path: "./src/shaders/axes_vertex.glsl"
    }
}

mod fs_axes {
    vulkano_shaders::shader! {
        ty: "fragment",
        path: "./src/shaders/axes_fragment.glsl"
    }
}

mod vs_particles {
    vulkano_shaders::shader! {
        ty: "vertex",
        path: "./src/shaders/particles_vertex.glsl"
    }
}

mod fs_particles {
    vulkano_shaders::shader! {
        ty: "fragment",
        path: "./src/shaders/particles_fragment.glsl"
    }
}

pub struct ParticleRenderPipeline {
    queue: Arc<Queue>,
    render_pass: Arc<RenderPass>,
    pipeline_axes: Arc<GraphicsPipeline>,
    pipeline_particles: Arc<GraphicsPipeline>,
    subpass: Subpass,
    axes_buffer: Subbuffer<[AxesVertex]>,
    particle_buffer: Subbuffer<[ParticleVertex]>,
    memory_allocator: Arc<StandardMemoryAllocator>,
    command_buffer_allocator: Arc<StandardCommandBufferAllocator>,
    camera: OrbitCamera,
    // aspect_ratio: f32,
}

impl ParticleRenderPipeline {
    pub fn new(
        queue: Arc<Queue>,
        image_format: vulkano::format::Format,
        allocator: &Arc<StandardMemoryAllocator>,
    ) -> Self {
        let render_pass = Self::create_render_pass(queue.device().clone(), image_format);
        let (pipeline_axes, pipeline_particles, subpass) =
            Self::create_pipeline(queue.device().clone(), render_pass.clone());
        let axes_buffer = Self::create_axes_buffer(allocator);
        let particle_buffer = Self::create_particle_buffer(allocator);
        let command_buffer_allocator = StandardCommandBufferAllocator::new(
            queue.device().clone(),
            StandardCommandBufferAllocatorCreateInfo {
                secondary_buffer_count: 32,

                ..Default::default()
            },
        )
        .into();
        let camera = OrbitCamera::new(INITIAL_POSITION, INITIAL_TARGET);
        Self {
            queue,
            render_pass,
            pipeline_axes,
            pipeline_particles,
            subpass,
            axes_buffer,
            particle_buffer,
            memory_allocator: allocator.clone(),
            command_buffer_allocator,
            camera,
            // aspect_ratio: 1.0,
        }
    }

    pub fn set_particles(&mut self, positions: &[[f32; 3]]) {
        let verts: Vec<ParticleVertex> = positions
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let color = match i % 5 {
                    0 => [1.0, 0.3, 0.2, 1.0], // Reddish color
                    1 => [0.2, 0.5, 1.0, 1.0], // Bluish color
                    2 => [1.0, 0.8, 0.2, 1.0], // Yellowish color
                    3 => [0.9, 0.4, 1.0, 1.0], // Purplish color
                    4 => [0.6, 1.0, 0.8, 1.0], // Cyanish color
                    _ => unreachable!(),
                };
                ParticleVertex {
                    position: *p,
                    color,
                }
            })
            .collect();
        let new_buf = Buffer::from_iter(
            self.memory_allocator.clone(),
            BufferCreateInfo {
                usage: BufferUsage::VERTEX_BUFFER | BufferUsage::TRANSFER_DST,
                ..Default::default()
            },
            AllocationCreateInfo {
                memory_type_filter: MemoryTypeFilter::PREFER_DEVICE
                    | MemoryTypeFilter::HOST_RANDOM_ACCESS,
                ..Default::default()
            },
            verts,
        )
        .unwrap();
        self.particle_buffer = new_buf;
    }

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

    fn create_render_pass(device: Arc<Device>, format: Format) -> Arc<RenderPass> {
        vulkano::ordered_passes_renderpass!(
            device,
            attachments: {
                color: {
                    format: format,
                    samples: SampleCount::Sample1,
                    load_op: Clear,
                    store_op: Store,
                }
            },
            passes: [
                { color: [color], depth_stencil: {}, input: [] },
                { color: [color], depth_stencil: {}, input: [] }
            ]
        )
        .unwrap()
    }

    pub fn gui_pass(&self) -> Subpass {
        Subpass::from(self.render_pass.clone(), 1).unwrap()
    }

    fn create_pipeline(
        device: Arc<Device>,
        render_pass: Arc<RenderPass>,
    ) -> (Arc<GraphicsPipeline>, Arc<GraphicsPipeline>, Subpass) {
        let subpass = Subpass::from(render_pass, 0).unwrap();
        let vs_axes = vs_axes::load(device.clone())
            .expect("failed to create shader module")
            .entry_point("main")
            .unwrap();
        let fs_axes = fs_axes::load(device.clone())
            .expect("failed to create shader module")
            .entry_point("main")
            .unwrap();
        let vertex_input_state_axes = AxesVertex::per_vertex().definition(&vs_axes).unwrap();
        let axes_stages = [
            PipelineShaderStageCreateInfo::new(vs_axes),
            PipelineShaderStageCreateInfo::new(fs_axes),
        ];
        let axes_layout = PipelineLayout::new(
            device.clone(),
            PipelineDescriptorSetLayoutCreateInfo::from_stages(&axes_stages)
                .into_pipeline_layout_create_info(device.clone())
                .unwrap(),
        )
        .unwrap();
        let pipeline_axes = GraphicsPipeline::new(
            device.clone(),
            None,
            GraphicsPipelineCreateInfo {
                stages: axes_stages.into_iter().collect(),
                vertex_input_state: Some(vertex_input_state_axes),
                input_assembly_state: Some(InputAssemblyState {
                    topology: PrimitiveTopology::LineList,
                    ..Default::default()
                }),
                viewport_state: Some(ViewportState::default()),
                rasterization_state: Some(RasterizationState::default()),
                multisample_state: Some(MultisampleState::default()),
                color_blend_state: Some(ColorBlendState::with_attachment_states(
                    subpass.num_color_attachments(),
                    ColorBlendAttachmentState::default(),
                )),
                dynamic_state: [DynamicState::Viewport].into_iter().collect(),
                subpass: Some(subpass.clone().into()),
                ..GraphicsPipelineCreateInfo::layout(axes_layout)
            },
        )
        .unwrap();
        let vs_particles = vs_particles::load(device.clone())
            .expect("failed to create shader module")
            .entry_point("main")
            .unwrap();
        let fs_particles = fs_particles::load(device.clone())
            .expect("failed to create shader module")
            .entry_point("main")
            .unwrap();
        let vertex_input_state_particles = ParticleVertex::per_vertex()
            .definition(&vs_particles)
            .unwrap();
        let particles_stages = [
            PipelineShaderStageCreateInfo::new(vs_particles),
            PipelineShaderStageCreateInfo::new(fs_particles),
        ];
        let particles_layout = PipelineLayout::new(
            device.clone(),
            PipelineDescriptorSetLayoutCreateInfo::from_stages(&particles_stages)
                .into_pipeline_layout_create_info(device.clone())
                .unwrap(),
        )
        .unwrap();
        let cbas = ColorBlendAttachmentState {
            blend: Some(AttachmentBlend {
                src_color_blend_factor: BlendFactor::One,
                dst_color_blend_factor: BlendFactor::One,
                color_blend_op: BlendOp::Add,
                src_alpha_blend_factor: BlendFactor::One,
                dst_alpha_blend_factor: BlendFactor::One,
                alpha_blend_op: BlendOp::Add,
            }),
            ..Default::default()
        };
        let pipeline_particles = GraphicsPipeline::new(
            device.clone(),
            None,
            GraphicsPipelineCreateInfo {
                stages: particles_stages.into_iter().collect(),
                vertex_input_state: Some(vertex_input_state_particles),
                input_assembly_state: Some(InputAssemblyState {
                    topology: PrimitiveTopology::PointList,
                    ..Default::default()
                }),
                viewport_state: Some(ViewportState::default()),
                rasterization_state: Some(RasterizationState::default()),
                multisample_state: Some(MultisampleState::default()),
                color_blend_state: Some(ColorBlendState::with_attachment_states(
                    subpass.num_color_attachments(),
                    cbas,
                )),
                dynamic_state: [DynamicState::Viewport].into_iter().collect(),
                subpass: Some(subpass.clone().into()),
                ..GraphicsPipelineCreateInfo::layout(particles_layout)
            },
        )
        .unwrap();
        (pipeline_axes, pipeline_particles, subpass)
    }

    pub fn render(
        &mut self,
        before_future: Box<dyn GpuFuture>,
        image: Arc<ImageView>,
        gui: &mut Gui,
        scale: f64,
    ) -> Box<dyn GpuFuture> {
        let mut builder = AutoCommandBufferBuilder::primary(
            self.command_buffer_allocator.clone(),
            self.queue.queue_family_index(),
            CommandBufferUsage::OneTimeSubmit,
        )
        .unwrap();
        let dimensions = image.image().extent();
        let framebuffer = Framebuffer::new(
            self.render_pass.clone(),
            FramebufferCreateInfo {
                attachments: vec![image],
                ..Default::default()
            },
        )
        .unwrap();
        builder
            .begin_render_pass(
                RenderPassBeginInfo {
                    clear_values: vec![Some([0.0, 0.0, 0.0, 1.0].into())],
                    ..RenderPassBeginInfo::framebuffer(framebuffer)
                },
                SubpassBeginInfo {
                    contents: SubpassContents::SecondaryCommandBuffers,
                    ..Default::default()
                },
            )
            .unwrap();
        let mut secondary_builder = AutoCommandBufferBuilder::secondary(
            self.command_buffer_allocator.clone(),
            self.queue.queue_family_index(),
            CommandBufferUsage::MultipleSubmit,
            CommandBufferInheritanceInfo {
                render_pass: Some(self.subpass.clone().into()),
                ..Default::default()
            },
        )
        .unwrap();
        let viewport = Viewport {
            offset: [0.0, 0.0],
            extent: [dimensions[0] as f32, dimensions[1] as f32],
            depth_range: 0.0..=1.0,
        };
        let aspect_ratio = dimensions[0] as f32 / dimensions[1] as f32;
        let view_proj = self.compute_mvp_axes(aspect_ratio);
        let push_constants = AxesPushConstants {
            view_proj: view_proj.to_cols_array_2d(),
        };
        self.draw_axes(&mut secondary_builder, &viewport, &push_constants);
        let scale_factor = (scale / DEFAULT_SCALE_UI).powi(4) as f32;
        let view_proj = self.compute_mvp_particle(aspect_ratio, scale_factor);
        let size_scale = dimensions[1] as f32 * SIZE_RATIO * scale_factor;
        let push_constants = PushConstants {
            view_proj: view_proj.to_cols_array_2d(),
            size_scale: size_scale.into(),
        };
        self.draw_particles(&mut secondary_builder, &viewport, &push_constants);
        let cb = secondary_builder.build().unwrap();
        builder.execute_commands(cb).unwrap();
        builder
            .next_subpass(
                Default::default(),
                SubpassBeginInfo {
                    contents: SubpassContents::SecondaryCommandBuffers,
                    ..Default::default()
                },
            )
            .unwrap();
        let cb = gui.draw_on_subpass_image([dimensions[0], dimensions[1]]);
        builder.execute_commands(cb).unwrap();
        builder.end_render_pass(Default::default()).unwrap();
        let command_buffer = builder.build().unwrap();
        let after_future = before_future
            .then_execute(self.queue.clone(), command_buffer)
            .unwrap();
        after_future.boxed()
    }

    fn create_axes_buffer(allocator: &Arc<StandardMemoryAllocator>) -> Subbuffer<[AxesVertex]> {
        let mut vertices: Vec<AxesVertex> = Vec::new();
        let range = 2.0;
        let num_lines = 9;
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
        Buffer::from_iter(
            allocator.clone(),
            BufferCreateInfo {
                usage: BufferUsage::VERTEX_BUFFER,
                ..Default::default()
            },
            AllocationCreateInfo {
                memory_type_filter: MemoryTypeFilter::PREFER_DEVICE
                    | MemoryTypeFilter::HOST_RANDOM_ACCESS,
                ..Default::default()
            },
            vertices,
        )
        .unwrap()
    }

    fn create_particle_buffer(
        allocator: &Arc<StandardMemoryAllocator>,
    ) -> Subbuffer<[ParticleVertex]> {
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
        Buffer::from_iter(
            allocator.clone(),
            BufferCreateInfo {
                usage: BufferUsage::VERTEX_BUFFER | BufferUsage::TRANSFER_DST,
                ..Default::default()
            },
            AllocationCreateInfo {
                memory_type_filter: MemoryTypeFilter::PREFER_DEVICE
                    | MemoryTypeFilter::HOST_RANDOM_ACCESS,
                ..Default::default()
            },
            particles,
        )
        .unwrap()
    }

    fn draw_axes(
        &self,
        builder: &mut AutoCommandBufferBuilder<SecondaryAutoCommandBuffer>,
        viewport: &Viewport,
        push_constants: &AxesPushConstants,
    ) {
        builder
            .bind_pipeline_graphics(self.pipeline_axes.clone())
            .unwrap()
            .set_viewport(0, [viewport.clone()].into_iter().collect())
            .unwrap()
            .bind_vertex_buffers(0, self.axes_buffer.clone())
            .unwrap()
            .push_constants(
                self.pipeline_axes.layout().clone(),
                0,
                push_constants.clone(),
            )
            .unwrap();
        unsafe {
            builder
                .draw(self.axes_buffer.len() as u32, 1, 0, 0)
                .unwrap();
        }
    }

    fn draw_particles(
        &self,
        builder: &mut AutoCommandBufferBuilder<SecondaryAutoCommandBuffer>,
        viewport: &Viewport,
        push_constants: &PushConstants,
    ) {
        builder
            .bind_pipeline_graphics(self.pipeline_particles.clone())
            .unwrap()
            .set_viewport(0, [viewport.clone()].into_iter().collect())
            .unwrap()
            .bind_vertex_buffers(0, self.particle_buffer.clone())
            .unwrap()
            .push_constants(
                self.pipeline_particles.layout().clone(),
                0,
                push_constants.clone(),
            )
            .unwrap();
        unsafe {
            builder
                .draw(self.particle_buffer.len() as u32, 1, 0, 0)
                .unwrap();
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

#[cfg(test)]
mod tests {
    use super::*;
    use vulkano_util::context::{VulkanoConfig, VulkanoContext};

    #[test]
    fn test_pipeline_creation() {
        let context = VulkanoContext::new(VulkanoConfig::default());
        let pipeline = ParticleRenderPipeline::new(
            context.graphics_queue().clone(),
            Format::B8G8R8A8_UNORM,
            context.memory_allocator(),
        );
        assert!(Arc::strong_count(&pipeline.pipeline_axes) > 0);
    }
}
