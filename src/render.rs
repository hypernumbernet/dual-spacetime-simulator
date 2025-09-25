use crate::integration::Gui;
use cgmath::{Matrix4, Point3, Rad, Vector3, perspective};
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
    #[format(R32_SFLOAT)]
    size: f32,
}

#[repr(C)]
#[derive(Clone, Copy, BufferContents)]
struct PushConstants {
    view_proj: [[f32; 4]; 4],
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
    command_buffer_allocator: Arc<StandardCommandBufferAllocator>,
    camera_position: Point3<f32>,
    camera_target: Point3<f32>,
    camera_up: Vector3<f32>,
    aspect_ratio: f32,
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
        let axes_buffer = Buffer::from_iter(
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
            [
                // Red line: from (0,0,0) to (1,0,0)
                AxesVertex {
                    position: [0.0, 0.0, 0.0],
                    color: [1.0, 0.0, 0.0, 1.0],
                },
                AxesVertex {
                    position: [1.0, 0.0, 0.0],
                    color: [1.0, 0.0, 0.0, 1.0],
                },
                // Green line: from (0,0,0) to (0,1,0)
                AxesVertex {
                    position: [0.0, 0.0, 0.0],
                    color: [0.0, 1.0, 0.0, 1.0],
                },
                AxesVertex {
                    position: [0.0, 1.0, 0.0],
                    color: [0.0, 1.0, 0.0, 1.0],
                },
                // Blue line: from (0,0,0) to (0,0,1)
                AxesVertex {
                    position: [0.0, 0.0, 0.0],
                    color: [0.0, 0.0, 1.0, 1.0],
                },
                AxesVertex {
                    position: [0.0, 0.0, 1.0],
                    color: [0.0, 0.0, 1.0, 1.0],
                },
            ],
        )
        .unwrap();

        let particle_buffer = Self::create_particle_buffer(allocator);

        let command_buffer_allocator = StandardCommandBufferAllocator::new(
            queue.device().clone(),
            StandardCommandBufferAllocatorCreateInfo {
                secondary_buffer_count: 32,
                ..Default::default()
            },
        )
        .into();

        Self {
            queue,
            render_pass,
            pipeline_axes,
            pipeline_particles,
            subpass,
            axes_buffer,
            particle_buffer,
            command_buffer_allocator,
            camera_position: Point3::new(1.6, 1.6, 3.0),
            camera_target: Point3::new(0.0, 0.0, 0.0),
            camera_up: Vector3::new(-1.0, 0.0, 0.0),
            aspect_ratio: 1.0,
        }
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
                    ColorBlendAttachmentState::default(),
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

        self.draw_axes(&mut secondary_builder);

        self.draw_particles(&mut secondary_builder);

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

    fn create_particle_buffer(
        allocator: &Arc<StandardMemoryAllocator>,
    ) -> Subbuffer<[ParticleVertex]> {
        let mut particles = Vec::with_capacity(100);
        for _ in 0..100 {
            particles.push(ParticleVertex {
                position: [rand::random::<f32>() * 2.0 - 1.0; 3],
                color: [1.0, 1.0, 1.0, 1.0],
                size: 5.0 + rand::random::<f32>() * 10.0,
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

    fn draw_axes(&self, builder: &mut AutoCommandBufferBuilder<SecondaryAutoCommandBuffer>) {
        let view_proj = self.compute_view_proj();
        let push_constants = PushConstants {
            view_proj: view_proj.into(),
        };

        builder
            .bind_pipeline_graphics(self.pipeline_axes.clone())
            .unwrap();

        let viewport = Viewport {
            offset: [0.0, 0.0],
            extent: [self.aspect_ratio * 1000.0, 1000.0],
            depth_range: 0.0..=1.0,
        };
        builder
            .set_viewport(0, smallvec::smallvec![viewport])
            .unwrap();

        builder
            .bind_vertex_buffers(0, self.axes_buffer.clone())
            .unwrap();

        builder
            .push_constants(self.pipeline_axes.layout().clone(), 0, push_constants)
            .unwrap();

        unsafe {
            builder
                .draw(self.axes_buffer.len() as u32, 1, 0, 0)
                .unwrap();
        }
    }

    fn draw_particles(&self, builder: &mut AutoCommandBufferBuilder<SecondaryAutoCommandBuffer>) {
        let view_proj = self.compute_view_proj();
        let push_constants = PushConstants {
            view_proj: view_proj.into(),
        };

        builder
            .bind_pipeline_graphics(self.pipeline_particles.clone())
            .unwrap();

        let viewport = Viewport {
            offset: [0.0, 0.0],
            extent: [self.aspect_ratio * 1000.0, 1000.0],
            depth_range: 0.0..=1.0,
        };
        builder
            .set_viewport(0, smallvec::smallvec![viewport])
            .unwrap();

        builder
            .bind_vertex_buffers(0, self.particle_buffer.clone())
            .unwrap();

        builder
            .push_constants(self.pipeline_particles.layout().clone(), 0, push_constants)
            .unwrap();

        unsafe {
            builder
                .draw(self.particle_buffer.len() as u32, 1, 0, 0)
                .unwrap();
        }
    }

    fn compute_view_proj(&self) -> Matrix4<f32> {
        let view = Matrix4::look_at_rh(self.camera_position, self.camera_target, self.camera_up);
        let proj = perspective(
            Rad(std::f32::consts::FRAC_PI_4),
            self.aspect_ratio,
            0.1,
            100.0,
        );
        proj * view
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
