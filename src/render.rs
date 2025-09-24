use crate::integration::Gui;
use std::sync::Arc;
use cgmath::{perspective, Matrix4, Point3, Rad, Vector3};
use vulkano::{
    buffer::{Buffer, BufferContents, BufferCreateInfo, BufferUsage, Subbuffer},
    pipeline::Pipeline,
    command_buffer::{
        AutoCommandBufferBuilder, CommandBufferInheritanceInfo, CommandBufferUsage,
        RenderPassBeginInfo, SubpassBeginInfo, SubpassContents,
        allocator::{StandardCommandBufferAllocator, StandardCommandBufferAllocatorCreateInfo},
    },
    device::{Device, Queue},
    format::Format,
    image::{SampleCount, view::ImageView},
    memory::allocator::{AllocationCreateInfo, MemoryTypeFilter, StandardMemoryAllocator},
    pipeline::{
        DynamicState, GraphicsPipeline, PipelineLayout, PipelineShaderStageCreateInfo,
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
struct ParticleVertex {
    #[format(R32G32B32_SFLOAT)]
    position: [f32; 3],
    #[format(R32G32B32A32_SFLOAT)]
    color: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, BufferContents)]
struct PushConstants {
    view_proj: [[f32; 4]; 4],
}

mod vs {
    vulkano_shaders::shader! {
        ty: "vertex",
        path: "./src/shaders/vertex.glsl"
    }
}

mod fs {
    vulkano_shaders::shader! {
        ty: "fragment",
        path: "./src/shaders/fragment.glsl"
    }
}

pub struct ParticleRenderPipeline {
    queue: Arc<Queue>,
    render_pass: Arc<RenderPass>,
    pipeline: Arc<GraphicsPipeline>,
    subpass: Subpass,
    vertex_buffer: Subbuffer<[ParticleVertex]>,
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
        let (pipeline, subpass) =
            Self::create_pipeline(queue.device().clone(), render_pass.clone());
        let vertex_buffer = Buffer::from_iter(
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
                ParticleVertex {
                    position: [0.0, 0.0, 0.0],
                    color: [1.0, 0.0, 0.0, 1.0],
                },
                ParticleVertex {
                    position: [1.0, 0.0, 0.0],
                    color: [1.0, 0.0, 0.0, 1.0],
                },
                // Green line: from (0,0,0) to (0,1,0)
                ParticleVertex {
                    position: [0.0, 0.0, 0.0],
                    color: [0.0, 1.0, 0.0, 1.0],
                },
                ParticleVertex {
                    position: [0.0, 1.0, 0.0],
                    color: [0.0, 1.0, 0.0, 1.0],
                },
                // Blue line: from (0,0,0) to (0,0,1)
                ParticleVertex {
                    position: [0.0, 0.0, 0.0],
                    color: [0.0, 0.0, 1.0, 1.0],
                },
                ParticleVertex {
                    position: [0.0, 0.0, 1.0],
                    color: [0.0, 0.0, 1.0, 1.0],
                },
            ],
        )
        .unwrap();
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
            pipeline,
            subpass,
            vertex_buffer,
            command_buffer_allocator,
            camera_position: Point3::new(2.0, 2.0, 2.0),
            camera_target: Point3::new(0.0, 0.0, 0.0),
            camera_up: Vector3::new(0.0, 1.0, 0.0),
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
    ) -> (Arc<GraphicsPipeline>, Subpass) {
        let vs = vs::load(device.clone())
            .expect("failed to create shader module")
            .entry_point("main")
            .unwrap();
        let fs = fs::load(device.clone())
            .expect("failed to create shader module")
            .entry_point("main")
            .unwrap();
        let vertex_input_state = ParticleVertex::per_vertex().definition(&vs).unwrap();
        let stages = [
            PipelineShaderStageCreateInfo::new(vs),
            PipelineShaderStageCreateInfo::new(fs),
        ];
        let layout = PipelineLayout::new(
            device.clone(),
            PipelineDescriptorSetLayoutCreateInfo::from_stages(&stages)
                .into_pipeline_layout_create_info(device.clone())
                .unwrap(),
        )
        .unwrap();
        let subpass = Subpass::from(render_pass, 0).unwrap();
        (
            GraphicsPipeline::new(
                device,
                None,
                GraphicsPipelineCreateInfo {
                    stages: stages.into_iter().collect(),
                    vertex_input_state: Some(vertex_input_state),
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
                    ..GraphicsPipelineCreateInfo::layout(layout)
                },
            )
            .unwrap(),
            subpass,
        )
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
        secondary_builder
            .bind_pipeline_graphics(self.pipeline.clone())
            .unwrap()
            .set_viewport(
                0,
                [Viewport {
                    offset: [0.0, 0.0],
                    extent: [dimensions[0] as f32, dimensions[1] as f32],
                    depth_range: 0.0..=1.0,
                }]
                .into_iter()
                .collect(),
            )
            .unwrap()
            .bind_vertex_buffers(0, self.vertex_buffer.clone())
            .unwrap();

        self.aspect_ratio = dimensions[0] as f32 / dimensions[1] as f32;

        let view = Matrix4::look_at_rh(
            self.camera_position,
            self.camera_target,
            self.camera_up,
        );
        let proj = perspective(
            Rad(std::f32::consts::FRAC_PI_4),
            self.aspect_ratio,
            0.1,
            100.0,
        );
        let view_proj = proj * view;

        let push_constants = PushConstants {
            view_proj: view_proj.into(),
        };

        secondary_builder
            .push_constants(
                self.pipeline.layout().clone(),
                0,
                push_constants,
            )
            .unwrap();

        unsafe {
            secondary_builder
                .draw(self.vertex_buffer.len() as u32, 1, 0, 0)
                .unwrap();
        }
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
        assert!(Arc::strong_count(&pipeline.pipeline) > 0);
    }
}
