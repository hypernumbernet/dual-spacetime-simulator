use crate::utils::Allocators;
use ahash::AHashMap;
use egui::{ClippedPrimitive, Rect, TexturesDelta, epaint::Primitive};
use std::sync::Arc;
use vulkano::{
    DeviceSize, NonZeroDeviceSize,
    buffer::{
        Buffer, BufferContents, BufferCreateInfo, BufferUsage, Subbuffer,
        allocator::{SubbufferAllocator, SubbufferAllocatorCreateInfo},
    },
    command_buffer::{
        AutoCommandBufferBuilder, BufferImageCopy, CommandBufferInheritanceInfo,
        CommandBufferUsage, CopyBufferToImageInfo, PrimaryAutoCommandBuffer,
        PrimaryCommandBufferAbstract, SecondaryAutoCommandBuffer,
    },
    descriptor_set::{DescriptorSet, WriteDescriptorSet, layout::DescriptorSetLayout},
    device::Queue,
    format::{Format, NumericFormat},
    image::{
        Image, ImageAspects, ImageCreateInfo, ImageLayout, ImageSubresourceLayers, ImageType,
        ImageUsage, SampleCount,
        sampler::{
            ComponentMapping, ComponentSwizzle, Filter, Sampler, SamplerAddressMode,
            SamplerCreateInfo, SamplerMipmapMode,
        },
        view::{ImageView, ImageViewCreateInfo},
    },
    memory::{
        DeviceAlignment,
        allocator::{AllocationCreateInfo, DeviceLayout, MemoryTypeFilter},
    },
    pipeline::{
        DynamicState, GraphicsPipeline, Pipeline, PipelineBindPoint, PipelineLayout,
        PipelineShaderStageCreateInfo,
        graphics::{
            GraphicsPipelineCreateInfo,
            color_blend::{
                AttachmentBlend, BlendFactor, ColorBlendAttachmentState, ColorBlendState,
            },
            depth_stencil::{CompareOp, DepthState, DepthStencilState},
            input_assembly::InputAssemblyState,
            multisample::MultisampleState,
            rasterization::RasterizationState,
            vertex_input::{Vertex, VertexDefinition},
            viewport::{Scissor, Viewport, ViewportState},
        },
        layout::PipelineDescriptorSetLayoutCreateInfo,
    },
    render_pass::{RenderPass, Subpass},
    sync::GpuFuture,
};

const VERTICES_PER_QUAD: DeviceSize = 4;
const VERTEX_BUFFER_SIZE: DeviceSize = 1024 * 1024 * VERTICES_PER_QUAD;
const INDEX_BUFFER_SIZE: DeviceSize = 1024 * 1024 * 2;

type IndexBuffer = Subbuffer<[u32]>;

#[repr(C)]
#[derive(BufferContents, Vertex)]
pub struct EguiVertex {
    #[format(R32G32_SFLOAT)]
    pub position: [f32; 2],
    #[format(R32G32_SFLOAT)]
    pub tex_coords: [f32; 2],
    #[format(R8G8B8A8_UNORM)]
    pub color: [u8; 4],
}

pub struct Renderer {
    gfx_queue: Arc<Queue>,
    render_pass: Option<Arc<RenderPass>>,
    output_in_linear_colorspace: bool,
    #[allow(unused)]
    format: vulkano::format::Format,
    font_sampler: Arc<Sampler>,
    font_format: Format,
    allocators: Allocators,
    vertex_index_buffer_pool: SubbufferAllocator,
    pipeline: Arc<GraphicsPipeline>,
    subpass: Subpass,
    texture_desc_sets: AHashMap<egui::TextureId, Arc<DescriptorSet>>,
    texture_images: AHashMap<egui::TextureId, Arc<ImageView>>,
}

impl Renderer {
    pub fn new_with_subpass(
        gfx_queue: Arc<Queue>,
        final_output_format: Format,
        subpass: Subpass,
    ) -> Renderer {
        Self::new_internal(gfx_queue, final_output_format, subpass, None)
    }

    fn new_internal(
        gfx_queue: Arc<Queue>,
        final_output_format: Format,
        subpass: Subpass,
        render_pass: Option<Arc<RenderPass>>,
    ) -> Renderer {
        let output_in_linear_colorspace =
            final_output_format.numeric_format_color().unwrap() == NumericFormat::SRGB;
        let allocators = Allocators::new_default(gfx_queue.device());
        let vertex_index_buffer_pool = SubbufferAllocator::new(
            allocators.memory.clone(),
            SubbufferAllocatorCreateInfo {
                arena_size: INDEX_BUFFER_SIZE + VERTEX_BUFFER_SIZE,
                buffer_usage: BufferUsage::INDEX_BUFFER | BufferUsage::VERTEX_BUFFER,
                memory_type_filter: MemoryTypeFilter::PREFER_DEVICE
                    | MemoryTypeFilter::HOST_SEQUENTIAL_WRITE,
                ..Default::default()
            },
        );
        let pipeline = Self::create_pipeline(gfx_queue.clone(), subpass.clone());
        let font_sampler = Sampler::new(
            gfx_queue.device().clone(),
            SamplerCreateInfo {
                mag_filter: Filter::Linear,
                min_filter: Filter::Linear,
                address_mode: [SamplerAddressMode::ClampToEdge; 3],
                mipmap_mode: SamplerMipmapMode::Linear,
                ..Default::default()
            },
        )
        .unwrap();
        let font_format = Self::choose_font_format(gfx_queue.device());
        Renderer {
            gfx_queue,
            format: final_output_format,
            render_pass,
            vertex_index_buffer_pool,
            pipeline,
            subpass,
            texture_desc_sets: AHashMap::default(),
            texture_images: AHashMap::default(),
            output_in_linear_colorspace,
            font_sampler,
            font_format,
            allocators,
        }
    }

    pub fn has_renderpass(&self) -> bool {
        self.render_pass.is_some()
    }

    fn create_pipeline(gfx_queue: Arc<Queue>, subpass: Subpass) -> Arc<GraphicsPipeline> {
        let vs = vs::load(gfx_queue.device().clone())
            .expect("failed to create shader module")
            .entry_point("main")
            .unwrap();
        let fs = fs::load(gfx_queue.device().clone())
            .expect("failed to create shader module")
            .entry_point("main")
            .unwrap();
        let mut blend = AttachmentBlend::alpha();
        blend.src_color_blend_factor = BlendFactor::One;
        blend.src_alpha_blend_factor = BlendFactor::OneMinusDstAlpha;
        blend.dst_alpha_blend_factor = BlendFactor::One;
        let blend_state = ColorBlendState {
            attachments: vec![ColorBlendAttachmentState {
                blend: Some(blend),
                ..Default::default()
            }],
            ..ColorBlendState::default()
        };
        let has_depth_buffer = subpass
            .subpass_desc()
            .depth_stencil_attachment
            .as_ref()
            .is_some_and(|depth_stencil_attachment| {
                subpass.render_pass().attachments()[depth_stencil_attachment.attachment as usize]
                    .format
                    .aspects()
                    .intersects(ImageAspects::DEPTH)
            });
        let depth_stencil_state = if has_depth_buffer {
            Some(DepthStencilState {
                depth: Some(DepthState {
                    write_enable: false,
                    compare_op: CompareOp::Always,
                }),
                ..Default::default()
            })
        } else {
            None
        };
        let vertex_input_state = Some(EguiVertex::per_vertex().definition(&vs).unwrap());
        let stages = [
            PipelineShaderStageCreateInfo::new(vs),
            PipelineShaderStageCreateInfo::new(fs),
        ];
        let layout = PipelineLayout::new(
            gfx_queue.device().clone(),
            PipelineDescriptorSetLayoutCreateInfo::from_stages(&stages)
                .into_pipeline_layout_create_info(gfx_queue.device().clone())
                .unwrap(),
        )
        .unwrap();
        GraphicsPipeline::new(
            gfx_queue.device().clone(),
            None,
            GraphicsPipelineCreateInfo {
                stages: stages.into_iter().collect(),
                vertex_input_state,
                input_assembly_state: Some(InputAssemblyState::default()),
                viewport_state: Some(ViewportState::default()),
                rasterization_state: Some(RasterizationState::default()),
                multisample_state: Some(MultisampleState {
                    rasterization_samples: subpass.num_samples().unwrap_or(SampleCount::Sample1),
                    ..Default::default()
                }),
                color_blend_state: Some(blend_state),
                depth_stencil_state,
                dynamic_state: [DynamicState::Viewport, DynamicState::Scissor]
                    .into_iter()
                    .collect(),
                subpass: Some(subpass.into()),
                ..GraphicsPipelineCreateInfo::layout(layout)
            },
        )
        .unwrap()
    }

    fn sampled_image_desc_set(
        &self,
        layout: &Arc<DescriptorSetLayout>,
        image: Arc<ImageView>,
        sampler: Arc<Sampler>,
    ) -> Arc<DescriptorSet> {
        DescriptorSet::new(
            self.allocators.descriptor_set.clone(),
            layout.clone(),
            [WriteDescriptorSet::image_view_sampler(0, image, sampler)],
            [],
        )
        .unwrap()
    }

    pub fn unregister_image(&mut self, texture_id: egui::TextureId) {
        self.texture_desc_sets.remove(&texture_id);
        self.texture_images.remove(&texture_id);
    }

    fn choose_font_format(device: &vulkano::device::Device) -> Format {
        let supports_swizzle = !device
            .physical_device()
            .supported_extensions()
            .khr_portability_subset
            || device
                .physical_device()
                .supported_features()
                .image_view_format_swizzle;
        let is_supported = |device: &vulkano::device::Device, format: Format| {
            device
                .physical_device()
                .image_format_properties(vulkano::image::ImageFormatInfo {
                    format,
                    usage: ImageUsage::SAMPLED
                        | ImageUsage::TRANSFER_DST
                        | ImageUsage::TRANSFER_SRC,
                    ..Default::default()
                })
                .is_ok_and(|properties| properties.is_some())
        };
        if supports_swizzle && is_supported(device, Format::R8G8_UNORM) {
            Format::R8G8_UNORM
        } else {
            Format::R8G8B8A8_SRGB
        }
    }

    fn pack_font_data_into(&self, data: &egui::FontImage, into: &mut [u8]) {
        match self.font_format {
            Format::R8G8_UNORM => {
                let linear = data
                    .pixels
                    .iter()
                    .map(|f| (f.clamp(0.0, 1.0 - f32::EPSILON) * 256.0) as u8);
                let bytes = linear
                    .zip(data.srgba_pixels(None))
                    .flat_map(|(linear, srgb)| [linear, srgb.a()]);
                into.iter_mut()
                    .zip(bytes)
                    .for_each(|(into, from)| *into = from);
            }
            Format::R8G8B8A8_SRGB => {
                let bytes = data.srgba_pixels(None).flat_map(|color| color.to_array());
                into.iter_mut()
                    .zip(bytes)
                    .for_each(|(into, from)| *into = from);
            }
            _ => unreachable!(),
        }
    }

    fn image_size_bytes(&self, delta: &egui::epaint::ImageDelta) -> usize {
        match &delta.image {
            egui::ImageData::Color(c) => c.width() * c.height() * 4,
            egui::ImageData::Font(f) => {
                f.width()
                    * f.height()
                    * match self.font_format {
                        Format::R8G8_UNORM => 2,
                        Format::R8G8B8A8_SRGB => 4,
                        _ => unreachable!(),
                    }
            }
        }
    }

    fn update_texture_within(
        &mut self,
        id: egui::TextureId,
        delta: &egui::epaint::ImageDelta,
        stage: Subbuffer<[u8]>,
        mapped_stage: &mut [u8],
        cbb: &mut AutoCommandBufferBuilder<PrimaryAutoCommandBuffer>,
    ) {
        let format = match &delta.image {
            egui::ImageData::Color(image) => {
                assert_eq!(
                    image.width() * image.height(),
                    image.pixels.len(),
                    "Mismatch between texture size and texel count"
                );
                let bytes = image.pixels.iter().flat_map(|color| color.to_array());
                mapped_stage
                    .iter_mut()
                    .zip(bytes)
                    .for_each(|(into, from)| *into = from);
                Format::R8G8B8A8_SRGB
            }
            egui::ImageData::Font(image) => {
                self.pack_font_data_into(image, mapped_stage);
                self.font_format
            }
        };
        if let Some(pos) = delta.pos {
            let Some(existing_image) = self.texture_images.get(&id) else {
                panic!("attempt to write into non-existing image");
            };
            assert_eq!(existing_image.format(), format);

            cbb.copy_buffer_to_image(CopyBufferToImageInfo {
                regions: [BufferImageCopy {
                    image_offset: [pos[0] as u32, pos[1] as u32, 0],
                    image_extent: [delta.image.width() as u32, delta.image.height() as u32, 1],
                    image_subresource: ImageSubresourceLayers {
                        aspects: ImageAspects::COLOR,
                        mip_level: 0,
                        array_layers: 0..1,
                    },
                    ..Default::default()
                }]
                .into(),
                ..CopyBufferToImageInfo::buffer_image(stage, existing_image.image().clone())
            })
            .unwrap();
        } else {
            let img = {
                let extent = [delta.image.width() as u32, delta.image.height() as u32, 1];
                Image::new(
                    self.allocators.memory.clone(),
                    ImageCreateInfo {
                        image_type: ImageType::Dim2d,
                        format,
                        extent,
                        usage: ImageUsage::TRANSFER_DST | ImageUsage::SAMPLED,
                        initial_layout: ImageLayout::Undefined,
                        ..Default::default()
                    },
                    AllocationCreateInfo::default(),
                )
                .unwrap()
            };
            cbb.copy_buffer_to_image(CopyBufferToImageInfo::buffer_image(stage, img.clone()))
                .unwrap();
            let component_mapping = match format {
                Format::R8G8_UNORM => ComponentMapping {
                    r: ComponentSwizzle::Red,
                    g: ComponentSwizzle::Red,
                    b: ComponentSwizzle::Red,
                    a: ComponentSwizzle::Green,
                },
                _ => ComponentMapping::identity(),
            };
            let view = ImageView::new(
                img.clone(),
                ImageViewCreateInfo {
                    component_mapping,
                    ..ImageViewCreateInfo::from_image(&img)
                },
            )
            .unwrap();
            let layout = self.pipeline.layout().set_layouts().first().unwrap();
            let desc_set =
                self.sampled_image_desc_set(layout, view.clone(), self.font_sampler.clone());
            self.texture_desc_sets.insert(id, desc_set);
            self.texture_images.insert(id, view);
        };
    }

    fn update_textures(&mut self, sets: &[(egui::TextureId, egui::epaint::ImageDelta)]) {
        let total_size_bytes = sets
            .iter()
            .map(|(_, set)| self.image_size_bytes(set))
            .sum::<usize>()
            * 4;
        let total_size_bytes = u64::try_from(total_size_bytes).unwrap();
        let Ok(total_size_bytes) = vulkano::NonZeroDeviceSize::try_from(total_size_bytes) else {
            return;
        };
        let buffer = Buffer::new(
            self.allocators.memory.clone(),
            BufferCreateInfo {
                usage: BufferUsage::TRANSFER_SRC,
                ..Default::default()
            },
            AllocationCreateInfo {
                memory_type_filter: MemoryTypeFilter::PREFER_DEVICE
                    | MemoryTypeFilter::HOST_SEQUENTIAL_WRITE,
                ..Default::default()
            },
            DeviceLayout::new(total_size_bytes, DeviceAlignment::MIN).unwrap(),
        )
        .unwrap();
        let buffer = Subbuffer::new(buffer);
        let mut cbb = AutoCommandBufferBuilder::primary(
            self.allocators.command_buffer.clone(),
            self.gfx_queue.queue_family_index(),
            CommandBufferUsage::OneTimeSubmit,
        )
        .unwrap();
        {
            let mut writer = buffer.write().unwrap();

            let mut past_buffer_end = 0usize;

            for (id, delta) in sets {
                let image_size_bytes = self.image_size_bytes(delta);
                let range = past_buffer_end..(image_size_bytes + past_buffer_end);

                past_buffer_end += image_size_bytes;

                let stage = buffer.clone().slice(range.start as u64..range.end as u64);
                let mapped_stage = &mut writer[range];

                self.update_texture_within(*id, delta, stage, mapped_stage, &mut cbb);
            }
        }
        let command_buffer = cbb.build().unwrap();
        command_buffer
            .execute(self.gfx_queue.clone())
            .unwrap()
            .then_signal_fence_and_flush()
            .unwrap()
            .wait(None)
            .unwrap();
    }

    fn get_rect_scissor(
        &self,
        scale_factor: f32,
        framebuffer_dimensions: [u32; 2],
        rect: Rect,
    ) -> Scissor {
        let min = rect.min;
        let min = egui::Pos2 {
            x: min.x * scale_factor,
            y: min.y * scale_factor,
        };
        let min = egui::Pos2 {
            x: min.x.clamp(0.0, framebuffer_dimensions[0] as f32),
            y: min.y.clamp(0.0, framebuffer_dimensions[1] as f32),
        };
        let max = rect.max;
        let max = egui::Pos2 {
            x: max.x * scale_factor,
            y: max.y * scale_factor,
        };
        let max = egui::Pos2 {
            x: max.x.clamp(min.x, framebuffer_dimensions[0] as f32),
            y: max.y.clamp(min.y, framebuffer_dimensions[1] as f32),
        };
        Scissor {
            offset: [min.x.round() as u32, min.y.round() as u32],
            extent: [
                (max.x.round() - min.x) as u32,
                (max.y.round() - min.y) as u32,
            ],
        }
    }

    fn create_secondary_command_buffer_builder(
        &self,
    ) -> AutoCommandBufferBuilder<SecondaryAutoCommandBuffer> {
        AutoCommandBufferBuilder::secondary(
            self.allocators.command_buffer.clone(),
            self.gfx_queue.queue_family_index(),
            CommandBufferUsage::MultipleSubmit,
            CommandBufferInheritanceInfo {
                render_pass: Some(self.subpass.clone().into()),
                ..Default::default()
            },
        )
        .unwrap()
    }

    pub fn draw_on_subpass_image(
        &mut self,
        clipped_meshes: &[ClippedPrimitive],
        textures_delta: &TexturesDelta,
        scale_factor: f32,
        framebuffer_dimensions: [u32; 2],
    ) -> Arc<SecondaryAutoCommandBuffer> {
        self.update_textures(&textures_delta.set);
        let mut builder = self.create_secondary_command_buffer_builder();
        self.draw_egui(
            scale_factor,
            clipped_meshes,
            framebuffer_dimensions,
            &mut builder,
        );
        let buffer = builder.build().unwrap();
        for &id in &textures_delta.free {
            self.unregister_image(id);
        }
        buffer
    }

    fn upload_meshes(
        &mut self,
        clipped_meshes: &[ClippedPrimitive],
    ) -> Option<(Subbuffer<[EguiVertex]>, IndexBuffer)> {
        type Index = u32;
        const VERTEX_ALIGN: DeviceAlignment = DeviceAlignment::of::<EguiVertex>();
        const INDEX_ALIGN: DeviceAlignment = DeviceAlignment::of::<Index>();
        let meshes = clipped_meshes
            .iter()
            .filter_map(|mesh| match &mesh.primitive {
                Primitive::Mesh(m) => Some(m),
                _ => None,
            });
        let (total_vertices, total_size_bytes) = {
            let mut total_vertices = 0;
            let mut total_indices = 0;

            for mesh in meshes.clone() {
                total_vertices += mesh.vertices.len();
                total_indices += mesh.indices.len();
            }
            if total_indices == 0 || total_vertices == 0 {
                return None;
            }

            let total_size_bytes = total_vertices * std::mem::size_of::<EguiVertex>()
                + total_indices * std::mem::size_of::<Index>();
            (
                total_vertices,
                NonZeroDeviceSize::new(u64::try_from(total_size_bytes).unwrap()).unwrap(),
            )
        };
        let layout = DeviceLayout::new(total_size_bytes, VERTEX_ALIGN.max(INDEX_ALIGN)).unwrap();
        let buffer = self.vertex_index_buffer_pool.allocate(layout).unwrap();
        assert!(VERTEX_ALIGN >= INDEX_ALIGN);
        let (vertices, indices) = {
            let partition_bytes = total_vertices as u64 * std::mem::size_of::<EguiVertex>() as u64;
            (
                buffer
                    .clone()
                    .slice(..partition_bytes)
                    .reinterpret::<[EguiVertex]>(),
                buffer.slice(partition_bytes..).reinterpret::<[Index]>(),
            )
        };
        {
            let mut vertex_write = vertices.write().unwrap();
            vertex_write
                .iter_mut()
                .zip(
                    meshes
                        .clone()
                        .flat_map(|m| &m.vertices)
                        .copied()
                        .map(|from| EguiVertex {
                            position: from.pos.into(),
                            tex_coords: from.uv.into(),
                            color: from.color.to_array(),
                        }),
                )
                .for_each(|(into, from)| *into = from);
        }
        {
            let mut index_write = indices.write().unwrap();
            index_write
                .iter_mut()
                .zip(meshes.flat_map(|m| &m.indices).copied())
                .for_each(|(into, from)| *into = from);
        }
        Some((vertices, indices))
    }

    fn draw_egui(
        &mut self,
        scale_factor: f32,
        clipped_meshes: &[ClippedPrimitive],
        framebuffer_dimensions: [u32; 2],
        builder: &mut AutoCommandBufferBuilder<SecondaryAutoCommandBuffer>,
    ) {
        let push_constants = vs::PushConstants {
            screen_size: [
                framebuffer_dimensions[0] as f32 / scale_factor,
                framebuffer_dimensions[1] as f32 / scale_factor,
            ],
            output_in_linear_colorspace: self.output_in_linear_colorspace.into(),
        };
        let mesh_buffers = self.upload_meshes(clipped_meshes);
        let mut vertex_cursor = 0;
        let mut index_cursor = 0;
        let mut needs_full_rebind = true;
        let mut current_rect = None;
        let mut current_texture = None;
        for ClippedPrimitive {
            clip_rect,
            primitive,
        } in clipped_meshes
        {
            match primitive {
                Primitive::Mesh(mesh) => {
                    if mesh.vertices.is_empty() || mesh.indices.is_empty() {
                        index_cursor += mesh.indices.len() as u32;
                        vertex_cursor += mesh.vertices.len() as u32;
                        continue;
                    }
                    if needs_full_rebind {
                        needs_full_rebind = false;

                        let Some((vertices, indices)) = mesh_buffers.clone() else {
                            unreachable!()
                        };
                        builder
                            .bind_pipeline_graphics(self.pipeline.clone())
                            .unwrap()
                            .bind_index_buffer(indices)
                            .unwrap()
                            .bind_vertex_buffers(0, [vertices])
                            .unwrap()
                            .set_viewport(
                                0,
                                [Viewport {
                                    offset: [0.0, 0.0],
                                    extent: [
                                        framebuffer_dimensions[0] as f32,
                                        framebuffer_dimensions[1] as f32,
                                    ],
                                    depth_range: 0.0..=1.0,
                                }]
                                .into_iter()
                                .collect(),
                            )
                            .unwrap()
                            .push_constants(self.pipeline.layout().clone(), 0, push_constants)
                            .unwrap();
                    }
                    if current_texture != Some(mesh.texture_id) {
                        if self.texture_desc_sets.get(&mesh.texture_id).is_none() {
                            eprintln!("This texture no longer exists {:?}", mesh.texture_id);
                            continue;
                        }
                        current_texture = Some(mesh.texture_id);
                        let desc_set = self.texture_desc_sets.get(&mesh.texture_id).unwrap();
                        builder
                            .bind_descriptor_sets(
                                PipelineBindPoint::Graphics,
                                self.pipeline.layout().clone(),
                                0,
                                desc_set.clone(),
                            )
                            .unwrap();
                    };
                    if current_rect != Some(*clip_rect) {
                        current_rect = Some(*clip_rect);
                        let new_scissor =
                            self.get_rect_scissor(scale_factor, framebuffer_dimensions, *clip_rect);
                        builder
                            .set_scissor(0, [new_scissor].into_iter().collect())
                            .unwrap();
                    }
                    unsafe {
                        builder
                            .draw_indexed(
                                mesh.indices.len() as u32,
                                1,
                                index_cursor,
                                vertex_cursor as i32,
                                0,
                            )
                            .unwrap();
                    }
                    index_cursor += mesh.indices.len() as u32;
                    vertex_cursor += mesh.vertices.len() as u32;
                }
                Primitive::Callback(_) => {}
            }
        }
    }

    pub fn queue(&self) -> Arc<Queue> {
        self.gfx_queue.clone()
    }
}

mod vs {
    vulkano_shaders::shader! {
        ty: "vertex",
        src: "
#version 450

layout(location = 0) in vec2 position;
layout(location = 1) in vec2 tex_coords;
layout(location = 2) in vec4 color;

layout(location = 0) out vec4 v_color;
layout(location = 1) out vec2 v_tex_coords;

layout(push_constant) uniform PushConstants {
    vec2 screen_size;
    int output_in_linear_colorspace;
} push_constants;

void main() {
    gl_Position = vec4(
        2.0 * position.x / push_constants.screen_size.x - 1.0,
        2.0 * position.y / push_constants.screen_size.y - 1.0,
        0.0, 1.0
    );
    v_color = color;
    v_tex_coords = tex_coords;
}"
    }
}

mod fs {
    vulkano_shaders::shader! {
        ty: "fragment",
        src: "
#version 450

layout(location = 0) in vec4 v_color;
layout(location = 1) in vec2 v_tex_coords;

layout(location = 0) out vec4 f_color;

layout(binding = 0, set = 0) uniform sampler2D font_texture;

layout(push_constant) uniform PushConstants {
    vec2 screen_size;
    int output_in_linear_colorspace;
} push_constants;

vec3 srgb_from_linear(vec3 linear) {
    bvec3 cutoff = lessThan(linear, vec3(0.0031308));
    vec3 lower = linear * vec3(12.92);
    vec3 higher = vec3(1.055) * pow(linear, vec3(1./2.4)) - vec3(0.055);
    return mix(higher, lower, vec3(cutoff));
}

vec4 srgba_from_linear(vec4 linear) {
    return vec4(srgb_from_linear(linear.rgb), linear.a);
}

vec3 linear_from_srgb(vec3 srgb) {
    bvec3 cutoff = lessThan(srgb, vec3(0.04045));
    vec3 lower = srgb / vec3(12.92);
    vec3 higher = pow((srgb + vec3(0.055) / vec3(1.055)), vec3(2.4));
    return mix(higher, lower, vec3(cutoff));
}

vec4 linear_from_srgba(vec4 srgb) {
    return vec4(linear_from_srgb(srgb.rgb), srgb.a);
}

void main() {
    vec4 texture_color = srgba_from_linear(texture(font_texture, v_tex_coords));
    vec4 color = v_color * texture_color;

    if (push_constants.output_in_linear_colorspace == 1) {
        color = linear_from_srgba(color);
    }
    f_color = color;
}"
    }
}
