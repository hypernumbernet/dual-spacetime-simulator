use crate::renderer::Renderer;
use egui::{ClippedPrimitive, TexturesDelta};
use egui_winit::winit::event_loop::ActiveEventLoop;
use std::sync::Arc;
use vulkano::{
    command_buffer::SecondaryAutoCommandBuffer,
    device::Queue,
    format::{Format, NumericFormat},
    render_pass::Subpass,
    swapchain::Surface,
};
use winit::window::Window;

pub struct GuiConfig {
    pub allow_srgb_render_target: bool,
}

impl Default for GuiConfig {
    fn default() -> Self {
        GuiConfig {
            allow_srgb_render_target: false,
        }
    }
}

impl GuiConfig {
    pub fn validate(&self, output_format: Format) {
        if output_format.numeric_format_color().unwrap() == NumericFormat::SRGB {
            assert!(
                self.allow_srgb_render_target,
                "Using an output format with sRGB requires `GuiConfig::allow_srgb_render_target` \
                 to be set! Egui prefers UNORM render targets. Using sRGB will cause minor \
                 discoloration of UI elements due to blending in linear color space and not sRGB \
                 as Egui expects."
            );
        }
    }
}

pub struct Gui {
    pub egui_ctx: egui::Context,
    pub egui_winit: egui_winit::State,
    renderer: Renderer,
    surface: Arc<Surface>,
    shapes: Vec<egui::epaint::ClippedShape>,
    textures_delta: egui::TexturesDelta,
}

impl Gui {
    pub fn new_with_subpass(
        event_loop: &ActiveEventLoop,
        surface: Arc<Surface>,
        gfx_queue: Arc<Queue>,
        subpass: Subpass,
        output_format: Format,
        config: GuiConfig,
    ) -> Gui {
        config.validate(output_format);
        let renderer = Renderer::new_with_subpass(gfx_queue, output_format, subpass);
        Self::new_internal(event_loop, surface, renderer)
    }

    fn new_internal(
        event_loop: &ActiveEventLoop,
        surface: Arc<Surface>,
        renderer: Renderer,
    ) -> Gui {
        let max_texture_side = renderer
            .queue()
            .device()
            .physical_device()
            .properties()
            .max_image_dimension2_d as usize;
        let egui_ctx: egui_winit::egui::Context = Default::default();
        let theme = match egui_ctx.theme() {
            egui_winit::egui::Theme::Dark => winit::window::Theme::Dark,
            egui_winit::egui::Theme::Light => winit::window::Theme::Light,
        };
        let egui_winit = egui_winit::State::new(
            egui_ctx.clone(),
            egui_ctx.viewport_id(),
            event_loop,
            Some(surface_window(&surface).scale_factor() as f32),
            Some(theme),
            Some(max_texture_side),
        );
        Gui {
            egui_ctx,
            egui_winit,
            renderer,
            surface,
            shapes: vec![],
            textures_delta: Default::default(),
        }
    }

    fn pixels_per_point(&self) -> f32 {
        egui_winit::pixels_per_point(&self.egui_ctx, surface_window(&self.surface))
    }

    pub fn update(&mut self, winit_event: &winit::event::WindowEvent) -> bool {
        self.egui_winit
            .on_window_event(surface_window(&self.surface), winit_event)
            .consumed
    }

    pub fn immediate_ui(&mut self, layout_function: impl FnOnce(&mut Self)) {
        let raw_input = self
            .egui_winit
            .take_egui_input(surface_window(&self.surface));
        self.egui_ctx.begin_pass(raw_input);
        layout_function(self);
    }

    pub fn draw_on_subpass_image(
        &mut self,
        image_dimensions: [u32; 2],
    ) -> Arc<SecondaryAutoCommandBuffer> {
        if self.renderer.has_renderpass() {
            panic!(
                "Gui integration has been created with its own render pass, use `draw_on_image` \
                 instead"
            )
        }
        let (clipped_meshes, textures_delta) = self.extract_draw_data_at_frame_end();
        self.renderer.draw_on_subpass_image(
            &clipped_meshes,
            &textures_delta,
            self.pixels_per_point(),
            image_dimensions,
        )
    }

    fn extract_draw_data_at_frame_end(&mut self) -> (Vec<ClippedPrimitive>, TexturesDelta) {
        self.end_frame();
        let shapes = std::mem::take(&mut self.shapes);
        let textures_delta = std::mem::take(&mut self.textures_delta);
        let clipped_meshes = self.egui_ctx.tessellate(shapes, self.pixels_per_point());
        (clipped_meshes, textures_delta)
    }

    fn end_frame(&mut self) {
        let egui::FullOutput {
            platform_output,
            textures_delta,
            shapes,
            pixels_per_point: _,
            viewport_output: _,
        } = self.egui_ctx.end_pass();
        self.egui_winit
            .handle_platform_output(surface_window(&self.surface), platform_output);
        self.shapes = shapes;
        self.textures_delta = textures_delta;
    }

    pub fn context(&self) -> egui::Context {
        self.egui_ctx.clone()
    }
}

fn surface_window(surface: &Surface) -> &Window {
    surface.object().unwrap().downcast_ref::<Window>().unwrap()
}
