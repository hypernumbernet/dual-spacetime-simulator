use ash::vk;
use egui::ClippedPrimitive;
use egui_winit::winit::event_loop::ActiveEventLoop;
use winit::window::Window;

pub struct Gui {
    pub egui_ctx: egui::Context,
    pub egui_winit: egui_winit::State,
    renderer: egui_ash_renderer::Renderer,
    shapes: Vec<egui::epaint::ClippedShape>,
    textures_delta: egui::TexturesDelta,
    prepared_meshes: Vec<ClippedPrimitive>,
    prepared_textures_free: Vec<egui::TextureId>,
    pixels_per_point: f32,
    queue: vk::Queue,
    command_pool: vk::CommandPool,
}

impl Gui {
    /// Creates GUI integration state and initializes egui renderer resources for Vulkan.
    pub fn new(
        event_loop: &ActiveEventLoop,
        window: &Window,
        instance: &ash::Instance,
        physical_device: vk::PhysicalDevice,
        device: ash::Device,
        queue: vk::Queue,
        command_pool: vk::CommandPool,
        render_pass: vk::RenderPass,
        swapchain_format: vk::Format,
    ) -> Self {
        let is_srgb = matches!(
            swapchain_format,
            vk::Format::B8G8R8A8_SRGB
                | vk::Format::R8G8B8A8_SRGB
                | vk::Format::A8B8G8R8_SRGB_PACK32
        );
        let renderer = egui_ash_renderer::Renderer::with_default_allocator(
            instance,
            physical_device,
            device,
            render_pass,
            egui_ash_renderer::Options {
                srgb_framebuffer: is_srgb,
                enable_depth_test: false,
                enable_depth_write: false,
                in_flight_frames: 2,
            },
        )
        .expect("Failed to create egui-ash-renderer");

        let egui_ctx: egui::Context = Default::default();
        let theme = match egui_ctx.theme() {
            egui::Theme::Dark => winit::window::Theme::Dark,
            egui::Theme::Light => winit::window::Theme::Light,
        };
        let max_texture_side = 8192;

        let egui_winit = egui_winit::State::new(
            egui_ctx.clone(),
            egui_ctx.viewport_id(),
            event_loop,
            Some(window.scale_factor() as f32),
            Some(theme),
            Some(max_texture_side),
        );

        let pixels_per_point = window.scale_factor() as f32;

        Gui {
            egui_ctx,
            egui_winit,
            renderer,
            shapes: vec![],
            textures_delta: Default::default(),
            prepared_meshes: vec![],
            prepared_textures_free: vec![],
            pixels_per_point,
            queue,
            command_pool,
        }
    }

    /// Forwards a window event to egui and reports whether egui consumed it.
    pub fn update(&mut self, window: &Window, winit_event: &winit::event::WindowEvent) -> bool {
        self.egui_winit
            .on_window_event(window, winit_event)
            .consumed
    }

    /// Reports whether pointer interactions should be captured by the GUI layer.
    pub fn pointer_wants_input(&self) -> bool {
        self.egui_ctx.wants_pointer_input() || self.egui_ctx.is_pointer_over_area()
    }

    /// Begins an egui pass, runs immediate UI code, and keeps outputs for rendering.
    pub fn immediate_ui(&mut self, window: &Window, layout_function: impl FnOnce(&mut Self)) {
        let raw_input = self.egui_winit.take_egui_input(window);
        self.egui_ctx.begin_pass(raw_input);
        layout_function(self);
    }

    /// Returns a clone of the egui context for external UI construction.
    pub fn context(&self) -> egui::Context {
        self.egui_ctx.clone()
    }

    /// Reports whether egui requested another repaint after the last pass.
    pub fn wants_repaint(&self) -> bool {
        self.egui_ctx.has_requested_repaint()
    }

    /// Finalizes the current UI pass and prepares tessellated meshes and textures.
    pub fn prepare_frame(&mut self, window: &Window) {
        self.end_frame(window);
        let shapes = std::mem::take(&mut self.shapes);
        let textures_delta = std::mem::take(&mut self.textures_delta);

        self.pixels_per_point = egui_winit::pixels_per_point(&self.egui_ctx, window);
        self.prepared_meshes = self.egui_ctx.tessellate(shapes, self.pixels_per_point);
        self.prepared_textures_free = textures_delta.free.clone();

        if !textures_delta.set.is_empty() {
            self.renderer
                .set_textures(self.queue, self.command_pool, &textures_delta.set)
                .expect("Failed to set egui textures");
        }
    }

    /// Records draw commands for prepared egui meshes into the command buffer.
    pub fn draw(&mut self, command_buffer: vk::CommandBuffer, extent: vk::Extent2D) {
        self.renderer
            .cmd_draw(
                command_buffer,
                extent,
                self.pixels_per_point,
                &self.prepared_meshes,
            )
            .expect("Failed to record egui draw commands");
    }

    /// Releases textures queued for deletion after draw submission completes.
    pub fn finish_frame(&mut self) {
        let free = std::mem::take(&mut self.prepared_textures_free);
        if !free.is_empty() {
            self.renderer
                .free_textures(&free)
                .expect("Failed to free egui textures");
        }
    }

    /// Ends the egui pass and stores generated shapes and texture deltas.
    fn end_frame(&mut self, window: &Window) {
        let egui::FullOutput {
            platform_output,
            textures_delta,
            shapes,
            pixels_per_point: _,
            viewport_output: _,
        } = self.egui_ctx.end_pass();
        self.egui_winit
            .handle_platform_output(window, platform_output);
        self.shapes = shapes;
        self.textures_delta = textures_delta;
    }
}
