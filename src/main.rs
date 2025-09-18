//Hide the console window on Windows in release mode
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use crate::render::ParticleRenderPipeline;
use crate::types::UiState;
use crate::ui::draw_ui;
use egui_winit_vulkano::{Gui, GuiConfig};
use vulkano_util::{
    context::{VulkanoConfig, VulkanoContext},
    window::{VulkanoWindows, WindowDescriptor},
};
use winit::{
    application::ApplicationHandler,
    error::EventLoopError,
    event::WindowEvent,
    event_loop::{ActiveEventLoop, EventLoop},
};

mod render;
mod types;
mod ui;
mod ui_styles;

pub fn main() -> Result<(), EventLoopError> {
    let event_loop = EventLoop::new().unwrap();
    let mut app = App::default();
    event_loop.run_app(&mut app)
}

fn generate_window_title() -> String {
    let package_name = env!("CARGO_PKG_NAME");
    let package_version = env!("CARGO_PKG_VERSION");
    format!("{} v{}", package_name, package_version)
}

pub struct App {
    context: VulkanoContext,
    windows: VulkanoWindows,
    render_pipeline: Option<ParticleRenderPipeline>,
    gui: Option<Gui>,
    ui_state: UiState,
}

impl Default for App {
    fn default() -> Self {
        let context = VulkanoContext::new(VulkanoConfig::default());
        let windows = VulkanoWindows::default();
        Self {
            context,
            windows,
            render_pipeline: None,
            gui: None,
            ui_state: UiState::default(),
        }
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let descriptor = WindowDescriptor {
            title: generate_window_title(),
            ..WindowDescriptor::default()
        };
        self.windows
            .create_window(event_loop, &self.context, &descriptor, |ci| {
                ci.image_format = vulkano::format::Format::B8G8R8A8_UNORM;
                ci.min_image_count = ci.min_image_count.max(2);
            });
        let render_pipeline = ParticleRenderPipeline::new(
            self.context.graphics_queue().clone(),
            self.windows
                .get_primary_renderer_mut()
                .unwrap()
                .swapchain_format(),
            self.context.memory_allocator(),
        );
        self.gui = Some(Gui::new_with_subpass(
            event_loop,
            self.windows.get_primary_renderer_mut().unwrap().surface(),
            self.windows
                .get_primary_renderer_mut()
                .unwrap()
                .graphics_queue(),
            render_pipeline.gui_pass(),
            self.windows
                .get_primary_renderer_mut()
                .unwrap()
                .swapchain_format(),
            GuiConfig::default(),
        ));
        self.render_pipeline = Some(render_pipeline);
    }

    fn window_event(
        &mut self,
        event_loop: &winit::event_loop::ActiveEventLoop,
        window_id: winit::window::WindowId,
        event: WindowEvent,
    ) {
        let renderer = self.windows.get_renderer_mut(window_id).unwrap();
        let gui = self.gui.as_mut().unwrap();
        match event {
            WindowEvent::Resized(_) => {
                renderer.resize();
            }
            WindowEvent::ScaleFactorChanged { .. } => {
                renderer.resize();
            }
            WindowEvent::CloseRequested => {
                event_loop.exit();
            }
            WindowEvent::RedrawRequested => {
                gui.immediate_ui(|gui| {
                    let ctx = gui.context();
                    draw_ui(&mut self.ui_state, &ctx);
                });
                match renderer.acquire(None, |_| {}) {
                    Ok(future) => {
                        let after_future = self.render_pipeline.as_mut().unwrap().render(
                            future,
                            renderer.swapchain_image_view(),
                            gui,
                        );
                        renderer.present(after_future, true);
                    }
                    Err(vulkano::VulkanError::OutOfDate) => {
                        renderer.resize();
                    }
                    Err(e) => panic!("Failed to acquire swapchain future: {}", e),
                };
            }
            _ => (),
        }
        if window_id == renderer.window().id() {
            let _pass_events_to_game = !gui.update(&event);
        }
    }

    fn about_to_wait(&mut self, _event_loop: &winit::event_loop::ActiveEventLoop) {
        let renderer = self.windows.get_primary_renderer().unwrap();
        renderer.window().request_redraw();
    }
}
