//Hide the console window on Windows in release mode
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use crate::integration::{Gui, GuiConfig};
use crate::pipeline::ParticleRenderPipeline;
use crate::simulation::SimulationState;
use crate::types::UiState;
use crate::ui::draw_ui;
use vulkano_util::{
    context::{VulkanoConfig, VulkanoContext},
    window::{VulkanoWindows, WindowDescriptor},
};
use winit::{
    application::ApplicationHandler,
    error::EventLoopError,
    event::{MouseButton, WindowEvent},
    event_loop::{ActiveEventLoop, EventLoop},
};

mod camera;
mod integration;
mod pipeline;
mod renderer;
mod simulation;
mod types;
mod ui;
mod ui_styles;
mod utils;

pub fn main() -> Result<(), EventLoopError> {
    let event_loop = EventLoop::new()?;
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
    simulation_state: SimulationState,
    mouse_left_down: bool,
    mouse_right_down: bool,
    last_cursor_position: Option<(f64, f64)>,
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
            simulation_state: SimulationState::default(),
            mouse_left_down: false,
            mouse_right_down: false,
            last_cursor_position: None,
        }
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let resize_constraints = vulkano_util::window::WindowResizeConstraints {
            min_width: self.ui_state.min_window_width,
            min_height: self.ui_state.min_window_height,
            ..Default::default()
        };
        let descriptor = WindowDescriptor {
            title: generate_window_title(),
            resize_constraints,
            ..Default::default()
        };
        self.windows
            .create_window(event_loop, &self.context, &descriptor, |ci| {
                ci.image_format = vulkano::format::Format::B8G8R8A8_UNORM;
                ci.min_image_count = ci.min_image_count.max(2);
            });
        let primary_renderer = self.windows.get_primary_renderer().unwrap();
        let render_pipeline = ParticleRenderPipeline::new(
            self.context.graphics_queue().clone(),
            primary_renderer.swapchain_format(),
            self.context.memory_allocator(),
        );
        self.gui = Some(Gui::new_with_subpass(
            event_loop,
            primary_renderer.surface(),
            primary_renderer.graphics_queue(),
            render_pipeline.gui_pass(),
            primary_renderer.swapchain_format(),
            GuiConfig::default(),
        ));
        self.render_pipeline = Some(render_pipeline);
        self.simulation_state = SimulationState::new(self.ui_state.particle_count);
    }

    fn window_event(
        &mut self,
        event_loop: &winit::event_loop::ActiveEventLoop,
        window_id: winit::window::WindowId,
        event: WindowEvent,
    ) {
        let renderer = self.windows.get_renderer_mut(window_id).unwrap();
        let gui = self.gui.as_mut().unwrap();
        let Some(pipeline) = self.render_pipeline.as_mut() else {
            return;
        };
        match &event {
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
                        pipeline.set_particles(&self.simulation_state.particles);
                        let after_future = pipeline.render(
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
            if !gui.update(&event) {
                match &event {
                    WindowEvent::MouseInput { state, button, .. } => match button {
                        MouseButton::Left => {
                            self.mouse_left_down = *state == winit::event::ElementState::Pressed;
                            if !self.mouse_left_down {
                                self.last_cursor_position = None;
                            }
                        }
                        MouseButton::Right => {
                            self.mouse_right_down = *state == winit::event::ElementState::Pressed;
                            if !self.mouse_right_down {
                                self.last_cursor_position = None;
                            }
                        }
                        _ => {}
                    },
                    WindowEvent::CursorMoved { position, .. } => {
                        if self.mouse_left_down {
                            let (x, y) = (position.x, position.y);
                            if let Some((lx, ly)) = self.last_cursor_position {
                                pipeline.rotate_camera(x - lx, y - ly);
                            }
                            self.last_cursor_position = Some((x, y));
                        }
                        if self.mouse_right_down {
                            let (x, y) = (position.x, position.y);
                            if let Some((lx, ly)) = self.last_cursor_position {
                                pipeline.look_around(x - lx, y - ly);
                            }
                            self.last_cursor_position = Some((x, y));
                        }
                    }
                    _ => {}
                }
            }
        }
    }

    fn about_to_wait(&mut self, _event_loop: &winit::event_loop::ActiveEventLoop) {
        let renderer = self.windows.get_primary_renderer().unwrap();
        renderer.window().request_redraw();
    }
}
