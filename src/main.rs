//Hide the console window on Windows in release mode
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use crate::integration::{Gui, GuiConfig};
use crate::pipeline::ParticleRenderPipeline;
use crate::simulation::SimulationState;
use crate::types::UiState;
use crate::ui::draw_ui;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};
use vulkano_util::{
    context::{VulkanoConfig, VulkanoContext},
    window::{VulkanoWindows, WindowDescriptor},
};
use winit::{
    application::ApplicationHandler,
    dpi::PhysicalPosition,
    error::EventLoopError,
    event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent},
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

const DOUBLE_CLICK_MILLIS: u64 = 400;
const DOUBLE_CLICK_DIST: f64 = 25.0;

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
    simulation_state: Arc<RwLock<SimulationState>>,
    thread_pool: rayon::ThreadPool,
    mouse_left_down: bool,
    mouse_right_down: bool,
    mouse_middle_down: bool,
    last_cursor_position: Option<(f64, f64)>,
    last_left_click_time: Option<Instant>,
    last_left_click_pos: Option<(f64, f64)>,
    last_right_click_time: Option<Instant>,
    last_right_click_pos: Option<(f64, f64)>,
    last_advance: std::time::Instant,
}

impl Default for App {
    fn default() -> Self {
        let context = VulkanoContext::new(VulkanoConfig::default());
        let windows = VulkanoWindows::default();
        let thread_pool = rayon::ThreadPoolBuilder::new()
            .num_threads(num_cpus::get())
            .build()
            .expect("Failed to build Rayon thread pool");
        Self {
            context,
            windows,
            render_pipeline: None,
            gui: None,
            ui_state: UiState::default(),
            simulation_state: Arc::new(RwLock::new(SimulationState::default())),
            thread_pool,
            mouse_left_down: false,
            mouse_right_down: false,
            mouse_middle_down: false,
            last_cursor_position: None,
            last_left_click_time: None,
            last_left_click_pos: None,
            last_right_click_time: None,
            last_right_click_pos: None,
            last_advance: Instant::now(),
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
        *self.simulation_state.write().unwrap() = SimulationState::new(self.ui_state.particle_count);
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
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
                        let sim = self.simulation_state.read().unwrap();
                        pipeline.set_particles(&sim.particles);
                        let after_future =
                            pipeline.render(future, renderer.swapchain_image_view(), gui);
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
                        MouseButton::Left => self.left_button(state),
                        MouseButton::Right => self.right_button(state),
                        MouseButton::Middle => self.middle_button(state),
                        _ => {}
                    },
                    WindowEvent::CursorMoved { position, .. } => {
                        let (x, y) = (position.x, position.y);
                        if let Some((lx, ly)) = self.last_cursor_position {
                            if self.mouse_left_down {
                                pipeline.revolve_camera(x - lx, y - ly);
                            }
                            if self.mouse_right_down {
                                pipeline.look_around(x - lx, y - ly);
                            }
                            if self.mouse_middle_down {
                                let window_size = renderer.window().inner_size();
                                let center_x = window_size.width as f64 / 2.0;
                                let center_y = window_size.height as f64 / 2.0;
                                pipeline.rotate_camera(x, lx, y, ly, center_x, center_y);
                            }
                        }
                        self.last_cursor_position = Some((x, y));
                    }
                    WindowEvent::MouseWheel { delta, .. } => match delta {
                        MouseScrollDelta::LineDelta(_, y) => {
                            let zoom_factor = y * 0.1;
                            pipeline.zoom_camera(zoom_factor);
                        }
                        MouseScrollDelta::PixelDelta(PhysicalPosition { y, .. }) => {
                            let zoom_factor = y * 0.1;
                            pipeline.zoom_camera(zoom_factor as f32);
                        }
                    },
                    _ => {}
                }
            }
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        let renderer = self.windows.get_primary_renderer().unwrap();
        renderer.window().request_redraw();
        if let Some(pipeline) = self.render_pipeline.as_mut() {
            pipeline.update_animation();
        }
        self.simulation_action();
    }
}

impl App {
    fn simulation_action(&mut self) {
        if !self.ui_state.is_running {
            return;
        }
        let now = Instant::now();
        let dt = now.duration_since(self.last_advance).as_secs_f64();
        let target_fps = self.ui_state.max_fps as f64;
        if !self.ui_state.unlimited_fps && dt < 1.0 / target_fps {
            return;
        }
        let shared_sim = self.simulation_state.clone();
        self.thread_pool.spawn(move || {
            let mut sim = shared_sim.write().unwrap();
            sim.advance_time(1.0);
            std::thread::sleep(Duration::from_millis(100));
        });
        self.last_advance = now;
    }

    fn left_button(&mut self, state: &ElementState) {
        let pressed = *state == ElementState::Pressed;
        self.mouse_left_down = pressed;
        if pressed {
            let now = Instant::now();
            let max_dt = Duration::from_millis(DOUBLE_CLICK_MILLIS);
            let Some(click_pos) = self.last_cursor_position else {
                return;
            };
            let Some(last_click_time) = self.last_left_click_time else {
                self.last_left_click_time = Some(now);
                self.last_left_click_pos = Some(click_pos);
                return;
            };
            let dt = now.duration_since(last_click_time);
            let is_double = if dt <= max_dt {
                let mut close_enough = false;
                if let Some((px, py)) = self.last_left_click_pos {
                    let dx = px - click_pos.0;
                    let dy = py - click_pos.1;
                    let dist2 = dx * dx + dy * dy;
                    close_enough = dist2 <= DOUBLE_CLICK_DIST;
                }
                close_enough
            } else {
                false
            };
            if is_double {
                if let Some(pipeline) = self.render_pipeline.as_mut() {
                    pipeline.y_top();
                }
                self.last_left_click_time = None;
                self.last_left_click_pos = None;
            } else {
                self.last_left_click_time = Some(now);
                self.last_left_click_pos = Some(click_pos);
            }
        }
    }

    fn right_button(&mut self, state: &ElementState) {
        let pressed = *state == ElementState::Pressed;
        self.mouse_right_down = pressed;
        if pressed {
            let now = Instant::now();
            let max_dt = Duration::from_millis(DOUBLE_CLICK_MILLIS);
            let Some(click_pos) = self.last_cursor_position else {
                return;
            };
            let Some(last_click_time) = self.last_right_click_time else {
                self.last_right_click_time = Some(now);
                self.last_right_click_pos = Some(click_pos);
                return;
            };
            let dt = now.duration_since(last_click_time);
            let is_double = if dt <= max_dt {
                let mut close_enough = false;
                if let Some((px, py)) = self.last_right_click_pos {
                    let dx = px - click_pos.0;
                    let dy = py - click_pos.1;
                    let dist2 = dx * dx + dy * dy;
                    close_enough = dist2 <= DOUBLE_CLICK_DIST;
                }
                close_enough
            } else {
                false
            };
            if is_double {
                if let Some(pipeline) = self.render_pipeline.as_mut() {
                    pipeline.center_target_on_origin();
                }
                self.last_right_click_time = None;
                self.last_right_click_pos = None;
            } else {
                self.last_right_click_time = Some(now);
                self.last_right_click_pos = Some(click_pos);
            }
        }
    }

    fn middle_button(&mut self, state: &ElementState) {
        let pressed = *state == ElementState::Pressed;
        self.mouse_middle_down = pressed;
    }
}
