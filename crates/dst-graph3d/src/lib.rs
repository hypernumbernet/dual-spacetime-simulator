//! Library crate for `dst-graph3d` (binary entry in `main.rs`).
//! Exposes modules for integration tests under `tests/`.

pub mod camera;
pub mod display_buffer;
pub mod graph3d;
pub mod integration;
pub mod pipeline;
pub mod settings;
pub mod ui;
pub mod ui_state;
pub mod ui_styles;

use crate::graph3d::GraphType;
use crate::integration::Gui;
use crate::pipeline::ParticleRenderPipeline;
use crate::settings::AppSettings;
use crate::ui::draw_ui;
use crate::ui_state::{DragOwner, UiState};
use ash::vk;
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};
use vulkanvil::VulkanBase;
use winit::{
    application::ApplicationHandler,
    dpi::PhysicalPosition,
    error::EventLoopError,
    event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent},
    event_loop::{ActiveEventLoop, EventLoop},
    window::Window,
};

const DOUBLE_CLICK_MILLIS: u64 = 400;
const DOUBLE_CLICK_DIST: f64 = 25.0;
const DEFAULT_WINDOW_WIDTH: f32 = 1280.0;
const DEFAULT_WINDOW_HEIGHT: f32 = 800.0;

type GraphParams = (GraphType, u32, f64, f64);

struct Graph3dBuildResult {
    fp: u64,
    positions: Vec<[f32; 3]>,
    colors: Vec<[f32; 4]>,
    line_vertices: Vec<([f32; 3], [f32; 4])>,
}

/// Run the desktop application (window + Vulkan + UI loop).
pub fn run() -> Result<(), EventLoopError> {
    let event_loop = EventLoop::new()?;
    let mut app = App::default();
    event_loop.run_app(&mut app)
}

/// Builds the window title from crate name and version metadata.
fn generate_window_title() -> String {
    let package_name = env!("CARGO_PKG_NAME");
    let package_version = env!("CARGO_PKG_VERSION");
    format!("{} v{}", package_name, package_version)
}

pub struct App {
    // Drop order matters: gui and pipeline must be dropped before vulkan_base
    gui: Option<Gui>,
    render_pipeline: Option<ParticleRenderPipeline>,
    vulkan_base: Option<VulkanBase>,
    window: Option<Arc<Window>>,
    ui_state: Arc<RwLock<UiState>>,
    last_cursor_position: Option<(f64, f64)>,
    last_left_click_time: Option<Instant>,
    last_left_click_pos: Option<(f64, f64)>,
    last_right_click_time: Option<Instant>,
    last_right_click_pos: Option<(f64, f64)>,
    settings: AppSettings,
    last_graph3d_fingerprint: u64,
    cached_graph_params: GraphParams,
    cached_graph_fingerprint: u64,
    graph3d_pending_rx: Option<Receiver<Graph3dBuildResult>>,
    drag_owner: DragOwner,
}

impl Drop for App {
    /// Waits for all pending GPU work to finish before dropping GUI and pipeline resources.
    ///
    /// The last frame's command buffer may still be executing on the GPU when the event loop
    /// exits (e.g. via CloseRequested or request_exit). The gui (egui renderer) and
    /// render_pipeline own Vulkan resources (vertex buffers, textures, descriptor sets,
    /// pipelines, etc.) that are referenced by in-flight command buffers.
    ///
    /// Destroying them before the work completes violates Vulkan's rules and commonly
    /// results in ERROR_DEVICE_LOST (visible in the Drop impls of ParticleRenderPipeline
    /// and VulkanBase).
    ///
    /// We perform the wait here, while all resources are still alive. The waits inside
    /// the individual drops will then be fast and succeed.
    fn drop(&mut self) {
        if let Some(vb) = &self.vulkan_base {
            let _ = unsafe { vb.device.device_wait_idle() };
        }
    }
}

impl Default for App {
    /// Creates application state with loaded settings and default runtime resources.
    fn default() -> Self {
        let settings = AppSettings::load();
        let mut ui_state = UiState::default();
        ui_state.apply_settings(&settings);
        Self {
            window: None,
            vulkan_base: None,
            render_pipeline: None,
            gui: None,
            ui_state: Arc::new(RwLock::new(ui_state)),
            last_cursor_position: None,
            last_left_click_time: None,
            last_left_click_pos: None,
            last_right_click_time: None,
            last_right_click_pos: None,
            settings,
            last_graph3d_fingerprint: u64::MAX,
            cached_graph_params: (
                GraphType::SphericalFibonacciLattice,
                0,
                f64::NAN,
                f64::NAN,
            ),
            cached_graph_fingerprint: 0,
            graph3d_pending_rx: None,
            drag_owner: DragOwner::None,
        }
    }
}

impl ApplicationHandler for App {
    /// Creates window and graphics resources when the app is resumed by the event loop.
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let ui_state = self.ui_state.read().unwrap();

        let window_attrs = Window::default_attributes()
            .with_title(generate_window_title())
            .with_inner_size(winit::dpi::LogicalSize::new(
                DEFAULT_WINDOW_WIDTH,
                DEFAULT_WINDOW_HEIGHT,
            ))
            .with_min_inner_size(winit::dpi::LogicalSize::new(
                ui_state.min_window_width,
                ui_state.min_window_height,
            ));
        let window = Arc::new(event_loop.create_window(window_attrs).unwrap());

        if self.settings.start_maximized {
            window.set_maximized(true);
        }

        let vulkan_base = VulkanBase::new(
            &window,
            self.settings.mailbox_present_mode,
            c"DstGraph3D",
            vk::make_api_version(0, 0, 1, 0),
        );
        let render_pipeline = ParticleRenderPipeline::new(&vulkan_base);

        let gui = Gui::new(
            event_loop,
            &window,
            &vulkan_base.instance,
            vulkan_base.physical_device,
            vulkan_base.device.clone(),
            vulkan_base.graphics_queue,
            vulkan_base.command_pool,
            render_pipeline.render_pass(),
            vulkan_base.swapchain_format,
        );

        self.window = Some(window);
        self.render_pipeline = Some(render_pipeline);
        self.vulkan_base = Some(vulkan_base);
        self.gui = Some(gui);
    }

    /// Handles window, input, rendering, and camera events for each platform event.
    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: winit::window::WindowId,
        event: WindowEvent,
    ) {
        let Some(window) = self.window.as_ref() else {
            return;
        };
        let Some(vb) = self.vulkan_base.as_mut() else {
            return;
        };
        let gui = self.gui.as_mut().unwrap();
        let Some(pipeline) = self.render_pipeline.as_mut() else {
            return;
        };

        {
            let mut ui_state = self.ui_state.write().unwrap();
            if ui_state.request_exit {
                ui_state.request_exit = false;
                event_loop.exit();
                return;
            }
        }

        let render_settings = {
            let ui_state = self.ui_state.read().unwrap();
            (
                ui_state.lock_camera_up,
                ui_state.mailbox_present_mode,
                ui_state.link_point_size_to_scale,
                ui_state.show_grid,
                ui_state.particle_display_mode,
            )
        };
        pipeline.set_lock_camera_up(render_settings.0);

        match &event {
            WindowEvent::Resized(size) => {
                if size.width > 0 && size.height > 0 {
                    vb.recreate_swapchain(window);
                    pipeline.recreate_framebuffers(vb);
                }
            }
            WindowEvent::ScaleFactorChanged { .. } => {
                vb.recreate_swapchain(window);
                pipeline.recreate_framebuffers(vb);
            }
            WindowEvent::CloseRequested => {
                event_loop.exit();
            }
            WindowEvent::RedrawRequested => {
                gui.immediate_ui(window, |gui| {
                    let ctx = gui.context();
                    draw_ui(&self.ui_state, &mut self.settings, &ctx);
                });

                if vb.mailbox_present_mode != render_settings.1 {
                    vb.mailbox_present_mode = render_settings.1;
                    vb.recreate_swapchain(window);
                    pipeline.recreate_framebuffers(vb);
                }
                gui.prepare_frame(window);

                vb.wait_for_fence();

                let image_index = match vb.acquire_next_image() {
                    Ok((idx, _)) => idx,
                    Err(vk::Result::ERROR_OUT_OF_DATE_KHR) => {
                        vb.recreate_swapchain(window);
                        pipeline.recreate_framebuffers(vb);
                        return;
                    }
                    Err(e) => panic!("Failed to acquire swapchain image: {:?}", e),
                };

                vb.reset_fence();

                let cb = vb.current_command_buffer();
                let begin_ci = vk::CommandBufferBeginInfo::default()
                    .flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT);
                unsafe {
                    vb.device
                        .reset_command_buffer(cb, vk::CommandBufferResetFlags::empty())
                        .unwrap();
                    vb.device.begin_command_buffer(cb, &begin_ci).unwrap();
                }

                pipeline.render(
                    cb,
                    image_index as usize,
                    vb.swapchain_extent,
                    gui,
                    render_settings.2,
                    render_settings.3,
                    render_settings.4,
                );

                unsafe {
                    vb.device.end_command_buffer(cb).unwrap();
                }

                match vb.submit_and_present(image_index) {
                    Ok(true) | Err(vk::Result::ERROR_OUT_OF_DATE_KHR) => {
                        vb.recreate_swapchain(window);
                        pipeline.recreate_framebuffers(vb);
                    }
                    Ok(false) => {}
                    Err(e) => panic!("Failed to present: {:?}", e),
                }

                gui.finish_frame();
                vb.advance_frame();
            }
            _ => (),
        }

        let window_clone = window.clone();
        let ui_consumed = gui.update(&window_clone, &event);
        let ui_wants_pointer = gui.pointer_wants_input();

        match &event {
            WindowEvent::MouseInput { state, button, .. } => {
                let pressed = *state == ElementState::Pressed;
                if ui_wants_pointer || ui_consumed {
                    if pressed {
                        self.drag_owner = DragOwner::Ui;
                    } else {
                        self.drag_owner = DragOwner::None;
                    }
                } else if pressed {
                    match button {
                        MouseButton::Left => {
                            self.drag_owner = DragOwner::PendingSceneLeft;
                            self.left_button(state);
                        }
                        MouseButton::Right => {
                            self.drag_owner = DragOwner::PendingSceneRight;
                            self.right_button(state);
                        }
                        MouseButton::Middle => {
                            self.drag_owner = DragOwner::PendingSceneMiddle;
                        }
                        _ => {}
                    }
                } else {
                    match button {
                        MouseButton::Left => self.left_button(state),
                        MouseButton::Right => self.right_button(state),
                        _ => {}
                    }
                    self.drag_owner = DragOwner::None;
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                let (x, y) = (position.x, position.y);
                let ui_blocks = ui_wants_pointer || ui_consumed;
                if let Some(new_owner) = self.drag_owner.promote_from_pending(ui_blocks) {
                    self.drag_owner = new_owner;
                }
                if let Some((lx, ly)) = self.last_cursor_position {
                    match self.drag_owner {
                        DragOwner::SceneLeft => {
                            pipeline.revolve_camera(x - lx, y - ly);
                        }
                        DragOwner::SceneRight => {
                            pipeline.look_around(x - lx, y - ly);
                        }
                        DragOwner::SceneMiddle => {
                            let window_size = window.inner_size();
                            let center_x = window_size.width as f64 / 2.0;
                            let center_y = window_size.height as f64 / 2.0;
                            pipeline.rotate_camera(x, lx, y, ly, center_x, center_y);
                        }
                        DragOwner::None
                        | DragOwner::Ui
                        | DragOwner::PendingSceneLeft
                        | DragOwner::PendingSceneRight
                        | DragOwner::PendingSceneMiddle => {}
                    }
                }
                self.last_cursor_position = Some((x, y));
            }
            WindowEvent::MouseWheel { delta, .. } => {
                if !ui_wants_pointer && !ui_consumed {
                    match delta {
                        MouseScrollDelta::LineDelta(_, y) => {
                            pipeline.zoom_camera(y * 0.1);
                        }
                        MouseScrollDelta::PixelDelta(PhysicalPosition { y, .. }) => {
                            pipeline.zoom_camera(*y as f32 * 0.1);
                        }
                    }
                }
            }
            _ => {}
        }
    }

    /// Performs per-frame updates before the event loop waits for new events.
    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(pipeline) = self.render_pipeline.as_mut() {
            pipeline.update_animation();
        }

        let uis = self.ui_state.read().unwrap();
        let params = (
            uis.graph_type,
            uis.graph_sample_count,
            uis.graph_radius,
            uis.graph_velocity_scale,
        );
        drop(uis);

        let fp = if params != self.cached_graph_params {
            self.cached_graph_params = params;
            self.cached_graph_fingerprint = crate::graph3d::graph_params_fingerprint(
                params.0,
                params.1,
                params.2,
                params.3,
            );
            self.cached_graph_fingerprint
        } else {
            self.cached_graph_fingerprint
        };

        if let Some(rx) = self.graph3d_pending_rx.as_ref() {
            match rx.try_recv() {
                Ok(result) => {
                    self.graph3d_pending_rx = None;
                    if result.fp == fp {
                        if let Some(pipeline) = self.render_pipeline.as_mut() {
                            pipeline.set_particles(&result.positions, &result.colors);
                            pipeline.set_graph_lines(&result.line_vertices);
                        }
                        self.last_graph3d_fingerprint = result.fp;
                    }
                }
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => {
                    self.graph3d_pending_rx = None;
                }
            }
        }

        if fp != self.last_graph3d_fingerprint && self.graph3d_pending_rx.is_none() {
            let (tx, rx) = mpsc::channel::<Graph3dBuildResult>();
            let (gt, n, t, vs) = params;
            std::thread::spawn(move || {
                let geometry = crate::graph3d::build_graph_geometry(gt, n, t, vs);
                let _ = tx.send(Graph3dBuildResult {
                    fp,
                    positions: geometry.positions,
                    colors: geometry.colors,
                    line_vertices: geometry.line_vertices,
                });
            });
            self.graph3d_pending_rx = Some(rx);
        }

        if let Some(window) = self.window.as_ref() {
            window.request_redraw();
        }
    }
}

impl App {
    /// Returns `true` when a double-click was recognized (click history is cleared).
    fn try_consume_double_click(
        click_pos: (f64, f64),
        now: Instant,
        last_time: &mut Option<Instant>,
        last_pos: &mut Option<(f64, f64)>,
    ) -> bool {
        let max_dt = Duration::from_millis(DOUBLE_CLICK_MILLIS);
        let Some(prev_t) = *last_time else {
            *last_time = Some(now);
            *last_pos = Some(click_pos);
            return false;
        };
        let dt = now.duration_since(prev_t);
        let is_double = if dt <= max_dt {
            if let Some((px, py)) = *last_pos {
                let dx = px - click_pos.0;
                let dy = py - click_pos.1;
                let dist2 = dx * dx + dy * dy;
                dist2 <= DOUBLE_CLICK_DIST
            } else {
                false
            }
        } else {
            false
        };
        if is_double {
            *last_time = None;
            *last_pos = None;
            true
        } else {
            *last_time = Some(now);
            *last_pos = Some(click_pos);
            false
        }
    }

    /// Handles left-button press/release and double-click camera-up reset behavior.
    fn left_button(&mut self, state: &ElementState) {
        if *state != ElementState::Pressed {
            return;
        }
        let now = Instant::now();
        let Some(click_pos) = self.last_cursor_position else {
            return;
        };
        if Self::try_consume_double_click(
            click_pos,
            now,
            &mut self.last_left_click_time,
            &mut self.last_left_click_pos,
        ) && let Some(pipeline) = self.render_pipeline.as_mut()
        {
            pipeline.y_top();
        }
    }

    /// Handles right-button press/release and double-click target-centering behavior.
    fn right_button(&mut self, state: &ElementState) {
        if *state != ElementState::Pressed {
            return;
        }
        let now = Instant::now();
        let Some(click_pos) = self.last_cursor_position else {
            return;
        };
        if Self::try_consume_double_click(
            click_pos,
            now,
            &mut self.last_right_click_time,
            &mut self.last_right_click_pos,
        ) && let Some(pipeline) = self.render_pipeline.as_mut()
        {
            pipeline.center_target_on_origin();
        }
    }
}
