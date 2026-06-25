//! Library crate for `dst-graph3d` (binary entry in `main.rs`).
//! Exposes modules for integration tests under `tests/`.

pub mod display_buffer;
pub mod graph3d;
pub mod integration;
pub mod pipeline;
pub mod settings;
pub mod ui;
pub mod ui_state;
pub mod ui_styles;

use crate::graph3d::{GraphGeometry, GraphType};
use crate::integration::Gui;
use crate::pipeline::ParticleRenderPipeline;
use crate::settings::AppSettings;
use crate::ui::draw_ui;
use crate::ui_state::{DragOwner, UiState};
use ash::vk;
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};
use vulkanvil::{
    InputState, VulkanBase, apply_camera_mouse_wheel, spacecraft_scene_wheel_allowed,
    spacecraft_steer_offset, tick_orbit_camera, tick_spacecraft_steer_and_motion,
    toggle_spacecraft_steer_anchor as toggle_steer_anchor,
};
use winit::{
    application::ApplicationHandler,
    dpi::PhysicalPosition,
    error::EventLoopError,
    event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent},
    event_loop::{ActiveEventLoop, EventLoop},
    keyboard::{KeyCode, PhysicalKey},
    window::Window,
};

const DOUBLE_CLICK_MILLIS: u64 = 400;
const DOUBLE_CLICK_DIST: f64 = 25.0;
const DEFAULT_WINDOW_WIDTH: f32 = 1280.0;
const DEFAULT_WINDOW_HEIGHT: f32 = 800.0;

type GraphParams = (GraphType, u32, f64, f64);

struct GraphParamsCache {
    params: GraphParams,
    fingerprint: u64,
}

struct Graph3dBuildResult {
    generation: u64,
    fp: u64,
    geometry: GraphGeometry,
}

struct FrameRenderSettings {
    lock_camera_up: bool,
    mailbox_present_mode: bool,
    show_grid: bool,
    particle_display_mode: crate::ui_state::ParticleDisplayMode,
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
    graph_params_cache: GraphParamsCache,
    graph_build_generation: u64,
    graph3d_build_target_fp: Option<u64>,
    graph3d_pending_rx: Option<Receiver<Graph3dBuildResult>>,
    drag_owner: DragOwner,
    input: InputState,
    last_camera_tick: Option<Instant>,
    last_lock_camera_up: Option<bool>,
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
            graph_params_cache: GraphParamsCache {
                params: (
                    GraphType::SphericalFibonacciLattice,
                    0,
                    f64::NAN,
                    f64::NAN,
                ),
                fingerprint: 0,
            },
            graph_build_generation: 0,
            graph3d_build_target_fp: None,
            graph3d_pending_rx: None,
            drag_owner: DragOwner::None,
            input: InputState::default(),
            last_camera_tick: None,
            last_lock_camera_up: None,
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
            FrameRenderSettings {
                lock_camera_up: ui_state.lock_camera_up,
                mailbox_present_mode: ui_state.mailbox_present_mode,
                show_grid: ui_state.show_grid,
                particle_display_mode: ui_state.particle_display_mode,
            }
        };
        pipeline.set_lock_camera_up(render_settings.lock_camera_up);

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
                    draw_ui(&self.ui_state, &mut self.settings, &gui.egui_ctx);
                });

                if vb.mailbox_present_mode != render_settings.mailbox_present_mode {
                    vb.mailbox_present_mode = render_settings.mailbox_present_mode;
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
                    render_settings.show_grid,
                    render_settings.particle_display_mode,
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
                            if !self.ui_state.read().unwrap().lock_camera_up {
                                if let Some(pos) = self.last_cursor_position {
                                    Self::toggle_spacecraft_steer_anchor(
                                        window,
                                        &self.ui_state,
                                        pos,
                                    );
                                }
                            }
                            self.left_button(state);
                        }
                        MouseButton::Right => {
                            self.drag_owner = DragOwner::PendingSceneRight;
                            if !self.ui_state.read().unwrap().lock_camera_up {
                                if let Some(pos) = self.last_cursor_position {
                                    Self::sync_spacecraft_yaw_steer_anchor(
                                        window,
                                        &self.ui_state,
                                        Some(pos),
                                    );
                                }
                            }
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
                        MouseButton::Right => {
                            let lock_camera_up = self.ui_state.read().unwrap().lock_camera_up;
                            if !lock_camera_up {
                                Self::sync_spacecraft_yaw_steer_anchor(
                                    window,
                                    &self.ui_state,
                                    None,
                                );
                            }
                            self.right_button(state);
                        }
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
                let lock_camera_up = self.ui_state.read().unwrap().lock_camera_up;
                if let Some((lx, ly)) = self.last_cursor_position {
                    match self.drag_owner {
                        DragOwner::SceneLeft if lock_camera_up => {
                            pipeline.revolve_camera(x - lx, y - ly);
                        }
                        DragOwner::SceneLeft => {}
                        DragOwner::SceneRight if lock_camera_up => {
                            pipeline.look_around(x - lx, y - ly);
                        }
                        DragOwner::SceneRight => {}
                        DragOwner::SceneMiddle if lock_camera_up => {
                            let window_size = window.inner_size();
                            let center_x = window_size.width as f64 / 2.0;
                            let center_y = window_size.height as f64 / 2.0;
                            pipeline.rotate_camera(x, lx, y, ly, center_x, center_y);
                        }
                        DragOwner::None
                        | DragOwner::Ui
                        | DragOwner::SceneMiddle
                        | DragOwner::PendingSceneLeft
                        | DragOwner::PendingSceneRight
                        | DragOwner::PendingSceneMiddle => {}
                    }
                }
                self.last_cursor_position = Some((x, y));
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let (lock_camera_up, steer_anchor_active) = {
                    let ui = self.ui_state.read().unwrap();
                    (ui.lock_camera_up, ui.spacecraft_steer_anchor.is_some())
                };
                if spacecraft_scene_wheel_allowed(
                    lock_camera_up,
                    steer_anchor_active,
                    ui_wants_pointer || ui_consumed,
                ) {
                    match delta {
                        MouseScrollDelta::LineDelta(_, y) => {
                            apply_camera_mouse_wheel(pipeline.camera_mut(), lock_camera_up, *y);
                        }
                        MouseScrollDelta::PixelDelta(PhysicalPosition { y, .. }) => {
                            apply_camera_mouse_wheel(
                                pipeline.camera_mut(),
                                lock_camera_up,
                                *y as f32,
                            );
                        }
                    }
                }
            }
            WindowEvent::KeyboardInput { event, .. } => {
                if let PhysicalKey::Code(code) = event.physical_key {
                    self.input.key_event(code, event.state);
                    if event.physical_key == PhysicalKey::Code(KeyCode::Home)
                        && event.state == ElementState::Pressed
                        && !event.repeat
                        && !gui.keyboard_wants_input()
                    {
                        pipeline.center_target_on_origin();
                    }
                }
            }
            _ => {}
        }
    }

    /// Performs per-frame updates before the event loop waits for new events.
    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        let (lock_camera_up, params) = {
            let uis = self.ui_state.read().unwrap();
            (
                uis.lock_camera_up,
                (
                    uis.graph_type,
                    uis.graph_sample_count,
                    uis.graph_radius,
                    uis.graph_velocity_scale,
                ),
            )
        };
        let keyboard_blocked = self
            .gui
            .as_ref()
            .is_some_and(|gui| gui.keyboard_wants_input());
        if self.last_lock_camera_up != Some(lock_camera_up) {
            self.last_camera_tick = None;
            self.last_lock_camera_up = Some(lock_camera_up);
            if let Some(window) = self.window.as_ref() {
                Self::sync_spacecraft_steer_anchor(window, &self.ui_state, None);
                Self::sync_spacecraft_yaw_steer_anchor(window, &self.ui_state, None);
            } else {
                let mut uis = self.ui_state.write().unwrap();
                uis.spacecraft_steer_anchor = None;
                uis.spacecraft_yaw_steer_anchor = None;
            }
        }
        if let Some(pipeline) = self.render_pipeline.as_mut() {
            if !lock_camera_up {
                let now = Instant::now();
                let dt = self
                    .last_camera_tick
                    .map(|t| now.duration_since(t).as_secs_f32())
                    .unwrap_or(0.0);
                self.last_camera_tick = Some(now);
                let (yaw_steer_offset_x, anchor_steer_offset) = {
                    let uis = self.ui_state.read().unwrap();
                    let yaw_steer_offset_x = spacecraft_steer_offset(
                        uis.spacecraft_yaw_steer_anchor,
                        self.last_cursor_position,
                    )
                    .map(|(x, _)| x);
                    let anchor_steer_offset = spacecraft_steer_offset(
                        uis.spacecraft_steer_anchor,
                        self.last_cursor_position,
                    );
                    (yaw_steer_offset_x, anchor_steer_offset)
                };
                tick_spacecraft_steer_and_motion(
                    pipeline.camera_mut(),
                    yaw_steer_offset_x,
                    anchor_steer_offset,
                    dt,
                );
            }
            tick_orbit_camera(
                pipeline.camera_mut(),
                &self.input,
                lock_camera_up,
                keyboard_blocked,
            );
        }

        let fp = self.graph_params_fingerprint(params);

        if let Some(rx) = self.graph3d_pending_rx.as_ref() {
            match rx.try_recv() {
                Ok(result) => {
                    self.graph3d_pending_rx = None;
                    if result.generation == self.graph_build_generation && result.fp == fp {
                        if let Some(pipeline) = self.render_pipeline.as_mut() {
                            pipeline.set_particles(
                                &result.geometry.positions,
                                &result.geometry.colors,
                            );
                            pipeline.set_graph_lines(&result.geometry.line_vertices);
                        }
                        self.last_graph3d_fingerprint = result.fp;
                        self.graph3d_build_target_fp = None;
                    }
                }
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => {
                    self.graph3d_pending_rx = None;
                    self.graph3d_build_target_fp = None;
                }
            }
        }

        if fp != self.last_graph3d_fingerprint && self.graph3d_build_target_fp != Some(fp) {
            self.graph_build_generation = self.graph_build_generation.wrapping_add(1);
            let generation = self.graph_build_generation;
            self.graph3d_build_target_fp = Some(fp);
            let (gt, n, t, vs) = params;
            let (tx, rx) = mpsc::channel::<Graph3dBuildResult>();
            std::thread::spawn(move || {
                let geometry = crate::graph3d::build_graph_geometry(gt, n, t, vs);
                let _ = tx.send(Graph3dBuildResult {
                    generation,
                    fp,
                    geometry,
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
    fn sync_spacecraft_steer_anchor(
        window: &Window,
        ui_state: &Arc<RwLock<UiState>>,
        anchor: Option<(f64, f64)>,
    ) {
        ui_state.write().unwrap().spacecraft_steer_anchor = anchor.map(|(x, y)| [x, y]);
        window.request_redraw();
    }

    fn sync_spacecraft_yaw_steer_anchor(
        window: &Window,
        ui_state: &Arc<RwLock<UiState>>,
        anchor: Option<(f64, f64)>,
    ) {
        ui_state.write().unwrap().spacecraft_yaw_steer_anchor = anchor.map(|(x, y)| [x, y]);
        window.request_redraw();
    }

    fn toggle_spacecraft_steer_anchor(
        window: &Window,
        ui_state: &Arc<RwLock<UiState>>,
        pos: (f64, f64),
    ) {
        {
            let mut uis = ui_state.write().unwrap();
            toggle_steer_anchor(&mut uis.spacecraft_steer_anchor, pos);
        }
        window.request_redraw();
    }

    fn graph_params_fingerprint(&mut self, params: GraphParams) -> u64 {
        if params != self.graph_params_cache.params {
            self.graph_params_cache.params = params;
            self.graph_params_cache.fingerprint = crate::graph3d::graph_params_fingerprint(
                params.0,
                params.1,
                params.2,
                params.3,
            );
        }
        self.graph_params_cache.fingerprint
    }
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
        if !self.ui_state.read().unwrap().lock_camera_up {
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
        if !self.ui_state.read().unwrap().lock_camera_up {
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
