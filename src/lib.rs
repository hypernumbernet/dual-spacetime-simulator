//! Library crate for `dual-spacetime-simulator` (binary entry in `main.rs`).
//! Exposes modules for integration tests under `tests/`.

pub mod camera;
pub mod graph3d;
pub mod initial_condition;
pub mod integration;
pub mod math;
pub mod pipeline;
pub mod settings;
pub mod simulation;
pub mod tree;
pub mod ui;
pub mod ui_state;
pub mod ui_styles;
pub mod vulkan_base;

pub use pipeline::shader_blobs;

use crate::initial_condition::InitialCondition;
use crate::integration::Gui;
use crate::pipeline::ParticleRenderPipeline;
use crate::settings::AppSettings;
use crate::simulation::SimulationManager;
use crate::tree::Tree;
use crate::ui::draw_ui;
use crate::ui_state::{AppMode, DragOwner, GpuTreeRenderMode, UiState};
use crate::vulkan_base::VulkanBase;
use ash::vk;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};
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

/// Run the desktop application (window + Vulkan + UI loop).
pub fn run() -> Result<(), EventLoopError> {
    let event_loop = EventLoop::new()?;
    let mut app = App::default();
    spawn_simulation_worker(
        Arc::clone(&app.ui_state),
        Arc::clone(&app.simulation_manager),
        Arc::clone(&app.need_redraw),
        Arc::clone(&app.skip_redraw),
    );
    event_loop.run_app(&mut app)
}

/// Spawns a background thread that advances simulation state and schedules redraws.
pub fn spawn_simulation_worker(
    ui_state_clone: Arc<RwLock<UiState>>,
    simulation_manager: Arc<RwLock<SimulationManager>>,
    need_redraw: Arc<RwLock<bool>>,
    skip_redraw: Arc<RwLock<u32>>,
) {
    let thread_pool = rayon::ThreadPoolBuilder::new()
        .num_threads(num_cpus::get())
        .build()
        .unwrap();
    std::thread::spawn(move || {
        let mut last_advance = Instant::now();
        let mut last_fps = Instant::now();
        let mut prev_frame: i64 = 1;
        loop {
            if *need_redraw.read().unwrap() {
                std::thread::sleep(Duration::from_millis(16));
                continue;
            }
            let ui_state = ui_state_clone.read().unwrap();
            let is_running = ui_state.is_running;
            let app_mode = ui_state.app_mode;
            let max_fps = ui_state.max_fps;
            let time_per_frame = ui_state.time_per_frame;
            let is_reset_requested = ui_state.is_reset_requested;
            let selected_initial_condition = ui_state.initial_condition.clone();
            let simulation_type = ui_state.simulation_type;
            let skip = ui_state.skip;
            let particle_count = ui_state.particle_count;
            let scale = ui_state.scale;
            drop(ui_state);
            if is_reset_requested {
                simulation_manager.read().unwrap().reset(
                    selected_initial_condition,
                    simulation_type,
                    particle_count,
                    scale,
                );
                let mut ui_state = ui_state_clone.write().unwrap();
                ui_state.frame = 1;
                ui_state.simulation_time = 0.0;
                ui_state.is_reset_requested = false;
                drop(ui_state);
                need_redraw.write().unwrap().clone_from(&true);
                skip_redraw.write().unwrap().clone_from(&skip);
                continue;
            }
            let now = Instant::now();
            let dt = now.duration_since(last_fps).as_secs_f64();
            if dt >= 1.0 {
                let mut ui_state = ui_state_clone.write().unwrap();
                ui_state.fps = if ui_state.frame - prev_frame > 0 {
                    ui_state.frame - prev_frame
                } else {
                    0
                };
                prev_frame = ui_state.frame;
                drop(ui_state);
                last_fps = now;
            }
            if !is_running || app_mode != AppMode::Simulation {
                std::thread::sleep(Duration::from_millis(16));
                continue;
            }
            let dt = now.duration_since(last_advance).as_secs_f64();
            let target_fps = max_fps as f64;
            if dt < 1.0 / target_fps {
                continue;
            }
            thread_pool.install(|| {
                simulation_manager.read().unwrap().advance(time_per_frame);
            });
            if *skip_redraw.read().unwrap() < 1 {
                let mut sr = skip_redraw.write().unwrap();
                *sr = skip;
                need_redraw.write().unwrap().clone_from(&true);
            } else {
                let mut sr = skip_redraw.write().unwrap();
                *sr -= 1;
            }
            last_advance = now;
            let mut ui_state = ui_state_clone.write().unwrap();
            ui_state.frame += 1;
            ui_state.simulation_time += time_per_frame;
        }
    });
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
    simulation_manager: Arc<RwLock<SimulationManager>>,
    positions: Vec<[f32; 3]>,
    colors: Vec<[f32; 4]>,
    need_redraw: Arc<RwLock<bool>>,
    skip_redraw: Arc<RwLock<u32>>,
    mouse_left_down: bool,
    mouse_right_down: bool,
    mouse_middle_down: bool,
    last_cursor_position: Option<(f64, f64)>,
    last_left_click_time: Option<Instant>,
    last_left_click_pos: Option<(f64, f64)>,
    last_right_click_time: Option<Instant>,
    last_right_click_pos: Option<(f64, f64)>,
    settings: AppSettings,
    last_graph3d_fingerprint: u64,
    last_gpu_tree_fingerprint: u64,
    prev_app_mode: AppMode,
    drag_owner: DragOwner,
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
            simulation_manager: Arc::new(RwLock::new(SimulationManager::default())),
            positions: Vec::new(),
            colors: Vec::new(),
            need_redraw: Arc::new(RwLock::new(true)),
            skip_redraw: Arc::new(RwLock::new(0)),
            mouse_left_down: false,
            mouse_right_down: false,
            mouse_middle_down: false,
            last_cursor_position: None,
            last_left_click_time: None,
            last_left_click_pos: None,
            last_right_click_time: None,
            last_right_click_pos: None,
            settings,
            last_graph3d_fingerprint: u64::MAX,
            last_gpu_tree_fingerprint: 0,
            prev_app_mode: AppMode::default(),
            drag_owner: DragOwner::None,
        }
    }
}

impl ApplicationHandler for App {
    /// Creates window and graphics resources when the app is resumed by the event loop.
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let ui_state = self.ui_state.write().unwrap();

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

        let vulkan_base = VulkanBase::new(&window, self.settings.mailbox_present_mode);
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

        let initial_condition = InitialCondition::default();
        let particle_count = ui_state.particle_count;
        let scale = ui_state.scale;
        let sim_type = ui_state.simulation_type;
        self.simulation_manager.write().unwrap().reset(
            initial_condition,
            sim_type,
            particle_count,
            scale,
        );
        self.skip_redraw.write().unwrap().clone_from(&ui_state.skip);
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

        let lock_camera_up = {
            let ui_state = self.ui_state.read().unwrap();
            ui_state.lock_camera_up
        };
        pipeline.set_lock_camera_up(lock_camera_up);

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
                let desired_mailbox_present_mode = {
                    let ui_state = self.ui_state.read().unwrap();
                    ui_state.mailbox_present_mode
                };
                if vb.mailbox_present_mode != desired_mailbox_present_mode {
                    vb.mailbox_present_mode = desired_mailbox_present_mode;
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

                let ui_state = self.ui_state.read().unwrap();
                let scale = ui_state.scale_gauge;
                let link_point_size_to_scale = ui_state.link_point_size_to_scale;
                let show_grid = ui_state.show_grid;
                let app_mode = ui_state.app_mode;
                let gpu_tree_render_mode = if app_mode == AppMode::GpuTree {
                    ui_state.gpu_tree_render_mode
                } else {
                    GpuTreeRenderMode::Polygons
                };
                drop(ui_state);

                pipeline.render(
                    cb,
                    image_index as usize,
                    vb.swapchain_extent,
                    gui,
                    scale,
                    link_point_size_to_scale,
                    show_grid,
                    app_mode,
                    gpu_tree_render_mode,
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
                if pressed {
                    if ui_wants_pointer || ui_consumed {
                        self.drag_owner = DragOwner::Ui;
                        self.clear_mouse_drag_flags();
                    } else {
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
                                self.middle_button(state);
                            }
                            _ => {}
                        }
                    }
                } else {
                    match button {
                        MouseButton::Left => self.left_button(state),
                        MouseButton::Right => self.right_button(state),
                        MouseButton::Middle => self.middle_button(state),
                        _ => {}
                    }
                    self.drag_owner = DragOwner::None;
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                let (x, y) = (position.x, position.y);
                let ui_blocks = ui_wants_pointer || ui_consumed;
                if let Some(new_owner) = self.drag_owner.promote_from_pending(ui_blocks) {
                    if new_owner == DragOwner::Ui {
                        self.mouse_left_down = false;
                        self.mouse_right_down = false;
                        self.mouse_middle_down = false;
                    }
                    self.drag_owner = new_owner;
                }
                if let Some((lx, ly)) = self.last_cursor_position {
                    match self.drag_owner {
                        DragOwner::SceneLeft => pipeline.revolve_camera(x - lx, y - ly),
                        DragOwner::SceneRight => pipeline.look_around(x - lx, y - ly),
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
                            let zoom_factor = y * 0.1;
                            pipeline.zoom_camera(zoom_factor);
                        }
                        MouseScrollDelta::PixelDelta(PhysicalPosition { y, .. }) => {
                            let zoom_factor = y * 0.1;
                            pipeline.zoom_camera(zoom_factor as f32);
                        }
                    }
                }
            }
            _ => {}
        }
    }

    /// Performs per-frame updates before the event loop waits for new events.
    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(window) = self.window.as_ref() {
            window.request_redraw();
        }
        if let Some(pipeline) = self.render_pipeline.as_mut() {
            pipeline.update_animation();
        }
        let app_mode = self.ui_state.read().unwrap().app_mode;
        let prev = self.prev_app_mode;
        if prev == AppMode::Graph3D && app_mode != AppMode::Graph3D {
            if let Some(pipeline) = self.render_pipeline.as_mut() {
                pipeline.set_graph_lines(&[]);
            }
        }
        if prev != app_mode && app_mode == AppMode::Simulation {
            // Force a fresh simulation redraw when returning from non-simulation views.
            if let Some(pipeline) = self.render_pipeline.as_mut() {
                pipeline.set_graph_lines(&[]);
            }
            self.last_graph3d_fingerprint = u64::MAX;
            self.last_gpu_tree_fingerprint = 0;
            *self.need_redraw.write().unwrap() = true;
        }
        if prev == AppMode::GpuTree && app_mode == AppMode::Graph3D {
            // Rebuild Graph3D data even when parameters are unchanged after leaving GpuTree.
            self.last_graph3d_fingerprint = u64::MAX;
        }
        if app_mode == AppMode::GpuTree {
            let uis = self.ui_state.read().unwrap();
            let fp = uis.gpu_tree_fingerprint();
            let layout = uis.gpu_tree_layout;
            let render_mode = uis.gpu_tree_render_mode;
            let params = uis.gpu_tree_params;
            drop(uis);

            let needs_update = prev != AppMode::GpuTree || fp != self.last_gpu_tree_fingerprint;
            if needs_update {
                if let Some(pipeline) = self.render_pipeline.as_mut() {
                    pipeline.set_particles(&[], &[]);
                    pipeline.set_graph_lines(&[]);
                    match render_mode {
                        GpuTreeRenderMode::Lines => {
                            let line_verts = Tree::generate_vertices_for_layout(layout, params);
                            pipeline.set_graph_lines(&line_verts);
                        }
                        GpuTreeRenderMode::Polygons => {
                            pipeline.compute_tree_vertices(params, layout);
                        }
                    }
                }
                self.last_gpu_tree_fingerprint = fp;
            }
            self.prev_app_mode = app_mode;
            return;
        }
        self.prev_app_mode = app_mode;

        if app_mode == AppMode::Graph3D {
            let uis = self.ui_state.read().unwrap();
            let fp = crate::graph3d::graph_params_fingerprint(
                uis.graph_type,
                uis.graph_sample_count,
                uis.graph_t_slice,
                uis.graph_velocity_scale,
                uis.graph_phi,
            );
            let (gt, n, t, vs, phi) = (
                uis.graph_type,
                uis.graph_sample_count,
                uis.graph_t_slice,
                uis.graph_velocity_scale,
                uis.graph_phi,
            );
            drop(uis);
            if fp != self.last_graph3d_fingerprint {
                let (pos, col) = crate::graph3d::build_points(gt, n, t, vs, phi);
                let line_verts = crate::graph3d::build_graph_line_vertices(gt, n, t, vs, phi);
                if let Some(pipeline) = self.render_pipeline.as_mut() {
                    pipeline.set_particles(&pos, &col);
                    pipeline.set_graph_lines(&line_verts);
                }
                self.last_graph3d_fingerprint = fp;
            }
            return;
        }
        self.last_graph3d_fingerprint = u64::MAX;
        self.last_gpu_tree_fingerprint = 0;
        if *self.need_redraw.read().unwrap() == false {
            return;
        }
        if let Ok(manager) = self.simulation_manager.try_read() {
            let particles = manager.particles();
            self.positions = particles
                .iter()
                .map(|p| {
                    [
                        p.position.x as f32,
                        p.position.y as f32,
                        p.position.z as f32,
                    ]
                })
                .collect();
            self.colors = particles.iter().map(|p| p.color).collect();
            self.need_redraw.write().unwrap().clone_from(&false);
            if let Some(pipeline) = self.render_pipeline.as_mut() {
                pipeline.set_particles(&self.positions, &self.colors);
            }
        }
    }
}

impl App {
    /// Clears all internal mouse drag button state flags.
    fn clear_mouse_drag_flags(&mut self) {
        self.mouse_left_down = false;
        self.mouse_right_down = false;
        self.mouse_middle_down = false;
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
        let pressed = *state == ElementState::Pressed;
        self.mouse_left_down = pressed;
        if pressed {
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
    }

    /// Handles right-button press/release and double-click target-centering behavior.
    fn right_button(&mut self, state: &ElementState) {
        let pressed = *state == ElementState::Pressed;
        self.mouse_right_down = pressed;
        if pressed {
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

    /// Handles middle-button press/release state tracking for camera roll gestures.
    fn middle_button(&mut self, state: &ElementState) {
        let pressed = *state == ElementState::Pressed;
        self.mouse_middle_down = pressed;
    }
}
