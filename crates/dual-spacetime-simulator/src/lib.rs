//! Library crate for `dual-spacetime-simulator` (binary entry in `main.rs`).
//! Exposes modules for integration tests under `tests/`.

pub mod camera;
pub mod gpu_simulation;
pub mod integration;
pub mod object_input;
pub mod particle_snapshot;
pub mod pipeline;
pub mod settings;
pub mod simulation;
pub mod solar_system_data;
pub mod ui;
pub mod ui_state;
pub mod ui_styles;

use crate::integration::Gui;
use crate::object_input::{ObjectInput, SolarSystemBuildError, build_solar_system_particles};
use crate::pipeline::ParticleRenderPipeline;
use crate::settings::AppSettings;
use crate::simulation::SimulationManager;
use crate::ui::{draw_ui, process_pending_snapshot_dialog};
use crate::ui_state::{DragOwner, PlacementMode, UiState};
use ash::vk;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};
use vulkanvil::VulkanBase;
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

/// Coordinates CPU→GPU particle buffer synchronization between the UI, worker, and render loop.
#[derive(Clone)]
struct GpuParticleSync {
    upload_pending: Arc<AtomicBool>,
    append_pending: Arc<AtomicBool>,
    advance_steps: Arc<AtomicU32>,
}

impl GpuParticleSync {
    fn new(initial_upload: bool) -> Self {
        Self {
            upload_pending: Arc::new(AtomicBool::new(initial_upload)),
            append_pending: Arc::new(AtomicBool::new(false)),
            advance_steps: Arc::new(AtomicU32::new(0)),
        }
    }

    fn request_full_upload(&self) {
        self.advance_steps.store(0, Ordering::Release);
        self.upload_pending.store(true, Ordering::Release);
        self.append_pending.store(false, Ordering::Release);
    }

    fn request_append_preserving(&self) {
        self.advance_steps.store(0, Ordering::Release);
        self.append_pending.store(true, Ordering::Release);
    }

    fn request_cpu_mode_upload(&self) {
        self.advance_steps.store(0, Ordering::Release);
        self.upload_pending.store(true, Ordering::Release);
    }

    fn has_pending_sync(&self) -> bool {
        self.upload_pending.load(Ordering::Acquire) || self.append_pending.load(Ordering::Acquire)
    }

    fn take_upload_pending(&self) -> bool {
        self.upload_pending.swap(false, Ordering::AcqRel)
    }

    fn append_pending(&self) -> bool {
        self.append_pending.load(Ordering::Acquire)
    }

    fn clear_append_pending(&self) {
        self.append_pending.store(false, Ordering::Release);
    }

    fn take_advance_steps(&self) -> u32 {
        self.advance_steps.swap(0, Ordering::AcqRel)
    }

    fn fetch_add_advance_step(&self) {
        self.advance_steps.fetch_add(1, Ordering::Release);
    }

    fn clear_advance_steps(&self) {
        self.advance_steps.store(0, Ordering::Release);
    }
}

/// Run the desktop application (window + Vulkan + UI loop).
pub fn run() -> Result<(), EventLoopError> {
    let event_loop = EventLoop::new()?;
    let mut app = App::default();
    spawn_simulation_worker(
        Arc::clone(&app.ui_state),
        Arc::clone(&app.simulation_manager),
        Arc::clone(&app.need_redraw),
        Arc::clone(&app.skip_redraw),
        app.gpu_particle_sync.clone(),
    );
    event_loop.run_app(&mut app)
}

/// Spawns a background thread that advances simulation state and schedules redraws.
pub(crate) fn spawn_simulation_worker(
    ui_state_clone: Arc<RwLock<UiState>>,
    simulation_manager: Arc<RwLock<SimulationManager>>,
    need_redraw: Arc<RwLock<bool>>,
    skip_redraw: Arc<RwLock<u32>>,
    gpu_particle_sync: GpuParticleSync,
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
            {
                let ui_state = ui_state_clone.read().unwrap();
                let is_reset_requested = ui_state.is_reset_requested;
                let is_add_particles_requested = ui_state.is_add_particles_requested;
                if is_reset_requested || is_add_particles_requested {
                    let selected_object_input = ui_state.object_input.clone();
                    let simulation_type = ui_state.simulation_type;
                    let skip = ui_state.skip;
                    let add_particle_count = ui_state.add_particle_count;
                    let scale = ui_state.scale;
                    let base_scale = ui_state.base_scale;
                    let add_center = ui_state.add_center;
                    let max_particle_count = ui_state.max_particle_count;
                    let uses_gpu = ui_state.uses_gpu_simulation();
                    let reset_repopulates = ui_state.reset_repopulates_particles();
                    let reset_object_input = ui_state.build_reset_object_input();
                    let placement_mode = ui_state.placement_mode;
                    let reset_log_abort = Arc::clone(&ui_state.reset_log.abort_requested);
                    drop(ui_state);
                    if is_reset_requested {
                        let mut reset_applied = false;
                        if reset_repopulates && placement_mode == PlacementMode::SolarSystem {
                            if let ObjectInput::SolarSystem {
                                scale,
                                start_year,
                                start_month,
                                start_day,
                                start_hour,
                            } = reset_object_input
                            {
                                let ui_state_for_log = Arc::clone(&ui_state_clone);
                                let need_redraw_for_log = Arc::clone(&need_redraw);
                                let log = move |line: &str| {
                                    {
                                        let mut ui_state = ui_state_for_log.write().unwrap();
                                        ui_state.append_reset_log(line);
                                    }
                                    need_redraw_for_log.write().unwrap().clone_from(&true);
                                };
                                match build_solar_system_particles(
                                    scale,
                                    start_year,
                                    start_month,
                                    start_day,
                                    start_hour,
                                    &log,
                                    reset_log_abort.as_ref(),
                                ) {
                                    Ok(particles) => {
                                        simulation_manager.read().unwrap().reset_from_particles(
                                            particles,
                                            simulation_type,
                                            base_scale,
                                        );
                                        reset_applied = true;
                                    }
                                    Err(SolarSystemBuildError::Aborted) => {
                                        let mut ui_state = ui_state_clone.write().unwrap();
                                        ui_state.append_reset_log("Aborted.");
                                        ui_state.finish_reset_log();
                                        ui_state.is_reset_requested = false;
                                        drop(ui_state);
                                        need_redraw.write().unwrap().clone_from(&true);
                                        continue;
                                    }
                                }
                            }
                        } else if reset_repopulates {
                            simulation_manager.read().unwrap().reset(
                                reset_object_input,
                                simulation_type,
                                add_particle_count,
                                base_scale,
                            );
                            reset_applied = true;
                        } else {
                            simulation_manager
                                .read()
                                .unwrap()
                                .clear(simulation_type, scale);
                            reset_applied = true;
                        }
                        let mut ui_state = ui_state_clone.write().unwrap();
                        if reset_applied {
                            ui_state.frame = 1;
                            ui_state.simulation_time = 0.0;
                        }
                        ui_state.is_reset_requested = false;
                        if placement_mode == PlacementMode::SolarSystem {
                            ui_state.finish_reset_log();
                        }
                        drop(ui_state);
                        if reset_applied {
                            gpu_particle_sync.request_full_upload();
                        }
                        need_redraw.write().unwrap().clone_from(&true);
                        skip_redraw.write().unwrap().clone_from(&skip);
                        continue;
                    }
                    simulation_manager.write().unwrap().append_particles(
                        selected_object_input,
                        simulation_type,
                        add_particle_count,
                        scale,
                        add_center,
                        base_scale,
                        max_particle_count,
                    );
                    let mut ui_state = ui_state_clone.write().unwrap();
                    ui_state.is_add_particles_requested = false;
                    drop(ui_state);
                    if uses_gpu {
                        gpu_particle_sync.request_append_preserving();
                    } else {
                        gpu_particle_sync.request_cpu_mode_upload();
                    }
                    need_redraw.write().unwrap().clone_from(&true);
                    skip_redraw.write().unwrap().clone_from(&skip);
                    continue;
                }
            }
            if *need_redraw.read().unwrap() {
                std::thread::sleep(Duration::from_millis(16));
                continue;
            }
            let ui_state = ui_state_clone.read().unwrap();
            let is_running = ui_state.is_running;
            let max_fps = ui_state.max_fps;
            let max_fps_unlimited = ui_state.max_fps_unlimited;
            let time_per_frame = ui_state.time_per_frame;
            let skip = ui_state.skip;
            let uses_gpu = ui_state.uses_gpu_simulation();
            drop(ui_state);
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
            if !is_running {
                std::thread::sleep(Duration::from_millis(16));
                continue;
            }
            let particle_count = simulation_manager.read().unwrap().particle_count();
            if !UiState::can_start_simulation(particle_count) {
                std::thread::sleep(Duration::from_millis(16));
                continue;
            }
            let dt = now.duration_since(last_advance).as_secs_f64();
            if !max_fps_unlimited {
                let target_fps = max_fps as f64;
                if dt < 1.0 / target_fps {
                    continue;
                }
            }
            if uses_gpu {
                gpu_particle_sync.fetch_add_advance_step();
            } else {
                thread_pool.install(|| {
                    simulation_manager.read().unwrap().advance(time_per_frame);
                });
            }
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
    need_redraw: Arc<RwLock<bool>>,
    skip_redraw: Arc<RwLock<u32>>,
    gpu_particle_sync: GpuParticleSync,
    mouse_left_down: bool,
    mouse_right_down: bool,
    mouse_middle_down: bool,
    last_cursor_position: Option<(f64, f64)>,
    last_left_click_time: Option<Instant>,
    last_left_click_pos: Option<(f64, f64)>,
    last_right_click_time: Option<Instant>,
    last_right_click_pos: Option<(f64, f64)>,
    settings: AppSettings,
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
            // Ignore error: on shutdown we just want to be as clean as possible.
            // Real device loss from a bad submit would have been observable earlier too.
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
            simulation_manager: Arc::new(RwLock::new(SimulationManager::default())),
            need_redraw: Arc::new(RwLock::new(true)),
            skip_redraw: Arc::new(RwLock::new(0)),
            gpu_particle_sync: GpuParticleSync::new(true),
            mouse_left_down: false,
            mouse_right_down: false,
            mouse_middle_down: false,
            last_cursor_position: None,
            last_left_click_time: None,
            last_left_click_pos: None,
            last_right_click_time: None,
            last_right_click_pos: None,
            settings,
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

        let vulkan_base = VulkanBase::new(
            &window,
            self.settings.mailbox_present_mode,
            c"DualSpacetimeSimulator",
            vk::make_api_version(0, 0, 2, 0),
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

        let object_input = ObjectInput::default();
        let add_particle_count = ui_state.add_particle_count;
        let scale = ui_state.scale;
        let sim_type = ui_state.simulation_type;
        self.simulation_manager.write().unwrap().reset(
            object_input,
            sim_type,
            add_particle_count,
            scale,
        );
        self.skip_redraw.write().unwrap().clone_from(&ui_state.skip);
        self.gpu_particle_sync.clear_advance_steps();
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
            WindowEvent::KeyboardInput { event, .. } => {
                // Pause shortcut that stays reachable even when heavy draw-skipping
                // makes the egui controls hard to click.
                if event.state == ElementState::Pressed
                    && matches!(
                        event.physical_key,
                        PhysicalKey::Code(KeyCode::Escape) | PhysicalKey::Code(KeyCode::Pause)
                    )
                {
                    self.ui_state.write().unwrap().is_running = false;
                }
            }
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
                    draw_ui(
                        &self.ui_state,
                        &self.simulation_manager,
                        &mut self.settings,
                        &ctx,
                    );
                });
                let desired_mailbox_present_mode = {
                    let ui_state = self.ui_state.read().unwrap();
                    pipeline.sync_add_center_marker(&ui_state);
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
                let particle_display_mode = ui_state.particle_display_mode;
                let uses_gpu = ui_state.uses_gpu_simulation();
                let time_per_frame = ui_state.time_per_frame;
                let simulation_type = ui_state.simulation_type;
                let sim_scale = ui_state.scale;
                drop(ui_state);

                let pending_steps = if uses_gpu {
                    self.gpu_particle_sync.take_advance_steps()
                } else {
                    0
                };
                if pending_steps > 0 {
                    pipeline.record_gpu_advance(
                        cb,
                        simulation_type,
                        time_per_frame,
                        sim_scale,
                        pending_steps,
                    );
                }

                pipeline.render(
                    cb,
                    image_index as usize,
                    vb.swapchain_extent,
                    gui,
                    scale,
                    link_point_size_to_scale,
                    show_grid,
                    particle_display_mode,
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
                if uses_gpu {
                    self.need_redraw.write().unwrap().clone_from(&false);
                }
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
                        MouseButton::Left => {
                            let is_scene_click =
                                matches!(self.drag_owner, DragOwner::PendingSceneLeft);
                            self.left_button(state);
                            if is_scene_click {
                                self.try_pick_particle();
                            }
                        }
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
            process_pending_snapshot_dialog(
                window,
                &self.ui_state,
                &self.simulation_manager,
                self.render_pipeline.as_ref(),
                &self.need_redraw,
            );
            window.request_redraw();
        }
        self.apply_pending_particle_buffer_reload();
        if let Some(pipeline) = self.render_pipeline.as_mut() {
            pipeline.update_animation();
        }
        if *self.need_redraw.read().unwrap() == false && !self.gpu_particle_sync.has_pending_sync()
        {
            return;
        }

        let uses_gpu = {
            let uis = self.ui_state.read().unwrap();
            let uses_gpu = uis.uses_gpu_simulation();
            if let Some(pipeline) = self.render_pipeline.as_mut() {
                pipeline.set_use_gpu_sim(uses_gpu);
            }
            uses_gpu
        };

        if self.gpu_particle_sync.take_upload_pending() {
            if let (Some(pipeline), Ok(manager)) = (
                self.render_pipeline.as_mut(),
                self.simulation_manager.try_read(),
            ) {
                pipeline.upload_particles(&manager.particles());
            }
        }

        // GPU-mode "Add": preserve simulated positions of existing particles by
        // reading back current GPU state before re-uploading the combined buffer.
        // Cleared only after a successful upload to avoid dropping the request.
        if self.gpu_particle_sync.append_pending() {
            let particles = self.simulation_manager.read().unwrap().particles();
            if let Some(pipeline) = self.render_pipeline.as_mut() {
                pipeline.add_particles_preserving_simulated(&particles);
            }
            self.gpu_particle_sync.clear_append_pending();
        }

        if uses_gpu {
            return;
        }

        if *self.need_redraw.read().unwrap() == false {
            return;
        }
        if let Ok(manager) = self.simulation_manager.try_read() {
            self.need_redraw.write().unwrap().clone_from(&false);
            if let Some(pipeline) = self.render_pipeline.as_mut() {
                pipeline.upload_particles(&manager.particles());
            }
        }
    }
}

impl App {
    /// Applies a snapshot-load request by scheduling a full GPU particle upload.
    fn apply_pending_particle_buffer_reload(&mut self) {
        let mut uis = self.ui_state.write().unwrap();
        let reload_requested = uis.take_particle_buffer_reload_requested();
        let uses_gpu = uis.uses_gpu_simulation();
        drop(uis);
        if reload_requested && uses_gpu {
            self.gpu_particle_sync.request_full_upload();
        }
    }

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

    /// Picks the particle closest to the last cursor position and stores it in UI state.
    ///
    /// Called on a left-button release that did not promote into a drag.
    /// Reads the most recent particle data from whichever simulation source
    /// (CPU manager or GPU buffer) the app is currently driving.
    fn try_pick_particle(&mut self) {
        let Some(click_pos) = self.last_cursor_position else {
            return;
        };
        let Some(vb) = self.vulkan_base.as_ref() else {
            return;
        };
        let Some(pipeline) = self.render_pipeline.as_ref() else {
            return;
        };
        let extent = vb.swapchain_extent;
        if extent.width == 0 || extent.height == 0 {
            return;
        }

        let (uses_gpu, scale_gauge) = {
            let uis = self.ui_state.read().unwrap();
            (uis.uses_gpu_simulation(), uis.scale_gauge)
        };

        let particles = if uses_gpu {
            pipeline.readback_particles()
        } else {
            self.simulation_manager.read().unwrap().particles()
        };

        if particles.is_empty() {
            return;
        }

        let click_x = click_pos.0 as f32;
        let click_y = click_pos.1 as f32;
        if let Some(idx) =
            pipeline.pick_nearest_particle(&particles, click_x, click_y, extent, scale_gauge)
        {
            let particle = particles[idx];
            let mut uis = self.ui_state.write().unwrap();
            uis.select_particle(idx, particle);
            drop(uis);
            self.need_redraw.write().unwrap().clone_from(&true);
        }
    }
}
