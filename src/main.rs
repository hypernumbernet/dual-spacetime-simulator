#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod camera;
mod graph3d;
mod initial_condition;
mod integration;
mod math;
mod pipeline;
mod settings;
mod simulation;
mod tree;
mod ui;
mod ui_state;
mod ui_styles;
mod vulkan_base;

use crate::initial_condition::InitialCondition;
use crate::integration::Gui;
use crate::pipeline::ParticleRenderPipeline;
use crate::settings::AppSettings;
use crate::simulation::SimulationManager;
use crate::tree::Tree;
use crate::ui::draw_ui;
use crate::ui_state::{AppMode, GpuTreeComputeMode, GpuTreeLayout, GpuTreeRenderMode, UiState};
use crate::vulkan_base::VulkanBase;
use ash::vk;
use glam::Vec3;
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

fn main() -> Result<(), EventLoopError> {
    let event_loop = EventLoop::new()?;
    let mut app = App::default();
    let ui_state_clone = Arc::clone(&app.ui_state);
    let simulation_manager_clone = Arc::clone(&app.simulation_manager);
    let simulation_manager_for_reset = Arc::clone(&simulation_manager_clone);
    let need_redraw = Arc::clone(&app.need_redraw);
    let skip_redraw = Arc::clone(&app.skip_redraw);
    let mut last_advance = Instant::now();
    let mut last_fps = Instant::now();
    let mut prev_frame: i64 = 1;
    let thread_pool = rayon::ThreadPoolBuilder::new()
        .num_threads(num_cpus::get())
        .build()
        .unwrap();
    std::thread::spawn(move || {
        let simulation_manager = simulation_manager_for_reset;
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
    event_loop.run_app(&mut app)
}

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
}

impl Default for App {
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
        }
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let ui_state = self.ui_state.write().unwrap();

        let window_attrs = Window::default_attributes()
            .with_title(generate_window_title())
            .with_min_inner_size(winit::dpi::LogicalSize::new(
                ui_state.min_window_width,
                ui_state.min_window_height,
            ));
        let window = Arc::new(event_loop.create_window(window_attrs).unwrap());

        if self.settings.start_maximized {
            window.set_maximized(true);
        }

        let vulkan_base = VulkanBase::new(&window);
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
        if !gui.update(&window_clone, &event) {
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
                            let window_size = window.inner_size();
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
        if app_mode == AppMode::GpuTree {
            let uis = self.ui_state.read().unwrap();
            let fp = uis.gpu_tree_fingerprint();
            let layout = uis.gpu_tree_layout;
            let render_mode = uis.gpu_tree_render_mode;
            let compute_mode = uis.gpu_tree_compute_mode;
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
                            if compute_mode == GpuTreeComputeMode::GPU {
                                if let Some(pipeline) = self.render_pipeline.as_mut() {
                                    pipeline.compute_tree_vertices(params, layout);
                                }
                            } else {
                                let tube_verts = match layout {
                                    GpuTreeLayout::Single => {
                                        let tree = Tree::generate(params);
                                        tree.generate_tube_vertices_at(Vec3::ZERO)
                                    }
                                    GpuTreeLayout::ForestOnGrid => {
                                        Tree::generate_forest_tube_vertices_on_axis_xz_grid(params)
                                    }
                                };
                                if let Some(pipeline) = self.render_pipeline.as_mut() {
                                    pipeline.set_tree_vertices(tube_verts);
                                }
                            }
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
