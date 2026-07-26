//! winit application: window, Vulkan, sim step, keyboard control, render loop.

use crate::control::{ControlMapper, KeySnapshot};
use crate::integration::Gui;
use crate::landing::LandingAutopilot;
use crate::mesh::{GRASS_METERS_PER_TILE, hud_text, random_target_xz};
use crate::target_landing::TargetLandingAutopilot;
use crate::renderer::{
    MIN_CAMERA_HEIGHT, MOON_SKY_COLOR, Renderer, SKY_COLOR, camera_view_proj, min_orbit_pitch,
    orbit_camera_far, orbit_eye_offset,
};
use crate::sim::{ControlCommand, RocketState, step_rocket};
use crate::ui::{ContentRegion, draw_params_panel};
use ash::vk;
use glam::Vec3;
use std::ffi::CString;
use std::sync::Arc;
use std::time::{Duration, Instant};
use vulkanvil::{InputState, OrbitCamera, VulkanBase, get_closest_perp_unit_to_y};
use winit::application::ApplicationHandler;
use winit::dpi::LogicalSize;
use winit::event::{DeviceEvent, DeviceId, ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::Window;

const MAX_DT: f32 = 1.0 / 30.0;
const FIXED_DT: f64 = 1.0 / 120.0;
const MOUSE_ORBIT_SENS: f32 = 0.005;
/// Render/update rate cap (also paired with FIFO present / vsync).
const TARGET_FPS: u32 = 60;
const FRAME_PERIOD: Duration = Duration::from_nanos(1_000_000_000 / TARGET_FPS as u64);
const CAM_DISTANCE_MIN: f32 = 20.0;
const CAM_DISTANCE_MAX: f32 = 400.0;
const CAM_PITCH_MAX: f32 = 1.2;

fn initial_orbit_camera(target: Vec3) -> OrbitCamera {
    let eye = target + orbit_eye_offset(0.8, 0.35, 80.0);
    let mut cam = OrbitCamera::new(eye, target);
    cam.set_lock_up(true);
    cam
}

/// Yaw / pitch / distance HUD values from the current orbit pose.
fn orbit_hud_angles(cam: &OrbitCamera) -> (f32, f32, f32) {
    let offset = cam.position - cam.target;
    let distance = offset.length().max(1e-6);
    let pitch = (offset.y / distance).asin();
    let yaw = offset.z.atan2(offset.x);
    (yaw, pitch, distance)
}

fn clamp_orbit_distance(cam: &mut OrbitCamera) {
    let distance = cam.orbit_distance().clamp(CAM_DISTANCE_MIN, CAM_DISTANCE_MAX);
    let dir = cam.view_relative().normalize_or_zero();
    if dir != Vec3::ZERO {
        cam.position = cam.target - dir * distance;
    }
}

fn clamp_orbit_above_ground(cam: &mut OrbitCamera) {
    let distance = cam.orbit_distance().max(1e-3);
    let pitch_floor = min_orbit_pitch(cam.target.y, distance, MIN_CAMERA_HEIGHT);
    let offset = cam.position - cam.target;
    let horiz = (offset.x * offset.x + offset.z * offset.z).sqrt().max(1e-6);
    let pitch = offset.y.atan2(horiz);
    let pitch = pitch.clamp(pitch_floor, CAM_PITCH_MAX);
    let yaw = offset.z.atan2(offset.x);
    cam.position = cam.target + orbit_eye_offset(yaw, pitch, distance);
    if cam.position.y < MIN_CAMERA_HEIGHT {
        cam.position.y = MIN_CAMERA_HEIGHT;
    }
    cam.up = get_closest_perp_unit_to_y(cam.position, cam.target);
}

pub struct App {
    /// Dropped before `renderer` / `vulkan_base` so egui Vulkan resources release cleanly.
    gui: Option<Gui>,
    renderer: Option<Renderer>,
    vulkan_base: Option<VulkanBase>,
    window: Option<Arc<Window>>,
    rocket: RocketState,
    control: ControlMapper,
    landing: LandingAutopilot,
    target_landing: TargetLandingAutopilot,
    input: InputState,
    last_frame: Option<Instant>,
    accum: f64,
    camera: OrbitCamera,
    fps: f32,
    fps_acc: f32,
    fps_frames: u32,
    needs_resize: bool,
    /// True while LMB or RMB is held — camera only follows the cursor during a drag.
    mouse_dragging: bool,
    /// Accumulated mouse motion applied only while `mouse_dragging` (no cursor grab).
    drag_delta: (f64, f64),
    /// Scroll-wheel zoom for this frame (positive = zoom in).
    scroll_zoom: f32,
    /// Earliest time the next rendered frame may start (60 FPS pacing).
    next_frame_at: Instant,
    /// Random T-mark target XZ — uniform annulus 100–8000 m from launch
    /// ([`crate::mesh::TARGET_DISTANCE_MIN_M`], [`crate::mesh::TARGET_DISTANCE_MAX_M`]).
    target_xz: [f32; 2],
}

impl Default for App {
    fn default() -> Self {
        let rocket = RocketState::resting_on_pad();
        let p = rocket.position();
        let target = Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32);
        Self {
            gui: None,
            renderer: None,
            vulkan_base: None,
            window: None,
            rocket,
            control: ControlMapper::default(),
            landing: LandingAutopilot::default(),
            target_landing: TargetLandingAutopilot::default(),
            input: InputState::default(),
            last_frame: None,
            accum: 0.0,
            camera: initial_orbit_camera(target),
            fps: 0.0,
            fps_acc: 0.0,
            fps_frames: 0,
            needs_resize: false,
            mouse_dragging: false,
            drag_delta: (0.0, 0.0),
            scroll_zoom: 0.0,
            next_frame_at: Instant::now(),
            target_xz: random_target_xz(),
        }
    }
}

impl Drop for App {
    fn drop(&mut self) {
        // Idle the device before tearing down Gui / Renderer / VulkanBase.
        if let Some(vb) = &self.vulkan_base {
            let _ = unsafe { vb.device.device_wait_idle() };
        }
        self.gui = None;
        self.renderer = None;
        self.vulkan_base = None;
    }
}

impl App {
    fn keys_from_input(&self) -> KeySnapshot {
        KeySnapshot {
            thrust_up: self.input.held(KeyCode::Space),
            // Ctrl only: hold to lower. C is a one-shot cut latch (see thrust_cut).
            thrust_down: self.input.held(KeyCode::ControlLeft)
                || self.input.held(KeyCode::ControlRight),
            thrust_full: self.input.just_pressed(KeyCode::KeyF),
            thrust_cut: self.input.just_pressed(KeyCode::KeyC),
            pitch_up: self.input.held(KeyCode::KeyW),
            pitch_down: self.input.held(KeyCode::KeyS),
            // A/D: roll, Q/E: yaw (swapped from classic A/D yaw layout).
            yaw_left: self.input.held(KeyCode::KeyQ),
            yaw_right: self.input.held(KeyCode::KeyE),
            roll_left: self.input.held(KeyCode::KeyA),
            roll_right: self.input.held(KeyCode::KeyD),
            reset: self.input.just_pressed(KeyCode::KeyR),
            toggle_landing: self.input.just_pressed(KeyCode::KeyL),
            toggle_target_landing: self.input.just_pressed(KeyCode::KeyT),
            toggle_moon_mode: self.input.just_pressed(KeyCode::KeyM),
            random_load_test: self.input.just_pressed(KeyCode::KeyY),
        }
    }

    fn frame(&mut self) {
        let Some(window) = self.window.clone() else {
            return;
        };
        let size = window.inner_size();
        if size.width == 0 || size.height == 0 {
            return;
        }

        let now = Instant::now();
        // Soft cap: skip work if we were woken earlier than the 60 FPS budget.
        if now < self.next_frame_at {
            return;
        }
        // Schedule from the ideal tick so small overruns don't permanently lag.
        self.next_frame_at = (self.next_frame_at + FRAME_PERIOD).max(now + FRAME_PERIOD / 2);

        let raw_dt = self
            .last_frame
            .map(|t| (now - t).as_secs_f32())
            .unwrap_or(FRAME_PERIOD.as_secs_f32());
        let dt = raw_dt.min(MAX_DT);
        self.last_frame = Some(now);

        self.fps_acc += raw_dt;
        self.fps_frames += 1;
        if self.fps_acc >= 0.25 {
            self.fps = self.fps_frames as f32 / self.fps_acc;
            self.fps_acc = 0.0;
            self.fps_frames = 0;
        }

        // Orbit only while a mouse button is held and dragged (cursor is never confined).
        let (mdx, mdy) = self.drag_delta;
        self.drag_delta = (0.0, 0.0);
        let cam_yaw_rate = self.input.axis(KeyCode::ArrowRight, KeyCode::ArrowLeft);
        let cam_pitch_rate = self.input.axis(KeyCode::ArrowUp, KeyCode::ArrowDown);
        let page_up = self.input.held(KeyCode::PageUp);
        let page_down = self.input.held(KeyCode::PageDown);
        let keys = self.keys_from_input();

        // vulkanvil OrbitCamera: quaternion revolve around target (same as dst-graph3d).
        // Mouse pitch is negated: DeviceEvent::MouseMotion Y is opposite the
        // cursor-space (y-down) deltas that dual-spacetime / dst-graph3d pass to revolve.
        let mut dyaw = cam_yaw_rate * dt * 1.2;
        let mut dpitch = cam_pitch_rate * dt * 1.0;
        if self.mouse_dragging {
            dyaw += mdx as f32 * MOUSE_ORBIT_SENS;
            dpitch += -mdy as f32 * MOUSE_ORBIT_SENS;
        }
        if dyaw != 0.0 || dpitch != 0.0 {
            self.camera.revolve(dyaw, dpitch);
        }
        if page_up {
            self.camera.zoom(40.0 * dt);
        }
        if page_down {
            self.camera.zoom(-40.0 * dt);
        }
        if self.scroll_zoom != 0.0 {
            self.camera.zoom(self.scroll_zoom * 8.0);
            self.scroll_zoom = 0.0;
        }
        clamp_orbit_distance(&mut self.camera);

        if keys.toggle_landing {
            self.landing.toggle();
            if self.landing.enabled {
                self.target_landing.disable();
            }
        }
        if keys.toggle_target_landing {
            self.target_landing.toggle();
            if self.target_landing.enabled {
                self.landing.disable();
            }
        }
        if (self.landing.enabled || self.target_landing.enabled) && keys.manual_control_active() {
            self.landing.disable();
            self.target_landing.disable();
        }
        if keys.toggle_moon_mode {
            self.rocket.moon_mode = !self.rocket.moon_mode;
        }

        if keys.reset {
            let moon_mode = self.rocket.moon_mode;
            self.rocket = RocketState::resting_on_pad();
            self.rocket.moon_mode = moon_mode;
            self.control = ControlMapper::default();
            self.landing.disable();
            self.target_landing.disable();
            self.target_xz = random_target_xz();
            let p = self.rocket.position();
            self.camera = initial_orbit_camera(Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32));
            if let Some(renderer) = self.renderer.as_mut() {
                renderer.set_target_xz(self.target_xz);
            }
        }

        if keys.random_load_test {
            let moon_mode = self.rocket.moon_mode;
            self.rocket = RocketState::random_load_test();
            self.rocket.moon_mode = moon_mode;
            self.control = ControlMapper::default();
            self.landing.disable();
            self.target_landing.enable();
        }

        let target_xz_f64 = [self.target_xz[0] as f64, self.target_xz[1] as f64];
        let using_autopilot = !self.rocket.destroyed
            && (self.target_landing.enabled || self.landing.enabled);
        let cmd = if self.rocket.destroyed {
            ControlCommand::default()
        } else if self.target_landing.enabled {
            self.target_landing
                .update(&self.rocket, target_xz_f64, dt as f64)
        } else if self.landing.enabled {
            self.landing.update(&self.rocket, dt as f64)
        } else {
            self.control.apply(&keys, dt as f64)
        };
        self.rocket.set_command(cmd);
        // Keep manual throttle in lockstep with L/T so exit resumes at the same level.
        if using_autopilot {
            self.control.adopt_throttle(cmd.throttle);
        }

        self.accum += dt as f64;
        while self.accum >= FIXED_DT {
            step_rocket(&mut self.rocket, FIXED_DT);
            self.accum -= FIXED_DT;
        }

        let pos = self.rocket.position();
        let target = Vec3::new(pos[0] as f32, pos[1] as f32, pos[2] as f32);
        // Keep orbit focus on the rocket CoM without changing relative view.
        let follow = target - self.camera.target;
        self.camera.target = target;
        self.camera.position += follow;
        clamp_orbit_above_ground(&mut self.camera);
        // Frustum + viewport share ContentRegion so the look-at sits in the
        // middle of the area to the right of the left panel.
        let content = ContentRegion::from_framebuffer(
            size.width as f32,
            size.height as f32,
            window.scale_factor() as f32,
        );
        let eye = self.camera.position;
        let far = orbit_camera_far(eye.y);
        let vp = camera_view_proj(eye, target, content.aspect, far);
        // Snap ground origin to the grass tile grid under the rocket so tiling stays stable.
        let tile = GRASS_METERS_PER_TILE;
        let ground_xz = [
            (pos[0] as f32 / tile).round() * tile,
            (pos[2] as f32 / tile).round() * tile,
        ];
        let hud = hud_text(&self.rocket, &self.landing, &self.target_landing, self.fps);
        let title = hud.lines().next().unwrap_or("PGA Rocket").to_string();
        window.set_title(&title);

        let needs_resize = self.needs_resize;
        let fps = self.fps;
        let (cam_yaw, cam_pitch, cam_distance) = orbit_hud_angles(&self.camera);
        let target_xz = self.target_xz;

        let (Some(vb), Some(renderer), Some(gui)) = (
            self.vulkan_base.as_mut(),
            self.renderer.as_mut(),
            self.gui.as_mut(),
        ) else {
            return;
        };

        if needs_resize {
            vb.recreate_swapchain(&window);
            renderer.recreate_size_dependent(vb);
        }
        self.needs_resize = false;

        // Build egui panel before recording the command buffer.
        gui.immediate_ui(&window, |gui| {
            let ctx = gui.context();
            draw_params_panel(
                &ctx,
                &mut self.rocket,
                &self.landing,
                &self.target_landing,
                fps,
                cam_yaw,
                cam_pitch,
                cam_distance,
                target_xz,
            );
        });
        gui.prepare_frame(&window);

        renderer.sync_rocket(&self.rocket, eye);
        renderer.set_hud(hud);

        let moon_mode = self.rocket.moon_mode;
        let sky_color = if moon_mode { MOON_SKY_COLOR } else { SKY_COLOR };
        let draw_result = renderer.draw(
            vb,
            vp,
            eye,
            ground_xz,
            target_xz,
            sky_color,
            content.left_inset_px,
            moon_mode,
            gui,
        );
        // Free egui textures even when the swapchain is out of date.
        gui.finish_frame();
        match draw_result {
            Ok(()) => {}
            Err(vk::Result::ERROR_OUT_OF_DATE_KHR) => {
                self.needs_resize = true;
            }
            Err(e) => {
                eprintln!("draw error: {e:?}");
            }
        }

        self.input.end_frame();
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }
        let attrs = Window::default_attributes()
            .with_title(
                "PGA Rocket — Space/Ctrl hold, F full, C cut, WASD/QE attitude, L land, T target, R reset",
            )
            .with_inner_size(LogicalSize::new(1280.0, 720.0));
        let window = Arc::new(event_loop.create_window(attrs).expect("create window"));

        let app_name = CString::new("pga-rocket").unwrap();
        // FIFO (vsync) present mode — mailbox=false avoids uncapped multi-thousand FPS.
        let vb = VulkanBase::new(
            &window,
            false,
            &app_name,
            vk::make_api_version(0, 0, 1, 0),
        );
        let mut renderer = Renderer::new(&vb);
        renderer.set_target_xz(self.target_xz);
        // No camera yet; the eye only sorts explosion FX, which is empty at startup.
        renderer.sync_rocket(&self.rocket, Vec3::new(0.0, 30.0, 80.0));

        let gui = Gui::new(
            event_loop,
            &window,
            &vb.instance,
            vb.physical_device,
            vb.device.clone(),
            vb.graphics_queue,
            vb.command_pool,
            renderer.render_pass(),
            vb.swapchain_format,
        );

        let now = Instant::now();
        self.window = Some(window);
        self.gui = Some(gui);
        self.vulkan_base = Some(vb);
        self.renderer = Some(renderer);
        self.last_frame = Some(now);
        self.next_frame_at = now;
        event_loop.set_control_flow(ControlFlow::WaitUntil(now + FRAME_PERIOD));
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _id: winit::window::WindowId,
        event: WindowEvent,
    ) {
        // Forward events to egui first so it can capture pointer/scroll over the panel.
        if let (Some(window), Some(gui)) = (self.window.as_ref(), self.gui.as_mut()) {
            let _ = gui.update(window, &event);
        }
        let ui_wants_pointer = self
            .gui
            .as_ref()
            .is_some_and(|g| g.pointer_wants_input());

        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(_) => {
                self.needs_resize = true;
            }
            WindowEvent::KeyboardInput { event, .. } => {
                if let PhysicalKey::Code(code) = event.physical_key {
                    if code == KeyCode::Escape && event.state == ElementState::Pressed {
                        event_loop.exit();
                        return;
                    }
                    self.input.key_event(code, event.state);
                }
            }
            WindowEvent::MouseInput { state, button, .. } => {
                // Left or right button drag orbits the camera; no cursor grab.
                // Skip orbit start when the pointer is over the egui panel.
                let is_orbit_button =
                    matches!(button, MouseButton::Left | MouseButton::Right);
                if is_orbit_button {
                    match state {
                        ElementState::Pressed => {
                            if !ui_wants_pointer {
                                self.mouse_dragging = true;
                                self.drag_delta = (0.0, 0.0);
                            }
                        }
                        ElementState::Released => {
                            self.mouse_dragging = false;
                            self.drag_delta = (0.0, 0.0);
                        }
                    }
                }
            }
            WindowEvent::MouseWheel { delta, .. } => {
                if ui_wants_pointer {
                    // Let egui ScrollArea handle the wheel over the panel.
                    return;
                }
                let steps = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y,
                    MouseScrollDelta::PixelDelta(p) => (p.y as f32) * 0.05,
                };
                self.scroll_zoom += steps;
            }
            WindowEvent::RedrawRequested => self.frame(),
            _ => {}
        }
    }

    fn device_event(
        &mut self,
        _event_loop: &ActiveEventLoop,
        _device_id: DeviceId,
        event: DeviceEvent,
    ) {
        // Raw motion is only used while a drag is active (see mouse_dragging).
        if let DeviceEvent::MouseMotion { delta } = event
            && self.mouse_dragging
        {
            self.drag_delta.0 += delta.0;
            self.drag_delta.1 += delta.1;
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        let now = Instant::now();
        if now < self.next_frame_at {
            // Sleep until the next 60 FPS slot instead of spinning.
            event_loop.set_control_flow(ControlFlow::WaitUntil(self.next_frame_at));
            return;
        }
        event_loop.set_control_flow(ControlFlow::WaitUntil(now + FRAME_PERIOD));
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}
