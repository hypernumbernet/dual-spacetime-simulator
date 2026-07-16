//! winit application: window, Vulkan, sim step, keyboard control, render loop.

use crate::control::{ControlMapper, KeySnapshot};
use crate::mesh::{GRASS_METERS_PER_TILE, hud_text};
use crate::renderer::{Renderer, SKY_COLOR, camera_view_proj};
use crate::sim::{RocketState, step_rocket};
use ash::vk;
use glam::Vec3;
use std::ffi::CString;
use std::sync::Arc;
use std::time::{Duration, Instant};
use vulkanvil::{InputState, VulkanBase};
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

pub struct App {
    renderer: Option<Renderer>,
    vulkan_base: Option<VulkanBase>,
    window: Option<Arc<Window>>,
    rocket: RocketState,
    control: ControlMapper,
    input: InputState,
    last_frame: Option<Instant>,
    accum: f64,
    cam_yaw: f32,
    cam_pitch: f32,
    cam_distance: f32,
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
}

impl Default for App {
    fn default() -> Self {
        Self {
            renderer: None,
            vulkan_base: None,
            window: None,
            rocket: RocketState::resting_on_pad(),
            control: ControlMapper::default(),
            input: InputState::default(),
            last_frame: None,
            accum: 0.0,
            cam_yaw: 0.8,
            cam_pitch: 0.35,
            cam_distance: 80.0,
            fps: 0.0,
            fps_acc: 0.0,
            fps_frames: 0,
            needs_resize: false,
            mouse_dragging: false,
            drag_delta: (0.0, 0.0),
            scroll_zoom: 0.0,
            next_frame_at: Instant::now(),
        }
    }
}

impl Drop for App {
    fn drop(&mut self) {
        if let Some(vb) = &self.vulkan_base {
            let _ = unsafe { vb.device.device_wait_idle() };
        }
    }
}

impl App {
    fn keys_from_input(&self) -> KeySnapshot {
        KeySnapshot {
            thrust_up: self.input.held(KeyCode::Space),
            thrust_down: self.input.held(KeyCode::ControlLeft)
                || self.input.held(KeyCode::ControlRight)
                || self.input.held(KeyCode::KeyC),
            pitch_up: self.input.held(KeyCode::KeyW),
            pitch_down: self.input.held(KeyCode::KeyS),
            yaw_left: self.input.held(KeyCode::KeyA),
            yaw_right: self.input.held(KeyCode::KeyD),
            roll_left: self.input.held(KeyCode::KeyQ),
            roll_right: self.input.held(KeyCode::KeyE),
            reset: self.input.just_pressed(KeyCode::KeyR),
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

        self.cam_yaw += cam_yaw_rate * dt * 1.2;
        self.cam_pitch = (self.cam_pitch + cam_pitch_rate * dt * 1.0).clamp(-1.2, 1.2);
        if page_up {
            self.cam_distance = (self.cam_distance - 40.0 * dt).max(20.0);
        }
        if page_down {
            self.cam_distance = (self.cam_distance + 40.0 * dt).min(400.0);
        }
        if self.mouse_dragging {
            self.cam_yaw += mdx as f32 * MOUSE_ORBIT_SENS;
            self.cam_pitch =
                (self.cam_pitch + mdy as f32 * MOUSE_ORBIT_SENS).clamp(-1.2, 1.2);
        }
        if self.scroll_zoom != 0.0 {
            self.cam_distance =
                (self.cam_distance - self.scroll_zoom * 8.0).clamp(20.0, 400.0);
            self.scroll_zoom = 0.0;
        }

        if keys.reset {
            self.rocket = RocketState::resting_on_pad();
            self.control = ControlMapper::default();
        }
        let cmd = self.control.apply(&keys, dt as f64);
        self.rocket.set_command(cmd);

        self.accum += dt as f64;
        while self.accum >= FIXED_DT {
            step_rocket(&mut self.rocket, FIXED_DT);
            self.accum -= FIXED_DT;
        }

        let pos = self.rocket.position();
        let target = Vec3::new(pos[0] as f32, pos[1] as f32, pos[2] as f32);
        let aspect = size.width as f32 / size.height.max(1) as f32;
        let (vp, eye) = camera_view_proj(
            target,
            self.cam_yaw,
            self.cam_pitch,
            self.cam_distance,
            aspect,
        );
        // Snap ground origin to the grass tile grid under the rocket so tiling stays stable.
        let tile = GRASS_METERS_PER_TILE;
        let ground_xz = [
            (pos[0] as f32 / tile).round() * tile,
            (pos[2] as f32 / tile).round() * tile,
        ];
        let hud = hud_text(&self.rocket, self.fps);
        let title = hud.lines().next().unwrap_or("PGA Rocket").to_string();
        window.set_title(&title);

        let needs_resize = self.needs_resize;
        let (Some(vb), Some(renderer)) = (self.vulkan_base.as_mut(), self.renderer.as_mut()) else {
            return;
        };

        if needs_resize {
            vb.recreate_swapchain(&window);
            renderer.recreate_size_dependent(vb);
        }
        self.needs_resize = false;

        renderer.sync_rocket(&self.rocket);
        renderer.set_hud(hud);

        match renderer.draw(vb, vp, eye, ground_xz, SKY_COLOR) {
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
            .with_title("PGA Rocket — Space/Ctrl throttle, WASD/QE attitude, R reset")
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
        renderer.sync_rocket(&self.rocket);

        let now = Instant::now();
        self.window = Some(window);
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
                let is_orbit_button =
                    matches!(button, MouseButton::Left | MouseButton::Right);
                if is_orbit_button {
                    match state {
                        ElementState::Pressed => {
                            self.mouse_dragging = true;
                            self.drag_delta = (0.0, 0.0);
                        }
                        ElementState::Released => {
                            self.mouse_dragging = false;
                            self.drag_delta = (0.0, 0.0);
                        }
                    }
                }
            }
            WindowEvent::MouseWheel { delta, .. } => {
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
