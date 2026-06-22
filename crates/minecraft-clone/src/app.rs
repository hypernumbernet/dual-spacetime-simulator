//! winit application: window, Vulkan, renderer, world streaming, and the frame loop.

use crate::chunk::chunk_of_pos;
use crate::input::InputState;
use crate::mesher::SUN_DIR;
use crate::player::Player;
use crate::renderer::{PushConstants, Renderer, SkyPushConstants};
use crate::world::World;
use ash::vk;
use glam::IVec3;
use std::sync::Arc;
use std::time::Instant;
use vulkanvil::VulkanBase;
use winit::application::ApplicationHandler;
use winit::dpi::LogicalSize;
use winit::event::{DeviceEvent, DeviceId, ElementState, MouseButton, WindowEvent};
use winit::event_loop::ActiveEventLoop;
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{CursorGrabMode, Window};

const MAX_DT: f32 = 1.0 / 30.0;
/// Render-distance step per PageUp/PageDown press (chunks).
const RD_STEP: i32 = 5;
/// HUD refresh interval (seconds); also the FPS averaging window.
const HUD_INTERVAL: f32 = 0.25;

/// Formats a byte count as mebibytes for the HUD.
fn fmt_mb(bytes: u64) -> String {
    format!("{:.1} MB", bytes as f64 / (1024.0 * 1024.0))
}

/// Recreates the swapchain + size-dependent resources, unless the window has no
/// drawable area (minimized). Creating a 0×0 swapchain is invalid and the resulting
/// unwrap panic aborts in release (panic = "abort" → STATUS_STACK_BUFFER_OVERRUN);
/// on restore, the Resized event triggers the recreation instead.
fn recreate_if_drawable(window: &Window, vb: &mut VulkanBase, renderer: &mut Renderer) {
    let size = window.inner_size();
    if size.width == 0 || size.height == 0 {
        return;
    }
    vb.recreate_swapchain(window);
    renderer.recreate_size_dependent(vb);
}

pub struct App {
    // Drop order: renderer before vulkan_base before window.
    renderer: Option<Renderer>,
    vulkan_base: Option<VulkanBase>,
    window: Option<Arc<Window>>,
    world: World,
    player: Player,
    input: InputState,
    last_frame: Option<Instant>,
    start_time: Instant,
    cursor_grabbed: bool,
    hud_visible: bool,
    hud_dirty: bool,
    hud_acc: f32,
    hud_frames: u32,
    fps: f32,
}

impl Default for App {
    fn default() -> Self {
        Self {
            renderer: None,
            vulkan_base: None,
            window: None,
            world: World::new(),
            player: Player::spawn(),
            input: InputState::default(),
            last_frame: None,
            start_time: Instant::now(),
            cursor_grabbed: false,
            hud_visible: true,
            hud_dirty: true,
            hud_acc: 0.0,
            hud_frames: 0,
            fps: 0.0,
        }
    }
}

impl Drop for App {
    /// Waits for all GPU work to finish before tearing down resources, avoiding
    /// ERROR_DEVICE_LOST (same rationale as the dual-spacetime-simulator crate).
    fn drop(&mut self) {
        if let Some(vb) = &self.vulkan_base {
            let _ = unsafe { vb.device.device_wait_idle() };
        }
    }
}

impl App {
    fn set_cursor_grab(&mut self, grab: bool) {
        let Some(window) = &self.window else { return };
        if grab {
            let _ = window.set_cursor_grab(CursorGrabMode::Confined);
            window.set_cursor_visible(false);
        } else {
            let _ = window.set_cursor_grab(CursorGrabMode::None);
            window.set_cursor_visible(true);
        }
        self.cursor_grabbed = grab;
    }

    fn frame(&mut self) {
        let (Some(window), Some(vb), Some(renderer)) = (
            self.window.as_ref(),
            self.vulkan_base.as_mut(),
            self.renderer.as_mut(),
        ) else {
            return;
        };

        // Minimized: no drawable surface — skip rendering (and any swapchain
        // recreation) until the window is restored.
        {
            let size = window.inner_size();
            if size.width == 0 || size.height == 0 {
                return;
            }
        }

        let now = Instant::now();
        let raw_dt = self
            .last_frame
            .map(|t| (now - t).as_secs_f32())
            .unwrap_or(0.0);
        let dt = raw_dt.min(MAX_DT);
        self.last_frame = Some(now);

        if self.cursor_grabbed {
            let (dx, dy) = self.input.mouse_delta;
            self.player.apply_mouse(dx, dy);
        }
        self.player.update(&self.input, &self.world, dt);

        // Developer controls: F3 toggles the HUD, PageUp/PageDown resize the streamed ring.
        if self.input.just_pressed(KeyCode::F3) {
            self.hud_visible = !self.hud_visible;
            if self.hud_visible {
                self.hud_dirty = true;
            } else {
                renderer.update_hud("");
            }
        }
        if self.input.just_pressed(KeyCode::PageUp) {
            self.world
                .set_render_distance(self.world.render_distance() + RD_STEP);
            self.hud_dirty = true;
        }
        if self.input.just_pressed(KeyCode::PageDown) {
            self.world
                .set_render_distance(self.world.render_distance() - RD_STEP);
            self.hud_dirty = true;
        }
        self.input.end_frame();

        let player_chunk = chunk_of_pos(self.player.pos);
        self.world.update_streaming(player_chunk, renderer);

        // HUD refresh on a fixed interval (the stat walks are O(loaded chunks)).
        self.hud_acc += raw_dt;
        self.hud_frames += 1;
        let fps_due = self.hud_acc >= HUD_INTERVAL;
        if fps_due {
            self.fps = self.hud_frames as f32 / self.hud_acc;
            self.hud_acc = 0.0;
            self.hud_frames = 0;
        }
        if self.hud_visible && (fps_due || self.hud_dirty) {
            let ws = self.world.stats();
            let rs = renderer.stats();
            let report = vb
                .allocator
                .as_ref()
                .unwrap()
                .lock()
                .unwrap()
                .generate_report();
            let pos = self.player.pos;
            let text = format!(
                "fps {:.1} ({:.2} ms)\n\
                 pos {:.1} {:.1} {:.1}  chunk {} {}\n\
                 render distance {} (PgUp/PgDn {:+}, F3 hud)\n\
                 chunks: {} loaded, {} meshed, {} with blocks\n\
                 queues: gen {}, mesh {}\n\
                 cpu chunk blocks: {}\n\
                 gpu chunk meshes: {} ({})\n\
                 gpu allocator: {} used / {} reserved",
                self.fps,
                if self.fps > 0.0 {
                    1000.0 / self.fps
                } else {
                    0.0
                },
                pos.x,
                pos.y,
                pos.z,
                player_chunk.x,
                player_chunk.y,
                self.world.render_distance(),
                RD_STEP,
                ws.loaded,
                ws.meshed,
                ws.resident_blocks,
                ws.pending_gen,
                ws.pending_mesh,
                fmt_mb(ws.block_bytes as u64),
                rs.chunk_meshes,
                fmt_mb(rs.mesh_bytes),
                fmt_mb(report.total_allocated_bytes),
                fmt_mb(report.total_reserved_bytes),
            );
            renderer.update_hud(&text);
            self.hud_dirty = false;
        }

        // Render.
        vb.wait_for_fence();
        renderer.collect_garbage();

        let image_index = match vb.acquire_next_image() {
            Ok((idx, _)) => idx,
            Err(vk::Result::ERROR_OUT_OF_DATE_KHR) => {
                recreate_if_drawable(window, vb, renderer);
                return;
            }
            Err(e) => panic!("Failed to acquire swapchain image: {e:?}"),
        };

        vb.reset_fence();

        let cb = vb.current_command_buffer();
        let begin = vk::CommandBufferBeginInfo::default()
            .flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT);
        unsafe {
            vb.device
                .reset_command_buffer(cb, vk::CommandBufferResetFlags::empty())
                .unwrap();
            vb.device.begin_command_buffer(cb, &begin).unwrap();
        }

        let aspect = vb.swapchain_extent.width as f32 / vb.swapchain_extent.height.max(1) as f32;
        let view_proj = self.player.view_proj(aspect).to_cols_array_2d();
        let eye = self.player.eye();
        let time = self.start_time.elapsed().as_secs_f32();
        let underwater = self
            .world
            .block(IVec3::new(
                eye.x.floor() as i32,
                eye.y.floor() as i32,
                eye.z.floor() as i32,
            ))
            .is_water();
        let pc = PushConstants::new(
            view_proj,
            [eye.x, eye.y, eye.z],
            time,
            underwater,
            self.world.render_distance(),
        );

        let fwd = self.player.forward();
        let right = self.player.right();
        let up = self.player.up_basis();
        let sun = SUN_DIR.normalize();
        let tan_half_fov = (70.0_f32.to_radians() * 0.5).tan();
        let sky = SkyPushConstants {
            cam_right: [right.x, right.y, right.z, 0.0],
            cam_up: [up.x, up.y, up.z, 0.0],
            cam_fwd: [fwd.x, fwd.y, fwd.z, 0.0],
            sun_dir: [sun.x, sun.y, sun.z, 0.0],
            params: [
                tan_half_fov,
                aspect,
                if underwater { 1.0 } else { 0.0 },
                0.0,
            ],
        };

        renderer.record(cb, image_index as usize, vb.swapchain_extent, &pc, &sky);

        unsafe { vb.device.end_command_buffer(cb).unwrap() };

        match vb.submit_and_present(image_index) {
            Ok(true) | Err(vk::Result::ERROR_OUT_OF_DATE_KHR) => {
                recreate_if_drawable(window, vb, renderer);
            }
            Ok(false) => {}
            Err(e) => panic!("Failed to present: {e:?}"),
        }

        vb.advance_frame();
        renderer.end_frame();
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }
        let attrs = Window::default_attributes()
            .with_title("minecraft-clone")
            .with_inner_size(LogicalSize::new(1280.0, 800.0));
        let window = Arc::new(event_loop.create_window(attrs).unwrap());

        let vb = VulkanBase::new(
            &window,
            false,
            c"MinecraftClone",
            vk::make_api_version(0, 0, 1, 0),
        );
        let renderer = Renderer::new(&vb);

        self.window = Some(window);
        self.vulkan_base = Some(vb);
        self.renderer = Some(renderer);
        self.set_cursor_grab(true);
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _id: winit::window::WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(_) | WindowEvent::ScaleFactorChanged { .. } => {
                if let (Some(window), Some(vb), Some(renderer)) = (
                    self.window.as_ref(),
                    self.vulkan_base.as_mut(),
                    self.renderer.as_mut(),
                ) {
                    recreate_if_drawable(window, vb, renderer);
                }
            }
            WindowEvent::KeyboardInput { event, .. } => {
                if let PhysicalKey::Code(code) = event.physical_key {
                    if code == KeyCode::Escape && event.state == ElementState::Pressed {
                        self.set_cursor_grab(false);
                    }
                    self.input.key_event(code, event.state);
                }
            }
            WindowEvent::MouseInput { state, button, .. } => {
                if button == MouseButton::Left
                    && state == ElementState::Pressed
                    && !self.cursor_grabbed
                {
                    self.set_cursor_grab(true);
                }
            }
            WindowEvent::Focused(false) => self.set_cursor_grab(false),
            WindowEvent::RedrawRequested => self.frame(),
            _ => {}
        }
    }

    fn device_event(&mut self, _event_loop: &ActiveEventLoop, _id: DeviceId, event: DeviceEvent) {
        if let DeviceEvent::MouseMotion { delta } = event {
            if self.cursor_grabbed {
                self.input.mouse_delta.0 += delta.0;
                self.input.mouse_delta.1 += delta.1;
            }
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}
