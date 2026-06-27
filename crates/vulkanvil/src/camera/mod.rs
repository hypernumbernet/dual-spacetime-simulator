mod orbit;
mod spacecraft;
mod trace;

pub use orbit::OrbitCamera;
pub use trace::trace_particle_from_behind;
pub use spacecraft::{
    apply_spacecraft_keyboard, apply_spacecraft_roll_pitch, apply_spacecraft_steer_from_offset,
    apply_spacecraft_wheel_thrust, apply_spacecraft_yaw_from_offset, reset_spacecraft_motion,
    spacecraft_scene_wheel_allowed, spacecraft_steer_inputs, spacecraft_steer_offset,
    tick_spacecraft_camera, tick_spacecraft_steer_and_motion,
    tick_spacecraft_steer_and_motion_from_anchors, toggle_spacecraft_steer_anchor,
    KEYBOARD_STEER_EQUIV_PX, KEYBOARD_STEER_RATE, STEER_RATE_PER_PX, THRUST_ACCEL, THRUST_DURATION,
    VELOCITY_STEER_THRESHOLD,
};

use crate::input::InputState;
use glam::Vec3;
use winit::keyboard::KeyCode;

pub const MIN_TRACE_FOLLOW_DISTANCE: f32 = 0.1;
pub const MAX_TRACE_FOLLOW_DISTANCE: f32 = 100.0;
pub const KEYBOARD_PAN_SPEED: f32 = 0.006;
pub const KEYBOARD_ORBIT_YAW_SPEED: f32 = 0.03;
pub const WHEEL_FORWARD_SPEED: f32 = 0.03;
pub const WHEEL_TRACE_DISTANCE_SPEED: f32 = 0.1;
pub const KEYBOARD_TRACE_DISTANCE_SPEED: f32 = 0.006;

/// Normalized keyboard axis values for orbit camera controls.
#[derive(Clone, Copy, Default)]
struct OrbitKeyboardAxes {
    forward: f32,
    right: f32,
    yaw: f32,
    vertical: f32,
    target_vertical: f32,
    target_horizontal: f32,
}

impl OrbitKeyboardAxes {
    fn is_zero(self) -> bool {
        self.forward == 0.0
            && self.right == 0.0
            && self.yaw == 0.0
            && self.vertical == 0.0
            && self.target_vertical == 0.0
            && self.target_horizontal == 0.0
    }
}

fn gather_orbit_keyboard_axes(input: &InputState, trace_active: bool) -> OrbitKeyboardAxes {
    OrbitKeyboardAxes {
        forward: input.axis(KeyCode::KeyW, KeyCode::KeyS),
        right: input.axis(KeyCode::KeyD, KeyCode::KeyA),
        yaw: input.axis(KeyCode::KeyE, KeyCode::KeyQ),
        vertical: input.space_shift_vertical_axis(trace_active),
        target_vertical: input.axis(KeyCode::ArrowDown, KeyCode::ArrowUp),
        target_horizontal: input.axis(KeyCode::ArrowLeft, KeyCode::ArrowRight),
    }
}

/// Advances camera animations and applies keyboard orbit controls.
pub fn tick_orbit_camera(
    camera: &mut OrbitCamera,
    input: &InputState,
    lock_camera_up: bool,
    keyboard_blocked: bool,
    suppress_space_shift: bool,
) {
    camera.update_animation();
    apply_orbit_keyboard(
        camera,
        input,
        lock_camera_up,
        keyboard_blocked,
        suppress_space_shift,
    );
}

/// Applies WASD pan, Q/E yaw, Space/Shift vertical move, and arrow-key target controls
/// when `lock_camera_up` is enabled and keyboard input is not blocked.
///
/// When `suppress_space_shift` is true (Trace On), Space and Shift are ignored.
///
/// Returns `true` when any camera motion was applied.
pub fn apply_orbit_keyboard(
    camera: &mut OrbitCamera,
    input: &InputState,
    lock_camera_up: bool,
    keyboard_blocked: bool,
    suppress_space_shift: bool,
) -> bool {
    if !lock_camera_up || keyboard_blocked {
        return false;
    }

    let axes = gather_orbit_keyboard_axes(input, suppress_space_shift);
    if axes.is_zero() {
        return false;
    }

    let relative = camera.view_relative();
    let distance = relative.length();
    let pan_speed = distance * KEYBOARD_PAN_SPEED;

    if axes.forward != 0.0 || axes.right != 0.0 {
        let mut forward_xz = Vec3::new(relative.x, 0.0, relative.z);
        if forward_xz.length_squared() <= f32::EPSILON {
            forward_xz = Vec3::NEG_Z;
        } else {
            forward_xz = forward_xz.normalize();
        }
        let right_xz = forward_xz.cross(Vec3::Y);
        if suppress_space_shift {
            if axes.forward != 0.0 {
                apply_trace_follow_distance_delta(
                    camera,
                    -axes.forward * distance * KEYBOARD_TRACE_DISTANCE_SPEED,
                );
            }
            if axes.right != 0.0 {
                let offset = right_xz * axes.right * pan_speed;
                camera.pan_xz(offset);
            }
        } else {
            let offset = (forward_xz * axes.forward + right_xz * axes.right) * pan_speed;
            camera.pan_xz(offset);
        }
    }
    if axes.yaw != 0.0 {
        camera.orbit_yaw(axes.yaw * KEYBOARD_ORBIT_YAW_SPEED);
    }
    if axes.vertical != 0.0 {
        camera.move_position_y(axes.vertical * pan_speed);
    }
    if axes.target_vertical != 0.0 {
        camera.move_target_y(axes.target_vertical * pan_speed);
    }
    if axes.target_horizontal != 0.0 {
        camera.move_target_around_position_y(axes.target_horizontal * KEYBOARD_ORBIT_YAW_SPEED);
    }

    true
}

/// Routes mouse-wheel input to orbit zoom, spacecraft thrust, or trace follow distance.
pub fn apply_camera_mouse_wheel(
    camera: &mut OrbitCamera,
    lock_camera_up: bool,
    scroll_y: f32,
    trace_active: bool,
) {
    if trace_active {
        if scroll_y.abs() <= f32::EPSILON {
            return;
        }
        let distance = camera.trace_follow_distance_or_default();
        apply_trace_follow_distance_delta(
            camera,
            -scroll_y * distance * WHEEL_TRACE_DISTANCE_SPEED,
        );
    } else if lock_camera_up {
        apply_wheel_forward(camera, scroll_y);
    } else {
        apply_spacecraft_wheel_thrust(camera, scroll_y);
    }
}

/// Adjusts the trace follow distance when Trace On is active.
pub fn apply_trace_follow_distance_delta(camera: &mut OrbitCamera, delta: f32) {
    camera.adjust_trace_follow_distance(delta);
}

/// Moves the camera along the view direction by a wheel delta scaled to orbit distance.
pub fn apply_wheel_forward(camera: &mut OrbitCamera, forward: f32) {
    if forward.abs() <= f32::EPSILON {
        return;
    }
    camera.move_forward(forward * camera.orbit_distance() * WHEEL_FORWARD_SPEED);
}
