mod orbit;

pub use orbit::OrbitCamera;

use crate::input::InputState;
use glam::Vec3;
use winit::keyboard::KeyCode;

pub const KEYBOARD_PAN_SPEED: f32 = 0.006;
pub const KEYBOARD_ORBIT_YAW_SPEED: f32 = 0.03;
pub const WHEEL_FORWARD_SPEED: f32 = 0.03;

/// Applies WASD pan, Q/E yaw, Space/Shift vertical move, and arrow-key target controls
/// when `lock_camera_up` is enabled and keyboard input is not blocked.
///
/// Returns `true` when any camera motion was applied.
pub fn apply_orbit_keyboard(
    camera: &mut OrbitCamera,
    input: &InputState,
    lock_camera_up: bool,
    keyboard_blocked: bool,
) -> bool {
    if !lock_camera_up || keyboard_blocked {
        return false;
    }

    let mut forward = 0.0f32;
    let mut right = 0.0f32;
    let mut yaw = 0.0f32;
    let mut vertical = 0.0f32;
    let mut target_vertical = 0.0f32;
    let mut target_horizontal = 0.0f32;

    if input.held(KeyCode::KeyW) {
        forward += 1.0;
    }
    if input.held(KeyCode::KeyS) {
        forward -= 1.0;
    }
    if input.held(KeyCode::KeyD) {
        right += 1.0;
    }
    if input.held(KeyCode::KeyA) {
        right -= 1.0;
    }
    if input.held(KeyCode::KeyQ) {
        yaw -= 1.0;
    }
    if input.held(KeyCode::KeyE) {
        yaw += 1.0;
    }
    if input.held(KeyCode::Space) {
        vertical -= 1.0;
    }
    if input.held(KeyCode::ShiftLeft) || input.held(KeyCode::ShiftRight) {
        vertical += 1.0;
    }
    if input.held(KeyCode::ArrowUp) {
        target_vertical -= 1.0;
    }
    if input.held(KeyCode::ArrowDown) {
        target_vertical += 1.0;
    }
    if input.held(KeyCode::ArrowLeft) {
        target_horizontal += 1.0;
    }
    if input.held(KeyCode::ArrowRight) {
        target_horizontal -= 1.0;
    }

    if forward == 0.0
        && right == 0.0
        && yaw == 0.0
        && vertical == 0.0
        && target_vertical == 0.0
        && target_horizontal == 0.0
    {
        return false;
    }

    let distance = (camera.target - camera.position).length();

    if forward != 0.0 || right != 0.0 {
        let view = (camera.target - camera.position).normalize_or_zero();
        let mut forward_xz = Vec3::new(view.x, 0.0, view.z);
        if forward_xz.length_squared() <= f32::EPSILON {
            forward_xz = Vec3::NEG_Z;
        } else {
            forward_xz = forward_xz.normalize();
        }
        let right_xz = forward_xz.cross(Vec3::Y).normalize();
        let speed = distance * KEYBOARD_PAN_SPEED;
        let offset = (forward_xz * forward + right_xz * right) * speed;
        camera.pan_xz(offset);
    }
    if yaw != 0.0 {
        camera.orbit_yaw(yaw * KEYBOARD_ORBIT_YAW_SPEED);
    }
    if vertical != 0.0 {
        let speed = distance * KEYBOARD_PAN_SPEED;
        camera.move_position_y(vertical * speed);
    }
    if target_vertical != 0.0 {
        let speed = distance * KEYBOARD_PAN_SPEED;
        camera.move_target_y(target_vertical * speed);
    }
    if target_horizontal != 0.0 {
        camera.move_target_around_position_y(target_horizontal * KEYBOARD_ORBIT_YAW_SPEED);
    }

    true
}

/// Moves the camera along the view direction by a wheel delta scaled to orbit distance.
pub fn apply_wheel_forward(camera: &mut OrbitCamera, forward: f32) {
    if forward.abs() <= f32::EPSILON {
        return;
    }
    let distance = (camera.target - camera.position).length();
    let speed = distance * WHEEL_FORWARD_SPEED;
    camera.move_forward(forward * speed);
}
