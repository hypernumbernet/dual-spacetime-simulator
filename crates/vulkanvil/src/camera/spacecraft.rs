use super::orbit::OrbitCamera;
use glam::{Quat, Vec3};
use std::f32::EPSILON;

pub const THRUST_DURATION: f32 = 0.4;
pub const THRUST_ACCEL: f32 = 0.5;
pub const MOUSE_RIGHT_DRAG_SENS: f32 = 0.001;
/// Roll/pitch rate (rad/s) per pixel of offset from the steer anchor.
pub const STEER_RATE_PER_PX: f32 = 0.005;
/// Below this speed, mouse steering zeros velocity and turns view in place.
pub const VELOCITY_STEER_THRESHOLD: f32 = 0.08;

const MAX_TICK_DT: f32 = 0.1;
const VELOCITY_STEER_THRESHOLD_SQ: f32 = VELOCITY_STEER_THRESHOLD * VELOCITY_STEER_THRESHOLD;

/// Clears spacecraft velocity and active thrust.
pub fn reset_spacecraft_motion(camera: &mut OrbitCamera) {
    camera.velocity = Vec3::ZERO;
    camera.thrust_remaining = 0.0;
    camera.thrust_sign = 0.0;
}

/// Starts or restarts a timed forward/backward thrust from mouse wheel input.
pub fn apply_spacecraft_wheel_thrust(camera: &mut OrbitCamera, direction: f32) {
    if direction.abs() <= EPSILON {
        return;
    }
    camera.thrust_sign = direction.signum();
    camera.thrust_remaining = THRUST_DURATION;
}

/// Integrates thrust, velocity, and inertial translation for spacecraft mode.
pub fn tick_spacecraft_camera(camera: &mut OrbitCamera, dt: f32) {
    let dt = dt.min(MAX_TICK_DT);
    if dt <= EPSILON {
        return;
    }

    let forward = camera.view_relative().normalize_or_zero();
    if forward == Vec3::ZERO {
        return;
    }

    if camera.thrust_remaining > 0.0 {
        let thrust_dt = dt.min(camera.thrust_remaining);
        camera.velocity += forward * camera.thrust_sign * THRUST_ACCEL * thrust_dt;
        camera.thrust_remaining -= thrust_dt;
    }

    let displacement = camera.velocity * dt;
    if displacement.length_squared() > EPSILON {
        camera.position += displacement;
        camera.target += displacement;
    }
}

/// Rotates velocity and view together, or turns in place when speed is negligible.
fn steer_spacecraft(camera: &mut OrbitCamera, axis: Vec3, angle: f32) {
    if axis.length_squared() <= EPSILON || angle.abs() <= EPSILON {
        return;
    }
    let axis = axis.normalize();
    let rotation = Quat::from_axis_angle(axis, angle);

    if camera.thrust_remaining <= 0.0
        && camera.velocity.length_squared() <= VELOCITY_STEER_THRESHOLD_SQ
    {
        camera.velocity = Vec3::ZERO;
    } else if camera.velocity.length_squared() > EPSILON {
        camera.velocity = rotation.mul_vec3(camera.velocity);
    }

    let relative = camera.view_relative();
    if relative.length_squared() <= EPSILON {
        return;
    }
    let position = camera.position;
    camera.target = position + rotation.mul_vec3(relative);
    camera.up = rotation.mul_vec3(camera.up);
}

/// Applies roll around the view axis and pitch around the view-right axis.
pub fn apply_spacecraft_roll_pitch(camera: &mut OrbitCamera, roll: f32, pitch: f32) {
    if roll.abs() <= EPSILON && pitch.abs() <= EPSILON {
        return;
    }

    let relative = camera.view_relative();
    if relative.length_squared() <= EPSILON {
        return;
    }

    let forward = relative.normalize();
    let mut up = camera.up;

    if roll.abs() > EPSILON {
        up = Quat::from_axis_angle(forward, roll).mul_vec3(up);
        camera.up = up;
    }
    if pitch.abs() > EPSILON {
        let mut right = forward.cross(up);
        if right.length_squared() <= EPSILON {
            return;
        }
        right = right.normalize();
        steer_spacecraft(camera, right, pitch);
    }
}

/// Applies roll/pitch from offset (pixels) from a screen-fixed steer anchor.
pub fn apply_spacecraft_steer_from_offset(
    camera: &mut OrbitCamera,
    offset_x: f32,
    offset_y: f32,
    dt: f32,
) {
    if dt <= EPSILON || (offset_x.abs() <= EPSILON && offset_y.abs() <= EPSILON) {
        return;
    }
    let roll = -offset_x * STEER_RATE_PER_PX * dt;
    let pitch = -offset_y * STEER_RATE_PER_PX * dt;
    apply_spacecraft_roll_pitch(camera, roll, pitch);
}

/// Screen-space offset from a steer anchor to the cursor, in pixels.
pub fn spacecraft_steer_offset(
    anchor: Option<[f64; 2]>,
    cursor: Option<(f64, f64)>,
) -> Option<(f32, f32)> {
    let [ax, ay] = anchor?;
    let (cx, cy) = cursor?;
    Some(((cx - ax) as f32, (cy - ay) as f32))
}

/// Toggles steer-anchor presence at `pos`.
pub fn toggle_spacecraft_steer_anchor(anchor: &mut Option<[f64; 2]>, pos: (f64, f64)) {
    if anchor.is_some() {
        *anchor = None;
    } else {
        *anchor = Some([pos.0, pos.1]);
    }
}

/// Applies anchor steering then integrates spacecraft motion for one frame.
pub fn tick_spacecraft_steer_and_motion(
    camera: &mut OrbitCamera,
    steer_offset: Option<(f32, f32)>,
    dt: f32,
) {
    if let Some((offset_x, offset_y)) = steer_offset {
        apply_spacecraft_steer_from_offset(camera, offset_x, offset_y, dt);
    }
    tick_spacecraft_camera(camera, dt);
}

/// Returns whether mouse wheel should affect the 3D scene camera.
pub fn spacecraft_scene_wheel_allowed(
    lock_camera_up: bool,
    steer_anchor_active: bool,
    ui_blocks_pointer: bool,
) -> bool {
    if lock_camera_up {
        !ui_blocks_pointer
    } else {
        steer_anchor_active || !ui_blocks_pointer
    }
}

/// Steers velocity yaw from horizontal drag; vertical movement is ignored.
pub fn apply_spacecraft_mouse_right(camera: &mut OrbitCamera, dx: f32) {
    let yaw = -dx * MOUSE_RIGHT_DRAG_SENS;
    if yaw.abs() <= EPSILON {
        return;
    }

    let relative = camera.view_relative();
    if relative.length_squared() <= EPSILON {
        return;
    }

    steer_spacecraft(camera, camera.up, yaw);
}
