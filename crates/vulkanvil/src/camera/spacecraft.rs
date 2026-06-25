use super::orbit::OrbitCamera;
use glam::{Quat, Vec3};
use std::f32::EPSILON;

pub const THRUST_DURATION: f32 = 0.4;
pub const THRUST_ACCEL: f32 = 2.0;
pub const MOUSE_LEFT_DRAG_SENS: f32 = 0.003;
pub const MOUSE_RIGHT_DRAG_SENS: f32 = 0.001;
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

    if camera.velocity.length_squared() <= VELOCITY_STEER_THRESHOLD_SQ {
        camera.velocity = Vec3::ZERO;
    } else {
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

/// Applies roll (horizontal drag) and steers velocity pitch (vertical drag).
pub fn apply_spacecraft_mouse_left(camera: &mut OrbitCamera, dx: f32, dy: f32) {
    let roll = -dx * MOUSE_LEFT_DRAG_SENS;
    let pitch = -dy * MOUSE_LEFT_DRAG_SENS;
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
