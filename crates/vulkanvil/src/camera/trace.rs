use glam::Vec3;
use std::f32::EPSILON;

use super::orbit::{OrbitCamera, clamp_pitch, get_closest_perp_unit_to_y};

const MIN_TRACE_VELOCITY_SQ: f32 = 1e-12;

fn trace_forward(camera: &OrbitCamera, particle_velocity: Vec3) -> Vec3 {
    if particle_velocity.length_squared() > MIN_TRACE_VELOCITY_SQ {
        particle_velocity.normalize()
    } else {
        camera.view_relative().normalize_or_zero()
    }
}

fn up_perpendicular_to_forward(forward: Vec3, prior_up: Vec3) -> Vec3 {
    if forward.length_squared() <= EPSILON {
        return prior_up.normalize_or_zero();
    }
    let forward = forward.normalize();
    let up = prior_up - forward * prior_up.dot(forward);
    if up.length_squared() > EPSILON {
        return up.normalize();
    }
    let mut right = forward.cross(Vec3::Y);
    if right.length_squared() <= EPSILON {
        right = forward.cross(Vec3::X);
    }
    right.normalize().cross(forward).normalize_or_zero()
}

/// Places the camera behind a moving particle, looking toward it along its velocity.
///
/// `particle_position` is in simulation world space; `visual_scale` maps it into the
/// same scaled space used by particle rendering. Lock-up mode preserves orbit distance
/// and pitch limits; free mode also syncs spacecraft velocity and clears active thrust.
pub fn trace_particle_from_behind(
    camera: &mut OrbitCamera,
    particle_position: Vec3,
    particle_velocity: Vec3,
    visual_scale: f32,
) {
    let forward = trace_forward(camera, particle_velocity);
    if forward == Vec3::ZERO {
        return;
    }

    let scaled_position = particle_position * visual_scale;
    let distance = camera.clamped_trace_follow_distance() * visual_scale;
    let relative = forward * distance;
    let lock_up = camera.lock_up();
    camera.target = scaled_position;
    camera.position = if lock_up {
        scaled_position - clamp_pitch(relative)
    } else {
        scaled_position - relative
    };

    if lock_up {
        camera.up = get_closest_perp_unit_to_y(camera.position, camera.target);
    } else {
        camera.up = up_perpendicular_to_forward(forward, camera.up);
        camera.velocity = particle_velocity;
        camera.thrust_remaining = 0.0;
    }
}
