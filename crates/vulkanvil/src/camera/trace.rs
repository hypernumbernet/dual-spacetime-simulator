use glam::Vec3;
use std::f32::EPSILON;

use super::orbit::{OrbitCamera, clamp_pitch, get_closest_perp_unit_to_y};

const MIN_TRACE_VELOCITY_SQ: f32 = 1e-12;

fn trace_forward(camera: &OrbitCamera, particle_velocity: Vec3) -> Vec3 {
    if particle_velocity.length_squared() > MIN_TRACE_VELOCITY_SQ {
        return particle_velocity.normalize();
    }
    camera.view_relative().normalize_or_zero()
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
/// Lock-up mode preserves orbit distance and pitch limits; free mode also syncs
/// spacecraft velocity and clears active thrust.
pub fn trace_particle_from_behind(
    camera: &mut OrbitCamera,
    particle_position: Vec3,
    particle_velocity: Vec3,
) {
    let distance = camera.orbit_distance().max(0.1);
    let forward = trace_forward(camera, particle_velocity);
    if forward == Vec3::ZERO {
        return;
    }

    camera.target = particle_position;
    let mut relative = forward * distance;

    if camera.lock_up() {
        relative = clamp_pitch(relative);
        camera.position = camera.target - relative;
        camera.up = get_closest_perp_unit_to_y(camera.position, camera.target);
    } else {
        camera.position = camera.target - relative;
        camera.up = up_perpendicular_to_forward(forward, camera.up);
        camera.velocity = particle_velocity;
        camera.thrust_remaining = 0.0;
    }
}
