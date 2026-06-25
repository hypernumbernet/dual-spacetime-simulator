use glam::Vec3;
use std::thread;
use std::time::Duration;
use vulkanvil::OrbitCamera;

fn run_origin_center_animation(cam: &mut OrbitCamera, steps: u32) {
    cam.center_target_on_origin();
    for _ in 0..steps {
        thread::sleep(Duration::from_millis(10));
        cam.update_animation();
    }
}

#[test]
fn center_target_on_origin_lock_up_preserves_distance() {
    let pos = Vec3::new(4.0, 2.0, 6.0);
    let target = Vec3::new(1.5, -0.5, 2.0);
    let mut cam = OrbitCamera::new(pos, target);
    cam.set_lock_up(true);
    let initial_distance = (cam.target - cam.position).length();

    run_origin_center_animation(&mut cam, 60);

    let final_distance = (cam.target - cam.position).length();
    assert!(
        (initial_distance - final_distance).abs() < 1e-3,
        "distance changed: {initial_distance} -> {final_distance}"
    );
    assert!(
        cam.target.length() < 0.06,
        "target not at origin: {:?}",
        cam.target
    );
    let view_dir = (cam.target - cam.position).normalize();
    let goal_dir = (-cam.position).normalize();
    assert!(
        view_dir.dot(goal_dir) > 0.99,
        "view not aimed at origin: view={view_dir:?} goal={goal_dir:?}"
    );
}

#[test]
fn center_target_on_origin_lock_up_off_keeps_position_moves_target() {
    let pos = Vec3::new(4.0, 2.0, 6.0);
    let target = Vec3::new(1.5, -0.5, 2.0);
    let mut cam = OrbitCamera::new(pos, target);

    run_origin_center_animation(&mut cam, 30);

    assert!(
        (cam.position - pos).length() < 1e-3,
        "position moved: {:?}",
        cam.position
    );
    assert!(
        cam.target.length() < target.length(),
        "target did not move toward origin: {:?}",
        cam.target
    );
}

#[test]
fn initial_up_orthogonal_to_view_ray() {
    let pos = Vec3::new(1.6, -1.6, 3.0);
    let target = Vec3::ZERO;
    let cam = OrbitCamera::new(pos, target);
    let dir = (target - pos).normalize();
    let dot = cam.up.dot(dir).abs();
    assert!(dot < 1e-4, "up·dir = {dot}");
}

#[test]
fn move_forward_preserves_distance() {
    let mut cam = OrbitCamera::new(Vec3::new(1.0, 2.0, 5.0), Vec3::new(0.5, -1.0, 1.0));
    let before = (cam.target - cam.position).length();
    cam.move_forward(0.7);
    let after = (cam.target - cam.position).length();
    assert!((before - after).abs() < 1e-5);
}

#[test]
fn move_forward_moves_both() {
    let pos = Vec3::new(1.0, 2.0, 5.0);
    let target = Vec3::new(0.5, -1.0, 1.0);
    let mut cam = OrbitCamera::new(pos, target);
    let direction = (target - pos).normalize();
    let delta = 0.7;
    cam.move_forward(delta);
    let offset = direction * delta;
    assert!((cam.position - (pos + offset)).length() < 1e-5);
    assert!((cam.target - (target + offset)).length() < 1e-5);
}

#[test]
fn move_forward_zero_is_noop() {
    let pos = Vec3::new(2.0, 1.0, 3.0);
    let target = Vec3::ZERO;
    let mut cam = OrbitCamera::new(pos, target);
    let before_p = cam.position;
    let before_t = cam.target;
    cam.move_forward(0.0);
    assert!((cam.position - before_p).length() < 1e-5);
    assert!((cam.target - before_t).length() < 1e-5);
}

#[test]
fn zoom_clamps_distance() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    cam.zoom(1000.0);
    let d = (cam.target - cam.position).length();
    assert!((d - 0.1).abs() < 1e-3);
}

#[test]
fn revolve_zero_is_noop() {
    let pos = Vec3::new(2.0, 1.0, 3.0);
    let target = Vec3::ZERO;
    let mut cam = OrbitCamera::new(pos, target);
    let before_p = cam.position;
    let before_t = cam.target;
    let before_u = cam.up;
    cam.revolve(0.0, 0.0);
    assert!((cam.position - before_p).length() < 1e-5);
    assert!((cam.target - before_t).length() < 1e-5);
    assert!((cam.up - before_u).length() < 1e-5);
}

#[test]
fn rotate_does_nothing_when_lock_up() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 4.0), Vec3::ZERO);
    cam.set_lock_up(true);
    let up = cam.up;
    cam.rotate(0.5);
    assert!((cam.up - up).length() < 1e-5);
}

#[test]
fn pan_xz_preserves_distance() {
    let mut cam = OrbitCamera::new(Vec3::new(1.0, 2.0, 5.0), Vec3::new(0.5, -1.0, 1.0));
    let before = (cam.target - cam.position).length();
    cam.pan_xz(Vec3::new(3.0, 99.0, -2.0));
    let after = (cam.target - cam.position).length();
    assert!((before - after).abs() < 1e-5);
}

#[test]
fn pan_xz_changes_only_xz() {
    let mut cam = OrbitCamera::new(Vec3::new(1.0, 2.0, 5.0), Vec3::new(0.5, -1.0, 1.0));
    let before_y = (cam.position.y, cam.target.y);
    cam.pan_xz(Vec3::new(1.5, 4.0, -0.5));
    assert!((cam.position.y - before_y.0).abs() < 1e-5);
    assert!((cam.target.y - before_y.1).abs() < 1e-5);
    assert!((cam.position.x - 2.5).abs() < 1e-5);
    assert!((cam.position.z - 4.5).abs() < 1e-5);
}

#[test]
fn pan_xz_zero_is_noop() {
    let pos = Vec3::new(2.0, 1.0, 3.0);
    let target = Vec3::ZERO;
    let mut cam = OrbitCamera::new(pos, target);
    let before_p = cam.position;
    let before_t = cam.target;
    cam.pan_xz(Vec3::ZERO);
    assert!((cam.position - before_p).length() < 1e-5);
    assert!((cam.target - before_t).length() < 1e-5);
}

#[test]
fn orbit_yaw_keeps_target_fixed() {
    let target = Vec3::new(1.0, -0.5, 2.0);
    let mut cam = OrbitCamera::new(Vec3::new(4.0, 1.0, 0.0), target);
    cam.orbit_yaw(0.3);
    assert!((cam.target - target).length() < 1e-5);
}

#[test]
fn orbit_yaw_preserves_distance() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 2.0, 5.0), Vec3::ZERO);
    let before = (cam.target - cam.position).length();
    cam.orbit_yaw(0.4);
    let after = (cam.target - cam.position).length();
    assert!((before - after).abs() < 1e-5);
}

#[test]
fn move_position_y_keeps_target_fixed() {
    let target = Vec3::new(1.0, -0.5, 2.0);
    let mut cam = OrbitCamera::new(Vec3::new(4.0, 1.0, 0.0), target);
    cam.move_position_y(-0.7);
    assert!((cam.target - target).length() < 1e-5);
    assert!((cam.position.y - 0.3).abs() < 1e-5);
}

#[test]
fn move_position_y_zero_is_noop() {
    let pos = Vec3::new(2.0, 1.0, 3.0);
    let target = Vec3::ZERO;
    let mut cam = OrbitCamera::new(pos, target);
    let before_p = cam.position;
    cam.move_position_y(0.0);
    assert!((cam.position - before_p).length() < 1e-5);
}

#[test]
fn move_target_y_keeps_position_fixed() {
    let position = Vec3::new(4.0, 1.0, 0.0);
    let target = Vec3::new(1.0, -0.5, 2.0);
    let mut cam = OrbitCamera::new(position, target);
    cam.move_target_y(-0.7);
    assert!((cam.position - position).length() < 1e-5);
    assert!((cam.target.y - (-1.2)).abs() < 1e-5);
}

#[test]
fn move_target_y_zero_is_noop() {
    let pos = Vec3::new(2.0, 1.0, 3.0);
    let target = Vec3::ZERO;
    let mut cam = OrbitCamera::new(pos, target);
    let before_t = cam.target;
    cam.move_target_y(0.0);
    assert!((cam.target - before_t).length() < 1e-5);
}

#[test]
fn move_target_around_position_y_keeps_position_fixed() {
    let position = Vec3::new(4.0, 1.0, 0.0);
    let target = Vec3::new(1.0, -0.5, 2.0);
    let mut cam = OrbitCamera::new(position, target);
    cam.move_target_around_position_y(0.3);
    assert!((cam.position - position).length() < 1e-5);
}

#[test]
fn move_target_around_position_y_preserves_distance() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 2.0, 5.0), Vec3::ZERO);
    let before = (cam.target - cam.position).length();
    cam.move_target_around_position_y(0.4);
    let after = (cam.target - cam.position).length();
    assert!((before - after).abs() < 1e-5);
}

#[test]
fn move_target_around_position_y_zero_is_noop() {
    let pos = Vec3::new(2.0, 1.0, 3.0);
    let target = Vec3::ZERO;
    let mut cam = OrbitCamera::new(pos, target);
    let before_t = cam.target;
    cam.move_target_around_position_y(0.0);
    assert!((cam.target - before_t).length() < 1e-5);
}

use vulkanvil::{
    apply_spacecraft_mouse_left, apply_spacecraft_mouse_right, apply_spacecraft_wheel_thrust,
    reset_spacecraft_motion, tick_spacecraft_camera, THRUST_ACCEL, THRUST_DURATION,
    VELOCITY_STEER_THRESHOLD,
};

#[test]
fn spacecraft_mouse_left_keeps_position_changes_orientation() {
    let pos = Vec3::new(0.0, 0.0, 5.0);
    let target = Vec3::ZERO;
    let mut cam = OrbitCamera::new(pos, target);
    let before_up = cam.up;
    apply_spacecraft_mouse_left(&mut cam, 10.0, 10.0);
    assert!((cam.position - pos).length() < 1e-5);
    assert!((cam.target - target).length() > 1e-5 || !cam.up.abs_diff_eq(before_up, 1e-5));
}

#[test]
fn spacecraft_mouse_right_ignores_vertical_drag() {
    let pos = Vec3::new(0.0, 0.0, 5.0);
    let target = Vec3::ZERO;
    let mut cam = OrbitCamera::new(pos, target);
    let before_target = cam.target;
    let before_up = cam.up;
    apply_spacecraft_mouse_right(&mut cam, 0.0);
    assert!((cam.target - before_target).length() < 1e-5);
    assert!(cam.up.abs_diff_eq(before_up, 1e-5));

    apply_spacecraft_mouse_right(&mut cam, 20.0);
    assert!((cam.target - before_target).length() > 1e-5);
}

#[test]
fn spacecraft_wheel_thrust_builds_velocity_and_moves_camera() {
    let pos = Vec3::new(0.0, 0.0, 5.0);
    let target = Vec3::ZERO;
    let mut cam = OrbitCamera::new(pos, target);
    apply_spacecraft_wheel_thrust(&mut cam, 1.0);

    let dt = 0.05;
    tick_spacecraft_camera(&mut cam, dt);

    assert!(cam.velocity().length() > 0.0);
    let relative_before = target - pos;
    let relative_after = cam.target - cam.position;
    assert!((relative_before - relative_after).length() < 1e-5);
    assert!((cam.position - pos).length() > 0.0);
    assert!((cam.target - target).length() > 0.0);
}

#[test]
fn spacecraft_coasts_after_thrust_ends() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    apply_spacecraft_wheel_thrust(&mut cam, 1.0);
    let tick_dt = 0.05;
    let thrust_ticks = (THRUST_DURATION / tick_dt).ceil() as u32 + 1;
    for _ in 0..thrust_ticks {
        tick_spacecraft_camera(&mut cam, tick_dt);
    }
    let velocity_after_thrust = cam.velocity();
    assert!(velocity_after_thrust.length() > 0.0);

    let pos_after_thrust = cam.position;
    tick_spacecraft_camera(&mut cam, tick_dt);
    assert!((cam.velocity() - velocity_after_thrust).length() < 1e-5);
    assert!((cam.position - pos_after_thrust).length() > 0.0);
}

#[test]
fn reset_spacecraft_motion_clears_velocity() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    apply_spacecraft_wheel_thrust(&mut cam, 1.0);
    tick_spacecraft_camera(&mut cam, 0.1);
    assert!(cam.velocity().length() > 0.0);
    reset_spacecraft_motion(&mut cam);
    assert_eq!(cam.velocity(), Vec3::ZERO);
}

#[test]
fn spacecraft_thrust_applies_expected_acceleration() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    apply_spacecraft_wheel_thrust(&mut cam, 1.0);
    let dt = 0.1;
    tick_spacecraft_camera(&mut cam, dt);
    let forward = (cam.target - cam.position).normalize();
    let expected_speed = THRUST_ACCEL * dt;
    assert!((cam.velocity().length() - expected_speed).abs() < 1e-4);
    assert!((cam.velocity().normalize() - forward).length() < 1e-4);
}

#[test]
fn spacecraft_mouse_left_vertical_steers_velocity_when_moving() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    apply_spacecraft_wheel_thrust(&mut cam, 1.0);
    for _ in 0..5 {
        tick_spacecraft_camera(&mut cam, 0.1);
    }
    let velocity_before = cam.velocity();
    assert!(velocity_before.length() > VELOCITY_STEER_THRESHOLD);
    let position_before = cam.position;
    apply_spacecraft_mouse_left(&mut cam, 0.0, 50.0);
    assert!((cam.position - position_before).length() < 1e-5);
    assert!(cam.velocity().length() > 1e-5);
    assert!(
        (cam.velocity().length() - velocity_before.length()).abs() < 1e-4,
        "speed changed: {} -> {}",
        velocity_before.length(),
        cam.velocity().length()
    );
    assert!(
        (cam.velocity().normalize() - velocity_before.normalize()).length() > 1e-3,
        "velocity direction unchanged"
    );
}

#[test]
fn spacecraft_mouse_left_vertical_turns_in_place_when_slow() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    apply_spacecraft_wheel_thrust(&mut cam, 1.0);
    tick_spacecraft_camera(&mut cam, 0.01);
    assert!(cam.velocity().length() < VELOCITY_STEER_THRESHOLD);
    let forward_before = (cam.target - cam.position).normalize();
    apply_spacecraft_mouse_left(&mut cam, 0.0, 50.0);
    assert_eq!(cam.velocity(), Vec3::ZERO);
    let forward_after = (cam.target - cam.position).normalize();
    assert!(
        (forward_after - forward_before).length() > 1e-3,
        "view direction unchanged"
    );
}

#[test]
fn spacecraft_mouse_right_horizontal_steers_velocity_when_moving() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    apply_spacecraft_wheel_thrust(&mut cam, 1.0);
    for _ in 0..5 {
        tick_spacecraft_camera(&mut cam, 0.1);
    }
    let velocity_before = cam.velocity();
    assert!(velocity_before.length() > VELOCITY_STEER_THRESHOLD);
    let position_before = cam.position;
    apply_spacecraft_mouse_right(&mut cam, 50.0);
    assert!((cam.position - position_before).length() < 1e-5);
    assert!(
        (cam.velocity().length() - velocity_before.length()).abs() < 1e-4,
        "speed changed"
    );
    assert!(
        (cam.velocity().normalize() - velocity_before.normalize()).length() > 1e-3,
        "velocity direction unchanged"
    );
}

#[test]
fn spacecraft_mouse_right_horizontal_turns_in_place_when_slow() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    apply_spacecraft_wheel_thrust(&mut cam, 1.0);
    tick_spacecraft_camera(&mut cam, 0.01);
    assert!(cam.velocity().length() < VELOCITY_STEER_THRESHOLD);
    let forward_before = (cam.target - cam.position).normalize();
    apply_spacecraft_mouse_right(&mut cam, 50.0);
    assert_eq!(cam.velocity(), Vec3::ZERO);
    let forward_after = (cam.target - cam.position).normalize();
    assert!(
        (forward_after - forward_before).length() > 1e-3,
        "view direction unchanged"
    );
}
