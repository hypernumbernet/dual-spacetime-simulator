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
fn center_target_on_origin_lock_up_pinned_when_mode_toggles_mid_animation() {
    let pos = Vec3::new(4.0, 2.0, 6.0);
    let target = Vec3::new(1.5, -0.5, 2.0);
    let mut cam = OrbitCamera::new(pos, target);
    cam.set_lock_up(true);
    let reference_distance = cam.orbit_distance();

    cam.center_target_on_origin();
    for _ in 0..5 {
        thread::sleep(Duration::from_millis(10));
        cam.update_animation();
    }
    cam.set_lock_up(false);
    for _ in 0..55 {
        thread::sleep(Duration::from_millis(10));
        cam.update_animation();
    }

    let final_distance = cam.orbit_distance();
    assert!(
        (reference_distance - final_distance).abs() < 1e-3,
        "distance changed: {reference_distance} -> {final_distance}"
    );
    assert!(
        (cam.position - pos).length() > 0.1,
        "lock path should move position, not freeze it: {:?}",
        cam.position
    );
    assert!(
        cam.target.length() < 0.06,
        "target not at origin: {:?}",
        cam.target
    );
}

#[test]
fn center_target_on_origin_lock_up_off_then_lock_on_restores_reference_distance() {
    let pos = Vec3::new(4.0, 2.0, 6.0);
    let target = Vec3::new(1.5, -0.5, 2.0);
    let mut cam = OrbitCamera::new(pos, target);
    cam.set_lock_up(true);
    let reference_distance = cam.orbit_distance();
    cam.set_lock_up(false);

    run_origin_center_animation(&mut cam, 60);

    let view_dir_before = cam.view_relative().normalize();
    cam.set_lock_up(true);

    assert!(
        (cam.orbit_distance() - reference_distance).abs() < 1e-3,
        "distance not restored: reference={reference_distance} actual={}",
        cam.orbit_distance()
    );
    let view_dir_after = cam.view_relative().normalize();
    assert!(
        view_dir_after.dot(view_dir_before) > 0.99,
        "view direction changed: before={view_dir_before:?} after={view_dir_after:?}"
    );
}

#[test]
fn lock_mode_toggle_during_lock_home_animation_restores_invariants() {
    let pos = Vec3::new(4.0, 2.0, 6.0);
    let target = Vec3::new(1.5, -0.5, 2.0);
    let mut cam = OrbitCamera::new(pos, target);
    cam.set_lock_up(true);
    let reference_distance = cam.orbit_distance();

    cam.center_target_on_origin();
    for _ in 0..10 {
        thread::sleep(Duration::from_millis(10));
        cam.update_animation();
    }
    cam.set_lock_up(false);
    for _ in 0..10 {
        thread::sleep(Duration::from_millis(10));
        cam.update_animation();
    }
    cam.set_lock_up(true);

    assert!(
        (cam.orbit_distance() - reference_distance).abs() < 1e-3,
        "distance not restored after mode toggles: reference={reference_distance} actual={}",
        cam.orbit_distance()
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
    apply_spacecraft_keyboard, apply_spacecraft_roll_pitch, apply_spacecraft_steer_from_offset,
    apply_spacecraft_wheel_thrust, apply_spacecraft_yaw_from_offset, reset_spacecraft_motion,
    spacecraft_steer_inputs, tick_spacecraft_camera, tick_spacecraft_steer_and_motion_from_anchors,
    InputState, KEYBOARD_STEER_EQUIV_PX, THRUST_ACCEL, THRUST_DURATION, VELOCITY_STEER_THRESHOLD,
};
use winit::event::ElementState;
use winit::keyboard::KeyCode;

fn input_holding(keys: &[KeyCode]) -> InputState {
    let mut input = InputState::default();
    for &key in keys {
        input.key_event(key, ElementState::Pressed);
    }
    input
}

#[test]
fn spacecraft_motion_then_lock_on_preserves_reference_distance() {
    let pos = Vec3::new(0.0, 0.0, 5.0);
    let target = Vec3::ZERO;
    let mut cam = OrbitCamera::new(pos, target);
    cam.set_lock_up(true);
    let reference_distance = cam.orbit_distance();
    cam.set_lock_up(false);

    apply_spacecraft_wheel_thrust(&mut cam, 1.0);
    for _ in 0..5 {
        tick_spacecraft_camera(&mut cam, 0.1);
    }
    apply_spacecraft_steer_from_offset(&mut cam, 0.0, 50.0, 0.05);
    apply_spacecraft_yaw_from_offset(&mut cam, 30.0, 0.05);

    cam.set_lock_up(true);
    assert!(
        (cam.orbit_distance() - reference_distance).abs() < 1e-3,
        "distance changed: {reference_distance} -> {}",
        cam.orbit_distance()
    );
}

#[test]
fn spacecraft_roll_pitch_keeps_position_changes_orientation() {
    let pos = Vec3::new(0.0, 0.0, 5.0);
    let target = Vec3::ZERO;
    let mut cam = OrbitCamera::new(pos, target);
    let before_up = cam.up;
    apply_spacecraft_roll_pitch(&mut cam, 0.03, 0.03);
    assert!((cam.position - pos).length() < 1e-5);
    assert!((cam.target - target).length() > 1e-5 || !cam.up.abs_diff_eq(before_up, 1e-5));
}

#[test]
fn spacecraft_steer_from_offset_zero_is_noop() {
    let pos = Vec3::new(0.0, 0.0, 5.0);
    let target = Vec3::ZERO;
    let mut cam = OrbitCamera::new(pos, target);
    let before_up = cam.up;
    apply_spacecraft_steer_from_offset(&mut cam, 0.0, 0.0, 0.1);
    assert!((cam.target - target).length() < 1e-5);
    assert!(cam.up.abs_diff_eq(before_up, 1e-5));
}

#[test]
fn spacecraft_steer_from_offset_proportional_to_offset() {
    let pos = Vec3::new(0.0, 0.0, 5.0);
    let target = Vec3::ZERO;
    let mut cam_small = OrbitCamera::new(pos, target);
    let mut cam_large = OrbitCamera::new(pos, target);
    let dt = 0.1;
    apply_spacecraft_steer_from_offset(&mut cam_small, 10.0, 0.0, dt);
    apply_spacecraft_steer_from_offset(&mut cam_large, 20.0, 0.0, dt);
    let delta_small = (cam_small.up - Vec3::Y).length();
    let delta_large = (cam_large.up - Vec3::Y).length();
    assert!(delta_large > delta_small);
}

#[test]
fn spacecraft_yaw_from_offset_zero_is_noop() {
    let pos = Vec3::new(0.0, 0.0, 5.0);
    let target = Vec3::ZERO;
    let mut cam = OrbitCamera::new(pos, target);
    let before_target = cam.target;
    apply_spacecraft_yaw_from_offset(&mut cam, 0.0, 0.1);
    assert!((cam.target - before_target).length() < 1e-5);
}

#[test]
fn spacecraft_yaw_from_offset_changes_view() {
    let pos = Vec3::new(0.0, 0.0, 5.0);
    let target = Vec3::ZERO;
    let mut cam = OrbitCamera::new(pos, target);
    let before_target = cam.target;
    apply_spacecraft_yaw_from_offset(&mut cam, 20.0, 0.05);
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
    apply_spacecraft_steer_from_offset(&mut cam, 0.0, 50.0, 0.05);
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
fn spacecraft_steer_from_offset_vertical_turns_in_place_when_slow() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    assert!(cam.velocity().length() < VELOCITY_STEER_THRESHOLD);
    let forward_before = (cam.target - cam.position).normalize();
    apply_spacecraft_steer_from_offset(&mut cam, 0.0, 50.0, 0.05);
    assert_eq!(cam.velocity(), Vec3::ZERO);
    let forward_after = (cam.target - cam.position).normalize();
    assert!(
        (forward_after - forward_before).length() > 1e-3,
        "view direction unchanged"
    );
}

#[test]
fn spacecraft_steer_from_offset_allows_thrust_while_pitched() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    apply_spacecraft_wheel_thrust(&mut cam, 1.0);
    apply_spacecraft_steer_from_offset(&mut cam, 0.0, 50.0, 0.05);
    tick_spacecraft_camera(&mut cam, 0.05);
    assert!(cam.velocity().length() > 1e-4);
}

#[test]
fn spacecraft_yaw_from_offset_horizontal_steers_velocity_when_moving() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    apply_spacecraft_wheel_thrust(&mut cam, 1.0);
    for _ in 0..5 {
        tick_spacecraft_camera(&mut cam, 0.1);
    }
    let velocity_before = cam.velocity();
    assert!(velocity_before.length() > VELOCITY_STEER_THRESHOLD);
    let position_before = cam.position;
    apply_spacecraft_yaw_from_offset(&mut cam, 50.0, 0.05);
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
fn spacecraft_yaw_from_offset_horizontal_turns_in_place_when_slow() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    assert!(cam.velocity().length() < VELOCITY_STEER_THRESHOLD);
    let forward_before = (cam.target - cam.position).normalize();
    apply_spacecraft_yaw_from_offset(&mut cam, 50.0, 0.05);
    assert_eq!(cam.velocity(), Vec3::ZERO);
    let forward_after = (cam.target - cam.position).normalize();
    assert!(
        (forward_after - forward_before).length() > 1e-3,
        "view direction unchanged"
    );
}

#[test]
fn spacecraft_steer_inputs_prioritizes_yaw_anchor() {
    let yaw_anchor = Some([0.0, 0.0]);
    let plus_anchor = Some([0.0, 0.0]);
    let cursor = Some((50.0, 50.0));
    let (yaw_x, plus_offset) = spacecraft_steer_inputs(yaw_anchor, plus_anchor, cursor);
    assert_eq!(yaw_x, Some(50.0));
    assert_eq!(plus_offset, None);
}

#[test]
fn spacecraft_yaw_steer_priority_over_anchor() {
    let mut cam_yaw_only = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    let mut cam_pitch_only = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    let dt = 0.05;
    let cursor = Some((50.0, 50.0));
    tick_spacecraft_steer_and_motion_from_anchors(
        &mut cam_yaw_only,
        Some([0.0, 0.0]),
        Some([0.0, 0.0]),
        cursor,
        dt,
        &InputState::default(),
        false,
        false,
    );
    tick_spacecraft_steer_and_motion_from_anchors(
        &mut cam_pitch_only,
        None,
        Some([0.0, 0.0]),
        cursor,
        dt,
        &InputState::default(),
        false,
        false,
    );
    assert!(
        (cam_yaw_only.target.y - cam_pitch_only.target.y).abs() > 1e-3,
        "yaw priority should ignore anchor pitch"
    );
    assert!(
        (cam_yaw_only.target - cam_pitch_only.target).length() > 1e-3,
        "yaw-only and pitch-only should differ"
    );
}

#[test]
fn spacecraft_keyboard_pitch_w_changes_view() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    let forward_before = (cam.target - cam.position).normalize();
    let input = input_holding(&[KeyCode::KeyW]);
    apply_spacecraft_keyboard(&mut cam, &input, 0.05, false, false);
    let forward_after = (cam.target - cam.position).normalize();
    assert!(
        (forward_after - forward_before).length() > 1e-3,
        "W should pitch view down like cursor above anchor"
    );
}

#[test]
fn spacecraft_keyboard_pitch_w_matches_mouse_up_offset() {
    let dt = 0.05;
    let offset_y = -KEYBOARD_STEER_EQUIV_PX;
    let mut cam_keyboard = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    let mut cam_mouse = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    apply_spacecraft_keyboard(
        &mut cam_keyboard,
        &input_holding(&[KeyCode::KeyW]),
        dt,
        false,
        false,
    );
    apply_spacecraft_steer_from_offset(&mut cam_mouse, 0.0, offset_y, dt);
    assert!(
        (cam_keyboard.target - cam_mouse.target).length() < 1e-4,
        "keyboard W should match mouse-up pitch steer"
    );
}

#[test]
fn spacecraft_keyboard_roll_a_matches_mouse_left_offset() {
    let dt = 0.05;
    let offset_x = -KEYBOARD_STEER_EQUIV_PX;
    let mut cam_keyboard = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    let mut cam_mouse = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    apply_spacecraft_keyboard(
        &mut cam_keyboard,
        &input_holding(&[KeyCode::KeyA]),
        dt,
        false,
        false,
    );
    apply_spacecraft_steer_from_offset(&mut cam_mouse, offset_x, 0.0, dt);
    assert!(
        (cam_keyboard.up - cam_mouse.up).length() < 1e-4,
        "keyboard A should match mouse-left roll steer"
    );
}

#[test]
fn spacecraft_keyboard_yaw_q_matches_mouse_left_offset() {
    let dt = 0.05;
    let offset_x = -KEYBOARD_STEER_EQUIV_PX;
    let mut cam_keyboard = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    let mut cam_mouse = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    apply_spacecraft_keyboard(
        &mut cam_keyboard,
        &input_holding(&[KeyCode::KeyQ]),
        dt,
        false,
        false,
    );
    apply_spacecraft_yaw_from_offset(&mut cam_mouse, offset_x, dt);
    assert!(
        (cam_keyboard.target - cam_mouse.target).length() < 1e-4,
        "keyboard Q should match mouse-left yaw steer"
    );
}

#[test]
fn spacecraft_keyboard_space_builds_velocity() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    let input = input_holding(&[KeyCode::Space]);
    apply_spacecraft_keyboard(&mut cam, &input, 0.05, false, false);
    tick_spacecraft_camera(&mut cam, 0.05);
    assert!(cam.velocity().length() > 0.0);
}

#[test]
fn spacecraft_keyboard_blocked_is_noop() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    let before_target = cam.target;
    let input = input_holding(&[KeyCode::KeyW, KeyCode::Space]);
    assert!(!apply_spacecraft_keyboard(&mut cam, &input, 0.05, true, false));
    assert!((cam.target - before_target).length() < 1e-5);
    assert_eq!(cam.velocity(), Vec3::ZERO);
}

#[test]
fn spacecraft_keyboard_space_suppressed_during_trace() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    let input = input_holding(&[KeyCode::Space]);
    assert!(!apply_spacecraft_keyboard(&mut cam, &input, 0.05, false, true));
    tick_spacecraft_camera(&mut cam, 0.05);
    assert_eq!(cam.velocity(), Vec3::ZERO);
}

use vulkanvil::trace_particle_from_behind;

#[test]
fn trace_lock_up_places_camera_behind_velocity() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    cam.set_lock_up(true);
    let distance = cam.orbit_distance();
    let particle_pos = Vec3::new(10.0, 0.0, 0.0);
    let particle_vel = Vec3::new(1.0, 0.0, 0.0);

    trace_particle_from_behind(&mut cam, particle_pos, particle_vel, 1.0);

    assert!((cam.target - particle_pos).length() < 1e-5);
    let view = (cam.target - cam.position).normalize();
    assert!(view.dot(particle_vel.normalize()) > 0.99);
    assert!((cam.position - (particle_pos - particle_vel.normalize() * distance)).length() < 1e-3);
}

#[test]
fn trace_visual_scale_places_camera_at_scaled_target() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    cam.set_lock_up(true);
    cam.begin_trace_follow();
    let world_distance = cam.trace_follow_distance_or_default();
    let visual_scale = 2.0;
    let world_pos = Vec3::new(1.0, 0.0, 0.0);
    let scaled_pos = world_pos * visual_scale;
    let particle_vel = Vec3::new(1.0, 0.0, 0.0);

    trace_particle_from_behind(&mut cam, world_pos, particle_vel, visual_scale);

    assert!((cam.target - scaled_pos).length() < 1e-5);
    assert!((cam.orbit_distance() - world_distance * visual_scale).abs() < 1e-4);
}

#[test]
fn trace_lock_up_preserves_distance() {
    let mut cam = OrbitCamera::new(Vec3::new(1.0, 2.0, 6.0), Vec3::new(0.5, -1.0, 1.0));
    cam.set_lock_up(true);
    let before = cam.orbit_distance();

    trace_particle_from_behind(&mut cam, Vec3::new(3.0, 1.0, -2.0), Vec3::new(0.0, 0.0, 2.0), 1.0);

    assert!((cam.orbit_distance() - before).abs() < 1e-5);
}

#[test]
fn trace_lock_up_zero_velocity_uses_view() {
    let pos = Vec3::new(0.0, 0.0, 5.0);
    let target = Vec3::ZERO;
    let mut cam = OrbitCamera::new(pos, target);
    cam.set_lock_up(true);
    let view_before = cam.view_relative().normalize();

    trace_particle_from_behind(&mut cam, Vec3::new(4.0, 1.0, 2.0), Vec3::ZERO, 1.0);

    let view_after = cam.view_relative().normalize();
    assert!(view_after.dot(view_before) > 0.99);
}

#[test]
fn trace_free_mode_syncs_velocity() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    cam.set_lock_up(false);
    let particle_vel = Vec3::new(0.3, -0.1, 0.2);

    trace_particle_from_behind(&mut cam, Vec3::new(1.0, 2.0, 3.0), particle_vel, 1.0);

    assert!((cam.velocity() - particle_vel).length() < 1e-5);
}

#[test]
fn trace_free_mode_no_pitch_clamp() {
    let mut cam_locked = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    cam_locked.set_lock_up(true);
    let mut cam_free = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    cam_free.set_lock_up(false);
    let steep_velocity = Vec3::new(0.01, 1.0, 0.01);
    let particle_pos = Vec3::new(2.0, 5.0, 1.0);

    trace_particle_from_behind(&mut cam_locked, particle_pos, steep_velocity, 1.0);
    trace_particle_from_behind(&mut cam_free, particle_pos, steep_velocity, 1.0);

    let locked_horiz = (cam_locked.view_relative().x.powi(2) + cam_locked.view_relative().z.powi(2))
        .sqrt();
    let free_horiz = (cam_free.view_relative().x.powi(2) + cam_free.view_relative().z.powi(2)).sqrt();
    assert!(
        locked_horiz > free_horiz,
        "lock-up should clamp steep pitch: locked={locked_horiz} free={free_horiz}"
    );
}

use vulkanvil::{
    apply_camera_mouse_wheel, apply_orbit_keyboard, MAX_TRACE_FOLLOW_DISTANCE,
    MIN_TRACE_FOLLOW_DISTANCE,
};

#[test]
fn trace_follow_distance_clamps_to_limits() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    cam.set_trace_follow_distance_limits(MIN_TRACE_FOLLOW_DISTANCE, MAX_TRACE_FOLLOW_DISTANCE);
    cam.begin_trace_follow();
    cam.adjust_trace_follow_distance(-100.0);
    assert!(
        (cam.trace_follow_distance_or_default() - MIN_TRACE_FOLLOW_DISTANCE).abs() < 1e-5
    );
    cam.adjust_trace_follow_distance(1000.0);
    assert!(
        (cam.trace_follow_distance_or_default() - MAX_TRACE_FOLLOW_DISTANCE).abs() < 1e-5
    );
}

#[test]
fn trace_wheel_adjusts_follow_distance() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 10.0), Vec3::ZERO);
    cam.set_lock_up(true);
    cam.begin_trace_follow();
    let before = cam.trace_follow_distance_or_default();
    apply_camera_mouse_wheel(&mut cam, true, 1.0, true);
    let after_wheel = cam.trace_follow_distance_or_default();
    assert!(
        after_wheel < before,
        "scroll up should move closer: before={before} after={after_wheel}"
    );

    let particle_pos = Vec3::new(3.0, 0.0, 0.0);
    let particle_vel = Vec3::new(1.0, 0.0, 0.0);
    trace_particle_from_behind(&mut cam, particle_pos, particle_vel, 1.0);
    assert!((cam.orbit_distance() - after_wheel).abs() < 1e-4);
}

#[test]
fn trace_orbit_keyboard_w_s_adjusts_follow_distance() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 8.0), Vec3::ZERO);
    cam.set_lock_up(true);
    cam.begin_trace_follow();
    let before = cam.trace_follow_distance_or_default();

    apply_orbit_keyboard(
        &mut cam,
        &input_holding(&[KeyCode::KeyW]),
        true,
        false,
        true,
    );
    assert!(cam.trace_follow_distance_or_default() < before);

    let after_w = cam.trace_follow_distance_or_default();
    apply_orbit_keyboard(
        &mut cam,
        &input_holding(&[KeyCode::KeyS]),
        true,
        false,
        true,
    );
    assert!(cam.trace_follow_distance_or_default() > after_w);
}

#[test]
fn trace_spacecraft_keyboard_w_adjusts_follow_distance_not_pitch() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 6.0), Vec3::ZERO);
    cam.begin_trace_follow();
    let forward_before = (cam.target - cam.position).normalize();
    let distance_before = cam.trace_follow_distance_or_default();

    apply_spacecraft_keyboard(&mut cam, &input_holding(&[KeyCode::KeyW]), 0.05, false, true);

    assert!(cam.trace_follow_distance_or_default() < distance_before);
    assert!(
        (cam.target - cam.position).normalize().dot(forward_before) > 0.999,
        "W during trace should not pitch the view"
    );
}

#[test]
fn end_trace_follow_restores_lock_up_orbit_distance() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    cam.set_lock_up(true);
    let before = cam.orbit_distance();

    cam.begin_trace_follow();
    cam.adjust_trace_follow_distance(-4.0);
    let particle_pos = Vec3::new(3.0, 0.0, 0.0);
    trace_particle_from_behind(&mut cam, particle_pos, Vec3::new(1.0, 0.0, 0.0), 1.0);
    assert!(
        cam.orbit_distance() < before,
        "trace should have moved the camera closer"
    );
    let view_before_release = cam.view_relative().normalize();

    cam.end_trace_follow();

    assert!((cam.orbit_distance() - before).abs() < 1e-4);
    assert!((cam.target - particle_pos).length() < 1e-5, "release keeps target");
    assert!(cam.view_relative().normalize().dot(view_before_release) > 0.9999);
}

#[test]
fn end_trace_follow_free_mode_keeps_position() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    cam.set_lock_up(false);
    cam.begin_trace_follow();
    cam.adjust_trace_follow_distance(-4.0);
    trace_particle_from_behind(&mut cam, Vec3::new(3.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 1.0);
    let position = cam.position;

    cam.end_trace_follow();

    assert!((cam.position - position).length() < 1e-6);
}

#[test]
fn unlock_during_trace_then_trace_off_then_relock_restores_distance() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    cam.set_lock_up(true);
    let before = cam.orbit_distance();

    cam.begin_trace_follow();
    cam.adjust_trace_follow_distance(-4.0);
    trace_particle_from_behind(&mut cam, Vec3::new(3.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 1.0);
    assert!(cam.orbit_distance() < before);

    cam.set_lock_up(false);
    cam.end_trace_follow();
    cam.set_lock_up(true);

    assert!(
        (cam.orbit_distance() - before).abs() < 1e-4,
        "re-lock after unlocking mid-trace should restore the pre-trace distance: \
         before={before} after={}",
        cam.orbit_distance()
    );
}

#[test]
fn end_trace_follow_without_trace_is_noop() {
    let mut cam = OrbitCamera::new(Vec3::new(0.0, 0.0, 5.0), Vec3::ZERO);
    cam.set_lock_up(true);
    cam.zoom(2.0);
    let position = cam.position;

    cam.end_trace_follow();

    assert!((cam.position - position).length() < 1e-6);
}

const INITIAL_POSITION: Vec3 = Vec3::new(1.6, -1.6, 3.0);
const INITIAL_TARGET: Vec3 = Vec3::ZERO;

#[test]
fn reset_pose_restores_orbit_distance() {
    let initial = OrbitCamera::new(INITIAL_POSITION, INITIAL_TARGET);
    let initial_distance = initial.orbit_distance();

    let mut cam = OrbitCamera::new(Vec3::new(10.0, 5.0, 20.0), Vec3::new(3.0, 1.0, 2.0));
    cam.set_lock_up(true);
    cam.begin_trace_follow();
    cam.adjust_trace_follow_distance(5.0);

    cam.reset_pose(INITIAL_POSITION, INITIAL_TARGET);

    assert!((cam.position - INITIAL_POSITION).length() < 1e-5);
    assert!((cam.target - INITIAL_TARGET).length() < 1e-5);
    assert!((cam.orbit_distance() - initial_distance).abs() < 1e-5);
}

#[test]
fn reset_pose_clears_trace_follow() {
    let mut cam = OrbitCamera::new(INITIAL_POSITION, INITIAL_TARGET);
    cam.begin_trace_follow();
    cam.adjust_trace_follow_distance(2.0);
    assert!(cam.trace_follow_distance_or_default() != cam.orbit_distance());

    cam.reset_pose(INITIAL_POSITION, INITIAL_TARGET);

    assert_eq!(cam.trace_follow_distance_or_default(), cam.orbit_distance());
}
