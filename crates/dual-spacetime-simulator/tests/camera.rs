use dual_spacetime_simulator::camera::OrbitCamera;
use glam::Vec3;

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
