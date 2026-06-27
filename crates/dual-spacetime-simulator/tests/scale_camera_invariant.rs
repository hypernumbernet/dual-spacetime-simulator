use glam::Vec3;
use vulkanvil::OrbitCamera;

const INITIAL_POSITION: Vec3 = Vec3::new(1.6, -1.6, 3.0);
const INITIAL_TARGET: Vec3 = Vec3::ZERO;

#[test]
fn reset_pose_matches_fresh_initial_camera() {
    let initial = OrbitCamera::new(INITIAL_POSITION, INITIAL_TARGET);
    let expected_distance = initial.orbit_distance();

    let mut cam = OrbitCamera::new(INITIAL_POSITION, INITIAL_TARGET);
    cam.set_lock_up(true);
    cam.move_target_y(2.0);
    cam.move_forward(5.0);

    cam.reset_pose(INITIAL_POSITION, INITIAL_TARGET);

    assert!((cam.position - INITIAL_POSITION).length() < 1e-5);
    assert!((cam.target - INITIAL_TARGET).length() < 1e-5);
    assert!((cam.orbit_distance() - expected_distance).abs() < 1e-5);
}
