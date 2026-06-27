use dual_spacetime_simulator::particle_selection_marker::{
    compute_bracket_half_size, BRACKET_RADIUS_RATIO, MIN_HALF_SIZE_PX,
};
use dual_spacetime_simulator::pipeline::project_particle_screen_px;
use dual_spacetime_simulator::simulation::Particle;
use glam::{DVec3, Mat4, Vec3};

const INITIAL_CAMERA_POSITION: Vec3 = Vec3::new(1.6, -1.6, 3.0);
const TEST_SIZE_SCALE: f32 = 720.0 * 0.06;

fn test_mvp(aspect_ratio: f32, scale_factor: f32) -> Mat4 {
    let view = Mat4::look_at_rh(INITIAL_CAMERA_POSITION, Vec3::ZERO, Vec3::Y);
    let proj = Mat4::perspective_rh(std::f32::consts::FRAC_PI_4, aspect_ratio, 0.1, 100.0);
    let model = Mat4::from_scale(Vec3::splat(scale_factor));
    proj * view * model
}

fn origin_particle() -> Particle {
    Particle::from_kinematics(DVec3::ZERO, DVec3::ZERO, 1.0, [1.0, 1.0, 1.0, 1.0])
}

#[test]
fn compute_bracket_half_size_grows_when_closer() {
    let near = compute_bracket_half_size(1.0, TEST_SIZE_SCALE);
    let far = compute_bracket_half_size(10.0, TEST_SIZE_SCALE);
    assert!(near > far);
}

#[test]
fn compute_bracket_half_size_clamps_to_minimum() {
    let very_far = compute_bracket_half_size(1000.0, TEST_SIZE_SCALE);
    assert!((very_far - MIN_HALF_SIZE_PX).abs() < f32::EPSILON);
}

#[test]
fn compute_bracket_half_size_tracks_particle_diameter() {
    let view_depth = 1.0;
    let half = compute_bracket_half_size(view_depth, TEST_SIZE_SCALE);
    let expected = TEST_SIZE_SCALE / view_depth * BRACKET_RADIUS_RATIO;
    assert!((half - expected).abs() < f32::EPSILON);
}

#[test]
fn compute_bracket_half_size_scales_with_size_scale() {
    let view_depth = 0.8;
    let small = compute_bracket_half_size(view_depth, TEST_SIZE_SCALE * 0.5);
    let large = compute_bracket_half_size(view_depth, TEST_SIZE_SCALE);
    assert!((large - small * 2.0).abs() < f32::EPSILON);
}

#[test]
fn project_particle_at_origin_is_visible_with_default_camera() {
    let aspect_ratio = 16.0 / 9.0;
    let scale_factor = 1.0;
    let mvp = test_mvp(aspect_ratio, scale_factor);
    let width = 1280.0;
    let height = 720.0;
    let particle = origin_particle();

    let screen_px = project_particle_screen_px(&particle, mvp, width, height)
        .expect("origin particle should project inside the viewport");
    assert!(screen_px[0] >= 0.0 && screen_px[0] <= width);
    assert!(screen_px[1] >= 0.0 && screen_px[1] <= height);
}

#[test]
fn project_particle_behind_camera_is_hidden() {
    let aspect_ratio = 1.0;
    let scale_factor = 1.0;
    let mvp = test_mvp(aspect_ratio, scale_factor);
    let behind_camera = Particle {
        position: DVec3::new(
            INITIAL_CAMERA_POSITION.x as f64 * 2.0,
            INITIAL_CAMERA_POSITION.y as f64 * 2.0,
            INITIAL_CAMERA_POSITION.z as f64 * 2.0,
        ),
        ..origin_particle()
    };

    assert!(project_particle_screen_px(&behind_camera, mvp, 800.0, 600.0).is_none());
}
