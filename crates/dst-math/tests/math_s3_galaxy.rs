use dst_math::s3_galaxy::{
    galaxy_gravity_pair_ln, galaxy_radius_sim, integrate_orientation,
    orientation_from_disk_position, orientation_to_display_position, quaternion_exp,
    quaternion_log, radial_distance_ln, relative_quaternion,
};
use glam::{DQuat, DVec3};
use std::f64::consts::PI;

#[test]
fn exp_log_roundtrip_small_angle() {
    let axis = DVec3::new(0.0, 0.0, 1.0);
    let v = axis * 0.3;
    let q = quaternion_exp(v);
    let v_back = quaternion_log(q);
    assert!((v - v_back).length() < 1e-10);
}

#[test]
fn exp_log_roundtrip_tiny_angle() {
    // Angles far below acos() resolution (~1.5e-8) must survive the round trip;
    // near-field scenes (scene size ≪ R) live entirely in this regime.
    let v = DVec3::new(3.0e-12, -1.0e-12, 2.0e-12);
    let q = quaternion_exp(v);
    let v_back = quaternion_log(q);
    assert!(
        (v - v_back).length() < 1e-15 * v.length().max(1e-30),
        "tiny-angle log map lost precision: {v_back:?}"
    );
}

#[test]
fn near_field_pair_matches_newtonian() {
    // Scene small relative to R: S³ gravity must reduce to Newton's law.
    let scale = 1e10;
    let r_galaxy = galaxy_radius_sim(scale);
    let p_i = DVec3::new(5.0, 1.0, -2.0);
    let p_j = DVec3::new(-3.0, 0.5, 4.0);
    let q_i = orientation_from_disk_position(p_i, r_galaxy);
    let q_j = orientation_from_disk_position(p_j, r_galaxy);
    let mass_j = 1.0e30;
    let time_g = 1.0;
    let a = galaxy_gravity_pair_ln(q_i, q_j, mass_j, r_galaxy, time_g, 1e-12);

    let diff = p_j - p_i;
    let expected = time_g * mass_j / diff.length_squared() * diff.normalize();
    assert!(
        (a - expected).length() < 1e-6 * expected.length(),
        "expected Newtonian {expected:?}, got {a:?}"
    );
}

#[test]
fn gravity_computed_in_projected_chart() {
    // Gravity must act on the Ln-projected display positions (the on-screen
    // coordinates), not on S³ geodesic quantities.
    let r_galaxy = 1000.0;
    let q_i = quaternion_exp(DVec3::new(0.4, 0.7, -0.2));
    let q_j = quaternion_exp(DVec3::new(0.5, 0.6, -0.1));
    let p_i = orientation_to_display_position(q_i, r_galaxy);
    let p_j = orientation_to_display_position(q_j, r_galaxy);
    let mass_j = 1.0e6;
    let a = galaxy_gravity_pair_ln(q_i, q_j, mass_j, r_galaxy, 1.0, 1e-12);

    let d = p_j - p_i;
    let expected = mass_j / d.length_squared() * d.normalize();
    assert!(
        (a - expected).length() < 1e-9 * expected.length(),
        "expected chart-space {expected:?}, got {a:?}"
    );
}

#[test]
fn chart_gravity_distorted_far_from_origin() {
    // The projection distortion is the point of the model: for a close pair far
    // from the origin, the chart-space force direction must deviate from the
    // true S³ geodesic direction (while still pulling broadly toward j).
    let r_galaxy = 1000.0;
    let q_i = quaternion_exp(DVec3::new(0.4, 0.7, -0.2));
    let d_geo = DVec3::new(1.0, -2.0, 0.5).normalize();
    let eps_ang = 0.05;
    let q_j = (quaternion_exp(d_geo * eps_ang) * q_i).normalize();
    let a = galaxy_gravity_pair_ln(q_i, q_j, 1.0e6, r_galaxy, 1.0, 1e-12);

    let alignment = a.normalize().dot(d_geo);
    assert!(alignment > 0.0, "force should still point broadly toward j");
    assert!(
        alignment < 1.0 - 1e-6,
        "distortion vanished: chart force aligns exactly with the geodesic ({alignment})"
    );
}

#[test]
fn integrate_orientation_matches_linear_motion_near_field() {
    // Δp = v·dt must hold when the scene is small relative to R.
    let scale = 1e10;
    let r_galaxy = galaxy_radius_sim(scale);
    let p = DVec3::new(10.0, -4.0, 2.0);
    let vel = DVec3::new(3.0e-6, 1.0e-6, -2.0e-6);
    let dt = 3600.0;
    let q = orientation_from_disk_position(p, r_galaxy);
    let q_next = integrate_orientation(q, vel, r_galaxy, dt);
    let p_next = orientation_to_display_position(q_next, r_galaxy);
    let expected = p + vel * dt;
    assert!(
        (p_next - expected).length() < 1e-9 * expected.length(),
        "expected {expected:?}, got {p_next:?}"
    );
}

#[test]
fn radial_distance_at_galaxy_radius() {
    let r_galaxy = galaxy_radius_sim(1e20);
    let v = DVec3::new(PI * 0.5, 0.0, 0.0);
    let r = radial_distance_ln(v, r_galaxy);
    assert!((r - r_galaxy).abs() < 1e-6 * r_galaxy);
}

#[test]
fn gravity_sign_flips_beyond_galaxy_radius() {
    let r_galaxy = 1.0;
    let q_i = DQuat::IDENTITY;
    let time_g = 1.0;
    let mass = 1e10;
    let sign_within = |v_mag: f64| {
        let q_j = quaternion_exp(DVec3::new(v_mag, 0.0, 0.0));
        let q_rel = relative_quaternion(q_i, q_j);
        let v = quaternion_log(q_rel);
        let a = galaxy_gravity_pair_ln(q_i, q_j, mass, r_galaxy, time_g, 1e-12);
        a.dot(v.normalize())
    };
    assert!(sign_within(0.3) > 0.0);
    assert!(sign_within(PI * 0.55) < 0.0);
}

#[test]
fn gravity_attractive_within_galaxy_radius() {
    let q_i = DQuat::IDENTITY;
    let v_j = DVec3::new(PI * 0.25, 0.0, 0.0);
    let q_j = quaternion_exp(v_j);
    let q_rel = relative_quaternion(q_i, q_j);
    let v = quaternion_log(q_rel);
    let v_hat = v.normalize();
    let a = galaxy_gravity_pair_ln(q_i, q_j, 1e10, 10.0, 1.0, 1e-12);
    assert!(a.dot(v_hat) > 0.0, "expected positive sign along v_hat, got {a:?}");
}

#[test]
fn gravity_direction_along_ln_v_hat() {
    let r_galaxy = 1.0;
    let q_i = DQuat::IDENTITY;
    let v_j = DVec3::new(0.1, 0.2, 0.0);
    let q_j = quaternion_exp(v_j);
    let a = galaxy_gravity_pair_ln(q_i, q_j, 1e10, r_galaxy, 1.0, 1e-12);
    let q_rel = relative_quaternion(q_i, q_j);
    let v = quaternion_log(q_rel);
    let v_hat = v.normalize();
    let alignment = a.normalize().dot(v_hat);
    assert!(alignment > 0.99, "acceleration should align with v_hat, got {alignment}");
}

#[test]
fn orientation_from_disk_position_at_radius() {
    let r_galaxy = 100.0;
    let pos = DVec3::new(r_galaxy * 0.5, 0.0, 0.0);
    let q = orientation_from_disk_position(pos, r_galaxy);
    let v = quaternion_log(q);
    let r = radial_distance_ln(v, r_galaxy);
    assert!((r - pos.x).abs() < 1e-6 * r_galaxy);
}

#[test]
fn orientation_position_roundtrip_3d() {
    let r_galaxy = 100.0;
    let pos = DVec3::new(30.0, 5.0, 40.0);
    let q = orientation_from_disk_position(pos, r_galaxy);
    let restored = orientation_to_display_position(q, r_galaxy);
    assert!((restored - pos).length() < 1e-6 * pos.length());
}

#[test]
fn orientation_position_roundtrip_xz_disk_with_y_thickness() {
    let r_galaxy = 100.0;
    let pos = DVec3::new(40.0, 3.0, 30.0);
    let q = orientation_from_disk_position(pos, r_galaxy);
    let restored = orientation_to_display_position(q, r_galaxy);
    assert!(restored.y != 0.0, "y component must be preserved");
    assert!((restored.y - pos.y).abs() < 1e-9 * pos.length());
    assert!((restored - pos).length() < 1e-6 * pos.length());
}
