use dst_math::gravity::{
    dst_gravity_velocity_delta, s3_log_from_rotation_angle, unit_quaternion_from_momentum_axis,
    unit_quaternion_ln,
};
use glam::DVec3;

const G: f64 = 6.6743e-11;
const C: f64 = 299_792_458.0;
const EPSILON: f64 = 1e-10;

#[test]
fn s3_log_positive_below_pi() {
    let angle = 0.5 * std::f64::consts::PI;
    let ln = s3_log_from_rotation_angle(angle, DVec3::X);
    assert!(ln.x > 0.0);
    assert!((ln.x - 0.25 * std::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn s3_log_flips_sign_past_pi() {
    let angle = 2.5 * std::f64::consts::PI;
    let ln = s3_log_from_rotation_angle(angle, DVec3::X);
    assert!(ln.x < 0.0, "ln should flip past antipodal hemisphere");
}

#[test]
fn unit_quaternion_ln_matches_direct_log() {
    let angle = 2.5 * std::f64::consts::PI;
    let q = unit_quaternion_from_momentum_axis(DVec3::X, angle);
    assert!(q.w < 0.0);
    let ln = unit_quaternion_ln(q, angle, DVec3::X);
    let direct = s3_log_from_rotation_angle(angle, DVec3::X);
    assert!((ln - direct).length() < 1e-12);
}

#[test]
fn dst_gravity_weak_field_attracts() {
    let mass = 1.0e24;
    let scale = 1e10;
    let light_speed = C / scale;
    let separation = 1.0e11;
    let delta = dst_gravity_velocity_delta(
        mass,
        mass,
        DVec3::new(separation, 0.0, 0.0),
        G,
        light_speed,
        1.0,
        EPSILON,
    );
    assert!(delta.x > 0.0, "weak field should attract: delta={delta:?}");
    assert!(delta.is_finite());
}

#[test]
fn dst_gravity_strong_field_repels() {
    let mass = 1.0e30;
    let scale = 1.0;
    let light_speed = C / scale;
    // p ~ G m²/r² must exceed m c so θ = atanh²(l) crosses π.
    let separation = 1.0e5;
    let delta = dst_gravity_velocity_delta(
        mass,
        mass,
        DVec3::new(separation, 0.0, 0.0),
        G,
        light_speed,
        1.0,
        EPSILON,
    );
    assert!(delta.x < 0.0, "strong field should repel: delta={delta:?}");
    assert!(delta.is_finite());
}