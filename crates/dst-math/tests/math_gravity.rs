use dst_math::gravity::{
    acceleration_at, dst_gravity_velocity_delta, dual_rotor_accel_at, killing_scalar,
    momentum_to_s3_angle, proper_time_rate, repulsion_shell_dual_rotor,
    horizon_reference_momentum, schwarzschild_radius, signed_interaction_weight,
    strong_field_gate, torsion_mismatch,
    s3_log_from_rotation_angle, unit_quaternion_from_momentum_axis, unit_quaternion_ln,
    usual_boost_from_velocity,
};
use glam::DVec3;

const G: f64 = 6.6743e-11;
const C: f64 = 299_792_458.0;
const EPSILON: f64 = 1e-10;

#[test]
fn torsion_mismatch_zero_when_phi_matches_velocity_boost() {
    let velocity = DVec3::new(0.5, 0.0, 0.0);
    let phi = usual_boost_from_velocity(velocity);
    let delta = torsion_mismatch(phi, velocity);
    assert!(delta.length() < 1e-12);
}

#[test]
fn proper_time_rate_is_one_at_zero_mismatch() {
    assert!((proper_time_rate(DVec3::ZERO) - 1.0).abs() < 1e-12);
}

#[test]
fn killing_scalar_negative_when_phi_dominates() {
    let theta = DVec3::ZERO;
    let phi = DVec3::new(1.0, 0.0, 0.0);
    assert!(killing_scalar(theta, phi) < 0.0);
}

#[test]
fn signed_weight_repulsive_in_strong_field_when_j_negative() {
    let phi = DVec3::new(1.0, 0.0, 0.0);
    let weight = signed_interaction_weight(-0.5, phi, phi, G, 0.1);
    assert!(weight < 0.0);
    assert!(weight.abs() > signed_interaction_weight(-0.5, phi, phi, G, 100.0).abs());
}

#[test]
fn strong_field_gate_increases_as_distance_decreases() {
    let near = strong_field_gate(G, 0.1);
    let far = strong_field_gate(G, 100.0);
    assert!(near > far);
    assert!((near - 1.0).abs() < 0.1);
}

#[test]
fn two_body_acceleration_points_toward_other_particle_when_j_nonnegative() {
    let positions = vec![DVec3::ZERO, DVec3::new(1.0e11, 0.0, 0.0)];
    let masses = vec![1.0e30, 1.0e30];
    let usual_boosts = vec![DVec3::ZERO, DVec3::ZERO];
    let dual_rotors = vec![DVec3::ZERO, DVec3::ZERO];
    let accel = acceleration_at(
        0,
        &positions,
        &masses,
        &usual_boosts,
        &dual_rotors,
        G,
        EPSILON,
    );
    assert!(accel.x > 0.0);
    assert!(accel.is_finite());
}

#[test]
fn repulsion_shell_pushes_outward_in_strong_field() {
    let positions = vec![DVec3::ZERO, DVec3::new(0.5, 0.0, 0.0)];
    let masses = vec![1.0e3, 1.0e2];
    let usual_boosts = vec![DVec3::ZERO, DVec3::ZERO];
    let dual_rotors = vec![DVec3::ZERO, repulsion_shell_dual_rotor(1.0)];
    let accel = acceleration_at(
        1,
        &positions,
        &masses,
        &usual_boosts,
        &dual_rotors,
        G,
        EPSILON,
    );
    assert!(accel.x > 0.0, "shell particle should be repelled from center");
}

#[test]
fn momentum_to_s3_angle_equals_pi_at_event_horizon() {
    let mass = 1.0e30;
    let rs = schwarzschild_radius(mass, mass, G, C);
    let dt = 1.0;
    let p_at_horizon = G * mass * mass / (rs * rs) * dt;
    let angle = momentum_to_s3_angle(p_at_horizon, rs, rs);
    assert!((angle - std::f64::consts::PI).abs() < 1e-12);
    let p_ref = horizon_reference_momentum(p_at_horizon, rs, rs);
    assert!((p_at_horizon - p_ref).abs() < 1e-12 * p_at_horizon);
}

#[test]
fn momentum_to_s3_angle_scales_with_momentum_ratio() {
    let mass = 1.0e30;
    let rs = schwarzschild_radius(mass, mass, G, C);
    let r = 2.0 * rs;
    let dt = 1.0;
    let p = G * mass * mass / (r * r) * dt;
    let p_horizon = horizon_reference_momentum(p, r, rs);
    let angle = momentum_to_s3_angle(p, r, rs);
    assert!((angle - std::f64::consts::PI * p / p_horizon).abs() < 1e-12);
    assert!((angle - 0.25 * std::f64::consts::PI).abs() < 1e-12);
}

#[test]
fn unit_quaternion_ln_flips_sign_past_antipodal_hemisphere() {
    let angle = 2.5 * std::f64::consts::PI;
    let q_neg = unit_quaternion_from_momentum_axis(DVec3::X, angle);
    assert!(q_neg.w < 0.0);
    let ln = unit_quaternion_ln(q_neg, angle, DVec3::X);
    assert!(ln.x < 0.0, "ln should point opposite to momentum axis past horizon");
    let direct = s3_log_from_rotation_angle(angle, DVec3::X);
    assert!((ln - direct).length() < 1e-12);
}

#[test]
fn dst_gravity_velocity_inverts_direction_inside_event_horizon() {
    let mass = 1.0e30;
    let rs = schwarzschild_radius(mass, mass, G, C);
    let dt = 1.0;

    let dv_far = dst_gravity_velocity_delta(
        mass,
        mass,
        DVec3::new(10.0 * rs, 0.0, 0.0),
        G,
        C,
        dt,
        EPSILON,
    );
    let dv_near = dst_gravity_velocity_delta(
        mass,
        mass,
        DVec3::new(0.4 * rs, 0.0, 0.0),
        G,
        C,
        dt,
        EPSILON,
    );

    eprintln!(
        "dst_gravity_velocity_delta far: dv=({:.6e}, {:.6e}, {:.6e}) |dv|={:.6e}",
        dv_far.x,
        dv_far.y,
        dv_far.z,
        dv_far.length()
    );
    eprintln!(
        "dst_gravity_velocity_delta near: dv=({:.6e}, {:.6e}, {:.6e}) |dv|={:.6e}",
        dv_near.x,
        dv_near.y,
        dv_near.z,
        dv_near.length()
    );
    assert!(dv_far.x > 0.0, "weak field should attract along +x");
    assert!(dv_near.x < 0.0, "inside horizon scale should repel along -x");
    assert!(dv_far.is_finite());
    assert!(dv_near.is_finite());
    assert!(dv_far.length() > 0.0);
    assert!(dv_near.length() > 0.0);

    let r_far = 10.0 * rs;
    let r_near = 0.4 * rs;
    let exchange_far = G * mass / (r_far * r_far) * dt;
    let exchange_near = G * mass / (r_near * r_near) * dt;
    let half_near =
        0.5 * std::f64::consts::PI * (rs / r_near).powi(2);
    let expected_near = half_near * exchange_near;
    assert!(
        (dv_far.length() - exchange_far).abs() < 1e-6 * exchange_far,
        "weak field magnitude should match Newtonian exchange: |dv|={} expected={}",
        dv_far.length(),
        exchange_far
    );
    assert!(
        (dv_near.length() - expected_near).abs() < 1e-6 * expected_near,
        "strong field should use full Ln scale (θ/2)·p/m: |dv|={} expected={}",
        dv_near.length(),
        expected_near
    );
    assert!(
        dv_near.length() > exchange_near,
        "repulsion inside horizon should exceed bare exchange magnitude"
    );
}

#[test]
fn dual_rotor_accel_restores_mismatch_when_isolated() {
    let positions = vec![DVec3::ZERO];
    let masses = vec![2.0];
    let velocity = DVec3::new(0.3, 0.0, 0.0);
    let phi = DVec3::ZERO;
    let dual_rotors = vec![phi];
    let delta = torsion_mismatch(phi, velocity);
    let accel = dual_rotor_accel_at(
        0,
        &positions,
        &masses,
        &dual_rotors,
        DVec3::ZERO,
        delta,
        G,
        EPSILON,
    );
    assert!((accel + G * delta).length() < 1e-9);
}
