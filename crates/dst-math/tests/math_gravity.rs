use dst_math::gravity::{
    acceleration_at, dual_rotor_accel_at, killing_scalar, proper_time_rate,
    repulsion_shell_dual_rotor, signed_interaction_weight, strong_field_gate,
    torsion_mismatch, usual_boost_from_velocity,
};
use glam::DVec3;

const G: f64 = 6.6743e-11;
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
