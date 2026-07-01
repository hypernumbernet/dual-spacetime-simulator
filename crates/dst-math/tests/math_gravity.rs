use dst_math::gravity::{
    dst_gravity_step_at, gravitational_potential_at, gravity_sign_from_time_dilation,
    k_scale_from_light_speed, newtonian_gravity_pair, time_dilation,
    update_time_delay_for_particle,
};
use glam::DVec3;

const G: f64 = 6.6743e-11;
const C: f64 = 299_792_458.0;
const EPSILON: f64 = 1e-10;

#[test]
fn gravitational_potential_two_body() {
    let mass = 1.0e30;
    let separation = 1.0e11;
    let positions = vec![DVec3::ZERO, DVec3::new(separation, 0.0, 0.0)];
    let masses = vec![mass, 1.0e24];

    let phi = gravitational_potential_at(1, &positions, &masses, G, EPSILON);
    let expected = -G * mass / (separation + EPSILON);
    assert!((phi - expected).abs() < 1e-6 * expected.abs().max(1.0));
}

#[test]
fn guide_verification_lambda_eff_and_dilation() {
    let central_mass = 1.0e30;
    let test_mass = 1.0e24;
    let scale = 1e10;
    let separation = 1.0e11 / scale;
    let positions = vec![DVec3::ZERO, DVec3::new(separation, 0.0, 0.0)];
    let masses = vec![central_mass / scale.powi(3), test_mass / scale.powi(3)];

    let phi = gravitational_potential_at(1, &positions, &masses, G, EPSILON);
    let light_speed_sim = C / scale;
    let k_scale = k_scale_from_light_speed(light_speed_sim);
    let lambda_eff = k_scale * phi;
    let dilation = time_dilation(lambda_eff);

    let expected_phi = -G * (central_mass / scale.powi(3)) / (separation + EPSILON);
    let expected_lambda = k_scale * expected_phi;

    assert!((lambda_eff - expected_lambda).abs() < 1e-12 * expected_lambda.abs().max(1.0));
    assert!(dilation.is_finite());
    assert!(dilation >= -1.0 && dilation <= 1.0);
}

#[test]
fn newtonian_gravity_pair_matches_separate_formulas() {
    let g = G;
    let time_g = G * 1.0;
    let pos_i = DVec3::ZERO;
    let pos_j = DVec3::new(1.0e11, 0.0, 0.0);
    let mass_j = 1.0e30;

    let (phi_j, accel) = newtonian_gravity_pair(pos_i, pos_j, mass_j, g, time_g, EPSILON);
    let expected_phi = -g * mass_j / (1.0e11 + EPSILON);
    let diff = pos_j - pos_i;
    let expected_accel = time_g * mass_j / diff.length_squared() * diff.normalize();

    assert!((phi_j - expected_phi).abs() < 1e-6 * expected_phi.abs());
    assert!((accel - expected_accel).length() < 1e-6 * expected_accel.length().max(1.0));
}

#[test]
fn dst_gravity_step_at_single_pass_matches_potential_and_sign() {
    let scale = 1e10_f64;
    let central_mass = 1.989e30 / scale.powi(3);
    let test_mass = 1.0e24 / scale.powi(3);
    let separation = 1.496e11 / scale;
    let positions = vec![DVec3::ZERO, DVec3::new(separation, 0.0, 0.0)];
    let masses = vec![central_mass, test_mass];
    let k_scale = k_scale_from_light_speed(C / scale);
    let dt = 1.0;

    let (velocity_delta, lambda_eff, proper_time_delta) = dst_gravity_step_at(
        1,
        &positions,
        &masses,
        G,
        G * dt,
        k_scale,
        EPSILON,
        dt,
    );

    let phi = gravitational_potential_at(1, &positions, &masses, G, EPSILON);
    let expected_lambda = k_scale * phi;
    let dilation = time_dilation(expected_lambda);

    assert!((lambda_eff - expected_lambda).abs() < 1e-12 * expected_lambda.abs().max(1.0));
    assert!((proper_time_delta - dt * dilation).abs() < 1e-12);
    assert!(velocity_delta.is_finite());
    assert!(velocity_delta.x < 0.0, "weak field should accelerate toward center");
    assert_eq!(gravity_sign_from_time_dilation(dilation), 1.0);
}

#[test]
fn gravity_sign_flips_when_dilation_negative() {
    assert_eq!(gravity_sign_from_time_dilation(1.0), 1.0);
    assert_eq!(gravity_sign_from_time_dilation(0.0), 1.0);
    assert_eq!(gravity_sign_from_time_dilation(-0.5), -1.0);
    assert!(time_dilation(std::f64::consts::PI) < 0.0);
    assert_eq!(gravity_sign_from_time_dilation(time_dilation(std::f64::consts::PI)), -1.0);
}

#[test]
fn update_time_delay_accumulates_proper_time() {
    let mut proper_time = 0.0;
    let mut lambda_eff = 0.0;
    let phi = -1.0e-6;
    let k_scale = 2.0;
    let dt = 1.0;

    update_time_delay_for_particle(&mut proper_time, &mut lambda_eff, phi, k_scale, dt);

    assert!((lambda_eff - (-2.0e-6)).abs() < 1e-15);
    assert!((proper_time - dt * (-2.0e-6_f64).cos()).abs() < 1e-15);
}
