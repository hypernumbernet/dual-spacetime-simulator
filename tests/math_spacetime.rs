//! Extra integration coverage for `math::spacetime` (see also crate-local `#[cfg(test)]`).

use dual_spacetime_simulator::math::spacetime::{
    Spacetime, lorentz_boost_matrix_from_velocity, rapidity_from_momentum, rapidity_vector,
};
use glam::{DMat4, DVec3, DVec4};

#[test]
fn lorentz_transformation_matrix_zero_velocity_is_identity() {
    let m = lorentz_boost_matrix_from_velocity(DVec3::ZERO, 1.0).unwrap();
    assert_eq!(m, DMat4::IDENTITY);
}

#[test]
fn lorentz_transformation_matrix_x_boost_beta_half() {
    let c_inv = 1.0_f64;
    let v = DVec3::new(0.5, 0.0, 0.0);
    let m = lorentz_boost_matrix_from_velocity(v, c_inv).unwrap();
    let beta = 0.5_f64;
    let gamma = 1.0 / (1.0 - beta * beta).sqrt();
    let gc = -gamma * c_inv;
    let gxc = v.x * gc;
    let g1 = gamma - 1.0;

    let expected = DMat4::from_cols(
        DVec4::new(gamma, gxc, 0.0, 0.0),
        DVec4::new(gxc, 1.0 + g1, 0.0, 0.0),
        DVec4::new(0.0, 0.0, 1.0, 0.0),
        DVec4::new(0.0, 0.0, 0.0, 1.0),
    );
    assert_eq!(m, expected);
}

#[test]
fn lorentz_boost_matrix_from_velocity_rejects_light_speed() {
    let m = lorentz_boost_matrix_from_velocity(DVec3::new(1.0, 0.0, 0.0), 1.0);
    assert!(m.is_err());
}

#[test]
fn apply_lorentz_transform_by_velocity_zero_is_noop() {
    let mut st = Spacetime::new(1.0, 0.5, -0.25, 0.125);
    let original = st;
    st.apply_lorentz_transform_by_velocity(DVec3::ZERO, 1.0 / 299_792_458.0);
    assert!(st.fuzzy_compare(original));
}

#[test]
fn apply_lorentz_transform_by_rapidity_zero_is_noop() {
    let mut st = Spacetime::new(2.0, 0.25, -0.5, 0.75);
    let original = st;
    st.apply_lorentz_transform_by_rapidity(DVec3::ZERO);
    assert!(st.fuzzy_compare(original));
}

#[test]
fn rapidity_from_momentum_zero_returns_zero() {
    assert_eq!(rapidity_from_momentum(DVec3::ZERO, 1.0, 1.0), DVec3::ZERO);
}

#[test]
fn rapidity_vector_zero_returns_zero() {
    assert_eq!(rapidity_vector(DVec3::ZERO, 1.0), DVec3::ZERO);
}

#[test]
fn apply_lorentz_transform_with_identity_spacetime_is_noop() {
    let mut st = Spacetime::new(1.0, 2.0, 3.0, 4.0);
    let original = st;
    let g = Spacetime::identity();
    st.apply_lorentz_transform(g);
    assert!(st.fuzzy_compare(original));
}

#[test]
fn exp_versor_matches_exp_vector_form() {
    let a = 0.37;
    let v = DVec3::new(0.6, -0.2, 0.1).normalize();
    let e1 = Spacetime::exp_versor(v.x, v.y, v.z, a);
    let e2 = Spacetime::exp(a, v);
    assert!(e1.fuzzy_compare(e2));
}
