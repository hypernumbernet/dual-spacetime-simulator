//! Extra integration coverage for `math::spacetime` (see also crate-local `#[cfg(test)]`).

use dual_spacetime_simulator::math::spacetime::{
    rapidity_from_momentum, rapidity_vector, Spacetime,
};
use glam::DVec3;

#[test]
fn lorentz_transformation_v_zero_is_noop() {
    let mut st = Spacetime::new(1.0, 0.5, -0.25, 0.125);
    let original = st;
    st.lorentz_transformation_v(DVec3::ZERO, 1.0 / 299_792_458.0);
    assert!(st.fuzzy_compare(original));
}

#[test]
fn lorentz_transformation_rapidity_zero_is_noop() {
    let mut st = Spacetime::new(2.0, 0.25, -0.5, 0.75);
    let original = st;
    st.lorentz_transformation_rapidity(DVec3::ZERO);
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
fn lorentz_transformation_with_identity_spacetime_is_noop() {
    let mut st = Spacetime::new(1.0, 2.0, 3.0, 4.0);
    let original = st;
    let g = Spacetime::identity();
    st.lorentz_transformation(g);
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
