use dst_math::bivector::BivectorBoost;

#[test]
fn norm_squared_matches_norm_squared() {
    let b = BivectorBoost::new(0.3, -0.4, 0.5);
    let n = b.norm();
    assert!((b.norm_squared() - n * n).abs() < 1e-12);
}

#[test]
fn exp_zero_is_identity_boost() {
    let e = BivectorBoost::new(0.0, 0.0, 0.0).exp();
    assert!((e.scalar - 1.0).abs() < 1e-12);
    assert!((e.i + e.j + e.k).abs() < 1e-12);
}

#[test]
fn from_velocity_zero_no_nan() {
    let b = BivectorBoost::from_velocity(0.0, 0.0, 0.0);
    assert!(b.i.is_finite() && b.j.is_finite() && b.k.is_finite());
    assert_eq!(b.norm(), 0.0);
}

#[test]
fn exp_satisfies_hyperbolic_identity() {
    let b = BivectorBoost::new(0.12, -0.07, 0.21);
    let e = b.exp();
    let lhs = e.scalar * e.scalar - e.i * e.i - e.j * e.j - e.k * e.k;
    assert!((lhs - 1.0).abs() < 1e-9);
}

#[test]
fn from_velocity_consistent_with_manual_phi() {
    let vx = 0.2;
    let vy = -0.1;
    let vz = 0.05;
    let b = BivectorBoost::from_velocity(vx, vy, vz);
    let speed = (vx * vx + vy * vy + vz * vz).sqrt();
    let phi = speed.atanh();
    let inv = phi / speed;
    assert!((b.i - inv * vx).abs() < 1e-9);
    assert!((b.j - inv * vy).abs() < 1e-9);
    assert!((b.k - inv * vz).abs() < 1e-9);
}
