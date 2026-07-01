//! DST gravity: momentum exchange mapped through S³ logarithm with natural repulsion flip.

use glam::{DQuat, DVec3};

use crate::spacetime::{rapidity_from_momentum, Spacetime};

const ANGLE_EPSILON: f64 = 1e-30;

/// S³ logarithm from a rotation angle and axis without angle wrapping.
///
/// For half-angle `θ/2 ≥ π/2` (i.e. `θ ≥ π`), the tangent vector flips sign,
/// producing repulsive inversion via the log branch cut.
pub fn s3_log_from_rotation_angle(angle: f64, axis: DVec3) -> DVec3 {
    let axis_len = axis.length();
    if axis_len < ANGLE_EPSILON || angle.abs() < ANGLE_EPSILON {
        return DVec3::ZERO;
    }
    let unit = axis / axis_len;
    let half = 0.5 * angle;
    if half >= std::f64::consts::FRAC_PI_2 {
        -unit * half
    } else {
        unit * half
    }
}

/// Builds a unit quaternion on S³ from a momentum-exchange direction and rotation angle.
///
/// Constructed as `(cos θ/2, sin θ/2 · û)` without angle wrapping so the logarithm
/// branch can cross the equator at strong gravitational coupling.
pub fn unit_quaternion_from_momentum_axis(axis: DVec3, angle: f64) -> DQuat {
    let axis_len = axis.length();
    if axis_len < ANGLE_EPSILON || angle.abs() < ANGLE_EPSILON {
        return DQuat::IDENTITY;
    }
    let unit = axis / axis_len;
    let half = 0.5 * angle;
    let (s, c) = half.sin_cos();
    DQuat::from_xyzw(unit.x * s, unit.y * s, unit.z * s, c)
}

/// Principal-branch logarithm of a unit quaternion on S³ paired with its rotation angle.
///
/// `momentum_axis` supplies the exchange direction (not the signed imaginary part of `q`).
pub fn unit_quaternion_ln(q: DQuat, rotation_angle: f64, momentum_axis: DVec3) -> DVec3 {
    let _ = q;
    s3_log_from_rotation_angle(rotation_angle, momentum_axis)
}

/// Velocity increment from pairwise momentum exchange via the S³ logarithm map.
///
/// Pipeline: impulse → rapidity angle θ → quaternion → Ln → velocity.
/// All inputs use simulation units: `diff` and derived separations are length/scale,
/// masses are kg/scale³, and `light_speed` is c/scale.
pub fn dst_gravity_velocity_delta(
    mass_i: f64,
    mass_j: f64,
    diff: DVec3,
    g: f64,
    light_speed: f64,
    delta_seconds: f64,
    epsilon: f64,
) -> DVec3 {
    let distance_sq = diff.length_squared();
    if mass_i <= 0.0 || mass_j <= 0.0 {
        return DVec3::ZERO;
    }
    let r_soft_sq = distance_sq + epsilon * epsilon;
    if r_soft_sq <= 0.0 {
        return DVec3::ZERO;
    }
    let distance = r_soft_sq.sqrt();
    if distance < ANGLE_EPSILON {
        return DVec3::ZERO;
    }

    // Momentum exchange: p = G m_i m_j / r² dt r̂
    let impulse = diff * (g * mass_i * mass_j / r_soft_sq * delta_seconds / distance);
    let rapidity = rapidity_from_momentum(impulse, mass_i, light_speed);
    let theta = rapidity.length_squared();
    if theta < ANGLE_EPSILON {
        return DVec3::ZERO;
    }
    let axis = rapidity / theta;
    let q = unit_quaternion_from_momentum_axis(axis, theta);
    let xi = unit_quaternion_ln(q, theta, axis);
    Spacetime::velocities(2.0 * xi, light_speed)
}