//! DST gravity helpers: torsion mismatch, interaction kernel, accelerations, and S³ Ln gravity.

use glam::{DQuat, DVec3};

use crate::bivector::BivectorBoost;

/// Maximum |v|/c ratio passed to `atanh` when building θ(v).
const MAX_RAPIDITY_RATIO: f64 = 0.999_999_9;

/// Reference separation (sim units) where strong-field repulsion reaches ~50 % strength.
pub const STRONG_FIELD_RADIUS: f64 = 1.0;

/// Returns usual-sector boost components θ(v) from a velocity vector (|v| < 1 in natural units).
pub fn usual_boost_from_velocity(velocity: DVec3) -> DVec3 {
    let boost = BivectorBoost::from_velocity(velocity.x, velocity.y, velocity.z);
    DVec3::new(boost.i, boost.j, boost.k)
}

/// Returns usual-sector boost θ(v) for simulation-unit velocity and `inverse_light_speed` (= scale / c).
pub fn usual_boost_from_sim_velocity(velocity: DVec3, inverse_light_speed: f64) -> DVec3 {
    let speed_sq = velocity.length_squared();
    if speed_sq == 0.0 {
        return DVec3::ZERO;
    }
    let speed = speed_sq.sqrt();
    let mut ratio = speed * inverse_light_speed;
    if ratio >= MAX_RAPIDITY_RATIO {
        ratio = MAX_RAPIDITY_RATIO;
    }
    let rapidity = ratio.atanh();
    (rapidity / speed) * velocity
}

/// Torsion mismatch δθ = φ − θ(v) in natural velocity units.
pub fn torsion_mismatch(dual_rotor: DVec3, velocity: DVec3) -> DVec3 {
    dual_rotor - usual_boost_from_velocity(velocity)
}

/// Torsion mismatch δθ = φ − θ(v) for simulation-unit velocities.
pub fn torsion_mismatch_sim(
    dual_rotor: DVec3,
    velocity: DVec3,
    inverse_light_speed: f64,
) -> DVec3 {
    dual_rotor - usual_boost_from_sim_velocity(velocity, inverse_light_speed)
}

/// Killing scalar J ≈ ½ Σ(θ_a² − φ_a²) = ½(|θ|² − |φ|²).
///
/// J > 0: usual boost dominates (attraction). J < 0: dual rotation dominates (repulsion in strong fields).
pub fn killing_scalar(usual_boost: DVec3, dual_rotor: DVec3) -> f64 {
    0.5 * (usual_boost.length_squared() - dual_rotor.length_squared())
}

/// Smooth gate in [0, 1] from weak to strong gravitational field (G / r²).
pub fn strong_field_gate(g: f64, r_soft: f64) -> f64 {
    let field = g / (r_soft * r_soft);
    let reference = g / (STRONG_FIELD_RADIUS * STRONG_FIELD_RADIUS);
    let x = field / reference.max(1e-30);
    (x * x) / (1.0 + x * x)
}

/// Signed interaction weight: positive → attraction, negative → repulsion (gated by field strength).
pub fn signed_interaction_weight(
    j_local: f64,
    phi_i: DVec3,
    phi_j: DVec3,
    g: f64,
    r_soft: f64,
) -> f64 {
    let geom = 1.0 + (phi_i - phi_j).length_squared();
    if j_local >= 0.0 {
        geom
    } else {
        -geom * strong_field_gate(g, r_soft)
    }
}

/// Proper-time rate f = cos(|δθ| / 2).
pub fn proper_time_rate(delta_theta: DVec3) -> f64 {
    (0.5 * delta_theta.length()).cos()
}

/// Softened kernel K(r) = g / r_soft and its derivative dK/dr.
pub fn kernel_and_deriv(distance: f64, g: f64, epsilon: f64) -> (f64, f64) {
    let r_soft = (distance * distance + epsilon * epsilon).sqrt();
    let k = g / r_soft;
    let k_deriv = -g * distance / (r_soft * r_soft * r_soft);
    (k, k_deriv)
}

/// Computes position acceleration for particle `index` from the J_total gradient.
///
/// a_i = Σ_{j≠i} m_j · signed_weight · (g / r_soft³) · (r_j − r_i)
pub fn acceleration_at(
    index: usize,
    positions: &[DVec3],
    masses: &[f64],
    usual_boosts: &[DVec3],
    dual_rotors: &[DVec3],
    g: f64,
    epsilon: f64,
) -> DVec3 {
    let pos_i = positions[index];
    let phi_i = dual_rotors[index];
    let j_i = killing_scalar(usual_boosts[index], phi_i);
    let mut acceleration = DVec3::ZERO;

    for (j, (&pos_j, &mass_j)) in positions.iter().zip(masses.iter()).enumerate() {
        if index == j {
            continue;
        }
        let diff = pos_j - pos_i;
        let distance_sq = diff.length_squared();
        if distance_sq < epsilon {
            continue;
        }
        let r_soft = (distance_sq + epsilon * epsilon).sqrt();
        let weight = signed_interaction_weight(j_i, phi_i, dual_rotors[j], g, r_soft);
        let magnitude = mass_j * weight * g / (r_soft * r_soft * r_soft);
        acceleration += magnitude * diff;
    }

    acceleration
}

/// Computes dual-rotor angular acceleration φ̈ for particle `index`.
pub fn dual_rotor_accel_at(
    index: usize,
    positions: &[DVec3],
    masses: &[f64],
    dual_rotors: &[DVec3],
    dual_rotor_vel: DVec3,
    delta_theta: DVec3,
    g: f64,
    epsilon: f64,
) -> DVec3 {
    let mass_i = masses[index].max(epsilon);
    let omega = g.sqrt();
    let mut accel = -g * delta_theta - 2.0 * omega * dual_rotor_vel;

    let pos_i = positions[index];
    let phi_i = dual_rotors[index];
    for (j, (&pos_j, &mass_j)) in positions.iter().zip(masses.iter()).enumerate() {
        if index == j {
            continue;
        }
        let diff = pos_j - pos_i;
        let distance_sq = diff.length_squared();
        if distance_sq < epsilon {
            continue;
        }
        let (kernel, _) = kernel_and_deriv(distance_sq.sqrt(), g, epsilon);
        accel += -2.0 * (mass_j / mass_i) * kernel * (phi_i - dual_rotors[j]);
    }

    accel
}

/// Clamps dual-rotor state; repulsion-layer particles (J < 0) keep large φ magnitudes.
pub fn clamp_dual_rotor_state(
    dual_rotor: &mut DVec3,
    dual_rotor_vel: &mut DVec3,
    velocity: DVec3,
    inverse_light_speed: f64,
) {
    let theta = usual_boost_from_sim_velocity(velocity, inverse_light_speed);
    let j = killing_scalar(theta, *dual_rotor);

    let max_phi = if j < 0.0 {
        theta.length().max(1e-8).mul_add(100.0, 10.0)
    } else {
        theta.length().max(1e-8).mul_add(8.0, 0.1)
    };
    let phi_len = dual_rotor.length();
    if phi_len > max_phi {
        *dual_rotor *= max_phi / phi_len;
    }
    let max_vel = max_phi * 100.0;
    let vel_len = dual_rotor_vel.length();
    if vel_len > max_vel {
        *dual_rotor_vel *= max_vel / vel_len;
    }
}

/// Dual-rotor vector for a repulsion-shell particle (θ ≈ 0, J < 0).
pub fn repulsion_shell_dual_rotor(phi_magnitude: f64) -> DVec3 {
    DVec3::new(phi_magnitude.max(0.0), 0.0, 0.0)
}

/// Schwarzschild radius for a two-body system: r_s = 2 G (m₁ + m₂) / c².
pub fn schwarzschild_radius(mass_a: f64, mass_b: f64, g: f64, light_speed: f64) -> f64 {
    2.0 * g * (mass_a + mass_b) / (light_speed * light_speed)
}

/// S³ logarithm from a rotation angle and axis without `acos` branch cuts.
///
/// For half-angle `θ/2 ≥ π/2` (i.e. separation inside the calibrated event horizon),
/// the vector flips sign to produce repulsive inversion.
pub fn s3_log_from_rotation_angle(angle: f64, axis: DVec3) -> DVec3 {
    let axis_len = axis.length();
    if axis_len < 1e-30 || angle.abs() < 1e-30 {
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

/// Principal-branch logarithm of a unit quaternion on S³ paired with its rotation angle.
///
/// `momentum_axis` supplies the exchange direction (not the signed imaginary part of `q`).
pub fn unit_quaternion_ln(q: DQuat, rotation_angle: f64, momentum_axis: DVec3) -> DVec3 {
    let _ = q;
    s3_log_from_rotation_angle(rotation_angle, momentum_axis)
}

/// Momentum-exchange magnitude at `separation = schwarzschild_radius` for the same pair and dt.
///
/// Because `p ∝ 1/r²`, the horizon reference is `p_horizon = p · (r/r_s)²`.
pub fn horizon_reference_momentum(
    momentum_magnitude: f64,
    separation: f64,
    schwarzschild_r: f64,
) -> f64 {
    if separation <= 0.0 || schwarzschild_r <= 0.0 {
        return momentum_magnitude.max(0.0);
    }
    momentum_magnitude * (separation / schwarzschild_r).powi(2)
}

/// Maps momentum-exchange magnitude to a rotation angle on S³ via the horizon ratio `p/p_horizon`.
///
/// At `separation == schwarzschild_radius` we have `p == p_horizon` and the angle equals π,
/// placing the quaternion on the equator of S³ where the logarithm branch flips.
pub fn momentum_to_s3_angle(
    momentum_magnitude: f64,
    separation: f64,
    schwarzschild_r: f64,
) -> f64 {
    if separation <= 0.0 || schwarzschild_r <= 0.0 || momentum_magnitude <= 0.0 {
        return 0.0;
    }
    let p_horizon = horizon_reference_momentum(momentum_magnitude, separation, schwarzschild_r);
    std::f64::consts::PI * momentum_magnitude / p_horizon
}

/// Builds a unit quaternion on S³ from a momentum-exchange direction and S³ rotation angle.
///
/// Constructed directly as `(cos θ/2, sin θ/2 · û)` without angle wrapping so the logarithm
/// branch can cross the equator at the calibrated event-horizon scale.
pub fn unit_quaternion_from_momentum_axis(axis: DVec3, angle: f64) -> DQuat {
    let axis_len = axis.length();
    if axis_len < 1e-30 || angle.abs() < 1e-30 {
        return DQuat::IDENTITY;
    }
    let unit = axis / axis_len;
    let half = 0.5 * angle;
    let (s, c) = half.sin_cos();
    DQuat::from_xyzw(unit.x * s, unit.y * s, unit.z * s, c)
}

/// Velocity increment from pairwise momentum exchange via the S³ logarithm map.
///
/// The exchange magnitude `p/m_i = G m_j / r² · dt` matches Newtonian gravity in the
/// weak field. The S³ logarithm supplies only the direction (including the horizon flip);
/// multiplying `ln` by `p/m_i` directly would suppress weak-field gravity by `(r_s/r)²` and
/// amplify strong-field repulsion by the same factor.
///
/// All inputs use simulation units: `diff` and derived separations are length/scale,
/// masses are kg/scale³, and `light_speed` is c/scale. The Schwarzschild radius and
/// momentum exchange are evaluated consistently so the inversion occurs at r = r_s in
/// simulation coordinates (matching the GPU `light_speed_per_scale` push constant).
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
    if mass_i <= 0.0 {
        return DVec3::ZERO;
    }
    let r_soft_sq = distance_sq + epsilon * epsilon;
    if r_soft_sq <= 0.0 {
        return DVec3::ZERO;
    }
    let distance = r_soft_sq.sqrt();
    let rs = schwarzschild_radius(mass_i, mass_j, g, light_speed);
    // p = G m_i m_j / r² dt  and  p/m_i = G m_j / r² dt — avoid m_i·m_j product for f32 GPU parity.
    let momentum_per_mass_i = g * mass_j / r_soft_sq * delta_seconds;
    let angle = momentum_to_s3_angle(
        momentum_per_mass_i * mass_i,
        distance,
        rs,
    );
    let q = unit_quaternion_from_momentum_axis(diff, angle);
    let ln = unit_quaternion_ln(q, angle, diff);
    let ln_len = ln.length();
    if ln_len < 1e-30 {
        return DVec3::ZERO;
    }
    (ln / ln_len) * momentum_per_mass_i
}
