//! DST gravity helpers: torsion mismatch, interaction kernel, and accelerations.

use glam::DVec3;

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
