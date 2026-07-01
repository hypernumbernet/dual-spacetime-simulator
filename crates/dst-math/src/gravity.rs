//! DST gravity: Newtonian potential drives oscillating time delay via cos(λ_eff).

use glam::DVec3;

/// Scaling constant k = 2/c² in simulation units (c_sim = c/scale).
pub fn k_scale_from_light_speed(light_speed_sim: f64) -> f64 {
    2.0 / (light_speed_sim * light_speed_sim)
}

/// Per-pair potential (linear softening) and Newtonian acceleration toward `j`.
pub fn newtonian_gravity_pair(
    pos_i: DVec3,
    pos_j: DVec3,
    mass_j: f64,
    g: f64,
    time_g: f64,
    epsilon: f64,
) -> (f64, DVec3) {
    let diff = pos_j - pos_i;
    let distance_sq = diff.length_squared();
    let distance = distance_sq.sqrt();
    let phi_j = -g * mass_j / (distance + epsilon);
    let acceleration = if distance_sq < epsilon {
        DVec3::ZERO
    } else {
        time_g * mass_j / distance_sq * (diff / distance)
    };
    (phi_j, acceleration)
}

/// Newtonian gravitational potential at particle `i`: Φ = -Σ G m_j / (|r_ij| + ε).
pub fn gravitational_potential_at(
    i: usize,
    positions: &[DVec3],
    masses: &[f64],
    g: f64,
    epsilon: f64,
) -> f64 {
    let pos_i = positions[i];
    positions
        .iter()
        .enumerate()
        .filter(|(j, _)| *j != i)
        .map(|(j, &pos_j)| newtonian_gravity_pair(pos_i, pos_j, masses[j], g, 0.0, epsilon).0)
        .sum()
}

/// Effective time progression rate: dτ/dt = cos(λ_eff).
pub fn time_dilation(lambda_eff: f64) -> f64 {
    lambda_eff.cos()
}

/// Returns -1 when dτ/dt is negative (time-reversed sector), otherwise +1.
pub fn gravity_sign_from_time_dilation(dilation: f64) -> f64 {
    if dilation.is_sign_negative() {
        -1.0
    } else {
        1.0
    }
}

/// DST gravity velocity delta, λ_eff, and proper-time increment for particle `i`.
///
/// Single O(N) pass over neighbors: potential and acceleration share pairwise geometry.
pub fn dst_gravity_step_at(
    i: usize,
    positions: &[DVec3],
    masses: &[f64],
    g: f64,
    time_g: f64,
    k_scale: f64,
    epsilon: f64,
    delta_seconds: f64,
) -> (DVec3, f64, f64) {
    let pos_i = positions[i];
    let mut phi = 0.0;
    let mut acceleration = DVec3::ZERO;
    for (j, &pos_j) in positions.iter().enumerate() {
        if j == i {
            continue;
        }
        let (phi_j, accel_j) = newtonian_gravity_pair(pos_i, pos_j, masses[j], g, time_g, epsilon);
        phi += phi_j;
        acceleration += accel_j;
    }
    let lambda_eff = k_scale * phi;
    let dilation = time_dilation(lambda_eff);
    (
        gravity_sign_from_time_dilation(dilation) * acceleration,
        lambda_eff,
        delta_seconds * dilation,
    )
}

/// Updates λ_eff and accumulates proper time for one particle.
pub fn update_time_delay_for_particle(
    proper_time: &mut f64,
    lambda_eff: &mut f64,
    phi: f64,
    k_scale: f64,
    delta_seconds: f64,
) {
    *lambda_eff = k_scale * phi;
    *proper_time += delta_seconds * time_dilation(*lambda_eff);
}
