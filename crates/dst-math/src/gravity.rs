//! DST gravity: Newtonian potential drives oscillating time delay via cos(λ_eff).

use glam::DVec3;

/// Scaling constant k = 2/c² in simulation units (c_sim = c/scale).
pub fn k_scale_from_light_speed(light_speed_sim: f64) -> f64 {
    2.0 / (light_speed_sim * light_speed_sim)
}

/// Newtonian gravitational potential at particle `i`: Φ = -Σ G m_j / (|r_ij| + ε).
pub fn gravitational_potential_at(
    i: usize,
    positions: &[DVec3],
    masses: &[f64],
    g: f64,
    epsilon: f64,
) -> f64 {
    let mut phi = 0.0;
    for (j, &pos_j) in positions.iter().enumerate() {
        if j == i {
            continue;
        }
        let r = (positions[i] - pos_j).length() + epsilon;
        phi -= g * masses[j] / r;
    }
    phi
}

/// Effective time progression rate: dτ/dt = cos(λ_eff).
pub fn time_dilation(lambda_eff: f64) -> f64 {
    lambda_eff.cos()
}

/// Returns -1 when dτ/dt is negative (time-reversed sector), otherwise +1.
pub fn gravity_sign_from_time_dilation(dilation: f64) -> f64 {
    if dilation < 0.0 {
        -1.0
    } else {
        1.0
    }
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
