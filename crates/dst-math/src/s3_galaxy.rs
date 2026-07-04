//! S³ galaxy gravity via Ln (log map) on unit quaternions.

use glam::{DQuat, DVec3};

/// Fixed galaxy radius in light years (observation-derived constant).
pub const GALAXY_RADIUS_LY: f64 = 50_000.0;

/// Speed of light in m/s (Julian year basis for light year).
pub const LIGHT_SPEED: f64 = 299_792_458.0;

/// Julian light year in meters.
pub const LY: f64 = LIGHT_SPEED * 365.25 * 86_400.0;

/// Galaxy radius R in simulation length units for the given world scale (meters per sim unit).
pub fn galaxy_radius_sim(scale_m: f64) -> f64 {
    GALAXY_RADIUS_LY * LY / scale_m
}

/// Log map S³ → su(2) ≅ ℝ³. Does not force shortest-path branch selection.
/// atan2-based: keeps full precision at small angles where acos(w) collapses to 0.
pub fn quaternion_log(q: DQuat) -> DVec3 {
    let xyz = DVec3::new(q.x, q.y, q.z);
    let vnorm = xyz.length();
    if vnorm == 0.0 {
        DVec3::ZERO
    } else {
        let half_angle = vnorm.atan2(q.w);
        xyz * (half_angle / vnorm)
    }
}

/// Exp map ℝ³ → S³. For v = (θ/2)·û, returns cos(θ/2) + sin(θ/2)·û.
pub fn quaternion_exp(v: DVec3) -> DQuat {
    let vmag = v.length();
    if vmag == 0.0 {
        return DQuat::IDENTITY;
    }
    let u = v / vmag;
    DQuat::from_xyzw(
        u.x * vmag.sin(),
        u.y * vmag.sin(),
        u.z * vmag.sin(),
        vmag.cos(),
    )
}

/// Space-frame displacement quaternion from particle i to j: q_j · q_i⁻¹.
/// Ln of this points from i toward j, in the same frame the orientation
/// integrator (left multiplication) moves in, so a close pair reduces to
/// Newtonian motion anywhere on S³ without frame mismatch.
pub fn relative_quaternion(q_i: DQuat, q_j: DQuat) -> DQuat {
    q_j * q_i.conjugate()
}

/// Radial distance from Ln vector: r = (2|v|/π) · R (180° rotation = R).
pub fn radial_distance_ln(v: DVec3, r_galaxy: f64) -> f64 {
    (2.0 * v.length() / std::f64::consts::PI) * r_galaxy
}

/// Pairwise gravity in the Ln-projected chart, from display positions
/// p = Ln(q)·(2R/π). Both distance and direction are taken in the projected
/// 3D space — NOT along the S³ geodesic — so the projection distortion of
/// magnitude and direction feeds the dynamics untouched. Combined with the
/// exp-map integrator on S³ this produces the DST distortion and chirality
/// effects; near the origin the chart is flat and this reduces exactly to
/// Newtonian gravity.
pub fn galaxy_gravity_pair_chart(
    p_i: DVec3,
    p_j: DVec3,
    mass_j: f64,
    r_galaxy: f64,
    time_g: f64,
    epsilon: f64,
) -> DVec3 {
    let d = p_j - p_i;
    let r = d.length();
    if r == 0.0 {
        return DVec3::ZERO;
    }
    let r_eff = r.max(epsilon);
    // Attractive within R, repulsive beyond R (DST sign reversal).
    let sign = if r > r_galaxy { -1.0 } else { 1.0 };
    (sign * time_g * mass_j / (r_eff * r_eff)) * (d / r)
}

/// Quaternion wrapper over [`galaxy_gravity_pair_chart`]: projects both
/// orientations to the display chart via the log map first.
pub fn galaxy_gravity_pair_ln(
    q_i: DQuat,
    q_j: DQuat,
    mass_j: f64,
    r_galaxy: f64,
    time_g: f64,
    epsilon: f64,
) -> DVec3 {
    let p_i = orientation_to_display_position(q_i, r_galaxy);
    let p_j = orientation_to_display_position(q_j, r_galaxy);
    galaxy_gravity_pair_chart(p_i, p_j, mass_j, r_galaxy, time_g, epsilon)
}

/// Total chart-space gravity at particle i from all neighbors.
/// `positions` are display positions p = Ln(q)·(2R/π), kept in sync with the
/// orientations by the integrator.
pub fn galaxy_gravity_step_at(
    i: usize,
    positions: &[DVec3],
    masses: &[f64],
    r_galaxy: f64,
    time_g: f64,
    epsilon: f64,
) -> DVec3 {
    let p_i = positions[i];
    positions
        .iter()
        .enumerate()
        .filter(|(j, _)| *j != i)
        .map(|(j, &p_j)| {
            galaxy_gravity_pair_chart(p_i, p_j, masses[j], r_galaxy, time_g, epsilon)
        })
        .sum()
}

/// Integrates orientation on S³ from a linear velocity in sim units per second:
/// q ← exp(vel·π/(2R)·dt) · q. Inverse-consistent with the display map
/// p = Ln(q)·(2R/π), so near-field motion satisfies Δp = vel·dt exactly.
pub fn integrate_orientation(q: DQuat, velocity: DVec3, r_galaxy: f64, dt: f64) -> DQuat {
    let v = velocity * (std::f64::consts::PI / (2.0 * r_galaxy) * dt);
    (quaternion_exp(v) * q).normalize()
}

/// Builds unit quaternion from a 3D position: v = p·π/(2R), q = exp(v).
pub fn orientation_from_disk_position(pos: DVec3, r_galaxy: f64) -> DQuat {
    let scale = std::f64::consts::PI / (2.0 * r_galaxy);
    let v = pos * scale;
    if v.length_squared() < 1e-40 {
        return DQuat::IDENTITY;
    }
    quaternion_exp(v)
}

/// Maps S³ orientation back to 3D display position: p = Ln(q)·(2R/π).
pub fn orientation_to_display_position(q: DQuat, r_galaxy: f64) -> DVec3 {
    let v = quaternion_log(q);
    if v.length_squared() < 1e-40 {
        return DVec3::ZERO;
    }
    v * (2.0 * r_galaxy / std::f64::consts::PI)
}
