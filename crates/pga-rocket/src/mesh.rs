//! CPU mesh builders for the grass ground plane and legged rocket body.

use crate::landing::LandingAutopilot;
use crate::sim::RocketState;
use crate::target_landing::TargetLandingAutopilot;
use bytemuck::{Pod, Zeroable};

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct Vertex {
    pub pos: [f32; 3],
    pub color: [f32; 3],
}

/// Translucent FX vertex: rgb premultiplied by alpha, straight destination alpha.
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct FxVertex {
    pub pos: [f32; 3],
    pub color: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct GroundVertex {
    pub pos: [f32; 3],
    pub uv: [f32; 2],
}

/// Half-extent of the local grass plane mesh (meters). Re-centered under the rocket
/// each frame; may be scaled in the vertex shader for high altitude (see
/// [`ground_plane_scale`]).
pub const GROUND_HALF_EXTENT: f32 = 1800.0;
/// World meters covered by one grass texture tile (minecraft-like 1 m block).
pub const GRASS_METERS_PER_TILE: f32 = 1.0;
/// Edge-fog start as a fraction of the effective ground half-extent (horizontal).
/// Fragments with radial distance / half_extent_world below this stay fully lit;
/// the rim fades to sky so the finite plane has no hard horizon.
pub const GROUND_EDGE_FOG_START: f32 = 0.72;
/// How much the ground disk grows with camera eye height: half_extent ≥ eye_y × this.
/// ~2.5 keeps a 45° FOV looking down mostly filled with terrain at high altitude.
pub const GROUND_EXTENT_PER_EYE_Y: f32 = 2.5;
/// Baseline far plane (meters) used near the pad / low flight.
pub const CAMERA_FAR_BASE: f32 = 4000.0;
/// Near plane (meters); keep small so the rocket stays sharp at orbit distance.
pub const CAMERA_NEAR: f32 = 0.5;

/// Effective world-space half-extent of the grass plane for a given eye height.
/// Floors at [`GROUND_HALF_EXTENT`] so low-altitude looks unchanged.
pub fn ground_half_extent_for_eye_height(eye_y: f32) -> f32 {
    let h = eye_y.max(0.0);
    GROUND_HALF_EXTENT.max(h * GROUND_EXTENT_PER_EYE_Y)
}

/// Vertex-shader scale applied to the local grass mesh so its world half-extent
/// matches [`ground_half_extent_for_eye_height`].
pub fn ground_plane_scale(eye_y: f32) -> f32 {
    ground_half_extent_for_eye_height(eye_y) / GROUND_HALF_EXTENT
}

/// Perspective far plane that keeps the scaled ground disk inside the frustum.
///
/// Uses eye height + diagonal of the ground square (half_extent × √2) with margin.
pub fn camera_far_for_eye_height(eye_y: f32) -> f32 {
    let half = ground_half_extent_for_eye_height(eye_y);
    let h = eye_y.max(0.0);
    // Corner of the plane is √2 farther horizontally than the half-edge.
    let slant = (h * h + 2.0 * half * half).sqrt();
    CAMERA_FAR_BASE.max(slant * 1.15)
}

/// Half-extent of the launch / target pad square (meters) → 60 m side.
/// Painted in `ground.frag` on the single grass plane (no separate pad mesh).
pub const LAUNCH_PAD_HALF_EXTENT: f32 = 30.0;
/// World meters covered by one paved texture tile (ground.frag PAD_METERS_PER_TILE).
pub const PAD_METERS_PER_TILE: f32 = 2.0;

/// Minimum horizontal range from launch origin to the random landing target (meters).
pub const TARGET_DISTANCE_MIN_M: f32 = 100.0;
/// Maximum horizontal range from launch origin to the random landing target (meters).
pub const TARGET_DISTANCE_MAX_M: f32 = 8000.0;
/// Target pad uses the same half-extent as the launch pad.
pub const TARGET_PAD_HALF_EXTENT: f32 = LAUNCH_PAD_HALF_EXTENT;
/// High-contrast paint color for pad letter marks (H / T) — must match ground.frag.
pub const PAD_MARK_COLOR: [f32; 3] = [0.95, 0.82, 0.12];

/// Flat local-space grass plane on y = 0, lightly subdivided for depth stability.
pub fn grass_ground_mesh(half_extent: f32, divisions: u32) -> (Vec<GroundVertex>, Vec<u32>) {
    let divs = divisions.max(1);
    let mut verts = Vec::with_capacity(((divs + 1) * (divs + 1)) as usize);
    let mut idx = Vec::with_capacity((divs * divs * 6) as usize);

    for iz in 0..=divs {
        let tz = iz as f32 / divs as f32;
        let z = -half_extent + tz * (2.0 * half_extent);
        for ix in 0..=divs {
            let tx = ix as f32 / divs as f32;
            let x = -half_extent + tx * (2.0 * half_extent);
            verts.push(GroundVertex {
                pos: [x, 0.0, z],
                // UV filled in the vertex shader from world XZ; keep local as fallback.
                uv: [x / GRASS_METERS_PER_TILE, z / GRASS_METERS_PER_TILE],
            });
        }
    }

    let stride = divs + 1;
    for iz in 0..divs {
        for ix in 0..divs {
            let i0 = iz * stride + ix;
            let i1 = i0 + 1;
            let i2 = i0 + stride;
            let i3 = i2 + 1;
            idx.extend_from_slice(&[i0, i2, i1, i1, i2, i3]);
        }
    }
    (verts, idx)
}

/// SplitMix64 step — deterministic u64 stream without extra dependencies.
fn splitmix64(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// Deterministic target in an annulus [`TARGET_DISTANCE_MIN_M`, `TARGET_DISTANCE_MAX_M`]
/// around the launch origin (uniform area in the ring; 100–8000 m).
pub fn target_xz_from_seed(seed: u64) -> [f32; 2] {
    let mut s = seed;
    // Avoid the all-zero fixed point of the additive constant path looking "stuck".
    if s == 0 {
        s = 0xA5A5_A5A5_A5A5_A5A5;
    }
    let u_angle = (splitmix64(&mut s) >> 11) as f64 / ((1u64 << 53) as f64);
    let u_radius = (splitmix64(&mut s) >> 11) as f64 / ((1u64 << 53) as f64);
    let theta = u_angle * std::f64::consts::TAU;
    let r_min = TARGET_DISTANCE_MIN_M as f64;
    let r_max = TARGET_DISTANCE_MAX_M as f64;
    let r = (u_radius * (r_max * r_max - r_min * r_min) + r_min * r_min).sqrt();
    [(r * theta.cos()) as f32, (r * theta.sin()) as f32]
}

/// Wall-clock seeded target for play sessions.
pub fn random_target_xz() -> [f32; 2] {
    use std::time::{SystemTime, UNIX_EPOCH};
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(1);
    target_xz_from_seed(nanos)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn target_from_seed_is_within_range() {
        for seed in [1u64, 42, 999, u64::MAX / 3] {
            let [x, z] = target_xz_from_seed(seed);
            let r = (x * x + z * z).sqrt();
            assert!(
                r >= TARGET_DISTANCE_MIN_M - 1e-2 && r <= TARGET_DISTANCE_MAX_M + 1e-2,
                "seed={seed} r={r}"
            );
        }
    }

    #[test]
    fn different_seeds_change_bearing_and_radius() {
        let a = target_xz_from_seed(1);
        let b = target_xz_from_seed(2);
        let c = target_xz_from_seed(9999);
        assert!(a != b || b != c);
        let radii: Vec<f32> = [1u64, 2, 9999, 12345, 67890]
            .iter()
            .map(|&seed| {
                let [x, z] = target_xz_from_seed(seed);
                (x * x + z * z).sqrt()
            })
            .collect();
        let min_r = radii.iter().copied().fold(f32::INFINITY, f32::min);
        let max_r = radii.iter().copied().fold(f32::NEG_INFINITY, f32::max);
        assert!(
            max_r - min_r > 100.0,
            "expected spread in radii, got min={min_r} max={max_r}"
        );
    }

    #[test]
    fn pad_half_extent_matches_shader_constant() {
        // ground.frag uses PAD_HALF = 30.0; keep Rust and GLSL in lockstep.
        assert!((LAUNCH_PAD_HALF_EXTENT - 30.0).abs() < 1e-6);
        assert!((TARGET_PAD_HALF_EXTENT - LAUNCH_PAD_HALF_EXTENT).abs() < 1e-6);
        assert!((PAD_METERS_PER_TILE - 2.0).abs() < 1e-6);
    }

    #[test]
    fn ground_scale_is_one_near_the_pad() {
        assert!((ground_plane_scale(0.0) - 1.0).abs() < 1e-6);
        assert!((ground_plane_scale(100.0) - 1.0).abs() < 1e-6);
        // Still under the floor: 1800 / 2.5 = 720 m eye height.
        assert!((ground_plane_scale(700.0) - 1.0).abs() < 1e-6);
        assert!((ground_half_extent_for_eye_height(50.0) - GROUND_HALF_EXTENT).abs() < 1e-3);
    }

    #[test]
    fn ground_extent_grows_with_high_altitude() {
        let half = ground_half_extent_for_eye_height(2000.0);
        assert!(half > GROUND_HALF_EXTENT);
        assert!((half - 2000.0 * GROUND_EXTENT_PER_EYE_Y).abs() < 1e-3);
        assert!(ground_plane_scale(2000.0) > 1.0);
        assert!(ground_plane_scale(5000.0) > ground_plane_scale(2000.0));
    }

    #[test]
    fn camera_far_covers_high_altitude_ground() {
        assert!((camera_far_for_eye_height(50.0) - CAMERA_FAR_BASE).abs() < 1e-3);
        let far_hi = camera_far_for_eye_height(3000.0);
        assert!(far_hi > CAMERA_FAR_BASE);
        // Far must exceed the slant range to a ground-plane corner.
        let half = ground_half_extent_for_eye_height(3000.0);
        let slant = (3000.0f32 * 3000.0 + 2.0 * half * half).sqrt();
        assert!(far_hi >= slant);
    }

    #[test]
    fn edge_fog_start_is_inside_unit_interval() {
        assert!(GROUND_EDGE_FOG_START > 0.0 && GROUND_EDGE_FOG_START < 1.0);
    }
}

/// Build a legged rocket mesh in world space from PGA-transformed body points.
pub fn rocket_mesh(state: &RocketState) -> (Vec<Vertex>, Vec<u32>) {
    if state.destroyed {
        return crate::explosion::explosion_opaque_mesh(state);
    }

    let mut verts = Vec::new();
    let mut idx = Vec::new();

    let hh = state.params.body_half_height as f32;
    let r = state.params.body_radius as f32;

    // Body = octagonal prism + nose cone + metal engine bell + 4 legs.
    let body_col = [0.75, 0.78, 0.82];
    // Cool metal (not orange/red — that was reading as a permanent flame).
    let nozzle_col = [0.28, 0.30, 0.34];
    let nozzle_rim_col = [0.42, 0.44, 0.48];
    let leg_col = [0.55, 0.55, 0.6];
    let nose_col = [0.9, 0.9, 0.95];

    let segs = 8u32;
    // Cylinder from y=-hh to y=+hh in body frame.
    let mut ring_lo = Vec::new();
    let mut ring_hi = Vec::new();
    for i in 0..segs {
        let a = (i as f32) * std::f32::consts::TAU / segs as f32;
        let (s, c) = a.sin_cos();
        ring_lo.push(body_to_world(state, [c * r, -hh, s * r]));
        ring_hi.push(body_to_world(state, [c * r, hh * 0.7, s * r]));
    }

    let base = 0u32;
    for p in &ring_lo {
        verts.push(Vertex {
            pos: *p,
            color: body_col,
        });
    }
    for p in &ring_hi {
        verts.push(Vertex {
            pos: *p,
            color: body_col,
        });
    }
    for i in 0..segs {
        let i0 = base + i;
        let i1 = base + (i + 1) % segs;
        let j0 = base + segs + i;
        let j1 = base + segs + (i + 1) % segs;
        idx.extend_from_slice(&[i0, i1, j1, i0, j1, j0]);
    }

    // Nose tip
    let nose = body_to_world(state, [0.0, hh, 0.0]);
    let nose_i = verts.len() as u32;
    verts.push(Vertex {
        pos: nose,
        color: nose_col,
    });
    for i in 0..segs {
        let i0 = base + segs + i;
        let i1 = base + segs + (i + 1) % segs;
        idx.extend_from_slice(&[i0, i1, nose_i]);
    }

    // Engine bell: open metal frustum (throat → exit), gimbal-tilted about throat.
    // Cache gimbal once per mesh build (avoids rebuilding rotor per vertex).
    let nozzle_xf = NozzleXform::from_state(state);
    let throat_r = r * 0.45;
    let exit_r = r * 0.95;
    let throat_y = -hh;
    let exit_y = state.params.nozzle_exit_y() as f32;
    let throat_base = verts.len() as u32;
    for i in 0..segs {
        let a = (i as f32) * std::f32::consts::TAU / segs as f32;
        let (s, c) = a.sin_cos();
        verts.push(Vertex {
            pos: nozzle_xf.map(state, [c * throat_r, throat_y, s * throat_r]),
            color: nozzle_col,
        });
    }
    let exit_base = verts.len() as u32;
    for i in 0..segs {
        let a = (i as f32) * std::f32::consts::TAU / segs as f32;
        let (s, c) = a.sin_cos();
        verts.push(Vertex {
            pos: nozzle_xf.map(state, [c * exit_r, exit_y, s * exit_r]),
            color: nozzle_rim_col,
        });
    }
    for i in 0..segs {
        let i0 = throat_base + i;
        let i1 = throat_base + (i + 1) % segs;
        let j0 = exit_base + i;
        let j1 = exit_base + (i + 1) % segs;
        idx.extend_from_slice(&[i0, j0, i1, i1, j0, j1]);
    }

    // Exhaust plume starts just past the bell exit; absent at zero throttle.
    append_exhaust_plume(
        &mut verts, &mut idx, state, &nozzle_xf, exit_y, exit_r, segs,
    );

    // Center-body roll RCS jets when commanded.
    append_rcs_plumes(&mut verts, &mut idx, state);

    // Landing legs: line from body attach to foot, thickened as thin boxes.
    for foot in &state.params.leg_feet {
        let foot_w = body_to_world(state, [foot[0] as f32, foot[1] as f32, foot[2] as f32]);
        let attach = body_to_world(
            state,
            [foot[0] as f32 * 0.35, -hh * 0.6, foot[2] as f32 * 0.35],
        );
        append_leg_prism(&mut verts, &mut idx, attach, foot_w, 0.18, leg_col);
    }

    (verts, idx)
}

/// Oriented box used for explosion debris shards (elongated along `dir`).
pub(crate) fn append_oriented_box(
    verts: &mut Vec<Vertex>,
    idx: &mut Vec<u32>,
    center: [f32; 3],
    dir: [f32; 3],
    half: f32,
    color: [f32; 3],
) {
    let len = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2])
        .sqrt()
        .max(1e-4);
    let d = [dir[0] / len, dir[1] / len, dir[2] / len];
    let up = if d[1].abs() < 0.9 {
        [0.0, 1.0, 0.0]
    } else {
        [1.0, 0.0, 0.0]
    };
    let px = [
        d[1] * up[2] - d[2] * up[1],
        d[2] * up[0] - d[0] * up[2],
        d[0] * up[1] - d[1] * up[0],
    ];
    let pl = (px[0] * px[0] + px[1] * px[1] + px[2] * px[2])
        .sqrt()
        .max(1e-4);
    let px = [px[0] / pl * half, px[1] / pl * half, px[2] / pl * half];
    let py = [
        d[1] * px[2] - d[2] * px[1],
        d[2] * px[0] - d[0] * px[2],
        d[0] * px[1] - d[1] * px[0],
    ];
    let pz = [d[0] * half * 0.6, d[1] * half * 0.6, d[2] * half * 0.6];

    let corners = [
        [
            center[0] - px[0] - py[0] - pz[0],
            center[1] - px[1] - py[1] - pz[1],
            center[2] - px[2] - py[2] - pz[2],
        ],
        [
            center[0] + px[0] - py[0] - pz[0],
            center[1] + px[1] - py[1] - pz[1],
            center[2] + px[2] - py[2] - pz[2],
        ],
        [
            center[0] + px[0] + py[0] - pz[0],
            center[1] + px[1] + py[1] - pz[1],
            center[2] + px[2] + py[2] - pz[2],
        ],
        [
            center[0] - px[0] + py[0] - pz[0],
            center[1] - px[1] + py[1] - pz[1],
            center[2] - px[2] + py[2] - pz[2],
        ],
        [
            center[0] - px[0] - py[0] + pz[0],
            center[1] - px[1] - py[1] + pz[1],
            center[2] - px[2] - py[2] + pz[2],
        ],
        [
            center[0] + px[0] - py[0] + pz[0],
            center[1] + px[1] - py[1] + pz[1],
            center[2] + px[2] - py[2] + pz[2],
        ],
        [
            center[0] + px[0] + py[0] + pz[0],
            center[1] + px[1] + py[1] + pz[1],
            center[2] + px[2] + py[2] + pz[2],
        ],
        [
            center[0] - px[0] + py[0] + pz[0],
            center[1] - px[1] + py[1] + pz[1],
            center[2] - px[2] + py[2] + pz[2],
        ],
    ];
    let base = verts.len() as u32;
    for c in &corners {
        verts.push(Vertex { pos: *c, color });
    }
    let faces = [
        [0, 1, 5, 4],
        [1, 2, 6, 5],
        [2, 3, 7, 6],
        [3, 0, 4, 7],
        [0, 3, 2, 1],
        [4, 5, 6, 7],
    ];
    for f in &faces {
        idx.extend_from_slice(&[
            base + f[0],
            base + f[1],
            base + f[2],
            base + f[0],
            base + f[2],
            base + f[3],
        ]);
    }
}

fn body_to_world(state: &RocketState, body: [f32; 3]) -> [f32; 3] {
    use crate::euclidean_pga::{extract_point, point};
    let p = point(body[0] as f64, body[1] as f64, body[2] as f64);
    let w = extract_point(&state.motor.sandwich(&p));
    [w[0] as f32, w[1] as f32, w[2] as f32]
}

/// Cached nozzle gimbal transform: one PGA rotor per mesh rebuild.
struct NozzleXform {
    rotor: crate::euclidean_pga::Multivector,
    pivot_y: f64,
    identity: bool,
}

impl NozzleXform {
    fn from_state(state: &RocketState) -> Self {
        let (pitch, yaw) = state.gimbal_angles();
        let identity = pitch.abs() < 1e-12 && yaw.abs() < 1e-12;
        Self {
            rotor: if identity {
                crate::euclidean_pga::Multivector::one()
            } else {
                state.gimbal_rotor()
            },
            pivot_y: -state.params.body_half_height,
            identity,
        }
    }

    fn map(&self, state: &RocketState, body: [f32; 3]) -> [f32; 3] {
        if self.identity {
            return body_to_world(state, body);
        }
        use crate::sim::rotate_vector_by_rotor;
        let rel = [
            body[0] as f64,
            body[1] as f64 - self.pivot_y,
            body[2] as f64,
        ];
        let rot = rotate_vector_by_rotor(&self.rotor, rel);
        body_to_world(
            state,
            [rot[0] as f32, (rot[1] + self.pivot_y) as f32, rot[2] as f32],
        )
    }
}

/// Adds a double-cone exhaust flame below the bell exit. Size ∝ throttle; nothing at 0.
fn append_exhaust_plume(
    verts: &mut Vec<Vertex>,
    idx: &mut Vec<u32>,
    state: &RocketState,
    nozzle_xf: &NozzleXform,
    bell_exit_y: f32,
    bell_exit_r: f32,
    segs: u32,
) {
    let thr = state.command.throttle.clamp(0.0, 1.0) as f32;
    if thr <= 1e-4 {
        return;
    }

    // Start slightly past the metal rim so flame does not z-fight the bell.
    let gap = 0.15;
    // Plume length scales with throttle (2× the original 1.5 + 12·thr curve).
    let length = 3.0 + 24.0 * thr;
    let base_r = bell_exit_r * (0.55 + 0.35 * thr);
    let mid_r = bell_exit_r * (0.75 + 0.55 * thr);
    let y0 = bell_exit_y - gap;
    let mid_y = y0 - length * 0.35;
    let tip_y = y0 - length;

    // Outer flame (orange → deep red at tip).
    let outer_hot = [1.0, 0.72, 0.18];
    let outer_cool = [0.95, 0.25, 0.05];
    let outer_tip = [0.55, 0.08, 0.02];
    append_flame_layer(
        verts,
        idx,
        state,
        nozzle_xf,
        segs,
        y0,
        mid_y,
        tip_y,
        base_r * 0.9,
        mid_r,
        outer_hot,
        outer_cool,
        outer_tip,
    );

    // Inner core (brighter, shorter, narrower).
    let core_len = length * 0.55;
    let core_mid_y = y0 - core_len * 0.4;
    let core_tip_y = y0 - core_len;
    let core_base_r = base_r * 0.45;
    let core_mid_r = mid_r * 0.4;
    let core_hot = [1.0, 0.97, 0.85];
    let core_mid = [1.0, 0.88, 0.35];
    let core_tip = [1.0, 0.55, 0.12];
    append_flame_layer(
        verts,
        idx,
        state,
        nozzle_xf,
        segs,
        y0,
        core_mid_y,
        core_tip_y,
        core_base_r,
        core_mid_r,
        core_hot,
        core_mid,
        core_tip,
    );
}

/// Two-stack cone (base ring → mid ring → tip) for a flame layer.
fn append_flame_layer(
    verts: &mut Vec<Vertex>,
    idx: &mut Vec<u32>,
    state: &RocketState,
    nozzle_xf: &NozzleXform,
    segs: u32,
    y0: f32,
    y1: f32,
    y2: f32,
    r0: f32,
    r1: f32,
    c0: [f32; 3],
    c1: [f32; 3],
    c2: [f32; 3],
) {
    let base0 = verts.len() as u32;
    for i in 0..segs {
        let a = (i as f32) * std::f32::consts::TAU / segs as f32;
        let (s, c) = a.sin_cos();
        verts.push(Vertex {
            pos: nozzle_xf.map(state, [c * r0, y0, s * r0]),
            color: c0,
        });
    }
    let base1 = verts.len() as u32;
    for i in 0..segs {
        let a = (i as f32) * std::f32::consts::TAU / segs as f32;
        let (s, c) = a.sin_cos();
        verts.push(Vertex {
            pos: nozzle_xf.map(state, [c * r1, y1, s * r1]),
            color: c1,
        });
    }
    let tip = verts.len() as u32;
    verts.push(Vertex {
        pos: nozzle_xf.map(state, [0.0, y2, 0.0]),
        color: c2,
    });

    for i in 0..segs {
        let i0 = base0 + i;
        let i1 = base0 + (i + 1) % segs;
        let j0 = base1 + i;
        let j1 = base1 + (i + 1) % segs;
        // Base → mid band
        idx.extend_from_slice(&[i0, j0, i1, i1, j0, j1]);
        // Mid → tip
        idx.extend_from_slice(&[j0, tip, j1]);
    }
}

/// Small orange jets at the four center roll thrusters when roll is commanded.
fn append_rcs_plumes(verts: &mut Vec<Vertex>, idx: &mut Vec<u32>, state: &RocketState) {
    let roll = state.command.roll.clamp(-1.0, 1.0);
    if roll.abs() <= 1e-4 {
        return;
    }
    let thrusters = state.roll_thrusters();
    let jet_len = 1.2 + 1.8 * roll.abs() as f32;
    // |force| is identical on all four when roll ≠ 0.
    let fmag = roll.abs() * state.params.rcs_thrust;
    if fmag < 1e-9 {
        return;
    }
    let inv_f = 1.0 / fmag;
    let color_hot = [1.0, 0.65, 0.2];
    let color_tip = [0.9, 0.25, 0.05];
    let offset = 0.12_f32;
    for t in &thrusters {
        let f = t.force_body;
        // Visual jet opposite the force (exhaust exits opposite reaction).
        let dir = [
            -(f[0] * inv_f) as f32,
            -(f[1] * inv_f) as f32,
            -(f[2] * inv_f) as f32,
        ];
        let base = [
            t.position_body[0] as f32,
            t.position_body[1] as f32,
            t.position_body[2] as f32,
        ];
        let tip = [
            base[0] + dir[0] * jet_len,
            base[1] + dir[1] * jet_len,
            base[2] + dir[2] * jet_len,
        ];
        let base_w = body_to_world(state, base);
        let tip_w = body_to_world(state, tip);
        // Tiny tetrahedron jet.
        let i0 = verts.len() as u32;
        verts.push(Vertex {
            pos: [base_w[0] + offset, base_w[1], base_w[2]],
            color: color_hot,
        });
        verts.push(Vertex {
            pos: [
                base_w[0] - offset * 0.5,
                base_w[1],
                base_w[2] + offset * 0.866,
            ],
            color: color_hot,
        });
        verts.push(Vertex {
            pos: [
                base_w[0] - offset * 0.5,
                base_w[1],
                base_w[2] - offset * 0.866,
            ],
            color: color_hot,
        });
        verts.push(Vertex {
            pos: tip_w,
            color: color_tip,
        });
        idx.extend_from_slice(&[
            i0,
            i0 + 1,
            i0 + 3,
            i0 + 1,
            i0 + 2,
            i0 + 3,
            i0 + 2,
            i0,
            i0 + 3,
        ]);
    }
}

fn append_leg_prism(
    verts: &mut Vec<Vertex>,
    idx: &mut Vec<u32>,
    a: [f32; 3],
    b: [f32; 3],
    radius: f32,
    color: [f32; 3],
) {
    // Build a simple 4-sided prism along a→b.
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let dz = b[2] - a[2];
    let len = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-4);
    let dir = [dx / len, dy / len, dz / len];
    // Pick a perpendicular.
    let up = if dir[1].abs() < 0.9 {
        [0.0, 1.0, 0.0]
    } else {
        [1.0, 0.0, 0.0]
    };
    let px = [
        dir[1] * up[2] - dir[2] * up[1],
        dir[2] * up[0] - dir[0] * up[2],
        dir[0] * up[1] - dir[1] * up[0],
    ];
    let pl = (px[0] * px[0] + px[1] * px[1] + px[2] * px[2])
        .sqrt()
        .max(1e-4);
    let px = [
        px[0] / pl * radius,
        px[1] / pl * radius,
        px[2] / pl * radius,
    ];
    let py = [
        dir[1] * px[2] - dir[2] * px[1],
        dir[2] * px[0] - dir[0] * px[2],
        dir[0] * px[1] - dir[1] * px[0],
    ];

    let base = verts.len() as u32;
    let corners = [
        [
            a[0] + px[0] + py[0],
            a[1] + px[1] + py[1],
            a[2] + px[2] + py[2],
        ],
        [
            a[0] - px[0] + py[0],
            a[1] - px[1] + py[1],
            a[2] - px[2] + py[2],
        ],
        [
            a[0] - px[0] - py[0],
            a[1] - px[1] - py[1],
            a[2] - px[2] - py[2],
        ],
        [
            a[0] + px[0] - py[0],
            a[1] + px[1] - py[1],
            a[2] + px[2] - py[2],
        ],
        [
            b[0] + px[0] + py[0],
            b[1] + px[1] + py[1],
            b[2] + px[2] + py[2],
        ],
        [
            b[0] - px[0] + py[0],
            b[1] - px[1] + py[1],
            b[2] - px[2] + py[2],
        ],
        [
            b[0] - px[0] - py[0],
            b[1] - px[1] - py[1],
            b[2] - px[2] - py[2],
        ],
        [
            b[0] + px[0] - py[0],
            b[1] + px[1] - py[1],
            b[2] + px[2] - py[2],
        ],
    ];
    for c in &corners {
        verts.push(Vertex { pos: *c, color });
    }
    let faces = [[0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]];
    for f in &faces {
        idx.extend_from_slice(&[
            base + f[0],
            base + f[1],
            base + f[2],
            base + f[0],
            base + f[2],
            base + f[3],
        ]);
    }
}

/// HUD text lines from simulation state (also used for window title / render-state checks).
pub fn hud_text(
    state: &RocketState,
    landing: &LandingAutopilot,
    target: &TargetLandingAutopilot,
    fps: f32,
) -> String {
    let p = state.position();
    let thr = state.command.throttle * 100.0;
    let contact = if state.contacting { "YES" } else { "no" };
    let status = if state.destroyed {
        format!("EXPLODED ({:.1} m/s impact)", state.last_impact_speed)
    } else if target.enabled {
        format!("T:{}", target.status_label())
    } else {
        format!("L:{}", landing.status_label())
    };
    format!(
        "PGA Rocket  |  alt={:.1} m  vel_y={:.1} m/s  thr={:.0}%  auto={}  contact={}  fps={:.0}\n\
         Space/Ctrl: hold thr  F: full  C: cut  W/S: pitch  Q/E: yaw  A/D: roll RCS  L: land  T: target-land  M: moon  R: reset\n\
         Drag LMB/RMB: orbit camera  Wheel: zoom  Arrows: orbit  Esc: quit",
        p[1], state.velocity[1], thr, status, contact, fps
    )
}
