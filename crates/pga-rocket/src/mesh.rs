//! CPU mesh builders for the grass ground plane and legged rocket body.

use crate::sim::RocketState;
use bytemuck::{Pod, Zeroable};

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct Vertex {
    pub pos: [f32; 3],
    pub color: [f32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct GroundVertex {
    pub pos: [f32; 3],
    pub uv: [f32; 2],
}

/// Half-extent of the local grass plane (meters). Re-centered under the rocket each frame.
pub const GROUND_HALF_EXTENT: f32 = 1800.0;
/// World meters covered by one grass texture tile (minecraft-like 1 m block).
pub const GRASS_METERS_PER_TILE: f32 = 1.0;
/// Fog distances (meters) — edge of the plane is fully fogged into the sky.
pub const GROUND_FOG_START: f32 = 350.0;
pub const GROUND_FOG_END: f32 = 1400.0;

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

/// Build a legged rocket mesh in world space from PGA-transformed body points.
pub fn rocket_mesh(state: &RocketState) -> (Vec<Vertex>, Vec<u32>) {
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

    // Engine bell: open metal frustum (throat → exit), not a solid cone tip.
    // Length comes from sim params so it stays consistent with leg clearance.
    let throat_r = r * 0.45;
    let exit_r = r * 0.95;
    let throat_y = -hh;
    let exit_y = state.params.nozzle_exit_y() as f32;
    let throat_base = verts.len() as u32;
    for i in 0..segs {
        let a = (i as f32) * std::f32::consts::TAU / segs as f32;
        let (s, c) = a.sin_cos();
        verts.push(Vertex {
            pos: body_to_world(state, [c * throat_r, throat_y, s * throat_r]),
            color: nozzle_col,
        });
    }
    let exit_base = verts.len() as u32;
    for i in 0..segs {
        let a = (i as f32) * std::f32::consts::TAU / segs as f32;
        let (s, c) = a.sin_cos();
        verts.push(Vertex {
            pos: body_to_world(state, [c * exit_r, exit_y, s * exit_r]),
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
    append_exhaust_plume(&mut verts, &mut idx, state, exit_y, exit_r, segs);

    // Landing legs: line from body attach to foot, thickened as thin boxes.
    for foot in &state.params.leg_feet {
        let foot_w = body_to_world(
            state,
            [foot[0] as f32, foot[1] as f32, foot[2] as f32],
        );
        let attach = body_to_world(
            state,
            [
                foot[0] as f32 * 0.35,
                -hh * 0.6,
                foot[2] as f32 * 0.35,
            ],
        );
        append_leg_prism(&mut verts, &mut idx, attach, foot_w, 0.18, leg_col);
    }

    (verts, idx)
}

fn body_to_world(state: &RocketState, body: [f32; 3]) -> [f32; 3] {
    use crate::euclidean_pga::{extract_point, point};
    let p = point(body[0] as f64, body[1] as f64, body[2] as f64);
    let w = extract_point(&state.motor.sandwich(&p));
    [w[0] as f32, w[1] as f32, w[2] as f32]
}

/// Adds a double-cone exhaust flame below the bell exit. Size ∝ throttle; nothing at 0.
fn append_exhaust_plume(
    verts: &mut Vec<Vertex>,
    idx: &mut Vec<u32>,
    state: &RocketState,
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
            pos: body_to_world(state, [c * r0, y0, s * r0]),
            color: c0,
        });
    }
    let base1 = verts.len() as u32;
    for i in 0..segs {
        let a = (i as f32) * std::f32::consts::TAU / segs as f32;
        let (s, c) = a.sin_cos();
        verts.push(Vertex {
            pos: body_to_world(state, [c * r1, y1, s * r1]),
            color: c1,
        });
    }
    let tip = verts.len() as u32;
    verts.push(Vertex {
        pos: body_to_world(state, [0.0, y2, 0.0]),
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
    let pl = (px[0] * px[0] + px[1] * px[1] + px[2] * px[2]).sqrt().max(1e-4);
    let px = [px[0] / pl * radius, px[1] / pl * radius, px[2] / pl * radius];
    let py = [
        dir[1] * px[2] - dir[2] * px[1],
        dir[2] * px[0] - dir[0] * px[2],
        dir[0] * px[1] - dir[1] * px[0],
    ];

    let base = verts.len() as u32;
    let corners = [
        [a[0] + px[0] + py[0], a[1] + px[1] + py[1], a[2] + px[2] + py[2]],
        [a[0] - px[0] + py[0], a[1] - px[1] + py[1], a[2] - px[2] + py[2]],
        [a[0] - px[0] - py[0], a[1] - px[1] - py[1], a[2] - px[2] - py[2]],
        [a[0] + px[0] - py[0], a[1] + px[1] - py[1], a[2] + px[2] - py[2]],
        [b[0] + px[0] + py[0], b[1] + px[1] + py[1], b[2] + px[2] + py[2]],
        [b[0] - px[0] + py[0], b[1] - px[1] + py[1], b[2] - px[2] + py[2]],
        [b[0] - px[0] - py[0], b[1] - px[1] - py[1], b[2] - px[2] - py[2]],
        [b[0] + px[0] - py[0], b[1] + px[1] - py[1], b[2] + px[2] - py[2]],
    ];
    for c in &corners {
        verts.push(Vertex {
            pos: *c,
            color,
        });
    }
    let faces = [
        [0, 1, 5, 4],
        [1, 2, 6, 5],
        [2, 3, 7, 6],
        [3, 0, 4, 7],
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

/// HUD text lines from simulation state (also used for window title / render-state checks).
pub fn hud_text(state: &RocketState, fps: f32) -> String {
    let p = state.position();
    let thr = state.command.throttle * 100.0;
    let contact = if state.contacting { "YES" } else { "no" };
    format!(
        "PGA Rocket  |  alt={:.1} m  vel_y={:.1} m/s  thr={:.0}%  contact={}  fps={:.0}\n\
         Space/Ctrl: throttle  W/S: pitch  A/D: roll  Q/E: yaw  R: reset\n\
         Drag LMB/RMB: orbit camera  Wheel: zoom  Arrows: orbit  Esc: quit",
        p[1],
        state.velocity[1],
        thr,
        contact,
        fps
    )
}
