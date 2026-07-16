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

    // Body = octagonal prism + nose cone + engine nozzle + 4 legs.
    let body_col = [0.75, 0.78, 0.82];
    let engine_col = [0.85, 0.35, 0.15];
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

    // Engine nozzle (inverted cone at bottom)
    let eng_r = r * 0.7;
    let eng_y0 = -hh;
    let eng_y1 = -hh - 1.2;
    let eng_base = verts.len() as u32;
    for i in 0..segs {
        let a = (i as f32) * std::f32::consts::TAU / segs as f32;
        let (s, c) = a.sin_cos();
        verts.push(Vertex {
            pos: body_to_world(state, [c * eng_r, eng_y0, s * eng_r]),
            color: engine_col,
        });
    }
    let eng_tip = verts.len() as u32;
    verts.push(Vertex {
        pos: body_to_world(state, [0.0, eng_y1, 0.0]),
        color: engine_col,
    });
    for i in 0..segs {
        let i0 = eng_base + i;
        let i1 = eng_base + (i + 1) % segs;
        idx.extend_from_slice(&[i0, eng_tip, i1]);
    }

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
         Space/Ctrl: throttle  W/S: pitch  A/D: yaw  Q/E: roll  R: reset  Esc: quit",
        p[1],
        state.velocity[1],
        thr,
        contact,
        fps
    )
}
