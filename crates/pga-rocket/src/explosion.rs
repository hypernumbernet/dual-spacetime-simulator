//! Crash explosion: opaque ballistic debris plus translucent fire/smoke/dust FX.
//!
//! Everything is a deterministic function of `explosion_age` and per-element
//! integer hashes, so both meshes are rebuilt each frame with no extra sim state.
//! The FX mesh uses premultiplied-alpha soft discs (center vertex at peak color,
//! rim vertices fully transparent) sorted back-to-front from the camera eye.

use crate::mesh::{FxVertex, Vertex, append_oriented_box};
use crate::sim::RocketState;

const G: f32 = 9.81;

// Flash
const FLASH_END: f32 = 0.15;
// Fireball
const FIREBALL_PUFFS: u32 = 7;
const FIREBALL_END: f32 = 2.2;
const FIREBALL_TAU: f32 = 0.35;
/// Overall fire size multiplier (flash + fireball) relative to body radius.
const FIREBALL_SCALE: f32 = 5.0;
// Smoke column
const SMOKE_PUFFS: u32 = 12;
const SMOKE_SPAWN_BASE: f32 = 0.20;
const SMOKE_SPAWN_STEP: f32 = 0.25;
const SMOKE_LIFE: f32 = 5.0;
const SMOKE_RISE: f32 = 3.5;
// Ground dust ring
const DUST_PUFFS: u32 = 10;
const DUST_END: f32 = 1.5;
const DUST_TAU: f32 = 0.5;
// Debris
pub const DEBRIS_COUNT: u32 = 240;
const DEBRIS_RESTITUTION: f32 = 0.40;
const DEBRIS_H_DAMP: f32 = 0.55;
const DEBRIS_MAX_BOUNCES: u32 = 3;
const DEBRIS_MIN_BOUNCE_SPEED: f32 = 2.0;
// Scorch decal
const SCORCH_Y: f32 = 0.04;
const FX_DISC_SEGS: u32 = 10;

/// Stable per-element pseudo-random in [0, 1).
fn hash01(seed: u32) -> f32 {
    let mut x = seed.wrapping_mul(0x9E37_79B9) ^ 0x85EB_CA6B;
    x ^= x >> 16;
    x = x.wrapping_mul(0x7FEB_352D);
    x ^= x >> 15;
    (x & 0x00FF_FFFF) as f32 / 16_777_216.0
}

/// Hash of (element index, channel).
fn h(i: u32, ch: u32) -> f32 {
    hash01(i.wrapping_mul(31).wrapping_add(ch).wrapping_add(1))
}

/// Exponential ease-out: 0 at t=0, asymptotically 1.
fn ease_out(t: f32, tau: f32) -> f32 {
    1.0 - (-t / tau).exp()
}

/// Fire color ramp; t in [0,1], 0 = hottest (white) to 1 = sooty dark red.
fn fire_ramp(t: f32) -> [f32; 3] {
    const STOPS: [[f32; 3]; 4] = [
        [1.00, 0.97, 0.85],
        [1.00, 0.75, 0.25],
        [0.95, 0.35, 0.06],
        [0.25, 0.06, 0.03],
    ];
    let s = t.clamp(0.0, 1.0) * 3.0;
    let i = (s as usize).min(2);
    let f = s - i as f32;
    let a = STOPS[i];
    let b = STOPS[i + 1];
    [
        a[0] + (b[0] - a[0]) * f,
        a[1] + (b[1] - a[1]) * f,
        a[2] + (b[2] - a[2]) * f,
    ]
}

fn normalize(v: [f32; 3]) -> [f32; 3] {
    let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt().max(1e-6);
    [v[0] / l, v[1] / l, v[2] / l]
}

fn cross3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Rodrigues rotation of `v` about unit axis `k` by `ang` radians.
fn rotate_about(v: [f32; 3], k: [f32; 3], ang: f32) -> [f32; 3] {
    let (s, c) = ang.sin_cos();
    let kxv = cross3(k, v);
    let kdv = k[0] * v[0] + k[1] * v[1] + k[2] * v[2];
    [
        v[0] * c + kxv[0] * s + k[0] * kdv * (1.0 - c),
        v[1] * c + kxv[1] * s + k[1] * kdv * (1.0 - c),
        v[2] * c + kxv[2] * s + k[2] * kdv * (1.0 - c),
    ]
}

/// State of one debris shard at `age` seconds after the explosion.
pub struct DebrisSample {
    pub center: [f32; 3],
    /// Orientation axis for the oriented box (tumbles until rest).
    pub axis: [f32; 3],
    /// Box half-size; also the rest height above the ground plane.
    pub half: f32,
    pub at_rest: bool,
    /// 0 = fresh/glowing ember, 1 = fully sooted.
    pub burn: f32,
}

/// Piecewise-analytic ballistic arc with up to `DEBRIS_MAX_BOUNCES` ground
/// bounces, then rest. Deterministic in (i, origin, body_r, age); never
/// returns a center below the shard's rest height.
pub fn debris_sample(i: u32, origin: [f32; 3], body_r: f32, age: f32) -> DebrisSample {
    let half = body_r * (0.20 + 0.25 * h(i, 3));
    let rest_h = half;

    let az = h(i, 0) * std::f32::consts::TAU;
    let el = 0.2618 + h(i, 1) * 1.0472; // 15°..75°
    let speed = 8.0 + h(i, 2) * 20.0;
    let dir0 = [az.cos() * el.cos(), el.sin(), az.sin() * el.cos()];
    let mut v = [dir0[0] * speed, dir0[1] * speed, dir0[2] * speed];

    let mut p = [
        origin[0] + (h(i, 5) * 2.0 - 1.0) * body_r * 0.5,
        (origin[1] + (h(i, 6) * 2.0 - 1.0) * body_r * 0.5).max(rest_h),
        origin[2] + (h(i, 7) * 2.0 - 1.0) * body_r * 0.5,
    ];

    let mut remaining = age.max(0.0);
    let mut elapsed = 0.0f32;
    let mut bounces = 0u32;
    let (center, at_rest, spin_time);
    loop {
        // Time until the shard falls back to rest height (positive root).
        let disc = v[1] * v[1] + 2.0 * G * (p[1] - rest_h);
        let t_hit = (v[1] + disc.max(0.0).sqrt()) / G;
        if remaining < t_hit {
            let t = remaining;
            center = [
                p[0] + v[0] * t,
                p[1] + v[1] * t - 0.5 * G * t * t,
                p[2] + v[2] * t,
            ];
            at_rest = false;
            spin_time = elapsed + t;
            break;
        }
        p[0] += v[0] * t_hit;
        p[2] += v[2] * t_hit;
        p[1] = rest_h;
        let impact_speed = G * t_hit - v[1]; // downward speed at impact (>= 0)
        elapsed += t_hit;
        remaining -= t_hit;
        bounces += 1;
        if impact_speed < DEBRIS_MIN_BOUNCE_SPEED || bounces > DEBRIS_MAX_BOUNCES {
            center = p;
            at_rest = true;
            spin_time = elapsed;
            break;
        }
        v[1] = impact_speed * DEBRIS_RESTITUTION;
        v[0] *= DEBRIS_H_DAMP;
        v[2] *= DEBRIS_H_DAMP;
    }

    let spin_axis = normalize([
        h(i, 10) * 2.0 - 1.0,
        h(i, 11) * 2.0 - 1.0,
        h(i, 12) * 2.0 - 1.0,
    ]);
    let spin_rate = 2.0 + 6.0 * h(i, 4);
    let axis = rotate_about(dir0, spin_axis, spin_rate * spin_time);

    DebrisSample {
        center,
        axis,
        half,
        at_rest,
        burn: (age / 1.5).clamp(0.0, 1.0),
    }
}

/// Opaque explosion geometry: tumbling ballistic debris shards.
pub fn explosion_opaque_mesh(state: &RocketState) -> (Vec<Vertex>, Vec<u32>) {
    let mut verts = Vec::new();
    let mut idx = Vec::new();
    if !state.destroyed {
        return (verts, idx);
    }
    let age = state.explosion_age as f32;
    let origin = [
        state.explosion_origin[0] as f32,
        state.explosion_origin[1] as f32,
        state.explosion_origin[2] as f32,
    ];
    let body_r = state.params.body_radius as f32;

    const EMBER: [f32; 3] = [1.0, 0.45, 0.10];
    const CHARRED: [f32; 3] = [0.16, 0.15, 0.14];
    for i in 0..DEBRIS_COUNT {
        let s = debris_sample(i, origin, body_r, age);
        let col = [
            EMBER[0] + (CHARRED[0] - EMBER[0]) * s.burn,
            EMBER[1] + (CHARRED[1] - EMBER[1]) * s.burn,
            EMBER[2] + (CHARRED[2] - EMBER[2]) * s.burn,
        ];
        append_oriented_box(&mut verts, &mut idx, s.center, s.axis, s.half, col);
    }
    (verts, idx)
}

/// One translucent soft-disc primitive collected before sorting.
struct Puff {
    center: [f32; 3],
    radius: f32,
    /// Premultiplied center color: rgb already multiplied by effective alpha
    /// (glow elements keep bright rgb with tiny alpha → additive look).
    center_rgba: [f32; 4],
    /// Ground-plane disc (+Y) instead of camera-facing billboard.
    ground: bool,
}

/// Translucent FX mesh (flash, fireball, smoke, dust ring, scorch decal),
/// sorted back-to-front from `cam_eye`. Empty when the rocket is intact.
pub fn explosion_fx_mesh(state: &RocketState, cam_eye: [f32; 3]) -> (Vec<FxVertex>, Vec<u32>) {
    let mut verts = Vec::new();
    let mut idx = Vec::new();
    if !state.destroyed {
        return (verts, idx);
    }
    let age = state.explosion_age as f32;
    let o = [
        state.explosion_origin[0] as f32,
        state.explosion_origin[1] as f32,
        state.explosion_origin[2] as f32,
    ];
    let br = state.params.body_radius as f32;

    let mut puffs: Vec<Puff> = Vec::with_capacity(32);

    // Persistent scorch decal on the ground (quick fade-in under the fireball).
    {
        let a = 0.85 * (age / 0.25).clamp(0.0, 1.0);
        let tint = [0.05, 0.04, 0.03];
        puffs.push(Puff {
            center: [o[0], SCORCH_Y, o[2]],
            radius: br * 4.5,
            center_rgba: [tint[0] * a, tint[1] * a, tint[2] * a, a],
            ground: true,
        });
    }

    // Initial white-hot flash (mostly additive).
    if age < FLASH_END {
        let k = 1.0 - age / FLASH_END;
        let i = 3.0 * k;
        puffs.push(Puff {
            center: [o[0], o[1] + br, o[2]],
            radius: br * (2.0 + 45.0 * age) * FIREBALL_SCALE,
            center_rgba: [1.0 * i, 0.98 * i, 0.9 * i, 0.10 * k],
            ground: false,
        });
    }

    // Fireball: glowing puffs that expand, rise, cool, and soot over.
    if age < FIREBALL_END {
        let end_fade = (1.0 - (age - 1.6) / 0.6).clamp(0.0, 1.0);
        for i in 0..FIREBALL_PUFFS {
            let heat = (age / FIREBALL_END + 0.25 * h(i, 9)).clamp(0.0, 1.0);
            let tint = fire_ramp(heat);
            let intensity = 1.8 * (1.0 - 0.7 * heat) * end_fade;
            let a = (0.08 + 0.47 * heat) * end_fade;
            let grow = ease_out(age, FIREBALL_TAU);
            puffs.push(Puff {
                center: [
                    o[0] + (h(i, 5) * 2.0 - 1.0) * br * 1.2 * FIREBALL_SCALE,
                    o[1] + h(i, 6) * br * 1.5 * FIREBALL_SCALE + 4.0 * age,
                    o[2] + (h(i, 7) * 2.0 - 1.0) * br * 1.2 * FIREBALL_SCALE,
                ],
                radius: br * (2.0 + 3.0 * h(i, 8)) * FIREBALL_SCALE * grow,
                center_rgba: [
                    tint[0] * intensity,
                    tint[1] * intensity,
                    tint[2] * intensity,
                    a,
                ],
                ground: false,
            });
        }
    }

    // Smoke column: staggered gray puffs with decelerating buoyant rise.
    for i in 0..SMOKE_PUFFS {
        let spawn = SMOKE_SPAWN_BASE + i as f32 * SMOKE_SPAWN_STEP;
        let life = age - spawn;
        if life <= 0.0 || life >= SMOKE_LIFE {
            continue;
        }
        let u = life / SMOKE_LIFE;
        let g = 0.22 + 0.18 * u;
        let a = 0.5 * (1.0 - u) * (life / 0.4).clamp(0.0, 1.0);
        puffs.push(Puff {
            center: [
                o[0] + (h(i, 20) * 2.0 - 1.0) * br + (h(i, 22) * 2.0 - 1.0) * life,
                o[1] + br * 0.5 + SMOKE_RISE * life * (1.0 - 0.5 * u),
                o[2] + (h(i, 21) * 2.0 - 1.0) * br + (h(i, 23) * 2.0 - 1.0) * life,
            ],
            radius: br * (1.2 + 4.5 * u),
            center_rgba: [g * a, g * a, g * 0.98 * a, a],
            ground: false,
        });
    }

    // Expanding ground dust ring.
    if age < DUST_END {
        let ring_r = br * 3.0 + br * 10.0 * ease_out(age, DUST_TAU);
        let a_base = 0.35 * (1.0 - age / DUST_END);
        let tint = [0.45, 0.38, 0.28];
        for i in 0..DUST_PUFFS {
            let ang = i as f32 / DUST_PUFFS as f32 * std::f32::consts::TAU + h(i, 12) * 0.6;
            puffs.push(Puff {
                center: [
                    o[0] + ang.cos() * ring_r,
                    SCORCH_Y + 0.02,
                    o[2] + ang.sin() * ring_r,
                ],
                radius: br * (1.0 + 1.5 * age),
                center_rgba: [tint[0] * a_base, tint[1] * a_base, tint[2] * a_base, a_base],
                ground: true,
            });
        }
    }

    // Back-to-front so premultiplied blending composites correctly.
    let d2 = |c: [f32; 3]| {
        let dx = c[0] - cam_eye[0];
        let dy = c[1] - cam_eye[1];
        let dz = c[2] - cam_eye[2];
        dx * dx + dy * dy + dz * dz
    };
    puffs.sort_by(|a, b| d2(b.center).total_cmp(&d2(a.center)));

    for p in &puffs {
        emit_soft_disc(&mut verts, &mut idx, p, cam_eye);
    }
    (verts, idx)
}

/// Triangle fan whose center vertex carries the peak color and whose rim is
/// fully transparent — vertex interpolation yields a soft round sprite.
fn emit_soft_disc(verts: &mut Vec<FxVertex>, idx: &mut Vec<u32>, p: &Puff, cam_eye: [f32; 3]) {
    let (right, up) = if p.ground {
        ([1.0, 0.0, 0.0], [0.0, 0.0, 1.0])
    } else {
        let n = normalize([
            cam_eye[0] - p.center[0],
            cam_eye[1] - p.center[1],
            cam_eye[2] - p.center[2],
        ]);
        let r = cross3([0.0, 1.0, 0.0], n);
        let rl = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();
        let right = if rl < 1e-4 {
            [1.0, 0.0, 0.0]
        } else {
            [r[0] / rl, r[1] / rl, r[2] / rl]
        };
        (right, cross3(n, right))
    };

    let base = verts.len() as u32;
    verts.push(FxVertex {
        pos: p.center,
        color: p.center_rgba,
    });
    for i in 0..FX_DISC_SEGS {
        let a = i as f32 / FX_DISC_SEGS as f32 * std::f32::consts::TAU;
        let (s, c) = a.sin_cos();
        verts.push(FxVertex {
            pos: [
                p.center[0] + (right[0] * c + up[0] * s) * p.radius,
                p.center[1] + (right[1] * c + up[1] * s) * p.radius,
                p.center[2] + (right[2] * c + up[2] * s) * p.radius,
            ],
            color: [0.0, 0.0, 0.0, 0.0],
        });
    }
    for i in 0..FX_DISC_SEGS {
        let r0 = base + 1 + i;
        let r1 = base + 1 + (i + 1) % FX_DISC_SEGS;
        idx.extend_from_slice(&[base, r0, r1]);
    }
}

/// Vertices emitted per soft disc (center + rim) — used by tests.
pub const FX_DISC_VERTS: u32 = FX_DISC_SEGS + 1;
