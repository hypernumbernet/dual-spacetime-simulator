//! 3D Graph mode: sample points from UI parameters for the particle buffer.

use crate::math::biquaternion::TetraQuaternion;
use crate::math::bivector::BivectorBoost;
use crate::ui_state::GraphType;
use std::hash::{Hash, Hasher};

const MAX_SAMPLES: u32 = 5000;
const GOLDEN: f64 = 1.618033988749895;

fn fibonacci_unit_direction(index: usize, n: usize) -> [f64; 3] {
    if n == 0 {
        return [0.0, 1.0, 0.0];
    }
    let t = (index as f64 + 0.5) / n as f64;
    let z = 1.0 - 2.0 * t;
    let r = (1.0 - z * z).max(0.0).sqrt();
    let theta = std::f64::consts::TAU * index as f64 * GOLDEN;
    let x = r * theta.cos();
    let y = r * theta.sin();
    [x, y, z]
}

fn clamp_samples(n: u32) -> usize {
    (n.clamp(1, MAX_SAMPLES)) as usize
}

fn graph_type_tag(gt: GraphType) -> u8 {
    match gt {
        GraphType::LightCone => 0,
        GraphType::RapidityField => 1,
        GraphType::BoostExponent => 2,
        GraphType::BivectorVisualization => 3,
        GraphType::QuaternionProjection => 4,
    }
}

/// パラメータが変わったときだけ GPU バッファを差し替えるためのフィンガープリント。
pub fn graph_params_fingerprint(
    graph_type: GraphType,
    graph_sample_count: u32,
    graph_t_slice: f64,
    graph_velocity_scale: f64,
    graph_phi: f64,
) -> u64 {
    let mut h = ahash::AHasher::default();
    graph_type_tag(graph_type).hash(&mut h);
    graph_sample_count.hash(&mut h);
    graph_t_slice.to_bits().hash(&mut h);
    graph_velocity_scale.to_bits().hash(&mut h);
    graph_phi.to_bits().hash(&mut h);
    h.finish()
}

pub fn build_points(
    graph_type: GraphType,
    graph_sample_count: u32,
    graph_t_slice: f64,
    graph_velocity_scale: f64,
    graph_phi: f64,
) -> (Vec<[f32; 3]>, Vec<[f32; 4]>) {
    let n = clamp_samples(graph_sample_count);
    let mut positions = Vec::with_capacity(n);
    let mut colors = Vec::with_capacity(n);

    match graph_type {
        GraphType::LightCone => {
            let t = graph_t_slice;
            let r = t.abs();
            for i in 0..n {
                let d = fibonacci_unit_direction(i, n);
                let px = (d[0] * r) as f32;
                let py = (d[1] * r) as f32;
                let pz = (d[2] * r) as f32;
                positions.push([px, py, pz]);
                let cr = (0.5 + 0.5 * d[0]) as f32;
                let cg = (0.5 + 0.5 * d[1]) as f32;
                let cb = (0.5 + 0.5 * d[2]) as f32;
                colors.push([cr, cg, cb, 1.0]);
            }
        }
        GraphType::RapidityField => {
            let vs = graph_velocity_scale;
            for i in 0..n {
                let d = fibonacci_unit_direction(i, n);
                let speed = (vs.abs()).min(0.999);
                let vx = d[0] * speed;
                let vy = d[1] * speed;
                let vz = d[2] * speed;
                let bv = if vx * vx + vy * vy + vz * vz < 1e-20 {
                    BivectorBoost::new(0.0, 0.0, 0.0)
                } else {
                    BivectorBoost::from_velocity(vx, vy, vz)
                };
                let s = vs as f32 * 0.25 + 0.25;
                positions.push([
                    (bv.i * vs) as f32,
                    (bv.j * vs) as f32,
                    (bv.k * vs) as f32,
                ]);
                colors.push([s, 0.4, 1.0 - s * 0.5, 1.0]);
            }
        }
        GraphType::BoostExponent => {
            let phi = graph_phi;
            let scale = graph_velocity_scale;
            for i in 0..n {
                let d = fibonacci_unit_direction(i, n);
                let b = BivectorBoost::new(d[0] * phi, d[1] * phi, d[2] * phi);
                let e = b.exp();
                positions.push([
                    (e.i * scale) as f32,
                    (e.j * scale) as f32,
                    (e.k * scale) as f32,
                ]);
                let cr = (e.scalar.tanh() * 0.5 + 0.5) as f32;
                colors.push([cr, 0.55, 0.9, 1.0]);
            }
        }
        GraphType::BivectorVisualization => {
            let phi = graph_phi;
            let scale = graph_velocity_scale;
            for i in 0..n {
                let d = fibonacci_unit_direction(i, n);
                positions.push([
                    (d[0] * phi * scale) as f32,
                    (d[1] * phi * scale) as f32,
                    (d[2] * phi * scale) as f32,
                ]);
                colors.push([0.3, 0.85, 0.45, 1.0]);
            }
        }
        GraphType::QuaternionProjection => {
            let mag = graph_phi;
            for i in 0..n {
                let a = i % 15;
                let b = ((i / 15) + (i % 7)) % 15;
                let q = TetraQuaternion::basis(a) * TetraQuaternion::basis(b);
                let ijk = q.ijk_coeffs();
                let px = ijk[0] * mag;
                let py = ijk[1] * mag;
                let pz = ijk[2] * mag;
                positions.push([px as f32, py as f32, pz as f32]);
                let ca = (a as f32) / 14.0;
                let cb = (b as f32) / 14.0;
                colors.push([ca, 0.35, cb, 1.0]);
            }
        }
    }

    (positions, colors)
}

/// `LineList` 用の頂点列（連続する 2 頂点が 1 線分）。`GraphType::LightCone` のみ非空。
pub fn build_graph_line_vertices(
    graph_type: GraphType,
    graph_sample_count: u32,
    graph_t_slice: f64,
    _graph_velocity_scale: f64,
    _graph_phi: f64,
) -> Vec<([f32; 3], [f32; 4])> {
    match graph_type {
        GraphType::LightCone => build_light_cone_line_vertices(graph_sample_count, graph_t_slice),
        _ => Vec::new(),
    }
}

fn build_light_cone_line_vertices(graph_sample_count: u32, graph_t_slice: f64) -> Vec<([f32; 3], [f32; 4])> {
    let n = clamp_samples(graph_sample_count);
    let r = graph_t_slice.abs();
    let origin = [0.0_f32, 0.0, 0.0];
    let mut out = Vec::with_capacity(n * 2);
    for i in 0..n {
        let d = fibonacci_unit_direction(i, n);
        let end = [(d[0] * r) as f32, (d[1] * r) as f32, (d[2] * r) as f32];
        let cr = (0.5 + 0.5 * d[0]) as f32;
        let cg = (0.5 + 0.5 * d[1]) as f32;
        let cb = (0.5 + 0.5 * d[2]) as f32;
        let c = [cr, cg, cb, 1.0];
        out.push((origin, c));
        out.push((end, c));
    }
    out
}
