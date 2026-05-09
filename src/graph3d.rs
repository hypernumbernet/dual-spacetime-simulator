//! 3D Graph mode: sample points from UI parameters for the particle buffer.

use crate::math::biquaternion::TetraQuaternion;
use crate::math::spacetime::{Spacetime, lorentz_transformation_matrix};
use crate::ui_state::GraphType;
use glam::{DVec3, DVec4};
use std::hash::{Hash, Hasher};

const MAX_SAMPLES: u32 = 5000;
const GOLDEN: f64 = 1.618033988749895;

/// Generates an approximately uniform unit direction on a sphere using Fibonacci sampling.
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

/// Clamps the requested sample count to a valid GPU-friendly range.
fn clamp_samples(n: u32) -> usize {
    (n.clamp(1, MAX_SAMPLES)) as usize
}

/// Converts each graph type into a stable numeric tag for hashing.
fn graph_type_tag(gt: GraphType) -> u8 {
    match gt {
        GraphType::SphericalFibonacciLattice => 0,
        GraphType::RapidityFieldMatrix => 1,
        GraphType::RapidityFieldBiquaternion => 2,
        GraphType::BivectorVisualization => 3,
        GraphType::QuaternionProjection => 4,
    }
}

/// Computes a stable fingerprint for graph parameters to detect buffer update needs.
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

/// Builds point positions and colors for the selected 3D graph visualization mode.
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
        GraphType::SphericalFibonacciLattice => {
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
        GraphType::RapidityFieldMatrix => {}
        GraphType::RapidityFieldBiquaternion => {}
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

/// Builds line-list vertices and colors for graph types that render line segments.
pub fn build_graph_line_vertices(
    graph_type: GraphType,
    graph_sample_count: u32,
    graph_t_slice: f64,
    graph_velocity_scale: f64,
    _graph_phi: f64,
) -> Vec<([f32; 3], [f32; 4])> {
    match graph_type {
        GraphType::SphericalFibonacciLattice => {
            build_light_cone_line_vertices(graph_sample_count, graph_t_slice)
        }
        GraphType::RapidityFieldMatrix => build_rapidity_field_line_vertices_with(
            graph_sample_count,
            graph_velocity_scale,
            rapidity_point_matrix,
        ),
        GraphType::RapidityFieldBiquaternion => build_rapidity_field_line_vertices_with(
            graph_sample_count,
            graph_velocity_scale,
            rapidity_point_biquaternion,
        ),
        _ => Vec::new(),
    }
}

/// Creates radial light-cone line segments from the origin to sampled sphere directions.
fn build_light_cone_line_vertices(
    graph_sample_count: u32,
    graph_t_slice: f64,
) -> Vec<([f32; 3], [f32; 4])> {
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

/// Builds Lorentz-transformed line mesh on xz grid and y-direction pillars.
fn build_rapidity_field_line_vertices_with(
    graph_sample_count: u32,
    graph_velocity_scale: f64,
    compute_point: fn(DVec3, f64, DVec4) -> Option<DVec4>,
) -> Vec<([f32; 3], [f32; 4])> {
    const RATE_XZ: f64 = 0.07;
    const RATE_Y: f64 = 0.05;
    const SPEED_OF_LIGHT_INV: f64 = 1.0;
    const MIN_GRID_SIZE: i32 = 6;
    const MAX_GRID_SIZE: i32 = 20;
    const GRID_BUCKET_COUNT: i32 = ((MAX_GRID_SIZE - MIN_GRID_SIZE) / 2) + 1; // 6,8,10,...,20
    const RAPIDITY_PILLAR_COLOR: [f32; 4] = [0.05, 0.95, 0.9, 1.0];
    const RAPIDITY_GRID_U_COLOR: [f32; 4] = [0.95, 0.2, 0.8, 1.0];
    const RAPIDITY_GRID_V_COLOR: [f32; 4] = [1.0, 0.7, 0.15, 1.0];
    let scale = graph_velocity_scale.abs().max(1e-9);
    let clamped = clamp_samples(graph_sample_count) as i32;
    let bucket = ((clamped - 1) * (GRID_BUCKET_COUNT - 1)) / (MAX_SAMPLES as i32 - 1);
    let grid_size = MIN_GRID_SIZE + bucket * 2;
    let spacetime_org = DVec4::new(1.0, 0.0, 0.0, 0.0);
    let mut out = Vec::with_capacity(4_500);

    for k in (-grid_size..=grid_size).step_by(2) {
        for j in (-grid_size..=grid_size).step_by(2) {
            let mut prev: Option<DVec4> = None;
            for i in (-grid_size..=grid_size).step_by(2) {
                let speed = DVec3::new(
                    scale * RATE_XZ * k as f64,
                    scale * RATE_Y * i as f64,
                    scale * RATE_XZ * j as f64,
                );
                if let Some(current) = compute_point(speed, SPEED_OF_LIGHT_INV, spacetime_org) {
                    if let Some(previous) = prev {
                        push_line(&mut out, previous, current, RAPIDITY_PILLAR_COLOR);
                    }
                    prev = Some(current);
                } else {
                    prev = None;
                }
            }
        }
    }

    for k in (-grid_size..=grid_size).step_by(2) {
        let mut prev: Option<DVec4> = None;
        for j in (-grid_size..=grid_size).step_by(2) {
            let speed = DVec3::new(scale * RATE_XZ * j as f64, 0.0, scale * RATE_XZ * k as f64);
            if let Some(current) = compute_point(speed, SPEED_OF_LIGHT_INV, spacetime_org) {
                if let Some(previous) = prev {
                    push_line(&mut out, previous, current, RAPIDITY_GRID_U_COLOR);
                }
                prev = Some(current);
            } else {
                prev = None;
            }
        }
    }

    for k in (-grid_size..=grid_size).step_by(2) {
        let mut prev: Option<DVec4> = None;
        for j in (-grid_size..=grid_size).step_by(2) {
            let speed = DVec3::new(scale * RATE_XZ * k as f64, 0.0, scale * RATE_XZ * j as f64);
            if let Some(current) = compute_point(speed, SPEED_OF_LIGHT_INV, spacetime_org) {
                if let Some(previous) = prev {
                    push_line(&mut out, previous, current, RAPIDITY_GRID_V_COLOR);
                }
                prev = Some(current);
            } else {
                prev = None;
            }
        }
    }

    out
}

/// Computes Lorentz boost using matrix multiplication.
fn rapidity_point_matrix(v: DVec3, speed_of_light_inv: f64, base: DVec4) -> Option<DVec4> {
    lorentz_transformation_matrix(v, speed_of_light_inv)
        .ok()
        .map(|matrix| matrix * base)
}

/// Computes Lorentz boost using biquaternion multiplication.
fn rapidity_point_biquaternion(v: DVec3, speed_of_light_inv: f64, base: DVec4) -> Option<DVec4> {
    let mut st = Spacetime::new(base.x, base.y, base.z, base.w);
    st.lorentz_transformation_v(-v, speed_of_light_inv);
    Some(DVec4::new(st.t, st.x, st.y, st.z))
}

/// Pushes a line segment between two spacetime points to the output vector.
fn push_line(out: &mut Vec<([f32; 3], [f32; 4])>, a: DVec4, b: DVec4, color: [f32; 4]) {
    out.push(([a.y as f32, a.z as f32, a.w as f32], color));
    out.push(([b.y as f32, b.z as f32, b.w as f32], color));
}
