//! Euclidean 3D Projective Geometric Algebra G(3,0,1).
//!
//! Generators: e0 (null, e0²=0), e1, e2, e3 (e_i²=+1).
//! Uses `dst_math::pga::basis_mul_with_metric` for the geometric product so
//! pose and geometry are true multivector operations, not renamed vectors.

use dst_math::pga::basis_mul_with_metric;
use std::ops::{Add, Mul, Neg, Sub};

/// Number of basis blades in G(3,0,1) (2^4).
pub const EPGA_DIM: usize = 16;

/// Metric: e0²=0, e1²=e2²=e3²=+1.
pub const EPGA_METRIC: [i8; 4] = [0, 1, 1, 1];

/// Basis indices (bitmask layout matching generator bits).
pub mod basis {
    pub const SCALAR: usize = 0;
    pub const E0: usize = 1; // 0b0001
    pub const E1: usize = 2; // 0b0010
    pub const E2: usize = 4; // 0b0100
    pub const E3: usize = 8; // 0b1000
    pub const E01: usize = 3;
    pub const E02: usize = 5;
    pub const E12: usize = 6;
    pub const E03: usize = 9;
    pub const E13: usize = 10;
    pub const E23: usize = 12;
    pub const E012: usize = 7;
    pub const E013: usize = 11;
    pub const E023: usize = 13;
    pub const E123: usize = 14;
    pub const E0123: usize = 15;
}

/// 16-component multivector for Euclidean PGA.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Multivector {
    coeffs: [f64; EPGA_DIM],
}

impl Multivector {
    pub fn new(coeffs: [f64; EPGA_DIM]) -> Self {
        Self { coeffs }
    }

    pub fn zero() -> Self {
        Self {
            coeffs: [0.0; EPGA_DIM],
        }
    }

    pub fn one() -> Self {
        let mut coeffs = [0.0; EPGA_DIM];
        coeffs[0] = 1.0;
        Self { coeffs }
    }

    pub fn basis(index: usize) -> Self {
        assert!(index < EPGA_DIM);
        let mut coeffs = [0.0; EPGA_DIM];
        coeffs[index] = 1.0;
        Self { coeffs }
    }

    pub fn coeff(&self, index: usize) -> f64 {
        self.coeffs[index]
    }

    pub fn coeffs(&self) -> &[f64; EPGA_DIM] {
        &self.coeffs
    }

    pub fn is_zero(&self, eps: f64) -> bool {
        self.coeffs.iter().all(|c| c.abs() < eps)
    }

    pub fn max_abs_diff(&self, other: &Self) -> f64 {
        self.coeffs
            .iter()
            .zip(other.coeffs.iter())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max)
    }

    /// Reverse (blade reversion sign).
    pub fn reverse(&self) -> Self {
        let mut out = Self::zero();
        for (i, &c) in self.coeffs.iter().enumerate() {
            // Grade k reverse sign is (-1)^{k(k-1)/2}.
            let k = i.count_ones();
            let sign = if (k * (k.saturating_sub(1)) / 2) % 2 == 0 {
                1.0
            } else {
                -1.0
            };
            out.coeffs[i] = c * sign;
        }
        out
    }

    pub fn scalar(&self) -> f64 {
        self.coeffs[0]
    }

    /// Geometric product via workspace metric helper (not G(3,1,1) table).
    pub fn geo(&self, rhs: &Self) -> Self {
        let mut result = [0.0; EPGA_DIM];
        for left in 0..EPGA_DIM {
            if self.coeffs[left] == 0.0 {
                continue;
            }
            for right in 0..EPGA_DIM {
                if rhs.coeffs[right] == 0.0 {
                    continue;
                }
                let (sign, out) = basis_mul_with_metric(left, right, &EPGA_METRIC);
                if sign == 0 {
                    continue;
                }
                result[out] += self.coeffs[left] * rhs.coeffs[right] * sign as f64;
            }
        }
        Self { coeffs: result }
    }

    /// Sandwich product M * X * ~M (rigid motion of element X).
    pub fn sandwich(&self, x: &Self) -> Self {
        self.geo(x).geo(&self.reverse())
    }

    /// Approximate unit motor renormalization (even-grade SE(3) motor).
    pub fn normalize_motor(&self) -> Self {
        // For unit motors ||s||^2 ≈ scalar(M * ~M).
        let n2 = self.geo(&self.reverse()).scalar();
        if n2.abs() < 1e-18 {
            return Self::one();
        }
        let inv = 1.0 / n2.sqrt();
        *self * inv
    }
}

impl Add for Multivector {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        let mut coeffs = [0.0; EPGA_DIM];
        for i in 0..EPGA_DIM {
            coeffs[i] = self.coeffs[i] + rhs.coeffs[i];
        }
        Self { coeffs }
    }
}

impl Sub for Multivector {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        let mut coeffs = [0.0; EPGA_DIM];
        for i in 0..EPGA_DIM {
            coeffs[i] = self.coeffs[i] - rhs.coeffs[i];
        }
        Self { coeffs }
    }
}

impl Neg for Multivector {
    type Output = Self;
    fn neg(self) -> Self {
        let mut coeffs = [0.0; EPGA_DIM];
        for i in 0..EPGA_DIM {
            coeffs[i] = -self.coeffs[i];
        }
        Self { coeffs }
    }
}

impl Mul<f64> for Multivector {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        let mut coeffs = [0.0; EPGA_DIM];
        for i in 0..EPGA_DIM {
            coeffs[i] = self.coeffs[i] * rhs;
        }
        Self { coeffs }
    }
}

impl Mul for Multivector {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        self.geo(&rhs)
    }
}

// --- Constructors for geometry ---

/// Plane ax + by + cz + d = 0 as d*e0 + a*e1 + b*e2 + c*e3.
pub fn plane(a: f64, b: f64, c: f64, d: f64) -> Multivector {
    let mut m = Multivector::zero();
    m.coeffs[basis::E0] = d;
    m.coeffs[basis::E1] = a;
    m.coeffs[basis::E2] = b;
    m.coeffs[basis::E3] = c;
    m
}

/// Infinite ground plane y = 0 (normal +Y): plane 0*x + 1*y + 0*z + 0 = 0 → e2.
pub fn ground_plane() -> Multivector {
    plane(0.0, 1.0, 0.0, 0.0)
}

/// Euclidean point (x,y,z) as trivector:
/// e123 − x e023 + y e013 − z e012.
pub fn point(x: f64, y: f64, z: f64) -> Multivector {
    let mut m = Multivector::zero();
    m.coeffs[basis::E123] = 1.0;
    m.coeffs[basis::E023] = -x;
    m.coeffs[basis::E013] = y;
    m.coeffs[basis::E012] = -z;
    m
}

/// Extract (x,y,z) from a Euclidean point multivector (homogeneous).
pub fn extract_point(p: &Multivector) -> [f64; 3] {
    let w = p.coeff(basis::E123);
    if w.abs() < 1e-18 {
        return [0.0, 0.0, 0.0];
    }
    // e123 − x e023 + y e013 − z e012  ⇒  x = −e023/w, y = e013/w, z = −e012/w
    [
        -p.coeff(basis::E023) / w,
        p.coeff(basis::E013) / w,
        -p.coeff(basis::E012) / w,
    ]
}

/// Translator motor for displacement (tx, ty, tz): 1 − ½(tx e01 + ty e02 + tz e03).
pub fn translator(tx: f64, ty: f64, tz: f64) -> Multivector {
    let mut m = Multivector::one();
    m.coeffs[basis::E01] = -0.5 * tx;
    m.coeffs[basis::E02] = -0.5 * ty;
    m.coeffs[basis::E03] = -0.5 * tz;
    m
}

/// Rotor about unit axis (ax,ay,az) by angle (radians), through the origin.
pub fn rotor(ax: f64, ay: f64, az: f64, angle: f64) -> Multivector {
    let half = 0.5 * angle;
    let s = half.sin();
    let c = half.cos();
    // Bivector dual to axis: B = ax e23 + ay e31 + az e12 = ax e23 − ay e13 + az e12
    let mut m = Multivector::zero();
    m.coeffs[basis::SCALAR] = c;
    m.coeffs[basis::E23] = -ax * s;
    m.coeffs[basis::E13] = ay * s;
    m.coeffs[basis::E12] = -az * s;
    m
}

/// Rigid pose motor from world translation + YXZ Euler angles (radians).
pub fn motor_from_pose(x: f64, y: f64, z: f64, pitch: f64, yaw: f64, roll: f64) -> Multivector {
    // Body axes: +Y up, +Z forward-ish, engine fires −Y in body frame.
    let r_yaw = rotor(0.0, 1.0, 0.0, yaw);
    let r_pitch = rotor(1.0, 0.0, 0.0, pitch);
    let r_roll = rotor(0.0, 0.0, 1.0, roll);
    let r = r_yaw.geo(&r_pitch).geo(&r_roll);
    let t = translator(x, y, z);
    // Apply rotation then translation in world: T * R
    t.geo(&r).normalize_motor()
}

/// Extract translation of the origin point under motor M.
pub fn motor_translation(m: &Multivector) -> [f64; 3] {
    let origin = point(0.0, 0.0, 0.0);
    extract_point(&m.sandwich(&origin))
}

/// Transform a body-frame direction (as free vector via ideal points difference proxy)
/// by rotating with the motor's rotational part. Uses sandwich on a direction bivector
/// constructed from two points.
pub fn motor_rotate_vector(m: &Multivector, v: [f64; 3]) -> [f64; 3] {
    // Map body vector v by sandwiching the point at v and subtracting origin image.
    let p = point(v[0], v[1], v[2]);
    let o = point(0.0, 0.0, 0.0);
    let pw = extract_point(&m.sandwich(&p));
    let ow = extract_point(&m.sandwich(&o));
    [pw[0] - ow[0], pw[1] - ow[1], pw[2] - ow[2]]
}

/// Compose incremental motor (left multiply) and renormalize: M' = dM * M.
pub fn compose_motors(delta: &Multivector, m: &Multivector) -> Multivector {
    delta.geo(m).normalize_motor()
}

/// Rotate a world-frame free vector into the body frame (inverse of [`motor_rotate_vector`]).
pub fn motor_inverse_rotate_vector(m: &Multivector, v: [f64; 3]) -> [f64; 3] {
    let inv = m.reverse();
    let p = point(v[0], v[1], v[2]);
    let o = point(0.0, 0.0, 0.0);
    let pw = extract_point(&inv.sandwich(&p));
    let ow = extract_point(&inv.sandwich(&o));
    [pw[0] - ow[0], pw[1] - ow[1], pw[2] - ow[2]]
}

/// Body +Y axis expressed in world coordinates under motor `m`.
pub fn motor_body_up_world(m: &Multivector) -> [f64; 3] {
    motor_rotate_vector(m, [0.0, 1.0, 0.0])
}

/// Small-angle attitude error in body frame: axis to rotate body +Y toward `desired_world`.
///
/// Returns `cross(body_up_world, desired_world)` mapped into the body frame.
pub fn attitude_error_body(m: &Multivector, desired_world: [f64; 3]) -> [f64; 3] {
    let up = motor_body_up_world(m);
    let err_world = [
        up[1] * desired_world[2] - up[2] * desired_world[1],
        up[2] * desired_world[0] - up[0] * desired_world[2],
        up[0] * desired_world[1] - up[1] * desired_world[0],
    ];
    motor_inverse_rotate_vector(m, err_world)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn e0_squares_to_zero() {
        let e0 = Multivector::basis(basis::E0);
        assert!(e0.geo(&e0).is_zero(1e-12));
    }

    #[test]
    fn e1_squares_to_one() {
        let e1 = Multivector::basis(basis::E1);
        let s = e1.geo(&e1);
        assert!((s.scalar() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn translator_moves_point() {
        let t = translator(3.0, -1.0, 2.0);
        let p = point(1.0, 0.0, 0.0);
        let q = extract_point(&t.sandwich(&p));
        assert!((q[0] - 4.0).abs() < 1e-9);
        assert!((q[1] - (-1.0)).abs() < 1e-9);
        assert!((q[2] - 2.0).abs() < 1e-9);
    }

    #[test]
    fn motor_inverse_rotate_roundtrip() {
        let m = motor_from_pose(1.0, 5.0, -2.0, 0.2, -0.15, 0.1);
        let v_body = [0.3, 0.9, -0.4];
        let v_world = motor_rotate_vector(&m, v_body);
        let back = motor_inverse_rotate_vector(&m, v_world);
        assert!((back[0] - v_body[0]).abs() < 1e-8);
        assert!((back[1] - v_body[1]).abs() < 1e-8);
        assert!((back[2] - v_body[2]).abs() < 1e-8);
    }
}
