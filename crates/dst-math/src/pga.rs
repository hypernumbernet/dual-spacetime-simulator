//! Projective Geometric Algebra G(3,1,1) (PGA).
//!
//! Basis elements are indexed by bitmasks: bit `k` corresponds to generator `e_k`.
//! Double Spacetime Theory metric assignment:
//! e0² = -1, e1² = e2² = e3² = +1, e4² = 0.

use std::ops::{Add, AddAssign, Mul, MulAssign, Neg, Sub, SubAssign};

/// Number of basis elements in G(3,1,1) PGA (2^5).
pub const PGA_DIM: usize = 32;

/// Metric square for each generator: e_k² = METRIC[k].
/// Assignment: e0²=-1, e1²=e2²=e3²=+1, e4²=0.
pub const PGA_METRIC: [i8; 5] = [-1, 1, 1, 1, 0];

/// Projective Geometric Algebra G(3,1,1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pga;

/// Singleton PGA instance.
pub const PGA: Pga = Pga;

impl Pga {
    /// Algebra dimension (32).
    pub const fn dimension() -> usize {
        PGA_DIM
    }

    /// Number of generators (5).
    pub const fn generator_count() -> usize {
        5
    }

    /// Metric array for all generators.
    pub fn metric_array() -> [i8; 5] {
        PGA_METRIC
    }

    /// Grade (popcount) of a basis index.
    pub fn grade(index: usize) -> u32 {
        index.count_ones()
    }

    /// Reverse sign for a basis blade, computed by multiplying generators in reverse order.
    pub fn reverse_sign(index: usize) -> i8 {
        reverse_basis_pga(index).0
    }

    /// Conjugate sign for a basis blade: reverse with odd-grade flip.
    pub fn conjugate_sign(index: usize) -> i8 {
        let rev = Self::reverse_sign(index);
        if Self::grade(index) % 2 == 1 {
            -rev
        } else {
            rev
        }
    }

    /// Basis index for null bivector N_mu = e4 ∧ e_mu (geometric product e4 * e_mu).
    pub fn null_bivector_index(mu: usize) -> usize {
        assert!(mu < 4, "mu must be 0..3");
        (1 << mu) | (1 << 4)
    }

    /// Multiplies two basis elements under the geometric product.
    pub fn basis_mul(left: usize, right: usize) -> (i8, usize) {
        assert!(left < PGA_DIM && right < PGA_DIM);
        PGA_MUL_TABLE[left][right]
    }
}

/// Geometric product of two basis blades using an explicit metric array.
pub fn basis_mul_with_metric(left: usize, right: usize, metric: &[i8]) -> (i8, usize) {
    if left == 0 {
        return (1, right);
    }
    if right == 0 {
        return (1, left);
    }

    let mut sign: i8 = 1;
    let mut a = left;
    let mut b = right;

    while b != 0 {
        let lb = b & b.wrapping_neg();
        let bit = lb.trailing_zeros() as usize;
        b ^= lb;

        let above = a >> (bit + 1);
        if above.count_ones() % 2 == 1 {
            sign = -sign;
        }

        if a & lb != 0 {
            sign *= metric[bit];
            a ^= lb;
        } else {
            a |= lb;
        }
    }

    (sign, a)
}

/// Reverses a PGA basis blade by multiplying generators in reverse order.
const fn reverse_basis_pga(index: usize) -> (i8, usize) {
    if index == 0 {
        return (1, 0);
    }

    let mut bits = [0usize; 5];
    let mut count = 0usize;
    let mut g = 0usize;
    while g < 5 {
        if index & (1 << g) != 0 {
            bits[count] = g;
            count += 1;
        }
        g += 1;
    }

    let mut sign: i8 = 1;
    let mut acc = 0usize;
    let mut i = count;
    while i > 0 {
        i -= 1;
        let (s, r) = basis_mul_const(acc, 1 << bits[i]);
        sign *= s;
        acc = r;
    }

    (sign, acc)
}

const fn compute_pga_mul_table() -> [[(i8, usize); PGA_DIM]; PGA_DIM] {
    let mut table = [[(0i8, 0usize); PGA_DIM]; PGA_DIM];
    let mut i = 0usize;
    while i < PGA_DIM {
        let mut j = 0usize;
        while j < PGA_DIM {
            table[i][j] = basis_mul_const(i, j);
            j += 1;
        }
        i += 1;
    }
    table
}

const fn basis_mul_const(left: usize, right: usize) -> (i8, usize) {
    if left == 0 {
        return (1, right);
    }
    if right == 0 {
        return (1, left);
    }

    let mut sign: i8 = 1;
    let mut a = left;
    let mut b = right;

    while b != 0 {
        let lb = b & (0usize.wrapping_sub(b));
        let bit = lb.trailing_zeros() as usize;
        b ^= lb;

        let above = a >> (bit + 1);
        if above.count_ones() % 2 == 1 {
            sign = -sign;
        }

        if a & lb != 0 {
            sign *= PGA_METRIC[bit];
            a ^= lb;
        } else {
            a |= lb;
        }
    }

    (sign, a)
}

/// Precomputed 32×32 multiplication table for G(3,1,1) PGA.
pub const PGA_MUL_TABLE: [[(i8, usize); PGA_DIM]; PGA_DIM] = compute_pga_mul_table();

/// Human-readable labels for all 32 basis elements.
pub const BASIS_LABELS: [&str; PGA_DIM] = [
    "1", "e0", "e1", "e0e1", "e2", "e0e2", "e1e2", "e0e1e2", "e3", "e0e3", "e1e3", "e0e1e3",
    "e2e3", "e0e2e3", "e1e2e3", "e0e1e2e3", "e4", "e0e4", "e1e4", "e0e1e4", "e2e4", "e0e2e4",
    "e1e2e4", "e0e1e2e4", "e3e4", "e0e3e4", "e1e3e4", "e0e1e3e4", "e2e3e4", "e0e2e3e4",
    "e1e2e3e4", "e0e1e2e3e4",
];

/// Returns the label for a basis index.
pub fn basis_label(index: usize) -> &'static str {
    assert!(index < PGA_DIM);
    BASIS_LABELS[index]
}

/// 32-dimensional multivector in G(3,1,1) PGA.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Multivector {
    coeffs: [f64; PGA_DIM],
}

impl Multivector {
    /// Creates a multivector from a full coefficient array.
    pub fn new(coeffs: [f64; PGA_DIM]) -> Self {
        Self { coeffs }
    }

    /// Additive zero.
    pub fn zero() -> Self {
        Self {
            coeffs: [0.0; PGA_DIM],
        }
    }

    /// Multiplicative identity (scalar 1).
    pub fn one() -> Self {
        let mut coeffs = [0.0; PGA_DIM];
        coeffs[0] = 1.0;
        Self { coeffs }
    }

    /// Unit basis element by index.
    pub fn basis(index: usize) -> Self {
        assert!(index < PGA_DIM);
        let mut coeffs = [0.0; PGA_DIM];
        coeffs[index] = 1.0;
        Self { coeffs }
    }

    /// Coefficient at basis index.
    pub fn coeff(&self, index: usize) -> f64 {
        self.coeffs[index]
    }

    /// Coefficient slice.
    pub fn coeffs(&self) -> &[f64; PGA_DIM] {
        &self.coeffs
    }

    /// Returns true when all coefficients are numerically near zero.
    pub fn is_zero(&self, eps: f64) -> bool {
        self.coeffs.iter().all(|c| c.abs() < eps)
    }

    /// Maximum absolute per-coefficient difference.
    pub fn max_abs_diff(&self, other: &Self) -> f64 {
        self.coeffs
            .iter()
            .zip(other.coeffs.iter())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max)
    }

    /// Grade-k projection.
    pub fn grade(&self, k: u32) -> Self {
        let mut out = Self::zero();
        for (i, &c) in self.coeffs.iter().enumerate() {
            if Pga::grade(i) == k {
                out.coeffs[i] = c;
            }
        }
        out
    }

    /// Reverse (reverses order of basis vectors in each blade).
    pub fn reverse(&self) -> Self {
        let mut out = Self::zero();
        for (i, &c) in self.coeffs.iter().enumerate() {
            out.coeffs[i] = c * Pga::reverse_sign(i) as f64;
        }
        out
    }

    /// Clifford conjugate.
    pub fn conjugate(&self) -> Self {
        let mut out = Self::zero();
        for (i, &c) in self.coeffs.iter().enumerate() {
            out.coeffs[i] = c * Pga::conjugate_sign(i) as f64;
        }
        out
    }

    /// Scalar part (grade 0).
    pub fn scalar(&self) -> f64 {
        self.coeffs[0]
    }
}

impl Add for Multivector {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        let mut coeffs = [0.0; PGA_DIM];
        for i in 0..PGA_DIM {
            coeffs[i] = self.coeffs[i] + rhs.coeffs[i];
        }
        Self { coeffs }
    }
}

impl Sub for Multivector {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        let mut coeffs = [0.0; PGA_DIM];
        for i in 0..PGA_DIM {
            coeffs[i] = self.coeffs[i] - rhs.coeffs[i];
        }
        Self { coeffs }
    }
}

impl Neg for Multivector {
    type Output = Self;

    fn neg(self) -> Self {
        let mut coeffs = [0.0; PGA_DIM];
        for i in 0..PGA_DIM {
            coeffs[i] = -self.coeffs[i];
        }
        Self { coeffs }
    }
}

impl Mul for Multivector {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self {
        let mut result = [0.0; PGA_DIM];

        for left in 0..PGA_DIM {
            if self.coeffs[left] == 0.0 {
                continue;
            }
            for right in 0..PGA_DIM {
                if rhs.coeffs[right] == 0.0 {
                    continue;
                }
                let (sign, out) = PGA_MUL_TABLE[left][right];
                result[out] += self.coeffs[left] * rhs.coeffs[right] * sign as f64;
            }
        }

        Self { coeffs: result }
    }
}

impl Mul<f64> for Multivector {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        let mut coeffs = [0.0; PGA_DIM];
        for i in 0..PGA_DIM {
            coeffs[i] = self.coeffs[i] * rhs;
        }
        Self { coeffs }
    }
}

impl AddAssign for Multivector {
    fn add_assign(&mut self, rhs: Self) {
        for i in 0..PGA_DIM {
            self.coeffs[i] += rhs.coeffs[i];
        }
    }
}

impl SubAssign for Multivector {
    fn sub_assign(&mut self, rhs: Self) {
        for i in 0..PGA_DIM {
            self.coeffs[i] -= rhs.coeffs[i];
        }
    }
}

impl MulAssign for Multivector {
    fn mul_assign(&mut self, rhs: Self) {
        *self = *self * rhs;
    }
}

/// One entry of the PGA basis multiplication table.
pub fn basis_mul(left: usize, right: usize) -> (i8, usize) {
    Pga::basis_mul(left, right)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pga_dimension_is_32() {
        assert_eq!(Pga::dimension(), 32);
    }

    #[test]
    fn e0_squared_is_minus_one() {
        let (sign, idx) = basis_mul(1, 1);
        assert_eq!(sign, -1);
        assert_eq!(idx, 0);
    }

    #[test]
    fn e4_squared_is_zero() {
        let (sign, idx) = basis_mul(16, 16);
        assert_eq!(sign, 0);
        assert_eq!(idx, 0);
    }

    #[test]
    fn null_bivector_squares_to_zero() {
        for mu in 0..4 {
            let n = Pga::null_bivector_index(mu);
            let (sign, idx) = basis_mul(n, n);
            assert_eq!(sign, 0, "N{mu} squared should vanish");
            assert_eq!(idx, 0);
        }
    }

    #[test]
    fn vectors_anticommute() {
        let e0 = Multivector::basis(1);
        let e1 = Multivector::basis(2);
        let sum = e0 * e1 + e1 * e0;
        assert!(sum.is_zero(1e-10));
    }

    #[test]
    fn reverse_of_product() {
        let a = Multivector::basis(3) + Multivector::basis(5) * 0.5;
        let b = Multivector::basis(17) + Multivector::basis(2);
        let left = (a * b).reverse();
        let right = b.reverse() * a.reverse();
        assert!(left.max_abs_diff(&right) < 1e-9);
    }
}
