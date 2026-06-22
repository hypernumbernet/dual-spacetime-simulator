//! Unified abstraction layer for algebras.
//!
//! Foundation for first-class support of PGA, G(p,q,r), Cayley-Dickson algebras,
//! and their tensor products in Double Spacetime Theory research.

use dst_math::pga::{PGA_DIM, PGA_MUL_TABLE, basis_mul_with_metric};

/// Signature of a general geometric algebra G(p, q, r) (non-PGA).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct GSignature {
    p: usize,
    q: usize,
    r: usize,
}

impl GSignature {
    const fn dimension(self) -> usize {
        1usize << (self.p + self.q + self.r)
    }

    const fn generator_count(self) -> usize {
        self.p + self.q + self.r
    }

    fn metric(self, generator: usize) -> i8 {
        assert!(generator < self.generator_count());
        if generator < self.p {
            1
        } else if generator < self.p + self.q {
            -1
        } else {
            0
        }
    }

    fn metric_array(self) -> Vec<i8> {
        (0..self.generator_count())
            .map(|k| self.metric(k))
            .collect()
    }

    fn basis_mul(self, left: usize, right: usize) -> (i8, usize) {
        assert!(left < self.dimension() && right < self.dimension());
        basis_mul_with_metric(left, right, &self.metric_array())
    }
}

/// Supported algebra kinds.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Algebra {
    /// G(3,1,1) Projective Geometric Algebra (PGA)
    Pga,

    /// General geometric algebra G(p, q, r) = Cl(p, q, r) (non-PGA).
    G { p: usize, q: usize, r: usize },

    /// Algebra via Cayley-Dickson construction (2, 4, 8, 16, ...).
    /// `dimension` is 2^n.
    CayleyDickson { dimension: usize },

    /// Tensor product algebra (e.g. H ⊗ H).
    TensorProduct {
        left: Box<Algebra>,
        right: Box<Algebra>,
    },
}

impl Algebra {
    /// Returns the total dimension of the basis.
    pub fn dimension(&self) -> usize {
        match self {
            Algebra::Pga => PGA_DIM,
            Algebra::G { p, q, r } => 2usize.pow((p + q + r) as u32),
            Algebra::CayleyDickson { dimension } => *dimension,
            Algebra::TensorProduct { left, right } => left.dimension() * right.dimension(),
        }
    }

    /// Standard name (for REPL display).
    pub fn name(&self) -> String {
        match self {
            Algebra::Pga => "PGA".to_string(),
            Algebra::G { p, q, r } => format!("G({},{},{})", p, q, r),
            Algebra::CayleyDickson { dimension } => match dimension {
                2 => "C".to_string(),
                4 => "H".to_string(),
                8 => "O".to_string(),
                16 => "S".to_string(),
                _ => format!("CD{}", dimension),
            },
            Algebra::TensorProduct { left, right } => format!("{}⊗{}", left.name(), right.name()),
        }
    }

    /// G(3,1,1) PGA shortcut.
    pub fn pga() -> Self {
        Algebra::Pga
    }

    /// Returns basis multiplication `(sign, result_index)`. Returns `None` for unsupported algebras.
    pub fn basis_mul(&self, left: usize, right: usize) -> Option<(i8, usize)> {
        match self {
            Algebra::Pga if left < PGA_DIM && right < PGA_DIM => Some(PGA_MUL_TABLE[left][right]),
            Algebra::G { p, q, r } => {
                let sig = GSignature {
                    p: *p,
                    q: *q,
                    r: *r,
                };
                let dim = sig.dimension();
                if left < dim && right < dim {
                    Some(sig.basis_mul(left, right))
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    /// Reference to the 32×32 multiplication table for PGA (returns `None` if not applicable).
    pub fn pga_mul_table(&self) -> Option<&'static [[(i8, usize); PGA_DIM]; PGA_DIM]> {
        match self {
            Algebra::Pga => Some(&PGA_MUL_TABLE),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dimension_pga_and_g() {
        let pga = Algebra::pga();
        assert_eq!(pga.dimension(), 32);
        let cl31 = Algebra::G { p: 3, q: 1, r: 0 };
        assert_eq!(cl31.dimension(), 16);
    }

    #[test]
    fn pga_is_dedicated_variant() {
        assert_eq!(Algebra::pga(), Algebra::Pga);
        assert_eq!(Algebra::pga().dimension(), 32);
    }

    #[test]
    fn pga_name() {
        assert_eq!(Algebra::pga().name(), "PGA");
    }

    #[test]
    fn g_name_for_cl31() {
        let cl31 = Algebra::G { p: 3, q: 1, r: 0 };
        assert_eq!(cl31.name(), "G(3,1,0)");
    }

    #[test]
    fn pga_basis_mul_via_algebra() {
        let alg = Algebra::pga();
        assert_eq!(alg.basis_mul(1, 1), Some((-1, 0)));
        assert_eq!(alg.basis_mul(16, 16), Some((0, 0)));
    }

    #[test]
    fn pga_mul_table_access() {
        let alg = Algebra::pga();
        let table = alg.pga_mul_table().expect("PGA table");
        assert_eq!(table[2][4], PGA_MUL_TABLE[2][4]);
    }

    #[test]
    fn name_tensor() {
        let hxh = Algebra::TensorProduct {
            left: Box::new(Algebra::CayleyDickson { dimension: 4 }),
            right: Box::new(Algebra::CayleyDickson { dimension: 4 }),
        };
        assert_eq!(hxh.name(), "H⊗H");
    }

    #[test]
    fn cayley_dickson_basis_mul_unsupported() {
        let h = Algebra::CayleyDickson { dimension: 4 };
        assert!(h.basis_mul(0, 1).is_none());
    }
}
