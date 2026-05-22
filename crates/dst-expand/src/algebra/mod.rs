//! 代数系の統一抽象化レイヤ
//!
//! Double Spacetime Theory 研究向けに、Clifford 代数、Cayley-Dickson 代数、
//! およびそれらのテンソル積を第一級で扱うための基盤。

/// サポートする代数系の種類
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Algebra {
    /// Clifford 代数 Cl(p, q, r) または G(p, q, r) = Cl(p, q, r)
    /// 例: G(3,1,1) for PGA
    Clifford { p: usize, q: usize, r: usize },

    /// Cayley-Dickson 構成による代数 (2, 4, 8, 16, ...)
    /// dimension は 2^n
    CayleyDickson { dimension: usize },

    /// テンソル積代数 (例: H ⊗ H)
    TensorProduct {
        left: Box<Algebra>,
        right: Box<Algebra>,
    },
}

impl Algebra {
    /// 基底の総次元数を返す
    pub fn dimension(&self) -> usize {
        match self {
            Algebra::Clifford { p, q, r } => 2usize.pow((p + q + r) as u32),
            Algebra::CayleyDickson { dimension } => *dimension,
            Algebra::TensorProduct { left, right } => left.dimension() * right.dimension(),
        }
    }

    /// 標準的な名前（REPL 表示用）
    pub fn name(&self) -> String {
        match self {
            Algebra::Clifford { p, q, r } => format!("G({},{},{})", p, q, r),
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
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dimension_clifford() {
        let g311 = Algebra::Clifford { p: 3, q: 1, r: 1 };
        assert_eq!(g311.dimension(), 32);
        let cl31 = Algebra::Clifford { p: 3, q: 1, r: 0 };
        assert_eq!(cl31.dimension(), 16);
    }

    #[test]
    fn name_tensor() {
        let hxh = Algebra::TensorProduct {
            left: Box::new(Algebra::CayleyDickson { dimension: 4 }),
            right: Box::new(Algebra::CayleyDickson { dimension: 4 }),
        };
        assert_eq!(hxh.name(), "H⊗H");
    }
}
