//! Symbolic basis products and sandwich expansions for G(3,1,1) PGA (32 basis elements).

use std::collections::BTreeMap;

use crate::biquaternion::{Coefficient, multiply_coeff_text};
use crate::coeff_format::format_coeff_display;
use dst_math::pga::{BASIS_LABELS, PGA_DIM, basis_mul};

/// Ordered product of PGA basis indices `0..32` (empty product = scalar 1).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct BasisMonomial {
    pub factors: Vec<usize>,
}

impl BasisMonomial {
    /// Scalar monomial (no basis factors).
    pub fn scalar() -> Self {
        Self {
            factors: Vec::new(),
        }
    }

    /// Single basis element at bitmask index `index` (`BASIS_LABELS[index]`).
    pub fn basis(index: usize) -> Self {
        assert!(index < PGA_DIM);
        Self {
            factors: vec![index],
        }
    }
}

/// One expanded term: coefficient string times a basis monomial.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExpandedTerm {
    pub coeff: Coefficient,
    pub monomial: BasisMonomial,
}

/// Fully expanded sum of basis monomials.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ExpandedProduct {
    pub terms: Vec<ExpandedTerm>,
}

impl ExpandedProduct {
    /// Merges another expanded product into this sum (no like-term combination yet).
    pub fn extend(&mut self, other: ExpandedProduct) {
        self.terms.extend(other.terms);
    }

    /// Multiplies every term of `self` by every term of `rhs`, expanding basis products.
    pub fn mul_assign(&mut self, rhs: &ExpandedProduct) {
        let left = std::mem::take(self);
        *self = multiply_expanded(&left, rhs);
    }
}

/// Expands the product of two single basis elements to one scalar, one basis monomial, or zero.
pub fn expand_basis_product(left: usize, right: usize) -> ExpandedProduct {
    assert!(left < PGA_DIM && right < PGA_DIM);
    let (sign, out) = basis_mul(left, right);
    if sign == 0 {
        return ExpandedProduct::default();
    }
    let coeff = if sign < 0 {
        Coefficient::one().negated()
    } else {
        Coefficient::one()
    };
    let monomial = if out == 0 {
        BasisMonomial::scalar()
    } else {
        BasisMonomial::basis(out)
    };
    ExpandedProduct {
        terms: vec![ExpandedTerm { coeff, monomial }],
    }
}

/// Expands an arbitrary basis monomial by multiplying bitmask indices left-to-right.
pub fn expand_basis_monomial(monomial: &BasisMonomial) -> ExpandedProduct {
    if monomial.factors.is_empty() {
        return ExpandedProduct {
            terms: vec![ExpandedTerm {
                coeff: Coefficient::one(),
                monomial: BasisMonomial::scalar(),
            }],
        };
    }

    let mut coeff = Coefficient::one();
    let mut acc: Option<usize> = None;

    for &f in &monomial.factors {
        assert!(f < PGA_DIM);
        match acc {
            None => acc = Some(f),
            Some(i) => {
                let (sign, out) = basis_mul(i, f);
                if sign == 0 {
                    return ExpandedProduct::default();
                }
                if sign < 0 {
                    coeff = coeff.negated();
                }
                acc = Some(out);
            }
        }
    }

    let monomial = match acc {
        None | Some(0) => BasisMonomial::scalar(),
        Some(i) => BasisMonomial::basis(i),
    };
    ExpandedProduct {
        terms: vec![ExpandedTerm { coeff, monomial }],
    }
}

/// Expands a monomial of PGA generators `e0`..`e4` (indices `0..5` in `factors`).
pub fn expand_generator_monomial(monomial: &BasisMonomial) -> ExpandedProduct {
    let blades: Vec<usize> = monomial.factors.iter().map(|&g| 1usize << g).collect();
    expand_basis_monomial(&BasisMonomial { factors: blades })
}

/// Expands sandwich `left * middle * right` with symbolic coefficients on each factor.
pub fn expand_sandwich(
    left: &BasisMonomial,
    left_coeff: Coefficient,
    middle: &BasisMonomial,
    middle_coeff: Coefficient,
    right: &BasisMonomial,
    right_coeff: Coefficient,
) -> ExpandedProduct {
    let mut left_exp = expand_basis_monomial(left);
    for t in &mut left_exp.terms {
        t.coeff = multiply_coeff_text(&left_coeff.0, &t.coeff.0).into();
    }
    let mut mid_exp = expand_basis_monomial(middle);
    for t in &mut mid_exp.terms {
        t.coeff = multiply_coeff_text(&middle_coeff.0, &t.coeff.0).into();
    }
    let mut right_exp = expand_basis_monomial(right);
    for t in &mut right_exp.terms {
        t.coeff = multiply_coeff_text(&right_coeff.0, &t.coeff.0).into();
    }
    left_exp.mul_assign(&mid_exp);
    left_exp.mul_assign(&right_exp);
    left_exp
}

fn multiply_expanded(left: &ExpandedProduct, right: &ExpandedProduct) -> ExpandedProduct {
    let mut out = ExpandedProduct::default();
    for lt in &left.terms {
        for rt in &right.terms {
            let mut combined = lt.monomial.factors.clone();
            combined.extend(&rt.monomial.factors);
            let mut piece = expand_basis_monomial(&BasisMonomial { factors: combined });
            let outer = multiply_coeff_text(&lt.coeff.0, &rt.coeff.0);
            for t in &mut piece.terms {
                t.coeff = multiply_coeff_text(&outer, &t.coeff.0).into();
            }
            out.extend(piece);
        }
    }
    out
}

/// Merges terms that share the same basis monomial into one term per monomial.
pub fn combine_like_terms(exp: ExpandedProduct) -> ExpandedProduct {
    let mut grouped: BTreeMap<Vec<usize>, Vec<String>> = BTreeMap::new();
    for term in exp.terms {
        grouped
            .entry(term.monomial.factors)
            .or_default()
            .push(term.coeff.0);
    }
    let terms = grouped
        .into_iter()
        .map(|(factors, coeffs)| ExpandedTerm {
            coeff: combine_coeff_sum(&coeffs).into(),
            monomial: BasisMonomial { factors },
        })
        .filter(|t| format_coeff_display(&t.coeff.0) != "0")
        .collect();
    ExpandedProduct { terms }
}

fn combine_coeff_sum(coeffs: &[String]) -> String {
    match coeffs.len() {
        0 => "0".to_string(),
        1 => coeffs[0].clone(),
        _ => coeffs
            .iter()
            .map(|c| format!("({c})"))
            .collect::<Vec<_>>()
            .join("+"),
    }
}

/// Renders the 32×32 basis multiplication table (Markdown) using `dst-math` labels.
pub fn mul_table_markdown() -> String {
    let mut s = String::from("| × |");
    for label in BASIS_LABELS {
        s.push_str(&format!(" {} |", label.trim()));
    }
    s.push('\n');
    s.push_str("|---|");
    for _ in 0..PGA_DIM {
        s.push_str("---|");
    }
    s.push('\n');
    for row in 0..PGA_DIM {
        s.push_str(&format!("| **{}** |", BASIS_LABELS[row].trim()));
        for col in 0..PGA_DIM {
            let (sign, out) = basis_mul(row, col);
            let cell = if sign == 0 {
                "0".to_string()
            } else if out == 0 {
                if sign > 0 { "+1".into() } else { "-1".into() }
            } else {
                let mut c = String::new();
                if sign < 0 {
                    c.push('-');
                }
                c.push_str(BASIS_LABELS[out].trim());
                c
            };
            s.push_str(&format!(" {cell} |"));
        }
        s.push('\n');
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use dst_math::pga::Pga;

    #[test]
    fn e0_squared_minus_one() {
        let exp = expand_basis_product(1, 1);
        assert_eq!(exp.terms.len(), 1);
        assert_eq!(exp.terms[0].coeff.0, "-1");
        assert!(exp.terms[0].monomial.factors.is_empty());
    }

    #[test]
    fn e4_squared_zero() {
        let exp = expand_basis_product(16, 16);
        assert!(exp.terms.is_empty());
    }

    #[test]
    fn null_bivector_squares_to_zero() {
        for mu in 0..4 {
            let n = Pga::null_bivector_index(mu);
            let exp = expand_basis_product(n, n);
            assert!(
                exp.terms.is_empty(),
                "N_{mu} squared should be zero, got {:?}",
                exp.terms
            );
        }
    }

    #[test]
    fn generator_e0_e0_matches_basis_mul() {
        let exp = expand_generator_monomial(&BasisMonomial {
            factors: vec![0, 0],
        });
        assert_eq!(exp.terms.len(), 1);
        assert_eq!(exp.terms[0].coeff.0, "-1");
    }

    #[test]
    fn generator_e4_e4_is_zero() {
        let exp = expand_generator_monomial(&BasisMonomial {
            factors: vec![4, 4],
        });
        assert!(exp.terms.is_empty());
    }

    #[test]
    fn combine_like_terms_merges_same_monomial() {
        let exp = ExpandedProduct {
            terms: vec![
                ExpandedTerm {
                    coeff: Coefficient::named("a"),
                    monomial: BasisMonomial::basis(1),
                },
                ExpandedTerm {
                    coeff: Coefficient::named("b"),
                    monomial: BasisMonomial::basis(1),
                },
            ],
        };
        let merged = combine_like_terms(exp);
        assert_eq!(merged.terms.len(), 1);
        assert_eq!(merged.terms[0].coeff.0, "(a)+(b)");
        assert_eq!(merged.terms[0].monomial.factors, vec![1]);
    }
}
