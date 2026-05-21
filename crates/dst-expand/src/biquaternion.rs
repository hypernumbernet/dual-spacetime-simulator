//! Symbolic basis products and sandwich expansions for the 15+1 tetraquaternion basis.

use std::collections::BTreeMap;

use dst_math::biquaternion::{BASIS_LABELS, basis_mul};

/// Coefficient factor in a symbolic term (variable name or numeric literal as text).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Coefficient(pub String);

impl From<String> for Coefficient {
    fn from(s: String) -> Self {
        Self(s)
    }
}

impl Coefficient {
    /// Unit scalar coefficient.
    pub fn one() -> Self {
        Self("1".into())
    }

    /// Named symbolic variable (e.g. rotor component label).
    pub fn named(name: impl Into<String>) -> Self {
        Self(name.into())
    }

    /// Negates the coefficient text when not already signed.
    pub fn negated(self) -> Self {
        let s = self.0;
        if s.starts_with('-') {
            Self(s.trim_start_matches('-').to_string())
        } else if s.starts_with('+') {
            Self(format!("-{}", s.trim_start_matches('+')))
        } else {
            Self(format!("-{s}"))
        }
    }
}

/// Ordered product of basis indices `0..15` (empty product = scalar 1).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct BasisMonomial {
    pub factors: Vec<usize>,
}

impl BasisMonomial {
    /// Scalar monomial (no basis factors).
    pub fn scalar() -> Self {
        Self { factors: Vec::new() }
    }

    /// Single basis element `BASIS_LABELS[i]`.
    pub fn basis(index: usize) -> Self {
        assert!(index < 15);
        Self {
            factors: vec![index],
        }
    }

    /// Appends another basis factor on the right.
    pub fn push_basis(mut self, index: usize) -> Self {
        assert!(index < 15);
        self.factors.push(index);
        self
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

/// Expands the product of two single basis elements to one scalar or one basis monomial.
pub fn expand_basis_product(left: usize, right: usize) -> ExpandedProduct {
    let (sign, out) = basis_mul(left, right);
    let coeff = if sign < 0 {
        Coefficient::one().negated()
    } else {
        Coefficient::one()
    };
    let monomial = if out == 0 {
        BasisMonomial::scalar()
    } else {
        BasisMonomial::basis(out - 1)
    };
    ExpandedProduct {
        terms: vec![ExpandedTerm { coeff, monomial }],
    }
}

/// Expands an arbitrary basis monomial by multiplying factors left-to-right on the table.
pub fn expand_basis_monomial(monomial: &BasisMonomial) -> ExpandedProduct {
    let mut coeff = Coefficient::one();
    let mut single: Option<usize> = None;

    for &f in &monomial.factors {
        match single {
            None => single = Some(f),
            Some(i) => {
                let (sign, out) = basis_mul(i, f);
                if sign < 0 {
                    coeff = coeff.negated();
                }
                single = if out == 0 { None } else { Some(out - 1) };
            }
        }
    }

    let monomial = match single {
        None => BasisMonomial::scalar(),
        Some(i) => BasisMonomial::basis(i),
    };
    ExpandedProduct {
        terms: vec![ExpandedTerm { coeff, monomial }],
    }
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

pub(crate) fn multiply_coeff_text(a: &str, b: &str) -> String {
    if a == "1" {
        return b.to_string();
    }
    if b == "1" {
        return a.to_string();
    }
    if a == "-1" {
        return format!("-{b}");
    }
    if b == "-1" {
        return format!("-{a}");
    }
    format!("({a})*({b})")
}

/// Renders the 15×15 basis multiplication table (Markdown) using `dst-math` labels.
pub fn mul_table_markdown() -> String {
    let mut s = String::from("| × |");
    for label in BASIS_LABELS {
        s.push_str(&format!(" {} |", label.trim()));
    }
    s.push('\n');
    s.push_str("|---|");
    for _ in 0..15 {
        s.push_str("---|");
    }
    s.push('\n');
    for row in 0..15 {
        s.push_str(&format!("| **{}** |", BASIS_LABELS[row].trim()));
        for col in 0..15 {
            let (sign, out) = basis_mul(row, col);
            let cell = if out == 0 {
                if sign > 0 { "+1".into() } else { "-1".into() }
            } else {
                let mut c = String::new();
                if sign < 0 {
                    c.push('-');
                }
                c.push_str(BASIS_LABELS[out - 1].trim());
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

    #[test]
    fn basis_ij_matches_table() {
        let exp = expand_basis_product(4, 5);
        assert_eq!(exp.terms.len(), 1);
        let (sign, out) = basis_mul(4, 5);
        let expected_coeff = if sign < 0 { "-1" } else { "1" };
        assert_eq!(exp.terms[0].coeff.0, expected_coeff);
        if out == 0 {
            assert!(exp.terms[0].monomial.factors.is_empty());
        } else {
            assert_eq!(exp.terms[0].monomial.factors, vec![out - 1]);
        }
    }

    #[test]
    fn combine_like_terms_merges_same_monomial() {
        let exp = ExpandedProduct {
            terms: vec![
                ExpandedTerm {
                    coeff: Coefficient::named("a"),
                    monomial: BasisMonomial::basis(0),
                },
                ExpandedTerm {
                    coeff: Coefficient::named("b"),
                    monomial: BasisMonomial::basis(0),
                },
            ],
        };
        let merged = combine_like_terms(exp);
        assert_eq!(merged.terms.len(), 1);
        assert_eq!(merged.terms[0].coeff.0, "(a)+(b)");
        assert_eq!(merged.terms[0].monomial.factors, vec![0]);
    }

    #[test]
    fn sandwich_j_j_j_is_minus_j() {
        let b = BasisMonomial::basis(0);
        let exp = expand_sandwich(
            &b,
            Coefficient::one(),
            &b,
            Coefficient::one(),
            &b,
            Coefficient::one(),
        );
        assert_eq!(exp.terms.len(), 1);
        assert_eq!(exp.terms[0].coeff.0, "-1");
        assert_eq!(exp.terms[0].monomial.factors, vec![0]);
    }
}
