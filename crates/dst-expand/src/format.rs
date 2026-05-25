//! Pretty-printing for expanded symbolic products (biquaternion and PGA).

use crate::biquaternion::{BasisMonomial, ExpandedProduct};
use crate::coeff_format::format_coeff_display;
use crate::pga::{BasisMonomial as PgaBasisMonomial, ExpandedProduct as PgaExpandedProduct};
use dst_math::biquaternion::BASIS_LABELS;
use dst_math::pga::BASIS_LABELS as PGA_BASIS_LABELS;

/// Formats an expanded product as a sum of basis monomials.
pub fn format_expanded(exp: &ExpandedProduct) -> String {
    if exp.terms.is_empty() {
        return "0".to_string();
    }
    let parts: Vec<String> = exp.terms.iter().map(format_term).collect();
    let mut out = parts[0].clone();
    for part in parts.iter().skip(1) {
        if part.starts_with('-') && !part.starts_with("(-") {
            out.push_str(" - ");
            out.push_str(part.strip_prefix('-').unwrap_or(part).trim_start());
        } else {
            out.push_str(" + ");
            out.push_str(part);
        }
    }
    out
}

fn coeff_needs_parens(coeff: &str) -> bool {
    coeff.contains('+') || coeff.chars().skip(1).any(|c| c == '-')
}

fn format_term(term: &crate::biquaternion::ExpandedTerm) -> String {
    let coeff = format_coeff_display(&term.coeff.0);
    let mono = format_monomial(&term.monomial);
    if coeff == "1" {
        mono
    } else if coeff == "-1" {
        if mono == "1" {
            "-1".to_string()
        } else {
            format!("-{mono}")
        }
    } else if mono == "1" {
        if coeff_needs_parens(&coeff) {
            format!("({coeff})")
        } else {
            coeff
        }
    } else {
        let coeff_part = if coeff_needs_parens(&coeff) {
            format!("({coeff})")
        } else {
            coeff
        };
        format!("{coeff_part}{mono}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::biquaternion::{BasisMonomial, Coefficient, ExpandedProduct, ExpandedTerm};

    #[test]
    fn format_sum_coeff_in_parens_before_basis() {
        use crate::expand_expr;
        let exp = expand_expr("(b-ai)(a+bi)").unwrap();
        assert_eq!(format_expanded(&exp), "2ab + (-aa+bb)[i]");
    }

    #[test]
    fn format_strips_coeff_parens_and_signs() {
        let exp = ExpandedProduct {
            terms: vec![ExpandedTerm {
                coeff: Coefficient("-(b)*(c)".into()),
                monomial: BasisMonomial::basis(4),
            }],
        };
        assert_eq!(format_expanded(&exp), "-bc[iI]");
    }
}

fn format_monomial(m: &BasisMonomial) -> String {
    if m.factors.is_empty() {
        return "1".to_string();
    }
    m.factors
        .iter()
        .map(|&i| format!("[{}]", BASIS_LABELS[i].trim()))
        .collect::<String>()
}

/// Formats a PGA expanded product as a sum of basis monomials.
pub fn format_pga_expanded(exp: &PgaExpandedProduct) -> String {
    if exp.terms.is_empty() {
        return "0".to_string();
    }
    let parts: Vec<String> = exp
        .terms
        .iter()
        .map(format_pga_term)
        .collect();
    let mut out = parts[0].clone();
    for part in parts.iter().skip(1) {
        if part.starts_with('-') && !part.starts_with("(-") {
            out.push_str(" - ");
            out.push_str(part.strip_prefix('-').unwrap_or(part).trim_start());
        } else {
            out.push_str(" + ");
            out.push_str(part);
        }
    }
    out
}

fn format_pga_term(term: &crate::pga::ExpandedTerm) -> String {
    let coeff = format_coeff_display(&term.coeff.0);
    let mono = format_pga_monomial(&term.monomial);
    if coeff == "1" {
        mono
    } else if coeff == "-1" {
        if mono == "1" {
            "-1".to_string()
        } else {
            format!("-{mono}")
        }
    } else if mono == "1" {
        if coeff_needs_parens(&coeff) {
            format!("({coeff})")
        } else {
            coeff
        }
    } else {
        let coeff_part = if coeff_needs_parens(&coeff) {
            format!("({coeff})")
        } else {
            coeff
        };
        format!("{coeff_part}{mono}")
    }
}

fn format_pga_monomial(m: &PgaBasisMonomial) -> String {
    if m.factors.is_empty() {
        return "1".to_string();
    }
    if m.factors.len() == 1 {
        return format!("[{}]", PGA_BASIS_LABELS[m.factors[0]].trim());
    }
    m.factors
        .iter()
        .map(|&i| format!("[{}]", PGA_BASIS_LABELS[i].trim()))
        .collect::<String>()
}
