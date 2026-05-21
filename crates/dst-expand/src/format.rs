//! Pretty-printing for [`ExpandedProduct`](crate::biquaternion::ExpandedProduct).

use crate::biquaternion::{BasisMonomial, ExpandedProduct};
use dst_math::biquaternion::BASIS_LABELS;

/// Formats an expanded product as a sum of basis monomials.
pub fn format_expanded(exp: &ExpandedProduct) -> String {
    if exp.terms.is_empty() {
        return "0".to_string();
    }
    exp.terms
        .iter()
        .map(format_term)
        .collect::<Vec<_>>()
        .join(" + ")
        .replace("+ -", "- ")
}

fn format_term(term: &crate::biquaternion::ExpandedTerm) -> String {
    let mono = format_monomial(&term.monomial);
    if term.coeff.0 == "1" {
        mono
    } else if term.coeff.0 == "-1" {
        if mono == "1" {
            "-1".to_string()
        } else {
            format!("-{mono}")
        }
    } else if mono == "1" {
        term.coeff.0.clone()
    } else {
        format!("{}*{}", term.coeff.0, mono)
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
