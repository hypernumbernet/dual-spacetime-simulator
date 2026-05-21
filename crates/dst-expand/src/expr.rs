//! Parser for coefficient-bearing tetraquaternion expressions.

use crate::biquaternion::{
    BasisMonomial, Coefficient, ExpandedProduct, ExpandedTerm, combine_like_terms,
    expand_basis_monomial, multiply_coeff_text,
};
use dst_math::biquaternion::BASIS_LABELS;

/// Error while parsing an expression.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParseError {
    pub message: String,
    pub offset: usize,
}

impl ParseError {
    fn new(offset: usize, message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            offset,
        }
    }
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "at offset {}: {}", self.offset, self.message)
    }
}

impl std::error::Error for ParseError {}

/// Basis label and index, sorted by label length descending for longest-match lexing.
const BASIS_TOKENS: [(&str, usize); 15] = [
    ("kI", 1),
    ("kJ", 2),
    ("kK", 3),
    ("iI", 4),
    ("iJ", 5),
    ("iK", 6),
    ("jI", 11),
    ("jJ", 12),
    ("jK", 13),
    ("I", 7),
    ("J", 8),
    ("K", 9),
    ("j", 0),
    ("k", 10),
    ("i", 14),
];

/// Parses an expression into an expanded product (without combining like terms).
pub fn parse_expr(input: &str) -> Result<ExpandedProduct, ParseError> {
    let mut p = Parser::new(input);
    let exp = p.parse_expr()?;
    p.skip_ws();
    if !p.at_end() {
        return Err(p.error("unexpected trailing input"));
    }
    Ok(exp)
}

/// Parses and expands an expression, merging like basis monomials.
pub fn expand_expr(input: &str) -> Result<ExpandedProduct, ParseError> {
    parse_expr(input).map(combine_like_terms)
}

struct Parser<'a> {
    input: &'a str,
    pos: usize,
}

impl<'a> Parser<'a> {
    fn new(input: &'a str) -> Self {
        Self { input, pos: 0 }
    }

    fn at_end(&self) -> bool {
        self.pos >= self.input.len()
    }

    fn peek(&self) -> Option<char> {
        self.input[self.pos..].chars().next()
    }

    fn skip_ws(&mut self) {
        while let Some(c) = self.peek() {
            if c.is_whitespace() {
                self.pos += c.len_utf8();
            } else {
                break;
            }
        }
    }

    fn error(&self, message: impl Into<String>) -> ParseError {
        ParseError::new(self.pos, message)
    }

    fn parse_expr(&mut self) -> Result<ExpandedProduct, ParseError> {
        self.parse_sum()
    }

    fn parse_sum(&mut self) -> Result<ExpandedProduct, ParseError> {
        let mut acc = self.parse_product()?;
        loop {
            self.skip_ws();
            let op = match self.peek() {
                Some('+') => {
                    self.pos += 1;
                    '+'
                }
                Some('-') => {
                    self.pos += 1;
                    '-'
                }
                _ => break,
            };
            let mut rhs = self.parse_product()?;
            if op == '-' {
                for t in &mut rhs.terms {
                    t.coeff = t.coeff.clone().negated();
                }
            }
            acc.extend(rhs);
        }
        Ok(acc)
    }

    fn parse_product(&mut self) -> Result<ExpandedProduct, ParseError> {
        let mut acc = self.parse_unary()?;
        loop {
            self.skip_ws();
            if self.at_end() || matches!(self.peek(), Some(')') | Some('+') | Some('-')) {
                break;
            }
            if self.peek() == Some('*') {
                self.pos += 1;
                self.skip_ws();
            }
            let rhs = self.parse_unary()?;
            acc.mul_assign(&rhs);
        }
        Ok(acc)
    }

    fn parse_unary(&mut self) -> Result<ExpandedProduct, ParseError> {
        self.skip_ws();
        if self.peek() == Some('-') {
            self.pos += 1;
            let mut exp = self.parse_unary()?;
            for t in &mut exp.terms {
                t.coeff = t.coeff.clone().negated();
            }
            return Ok(exp);
        }
        self.parse_primary()
    }

    fn parse_primary(&mut self) -> Result<ExpandedProduct, ParseError> {
        self.skip_ws();
        if self.peek() == Some('(') {
            self.pos += 1;
            let exp = self.parse_expr()?;
            self.skip_ws();
            if self.peek() != Some(')') {
                return Err(self.error("expected ')'"));
            }
            self.pos += 1;
            return Ok(exp);
        }
        let term = self.parse_term()?;
        Ok(ExpandedProduct {
            terms: vec![term],
        })
    }

    fn parse_term(&mut self) -> Result<ExpandedTerm, ParseError> {
        self.skip_ws();
        let start = self.pos;
        let coeff = self.try_parse_coeff();
        let mut factors = Vec::new();
        while let Some(idx) = self.try_parse_basis() {
            factors.push(idx);
        }
        if coeff.is_none() && factors.is_empty() {
            return Err(self.error(format!(
                "expected term (coefficient and/or basis); known bases: {}",
                basis_hint()
            )));
        }
        let coeff = coeff.unwrap_or_else(|| "1".to_string());
        if coeff.is_empty() {
            return Err(ParseError::new(start, "empty coefficient"));
        }
        let monomial = BasisMonomial { factors };
        if monomial.factors.len() <= 1 {
            return Ok(ExpandedTerm {
                coeff: Coefficient(coeff),
                monomial,
            });
        }
        let mut exp = expand_basis_monomial(&monomial);
        let mut term = exp
            .terms
            .pop()
            .expect("expanded monomial has one term");
        term.coeff = multiply_coeff_text(&coeff, &term.coeff.0).into();
        Ok(term)
    }

    fn try_parse_coeff(&mut self) -> Option<String> {
        let start = self.pos;
        self.skip_ws();
        if let Some(num) = self.try_parse_number() {
            return Some(num);
        }
        if !self.is_ident_start() {
            return None;
        }
        let end = self.ident_run_end();
        let slice = &self.input[start..end];
        if self.only_bases_in_range(start, end) {
            return None;
        }
        for k in 1..slice.len() {
            let coeff_part = &slice[..k];
            let rest = start + k;
            if self.only_bases_in_range(rest, end) {
                self.pos = rest;
                return Some(coeff_part.to_string());
            }
        }
        self.pos = end;
        Some(slice.to_string())
    }

    fn ident_run_end(&self) -> usize {
        let mut pos = self.pos;
        if let Some(c) = self.input[pos..].chars().next() {
            if c.is_ascii_alphabetic() || c == '_' {
                pos += c.len_utf8();
            } else {
                return pos;
            }
        }
        while let Some(c) = self.input[pos..].chars().next() {
            if c.is_ascii_alphanumeric() || c == '_' {
                pos += c.len_utf8();
            } else {
                break;
            }
        }
        pos
    }

    fn only_bases_in_range(&self, start: usize, end: usize) -> bool {
        if start >= end {
            return false;
        }
        let mut pos = start;
        while pos < end {
            match self.basis_match_at(pos) {
                Some((len, _)) => pos += len,
                None => return false,
            }
        }
        true
    }

    fn try_parse_basis(&mut self) -> Option<usize> {
        let start = self.pos;
        self.skip_ws();
        if let Some((len, idx)) = self.basis_match_at(self.pos) {
            self.pos += len;
            return Some(idx);
        }
        if self.pos != start {
            self.pos = start;
        }
        None
    }

    fn basis_match_at(&self, pos: usize) -> Option<(usize, usize)> {
        let rest = &self.input[pos..];
        for &(label, idx) in &BASIS_TOKENS {
            if rest.starts_with(label) {
                return Some((label.len(), idx));
            }
        }
        None
    }

    fn try_parse_number(&mut self) -> Option<String> {
        let start = self.pos;
        let bytes = self.input.as_bytes();
        let mut pos = self.pos;
        if pos < bytes.len() && (bytes[pos] == b'+' || bytes[pos] == b'-') {
            pos += 1;
        }
        let mut has_digit = false;
        while pos < bytes.len() && bytes[pos].is_ascii_digit() {
            has_digit = true;
            pos += 1;
        }
        if has_digit && pos < bytes.len() && bytes[pos] == b'.' {
            pos += 1;
            while pos < bytes.len() && bytes[pos].is_ascii_digit() {
                pos += 1;
            }
        }
        if !has_digit {
            return None;
        }
        let s = self.input[start..pos].to_string();
        self.pos = pos;
        Some(s)
    }

    fn is_ident_start(&self) -> bool {
        matches!(
            self.peek(),
            Some(c) if c.is_ascii_alphabetic() || c == '_'
        )
    }

}

fn basis_hint() -> String {
    BASIS_LABELS
        .iter()
        .map(|l| l.trim())
        .collect::<Vec<_>>()
        .join(", ")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::biquaternion::{expand_basis_product, expand_basis_monomial};

    #[test]
    fn parse_single_basis() {
        let exp = parse_expr("j").unwrap();
        assert_eq!(exp.terms.len(), 1);
        assert_eq!(exp.terms[0].coeff.0, "1");
        assert_eq!(exp.terms[0].monomial.factors, vec![0]);
    }

    #[test]
    fn parse_coeff_basis() {
        let exp = parse_expr("ai").unwrap();
        assert_eq!(exp.terms[0].coeff.0, "a");
        assert_eq!(exp.terms[0].monomial.factors, vec![14]);
    }

    #[test]
    fn parse_product_matches_mul() {
        let exp = parse_expr("ij").unwrap();
        let table = expand_basis_product(14, 0);
        assert_eq!(exp.terms.len(), 1);
        assert_eq!(exp.terms[0].coeff.0, table.terms[0].coeff.0);
        assert_eq!(exp.terms[0].monomial.factors, table.terms[0].monomial.factors);
    }

    #[test]
    fn parse_sum_two_terms() {
        let exp = parse_expr("ai+bkI").unwrap();
        assert_eq!(exp.terms.len(), 2);
        assert_eq!(exp.terms[0].coeff.0, "a");
        assert_eq!(exp.terms[0].monomial.factors, vec![14]);
        assert_eq!(exp.terms[1].coeff.0, "b");
        assert_eq!(exp.terms[1].monomial.factors, vec![1]);
    }

    #[test]
    fn parse_parenthesized_product_expands() {
        let exp = expand_expr("(ai+bkI)(cj+dkK)").unwrap();
        assert!(!exp.terms.is_empty());
        let manual_left = parse_expr("ai+bkI").unwrap();
        let manual_right = parse_expr("cj+dkK").unwrap();
        let mut manual = manual_left;
        manual.mul_assign(&manual_right);
        let manual = combine_like_terms(manual);
        assert_eq!(exp.terms.len(), manual.terms.len());
    }

    #[test]
    fn parse_unclosed_paren_errors() {
        assert!(parse_expr("(ai").is_err());
    }

    #[test]
    fn expand_expr_combines_like_terms() {
        let exp = expand_expr("(a)+(a)").unwrap();
        assert_eq!(exp.terms.len(), 1);
        assert_eq!(exp.terms[0].coeff.0, "(a)+(a)");
    }

    #[test]
    fn parse_explicit_star() {
        let a = parse_expr("a*i").unwrap();
        let b = parse_expr("ai").unwrap();
        assert_eq!(a.terms, b.terms);
    }

    #[test]
    fn parse_ki_is_k_then_i() {
        let exp = parse_expr("ki").unwrap();
        let table = expand_basis_monomial(&BasisMonomial {
            factors: vec![10, 14],
        });
        assert_eq!(exp.terms.len(), table.terms.len());
    }
}
