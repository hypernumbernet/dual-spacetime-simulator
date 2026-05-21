//! Parses symbolic coefficient strings, simplifies them, and formats without `*`.

use std::collections::BTreeMap;

#[derive(Clone, Debug, PartialEq, Eq)]
enum CoeffFactor {
    Atom(String),
    Sum(CoeffSum),
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct CoeffProduct {
    sign: i8,
    factors: Vec<CoeffFactor>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct CoeffSum {
    terms: Vec<CoeffProduct>,
}

/// Canonical product: numeric part times symbolic factors with exponents.
#[derive(Clone, Debug, PartialEq, Eq)]
struct CanonicalProduct {
    sign: i8,
    numeric: i64,
    factors: BTreeMap<String, u32>,
}

/// Formats a coefficient string (products/sums of symbolic factors) for display.
pub fn format_coeff_display(input: &str) -> String {
    match parse_coeff_sum(input) {
        Ok(sum) => format_sum(&simplify_sum(sum)),
        Err(_) => input.to_string(),
    }
}

fn parse_coeff_sum(input: &str) -> Result<CoeffSum, ()> {
    let mut p = CoeffParser::new(input);
    let first = p.parse_product()?;
    let mut terms = vec![first];
    loop {
        p.skip_ws();
        match p.peek() {
            Some('+') => {
                p.bump();
                terms.push(p.parse_product()?);
            }
            Some('-') => {
                p.bump();
                let mut term = p.parse_product()?;
                term.sign *= -1;
                terms.push(term);
            }
            _ => break,
        }
    }
    p.skip_ws();
    if !p.at_end() {
        return Err(());
    }
    Ok(CoeffSum { terms })
}

struct CoeffParser<'a> {
    input: &'a str,
    pos: usize,
}

impl<'a> CoeffParser<'a> {
    fn new(input: &'a str) -> Self {
        Self { input, pos: 0 }
    }

    fn at_end(&self) -> bool {
        self.pos >= self.input.len()
    }

    fn peek(&self) -> Option<char> {
        self.input[self.pos..].chars().next()
    }

    fn bump(&mut self) {
        if let Some(c) = self.peek() {
            self.pos += c.len_utf8();
        }
    }

    fn skip_ws(&mut self) {
        while let Some(c) = self.peek() {
            if c.is_whitespace() {
                self.bump();
            } else {
                break;
            }
        }
    }

    fn parse_product(&mut self) -> Result<CoeffProduct, ()> {
        let mut acc = self.parse_signed_factor()?;
        loop {
            self.skip_ws();
            if self.peek() != Some('*') {
                break;
            }
            self.bump();
            let rhs = self.parse_signed_factor()?;
            merge_product(&mut acc, rhs);
        }
        Ok(acc)
    }

    fn parse_signed_factor(&mut self) -> Result<CoeffProduct, ()> {
        let mut sign: i8 = 1;
        loop {
            self.skip_ws();
            match self.peek() {
                Some('+') => self.bump(),
                Some('-') => {
                    sign *= -1;
                    self.bump();
                }
                _ => break,
            }
        }
        match self.parse_factor()? {
            CoeffFactor::Sum(sum) if sum.terms.len() == 1 => {
                let mut t = sum.terms[0].clone();
                t.sign *= sign;
                Ok(t)
            }
            factor => Ok(CoeffProduct {
                sign,
                factors: vec![factor],
            }),
        }
    }

    fn parse_factor(&mut self) -> Result<CoeffFactor, ()> {
        self.skip_ws();
        if self.peek() == Some('(') {
            self.bump();
            let sum = parse_coeff_sum_from_parser(self)?;
            self.skip_ws();
            if self.peek() != Some(')') {
                return Err(());
            }
            self.bump();
            return Ok(CoeffFactor::Sum(sum));
        }
        let start = self.pos;
        if let Some(c) = self.peek() {
            if c.is_ascii_digit() || c == '.' {
                while matches!(self.peek(), Some(c) if c.is_ascii_digit() || c == '.') {
                    self.bump();
                }
            } else if c.is_ascii_alphabetic() || c == '_' {
                self.bump();
                while matches!(self.peek(), Some(c) if c.is_ascii_alphanumeric() || c == '_') {
                    self.bump();
                }
            } else {
                return Err(());
            }
        } else {
            return Err(());
        }
        if self.pos == start {
            return Err(());
        }
        Ok(CoeffFactor::Atom(self.input[start..self.pos].to_string()))
    }
}

fn parse_coeff_sum_from_parser(p: &mut CoeffParser<'_>) -> Result<CoeffSum, ()> {
    let first = p.parse_product()?;
    let mut terms = vec![first];
    loop {
        p.skip_ws();
        match p.peek() {
            Some('+') => {
                p.bump();
                terms.push(p.parse_product()?);
            }
            Some('-') => {
                p.bump();
                let mut term = p.parse_product()?;
                term.sign *= -1;
                terms.push(term);
            }
            _ => break,
        }
    }
    Ok(CoeffSum { terms })
}

fn merge_product(acc: &mut CoeffProduct, rhs: CoeffProduct) {
    acc.sign *= rhs.sign;
    for f in rhs.factors {
        match f {
            CoeffFactor::Sum(sum) if sum.terms.len() == 1 => {
                let t = sum.terms[0].clone();
                acc.sign *= t.sign;
                acc.factors.extend(t.factors);
            }
            other => acc.factors.push(other),
        }
    }
}

fn canonicalize_product(p: &CoeffProduct) -> Option<CanonicalProduct> {
    let mut numeric: i64 = 1;
    let mut factors: BTreeMap<String, u32> = BTreeMap::new();
    for f in &p.factors {
        match f {
            CoeffFactor::Atom(s) => {
                if let Ok(n) = s.parse::<i64>() {
                    numeric = numeric.saturating_mul(n);
                } else {
                    *factors.entry(s.clone()).or_insert(0) += 1;
                }
            }
            CoeffFactor::Sum(_) => return None,
        }
    }
    Some(CanonicalProduct {
        sign: p.sign,
        numeric,
        factors,
    })
}

fn simplify_sum(sum: CoeffSum) -> CoeffSum {
    let mut grouped: BTreeMap<BTreeMap<String, u32>, i64> = BTreeMap::new();
    let mut fallback: Vec<CoeffProduct> = Vec::new();

    for term in sum.terms {
        match canonicalize_product(&term) {
            Some(c) => {
                let contrib = (c.sign as i64).saturating_mul(c.numeric);
                *grouped.entry(c.factors).or_insert(0) += contrib;
            }
            None => fallback.push(term),
        }
    }

    let mut terms: Vec<CoeffProduct> = fallback;
    for (factors, total) in grouped {
        if total == 0 {
            continue;
        }
        let sign: i8 = if total < 0 { -1 } else { 1 };
        let numeric = total.unsigned_abs() as i64;
        terms.push(canonical_to_product(sign, numeric, factors));
    }

    CoeffSum { terms }
}

fn canonical_to_product(sign: i8, numeric: i64, factors: BTreeMap<String, u32>) -> CoeffProduct {
    let mut atoms: Vec<CoeffFactor> = Vec::new();
    if numeric != 1 {
        atoms.push(CoeffFactor::Atom(numeric.to_string()));
    }
    for (name, exp) in factors {
        for _ in 0..exp {
            atoms.push(CoeffFactor::Atom(name.clone()));
        }
    }
    CoeffProduct {
        sign,
        factors: atoms,
    }
}

fn format_sum(sum: &CoeffSum) -> String {
    if sum.terms.is_empty() {
        return "0".to_string();
    }
    let parts: Vec<String> = sum
        .terms
        .iter()
        .filter_map(|t| {
            if let Some(c) = canonicalize_product(t) {
                if c.numeric == 0 {
                    return None;
                }
                Some(format_canonical(&c))
            } else {
                Some(format_product(t))
            }
        })
        .collect();
    if parts.is_empty() {
        return "0".to_string();
    }
    let mut out = parts[0].clone();
    for part in parts.iter().skip(1) {
        if part.starts_with('-') {
            let tail = part.strip_prefix('-').unwrap_or(part).trim_start();
            out.push('-');
            out.push_str(tail);
        } else {
            out.push('+');
            out.push_str(part);
        }
    }
    out
}

fn format_canonical(c: &CanonicalProduct) -> String {
    let body = format_canonical_body(c);
    match c.sign {
        -1 if body == "1" => "-1".to_string(),
        -1 => format!("-{body}"),
        1 => body,
        _ => body,
    }
}

fn format_canonical_body(c: &CanonicalProduct) -> String {
    let mut s = String::new();
    if c.numeric != 1 {
        s.push_str(&c.numeric.to_string());
    }
    for (name, exp) in &c.factors {
        s.push_str(&format_factor_power(name, *exp));
    }
    if s.is_empty() {
        "1".to_string()
    } else {
        s
    }
}

fn format_factor_power(name: &str, exp: u32) -> String {
    if exp == 0 {
        return String::new();
    }
    if exp == 1 {
        return name.to_string();
    }
    if name.len() == 1 {
        return name.repeat(exp as usize);
    }
    format!("{name}^{exp}")
}

fn format_product(product: &CoeffProduct) -> String {
    if let Some(c) = canonicalize_product(product) {
        return format_canonical(&c);
    }
    if product.factors.is_empty() {
        return if product.sign < 0 {
            "-1".to_string()
        } else {
            "1".to_string()
        };
    }
    let body = format_product_body(product);
    match product.sign {
        -1 if body == "1" => "-1".to_string(),
        -1 => format!("-{body}"),
        1 => body,
        _ => body,
    }
}

fn format_product_body(product: &CoeffProduct) -> String {
    if let Some(c) = canonicalize_product(product) {
        return format_canonical_body(&c);
    }
    let mut parts: Vec<String> = product.factors.iter().map(format_factor).collect();
    match parts.len() {
        0 => "1".to_string(),
        1 => parts.pop().unwrap(),
        _ => parts.join(""),
    }
}

fn format_factor(factor: &CoeffFactor) -> String {
    match factor {
        CoeffFactor::Atom(s) => s.clone(),
        CoeffFactor::Sum(sum) => {
            let inner = format_sum(&simplify_sum(sum.clone()));
            if needs_parens(&inner) {
                format!("({inner})")
            } else {
                inner
            }
        }
    }
}

fn needs_parens(s: &str) -> bool {
    s.contains('+') || s.chars().skip(1).any(|c| c == '-')
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strips_product_parens_and_negatives() {
        assert_eq!(format_coeff_display("-(b)*(c)"), "-bc");
        assert_eq!(format_coeff_display("(a)*(-b)"), "-ab");
    }

    #[test]
    fn strips_sum_parens() {
        assert_eq!(format_coeff_display("(a)+(b)"), "a+b");
    }

    #[test]
    fn merges_like_products_in_sum() {
        assert_eq!(format_coeff_display("(a)+(a)"), "2a");
        assert_eq!(format_coeff_display("(a)*c+(a)*c"), "2ac");
        assert_eq!(format_coeff_display("a*c-a*c"), "0");
    }

    #[test]
    fn merges_duplicate_factors_in_product() {
        assert_eq!(format_coeff_display("(a)*(a)"), "aa");
        assert_eq!(format_coeff_display("2*(a)*(a)"), "2aa");
    }

    #[test]
    fn scalar_unchanged() {
        assert_eq!(format_coeff_display("1"), "1");
        assert_eq!(format_coeff_display("-1"), "-1");
        assert_eq!(format_coeff_display("a"), "a");
    }
}
