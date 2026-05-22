//! Symbolic expansion of tetraquaternion / biquaternion products using `dst-math` tables.
//!
//! Builds human-readable sums of basis monomials (for sandwich products and full component
//! expansions) without evaluating numeric coefficients.

pub mod algebra;
pub mod biquaternion;
pub mod coeff_format;
pub mod expr;
pub mod format;

pub use format::format_expanded;

pub use biquaternion::{
    BasisMonomial, Coefficient, ExpandedProduct, combine_like_terms, expand_basis_monomial,
    expand_basis_product, expand_sandwich, mul_table_markdown,
};

pub use expr::{ParseError, expand_expr, parse_expr};
