//! Symbolic expansion of tetraquaternion / biquaternion products using `dst-math` tables.
//!
//! Builds human-readable sums of basis monomials (for sandwich products and full component
//! expansions) without evaluating numeric coefficients.

pub mod biquaternion;
pub mod format;

pub use format::format_expanded;

pub use biquaternion::{
    BasisMonomial, Coefficient, ExpandedProduct, expand_basis_monomial, expand_basis_product,
    expand_sandwich, mul_table_markdown,
};
