//! Symbolic expansion of biquaternion products using `dst-math` tables.
//!
//! Builds human-readable sums of basis monomials (for sandwich products and full component
//! expansions) without evaluating numeric coefficients.

pub mod algebra;
pub mod biquaternion;
pub mod coeff_format;
pub mod expr;
pub mod format;
pub mod pga;
pub mod pga_expr;

pub use format::{format_expanded, format_pga_expanded};

pub use biquaternion::{
    BasisMonomial, Coefficient, ExpandedProduct, combine_like_terms, expand_basis_monomial,
    expand_basis_product, expand_sandwich, mul_table_markdown,
};

pub use expr::{ParseError, expand_expr, parse_expr};

pub use pga::{
    BasisMonomial as PgaBasisMonomial, ExpandedProduct as PgaExpandedProduct,
    combine_like_terms as combine_pga_like_terms, expand_basis_monomial as expand_pga_basis_monomial,
    expand_basis_product as expand_pga_basis_product, expand_generator_monomial,
    expand_sandwich as expand_pga_sandwich, mul_table_markdown as pga_mul_table_markdown,
};

pub use pga_expr::{expand_pga_expr, parse_pga_expr};
