//! Integration tests for G(3,1,1) PGA (`dst-math::pga`).

use dst_math::pga::{
    BASIS_LABELS, Multivector, PGA, PGA_DIM, PGA_METRIC, Pga, basis_mul, basis_mul_with_metric,
};

const EPS: f64 = 1e-9;

fn mv(index: usize) -> Multivector {
    Multivector::basis(index)
}

fn approx_eq(a: Multivector, b: Multivector) -> bool {
    a.max_abs_diff(&b) < EPS
}

// --- Metric ---

#[test]
fn pga_metric_assignment() {
    assert_eq!(PGA_METRIC, [-1, 1, 1, 1, 0]);
    assert_eq!(Pga::metric_array(), PGA_METRIC);
}

#[test]
fn e0_squared_minus_one() {
    assert_eq!(basis_mul(1, 1), (-1, 0));
}

#[test]
fn e1_e2_e3_squared_plus_one() {
    assert_eq!(basis_mul(2, 2), (1, 0));
    assert_eq!(basis_mul(4, 4), (1, 0));
    assert_eq!(basis_mul(8, 8), (1, 0));
}

#[test]
fn e4_squared_zero() {
    assert_eq!(basis_mul(16, 16), (0, 0));
}

#[test]
fn metric_via_pga() {
    let (sign, idx) = Pga::basis_mul(1, 1);
    assert_eq!((sign, idx), (-1, 0));
    let (sign, idx) = Pga::basis_mul(16, 16);
    assert_eq!((sign, idx), (0, 0));
}

// --- Anticommutation ---

#[test]
fn e0_e1_anticommute() {
    let sum = mv(1) * mv(2) + mv(2) * mv(1);
    assert!(sum.is_zero(EPS));
}

#[test]
fn e1_e2_anticommute() {
    let sum = mv(2) * mv(4) + mv(4) * mv(2);
    assert!(sum.is_zero(EPS));
}

#[test]
fn e3_e4_anticommute() {
    let sum = mv(8) * mv(16) + mv(16) * mv(8);
    assert!(sum.is_zero(EPS));
}

#[test]
fn bivector_e0e1_squared_minus_one() {
    assert_eq!(basis_mul(3, 3), (1, 0));
}

// --- Null bivectors ---

#[test]
fn null_bivector_indices() {
    assert_eq!(Pga::null_bivector_index(0), 17);
    assert_eq!(Pga::null_bivector_index(1), 18);
    assert_eq!(Pga::null_bivector_index(2), 20);
    assert_eq!(Pga::null_bivector_index(3), 24);
}

#[test]
fn null_bivectors_square_to_zero() {
    for mu in 0..4 {
        let n = Pga::null_bivector_index(mu);
        assert_eq!(basis_mul(n, n), (0, 0), "N{mu}^2");
        let nv = mv(n);
        assert!(!nv.is_zero(EPS));
        assert!((nv * nv).is_zero(EPS), "N{mu}^2 as multivector");
    }
}

#[test]
fn null_bivectors_anticommute() {
    for mu in 0..4 {
        for nu in 0..4 {
            if mu == nu {
                continue;
            }
            let n_mu = mv(Pga::null_bivector_index(mu));
            let n_nu = mv(Pga::null_bivector_index(nu));
            assert!((n_mu * n_nu + n_nu * n_mu).is_zero(EPS), "N{mu} N{nu}");
        }
    }
}

#[test]
fn e4_anticommutes_with_spatial_vectors() {
    let e1 = mv(2);
    let e4 = mv(16);
    assert!(approx_eq(e4 * e1, -(e1 * e4)));
}

// --- Identity and associativity ---

#[test]
fn scalar_is_identity() {
    let one = Multivector::one();
    for i in 0..PGA_DIM {
        let e = mv(i);
        assert!(approx_eq(one * e, e));
        assert!(approx_eq(e * one, e));
    }
}

#[test]
fn multiplication_is_associative_on_sample_triples() {
    let indices = [0usize, 1, 2, 3, 16, 17, 31];
    for &i in &indices {
        for &j in &indices {
            for &k in &indices {
                let a = mv(i);
                let b = mv(j);
                let c = mv(k);
                let left = (a * b) * c;
                let right = a * (b * c);
                assert!(
                    left.max_abs_diff(&right) < 1e-7,
                    "assoc failed for {i},{j},{k}"
                );
            }
        }
    }
}

// --- Distributivity ---

#[test]
fn left_distributivity() {
    let a = mv(1) * 2.0 + mv(3);
    let b = mv(2) + mv(16);
    let c = mv(4) + mv(17) * 0.5;
    let left = a * (b + c);
    let right = a * b + a * c;
    assert!(approx_eq(left, right));
}

#[test]
fn right_distributivity() {
    let a = mv(1) + mv(8);
    let b = mv(2) * 3.0;
    let c = mv(16) + mv(17);
    let left = (a + c) * b;
    let right = a * b + c * b;
    assert!(approx_eq(left, right));
}

// --- Grade projection ---

#[test]
fn grade_projection_vectors() {
    let m = mv(1) * 2.0 + mv(3) + Multivector::one() * 5.0;
    let g1 = m.grade(1);
    assert!((g1.coeff(1) - 2.0).abs() < EPS);
    assert!(g1.coeff(3).abs() < EPS);
    assert!(g1.coeff(0).abs() < EPS);
}

#[test]
fn grade_projection_bivector() {
    let m = mv(17) + mv(2) + mv(31);
    let g2 = m.grade(2);
    assert!((g2.coeff(17) - 1.0).abs() < EPS);
    assert!(g2.coeff(2).abs() < EPS);
    assert!(g2.coeff(31).abs() < EPS);
}

#[test]
fn grade_counts_match_dimension() {
    let mut by_grade = [0usize; 6];
    for i in 0..PGA_DIM {
        by_grade[Pga::grade(i) as usize] += 1;
    }
    assert_eq!(by_grade, [1, 5, 10, 10, 5, 1]);
}

// --- Reverse and conjugate ---

#[test]
fn reverse_of_vector_is_itself() {
    for i in [1, 2, 4, 8, 16] {
        assert!(approx_eq(mv(i).reverse(), mv(i)));
    }
}

#[test]
fn reverse_of_bivector_e1e2_negates() {
    let b = mv(6);
    assert!(approx_eq(b.reverse(), -b));
}

#[test]
fn reverse_of_bivector_e0e1_negates() {
    let b = mv(3);
    assert!(approx_eq(b.reverse(), -b));
}

#[test]
fn reverse_anti_homomorphism() {
    let a = mv(3) + mv(5) * 0.5;
    let b = mv(17) + mv(2);
    assert!(approx_eq((a * b).reverse(), b.reverse() * a.reverse()));
}

#[test]
fn conjugate_flips_odd_grades() {
    let v = mv(1) + mv(6) + mv(17);
    let c = v.conjugate();
    assert!((c.coeff(1) + 1.0).abs() < EPS);
    assert!((c.coeff(6) + 1.0).abs() < EPS);
    assert!((c.coeff(17) + 1.0).abs() < EPS);
}

// --- Labels ---

#[test]
fn basis_labels_cover_all_indices() {
    assert_eq!(BASIS_LABELS.len(), PGA_DIM);
    assert_eq!(BASIS_LABELS[0], "1");
    assert_eq!(BASIS_LABELS[1], "e0");
    assert_eq!(BASIS_LABELS[16], "e4");
    assert_eq!(BASIS_LABELS[31], "e0e1e2e3e4");
}

// --- Pseudoscalar ---

#[test]
fn pseudoscalar_squares_to_zero_due_to_null_generator() {
    let (sign, idx) = basis_mul(31, 31);
    assert_eq!((sign, idx), (0, 0));
    assert!((mv(31) * mv(31)).is_zero(EPS));
}

// --- Full table consistency ---

#[test]
fn mul_table_matches_dynamic_algorithm() {
    for i in 0..PGA_DIM {
        for j in 0..PGA_DIM {
            let from_table = basis_mul(i, j);
            let dynamic = basis_mul_with_metric(i, j, &PGA_METRIC);
            assert_eq!(from_table, dynamic, "mismatch at {i},{j}");
        }
    }
}

#[test]
fn pga_table_matches_basis_mul() {
    for i in 0..PGA_DIM {
        for j in 0..PGA_DIM {
            assert_eq!(Pga::basis_mul(i, j), basis_mul(i, j));
        }
    }
}

#[test]
fn pga_singleton() {
    assert_eq!(PGA, Pga);
}

// --- Scalar multiplication ---

#[test]
fn scalar_multiplication() {
    let v = mv(1) * 2.5;
    assert!((v.coeff(1) - 2.5).abs() < EPS);
    assert!(approx_eq(v * mv(2), mv(1) * mv(2) * 2.5));
}

// --- Addition ---

#[test]
fn addition_subtraction() {
    let a = mv(1) + mv(16);
    let b = mv(1) - mv(16);
    assert!(approx_eq(a + b, mv(1) * 2.0));
    assert!(approx_eq(a - b, mv(16) * 2.0));
}
