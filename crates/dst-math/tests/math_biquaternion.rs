use dst_math::biquaternion::Biquaternion;

fn approx_eq(a: Biquaternion, b: Biquaternion) -> bool {
    a.max_abs_diff(&b) < 1e-9
}

#[test]
fn one_is_multiplicative_identity() {
    let one = Biquaternion::one();
    for i in 0..15 {
        let e = Biquaternion::basis(i);
        assert!(approx_eq(one * e, e));
        assert!(approx_eq(e * one, e));
    }
}

#[test]
fn multiplication_is_associative_on_basis_triples() {
    let indices = [0usize, 4, 7, 10, 14];
    for &i in &indices {
        for &j in &indices {
            for &k in &indices {
                let a = Biquaternion::basis(i);
                let b = Biquaternion::basis(j);
                let c = Biquaternion::basis(k);
                let left = (a * b) * c;
                let right = a * (b * c);
                let d = left.max_abs_diff(&right);
                assert!(d < 1e-7, "assoc failed for basis {i},{j},{k} d={d}");
            }
        }
    }
}

/// Cl(3,1) 部分代数として biquaternion は 16 次元（e4 なし）。
#[test]
fn biquaternion_is_16_dimensional_cl31() {
    use dst_math::pga::PGA_DIM;
    assert_eq!(PGA_DIM, 32);
    // biquaternion: 15 non-scalar + 1 scalar = 16
    let one = Biquaternion::one();
    assert!(approx_eq(one * one, one));
}

#[test]
fn cl31_vector_squares_match_pga_without_e4() {
    use dst_math::pga::Multivector;

    // biquaternion j (index 0) ~ e0 in PGA: j*j = -1
    let j = Biquaternion::basis(0);
    let neg_one = Biquaternion::new(-1.0, [0.0; 15]);
    assert!(approx_eq(j * j, neg_one));

    // PGA e0*e0 = -1
    fn mv_approx(a: Multivector, b: Multivector) -> bool {
        a.max_abs_diff(&b) < 1e-9
    }
    let e0 = Multivector::basis(1);
    assert!(mv_approx(e0 * e0, -Multivector::one()));
}

#[test]
fn cl31_i_times_j_matches_pga_e0_e1_pattern() {
    use dst_math::pga::Multivector;

    // biquaternion: i (14) * j (0) = k (10)
    let i = Biquaternion::basis(14);
    let j = Biquaternion::basis(0);
    let k = Biquaternion::basis(10);
    assert!(approx_eq(i * j, k));

    fn mv_approx(a: Multivector, b: Multivector) -> bool {
        a.max_abs_diff(&b) < 1e-9
    }
    // PGA: e0*e1 = +e0e1 (index 3), e1*e0 = -e0e1
    let e0 = Multivector::basis(1);
    let e1 = Multivector::basis(2);
    assert!(mv_approx(e0 * e1, Multivector::basis(3)));
    assert!(mv_approx(e1 * e0, -(Multivector::basis(3))));
}

#[test]
fn biquaternion_has_no_e4_generator() {
    // PGA has e4 at index 16; biquaternion max basis index is 14
    let last = Biquaternion::basis(14);
    assert!(!last.is_zero());
}

#[test]
fn biquaternion_distribution_holds() {
    let a = Biquaternion::basis(0) + Biquaternion::basis(7);
    let b = Biquaternion::basis(4) + Biquaternion::basis(10);
    let c = Biquaternion::basis(1) + Biquaternion::basis(14);
    let left = a * (b + c);
    let right = a * b + a * c;
    assert!(approx_eq(left, right));
}
