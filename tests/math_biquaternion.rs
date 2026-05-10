use dual_spacetime_simulator::math::biquaternion::Biquaternion;

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
