use dual_spacetime_simulator::tree::{HermiteSpline, Tree, TreeParams, AXIS_XZ_GRID_LINE_COUNT};
use glam::Vec3;

#[test]
fn hermite_endpoints() {
    let s = HermiteSpline::new(
        Vec3::new(1.0, 2.0, 3.0),
        Vec3::new(4.0, 5.0, 6.0),
        Vec3::Y,
        Vec3::Z,
    );
    assert!((s.eval(0.0) - s.p0).length() < 1e-5);
    assert!((s.eval(1.0) - s.p1).length() < 1e-5);
}

#[test]
fn hermite_tangent_unit_length() {
    let s = HermiteSpline::new(Vec3::ZERO, Vec3::Y, Vec3::X * 0.5, Vec3::NEG_Y * 0.3);
    for t in [0.0f32, 0.25, 0.5, 0.75, 1.0] {
        let len = s.eval_tangent(t).length();
        assert!((len - 1.0).abs() < 1e-4, "t={t} len={len}");
    }
}

#[test]
fn tree_generate_is_deterministic_for_seed() {
    // Avoid `depth >= 3` leaf decoration that uses the global `rand::random` (not `SmallRng`).
    let p = TreeParams {
        seed: 12345,
        max_depth: 2,
        ..Default::default()
    };
    let a = Tree::generate(p);
    let b = Tree::generate(p);
    let va = a.generate_vertices_at(Vec3::ZERO);
    let vb = b.generate_vertices_at(Vec3::ZERO);
    assert_eq!(va.len(), vb.len());
    for (x, y) in va.iter().zip(vb.iter()) {
        assert_eq!(x.0, y.0);
        assert_eq!(x.1, y.1);
    }
}

#[test]
fn single_tree_produces_vertices() {
    let tree = Tree::generate(TreeParams::default());
    let v = tree.generate_vertices_at(Vec3::ZERO);
    assert!(!v.is_empty());
}

#[test]
fn forest_tube_grid_scales_with_grid_cells() {
    let params = TreeParams {
        seed: 7,
        max_depth: 2,
        branch_factor: 2,
        ..Default::default()
    };
    let single = Tree::generate(params).generate_tube_vertices_at(Vec3::ZERO);
    let forest = Tree::generate_forest_tube_vertices_on_axis_xz_grid(params);
    let n_cells = AXIS_XZ_GRID_LINE_COUNT * AXIS_XZ_GRID_LINE_COUNT;
    assert!(forest.len() > single.len() * n_cells / 2);
}
