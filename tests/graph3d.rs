use dual_spacetime_simulator::graph3d::{
    build_graph_line_vertices, build_points, graph_params_fingerprint,
};
use dual_spacetime_simulator::ui_state::GraphType;

#[test]
fn build_points_respects_sample_clamp() {
    let n = 100;
    let (pos, col) = build_points(
        GraphType::SphericalFibonacciLattice,
        n,
        1.0,
        1.0,
        1.0,
    );
    assert_eq!(pos.len(), n as usize);
    assert_eq!(col.len(), n as usize);
}

#[test]
fn fingerprint_stable_for_same_inputs() {
    let a = graph_params_fingerprint(GraphType::BoostExponent, 200, 0.1, 2.0, 0.5);
    let b = graph_params_fingerprint(GraphType::BoostExponent, 200, 0.1, 2.0, 0.5);
    assert_eq!(a, b);
}

#[test]
fn fingerprint_changes_with_phi_bits() {
    let a = graph_params_fingerprint(GraphType::BivectorVisualization, 50, 0.0, 1.0, 1.0);
    let b = graph_params_fingerprint(GraphType::BivectorVisualization, 50, 0.0, 1.0, 1.0 + f64::EPSILON);
    assert_ne!(a, b);
}

#[test]
fn light_cone_lines_have_two_vertices_per_sample() {
    let n = 32;
    let lines = build_graph_line_vertices(GraphType::SphericalFibonacciLattice, n, 2.0, 0.0, 0.0);
    assert_eq!(lines.len(), n as usize * 2);
}

#[test]
fn non_light_cone_lines_empty() {
    let lines = build_graph_line_vertices(GraphType::RapidityField, 100, 1.0, 1.0, 1.0);
    assert!(lines.is_empty());
}
