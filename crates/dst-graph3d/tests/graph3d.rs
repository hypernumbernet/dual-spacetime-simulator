use dst_graph3d::graph3d::{
    GraphType, build_graph_line_vertices, build_points, graph_params_fingerprint,
};

#[test]
fn build_points_respects_sample_clamp() {
    let n = 100;
    let (pos, col) = build_points(GraphType::SphericalFibonacciLattice, n, 1.0, 1.0);
    assert_eq!(pos.len(), n as usize);
    assert_eq!(col.len(), n as usize);
}

#[test]
fn fingerprint_stable_for_same_inputs() {
    let a = graph_params_fingerprint(GraphType::RapidityFieldBiquaternion, 200, 0.1, 2.0);
    let b = graph_params_fingerprint(GraphType::RapidityFieldBiquaternion, 200, 0.1, 2.0);
    assert_eq!(a, b);
}

#[test]
fn fingerprint_changes_with_t_slice_bits() {
    let a = graph_params_fingerprint(GraphType::SphericalFibonacciLattice, 50, 1.0, 1.0);
    let b = graph_params_fingerprint(
        GraphType::SphericalFibonacciLattice,
        50,
        1.0 + f64::EPSILON,
        1.0,
    );
    assert_ne!(a, b);
}

#[test]
fn light_cone_lines_have_two_vertices_per_sample() {
    let n = 32;
    let lines = build_graph_line_vertices(GraphType::SphericalFibonacciLattice, n, 2.0, 0.0);
    assert_eq!(lines.len(), n as usize * 2);
}

#[test]
fn rapidity_field_lines_have_expected_vertex_count() {
    let lines = build_graph_line_vertices(GraphType::RapidityFieldMatrix, 100, 1.0, 1.0);
    let grid_size = 6;
    let axis_count = (grid_size + 1) as usize;
    let expected_vertices =
        axis_count * axis_count * (axis_count - 1) * 2 + axis_count * (axis_count - 1) * 2 * 2;
    assert_eq!(lines.len(), expected_vertices);
    assert!(!lines.is_empty());
}

#[test]
fn rapidity_field_lines_increase_with_sample_count() {
    let low = build_graph_line_vertices(GraphType::RapidityFieldMatrix, 1, 1.0, 1.0);
    let high = build_graph_line_vertices(GraphType::RapidityFieldMatrix, 5000, 1.0, 1.0);
    assert!(high.len() > low.len());
}

#[test]
fn rapidity_field_biquaternion_lines_have_expected_vertex_count() {
    let lines = build_graph_line_vertices(GraphType::RapidityFieldBiquaternion, 100, 1.0, 1.0);
    let grid_size = 6;
    let axis_count = (grid_size + 1) as usize;
    let expected_vertices =
        axis_count * axis_count * (axis_count - 1) * 2 + axis_count * (axis_count - 1) * 2 * 2;
    assert_eq!(lines.len(), expected_vertices);
    assert!(!lines.is_empty());
}

#[test]
fn rapidity_field_biquaternion_matches_matrix_lines() {
    let matrix = build_graph_line_vertices(GraphType::RapidityFieldMatrix, 100, 1.0, 1.0);
    let biquaternion =
        build_graph_line_vertices(GraphType::RapidityFieldBiquaternion, 100, 1.0, 1.0);
    assert_eq!(matrix.len(), biquaternion.len());

    for (matrix_v, biquat_v) in matrix.iter().zip(biquaternion.iter()) {
        let (matrix_pos, matrix_col) = matrix_v;
        let (biquat_pos, biquat_col) = biquat_v;
        for i in 0..3 {
            assert!(
                (matrix_pos[i] - biquat_pos[i]).abs() <= 1e-4,
                "vertex position differs at index {i}: matrix={}, biquaternion={}",
                matrix_pos[i],
                biquat_pos[i]
            );
        }
        for i in 0..4 {
            assert!(
                (matrix_col[i] - biquat_col[i]).abs() <= f32::EPSILON,
                "vertex color differs at index {i}: matrix={}, biquaternion={}",
                matrix_col[i],
                biquat_col[i]
            );
        }
    }
}
