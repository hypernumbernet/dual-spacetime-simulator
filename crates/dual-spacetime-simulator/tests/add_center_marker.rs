use dual_spacetime_simulator::object_input::{ObjectInput, ObjectInputType};
use dual_spacetime_simulator::pipeline::build_add_center_marker;
use glam::DVec3;

const WHITE: [f32; 4] = [1.0, 1.0, 1.0, 1.0];

fn edge_colors(verts: &[([f32; 3], [f32; 4])]) -> Vec<[f32; 4]> {
    assert!(verts.len().is_multiple_of(2));
    verts
        .chunks_exact(2)
        .map(|edge| {
            assert_eq!(edge[0].1, edge[1].1);
            edge[0].1
        })
        .collect()
}

fn edge_is_diagonal(edge: [([f32; 3], [f32; 4]); 2]) -> bool {
    let delta = [
        (edge[1].0[0] - edge[0].0[0]).abs(),
        (edge[1].0[1] - edge[0].0[1]).abs(),
        (edge[1].0[2] - edge[0].0[2]).abs(),
    ];
    delta.iter().filter(|&&d| d > 1e-6).count() >= 2
}

#[test]
fn build_add_center_marker_has_octahedron_segments() {
    let verts = build_add_center_marker([1.0, 2.0, 3.0], 0.15);
    assert_eq!(verts.len(), 24);

    let colors = edge_colors(&verts);
    assert_eq!(colors.len(), 12);
    assert!(colors.iter().all(|&c| c == WHITE));
}

#[test]
fn build_add_center_marker_spans_each_axis() {
    let half = 0.15;
    let verts = build_add_center_marker([0.0, 0.0, 0.0], half);
    let xs: Vec<f32> = verts.iter().map(|(p, _)| p[0]).collect();
    let ys: Vec<f32> = verts.iter().map(|(p, _)| p[1]).collect();
    let zs: Vec<f32> = verts.iter().map(|(p, _)| p[2]).collect();
    assert!(xs.contains(&-half));
    assert!(xs.contains(&half));
    assert!(ys.contains(&-half));
    assert!(ys.contains(&half));
    assert!(zs.contains(&-half));
    assert!(zs.contains(&half));
}

#[test]
fn build_add_center_marker_all_edges_are_white() {
    let verts = build_add_center_marker([0.0, 0.0, 0.0], 0.15);
    for edge in verts.chunks_exact(2) {
        let pair: [([f32; 3], [f32; 4]); 2] = [edge[0], edge[1]];
        assert!(edge_is_diagonal(pair));
        assert_eq!(pair[0].1, WHITE);
        assert_eq!(pair[1].1, WHITE);
    }
}

#[test]
fn add_center_world_position_inverts_y_slider_sign() {
    let center = DVec3::new(2.0, -1.0, 3.0);
    let base_scale = 1e10;
    let world = ObjectInput::add_center_world_position(center, base_scale);
    assert_eq!(world, DVec3::new(2.0, 1.0, 3.0));
}

#[test]
fn add_center_effective_leaves_x_and_z_unchanged() {
    let center = DVec3::new(1.5, 2.0, -3.0);
    assert_eq!(
        ObjectInput::add_center_effective(center),
        DVec3::new(1.5, -2.0, -3.0)
    );
}

#[test]
fn add_center_marker_geometry_matches_world_position_and_half_extent() {
    let center = DVec3::new(2.0, -1.0, 3.0);
    let base_scale = 1e10;
    let (marker_center, half_extent) = ObjectInput::add_center_marker_geometry(center, base_scale);
    let world = ObjectInput::add_center_world_position(center, base_scale);
    assert_eq!(
        marker_center,
        [world.x as f32, world.y as f32, world.z as f32]
    );
    assert!((half_extent - ObjectInput::add_center_marker_half_extent(base_scale)).abs() < 1e-6);
}

#[test]
fn add_center_marker_half_extent_is_fifteen_percent_of_base_scale() {
    let base_scale = 1e10;
    let half = ObjectInput::add_center_marker_half_extent(base_scale);
    assert!((half - 0.15).abs() < 1e-6);
    let small_scale = 1.0;
    let half_small = ObjectInput::add_center_marker_half_extent(small_scale);
    assert!((half_small - 0.15).abs() < 1e-6);
}

#[test]
fn preview_group_extent_matches_random_sphere_radius() {
    let scale = 1e10;
    let input = ObjectInputType::RandomSphere.to_object_input(scale);
    let extent = input.preview_group_extent();
    assert!((extent - 1.0).abs() < 1e-6);
}
