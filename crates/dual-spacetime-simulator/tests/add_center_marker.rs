use dual_spacetime_simulator::object_input::{ObjectInput, ObjectInputType};
use dual_spacetime_simulator::pipeline::build_add_center_cross;
use glam::DVec3;

#[test]
fn build_add_center_cross_has_three_axis_segments() {
    let verts = build_add_center_cross([1.0, 2.0, 3.0], 0.3);
    assert_eq!(verts.len(), 6);
}

#[test]
fn build_add_center_cross_spans_each_axis() {
    let half = 0.3;
    let verts = build_add_center_cross([0.0, 0.0, 0.0], half);
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
fn add_center_world_position_matches_offset_formula() {
    let center = DVec3::new(2.0, -1.0, 3.0);
    let base_scale = 1e10;
    let world = ObjectInput::add_center_world_position(center, base_scale);
    assert_eq!(world, center);
}

#[test]
fn add_center_marker_half_extent_is_thirty_percent_of_base_scale() {
    let base_scale = 1e10;
    let half = ObjectInput::add_center_marker_half_extent(base_scale);
    assert!((half - 0.3).abs() < 1e-6);
    let small_scale = 1.0;
    let half_small = ObjectInput::add_center_marker_half_extent(small_scale);
    assert!((half_small - 0.3).abs() < 1e-6);
}

#[test]
fn preview_group_extent_matches_random_sphere_radius() {
    let scale = 1e10;
    let input = ObjectInputType::RandomSphere.to_object_input(scale);
    let extent = input.preview_group_extent();
    assert!((extent - 1.0).abs() < 1e-6);
}