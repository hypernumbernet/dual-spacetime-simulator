use dual_spacetime_simulator::object_input::{
    clamp_world_scale, ObjectInputType, SATELLITE_ORBIT_SCALE, SOLAR_SYSTEM_SCALE,
};
use dual_spacetime_simulator::ui_state::{BaseScaleUnit, UiState};

#[test]
fn default_base_scale_unit_is_km() {
    let ui = UiState::default();
    assert_eq!(ui.base_scale_unit, BaseScaleUnit::Km);
}

#[test]
fn base_scale_unit_roundtrip_all_units() {
    let meters = 1.5e22;
    for unit in BaseScaleUnit::ALL {
        let display = unit.from_meters(meters);
        let back = unit.to_meters(display);
        assert!((back - meters).abs() < 1e-3, "{unit}");
    }
}

#[test]
fn min_display_value_is_positive_for_all_units() {
    for unit in BaseScaleUnit::ALL {
        assert!(unit.min_display_value() > 0.0, "{unit}");
    }
}

#[test]
fn nm_min_display_value_is_one_hundredth() {
    assert_eq!(BaseScaleUnit::Nm.min_display_value(), 0.01);
}

#[test]
fn fm_min_display_value_is_one_hundredth() {
    assert_eq!(BaseScaleUnit::Fm.min_display_value(), 0.01);
}

#[test]
fn sub_nanometer_units_ordered_largest_first() {
    assert!(BaseScaleUnit::Nm.meters_per_unit() > BaseScaleUnit::Fm.meters_per_unit());
}

#[test]
fn astronomical_units_ordered_largest_first() {
    assert!(BaseScaleUnit::Mpc.meters_per_unit() > BaseScaleUnit::Pc.meters_per_unit());
    assert!(BaseScaleUnit::Pc.meters_per_unit() > BaseScaleUnit::Ly.meters_per_unit());
    assert!(BaseScaleUnit::Ly.meters_per_unit() > BaseScaleUnit::Au.meters_per_unit());
}

#[test]
fn apply_base_scale_edit_resets_to_one_on_unit_change() {
    let mut ui = UiState::default();
    ui.base_scale = 1e10;
    ui.apply_base_scale_edit(999.0, true);
    assert_eq!(ui.base_scale, clamp_world_scale(BaseScaleUnit::Km.to_meters(1.0)));
}

#[test]
fn build_object_input_uses_base_scale() {
    let mut ui = UiState::default();
    ui.base_scale = 42.0;
    let input = ui.build_object_input();
    assert_eq!(input.get_scale(), 42.0);
}

#[test]
fn object_input_type_change_preserves_base_scale_for_scaled_types() {
    let mut ui = UiState::default();
    ui.base_scale = 42.0;
    ui.object_input_type = ObjectInputType::SpiralDisk;
    ui.apply_object_input_type_change(ObjectInputType::RandomSphere);
    assert_eq!(ui.base_scale, 42.0);
    assert_eq!(ui.build_object_input().get_scale(), 42.0);
    ui.object_input_type = ObjectInputType::EllipticalOrbit;
    ui.apply_object_input_type_change(ObjectInputType::SpiralDisk);
    assert_eq!(ui.base_scale, 42.0);
    assert_eq!(ui.build_object_input().get_scale(), 42.0);
}

#[test]
fn object_input_type_change_sets_base_scale_for_solar_system_and_satellite_orbit() {
    let mut ui = UiState::default();
    ui.base_scale = 42.0;
    ui.object_input_type = ObjectInputType::SolarSystem;
    ui.apply_object_input_type_change(ObjectInputType::RandomSphere);
    assert_eq!(ui.base_scale, SOLAR_SYSTEM_SCALE);
    assert!(!ui.is_add_particles_enabled);

    ui.is_add_particles_enabled = true;
    ui.object_input_type = ObjectInputType::SatelliteOrbit;
    ui.apply_object_input_type_change(ObjectInputType::SolarSystem);
    assert_eq!(ui.base_scale, SATELLITE_ORBIT_SCALE);
    assert!(!ui.is_add_particles_enabled);
}

#[test]
fn object_input_type_change_syncs_scaled_parameters() {
    let mut ui = UiState::default();
    ui.base_scale = 42.0;
    ui.object_input_type = ObjectInputType::RandomSphere;
    ui.sync_scaled_object_input_parameters();
    assert!((ui.random_sphere.radius - 42.0).abs() < 1e-6);

    ui.object_input_type = ObjectInputType::RandomCube;
    ui.sync_scaled_object_input_parameters();
    assert!((ui.random_cube.cube_size - 84.0).abs() < 1e-6);

    ui.object_input_type = ObjectInputType::SpiralDisk;
    ui.sync_scaled_object_input_parameters();
    assert!((ui.spiral_disk.disk_radius - 63.0).abs() < 1e-6);
}

#[test]
fn base_scale_edit_syncs_scaled_parameters_for_active_type() {
    let mut ui = UiState::default();
    ui.object_input_type = ObjectInputType::RandomSphere;
    ui.base_scale = 1e10;
    ui.sync_scaled_object_input_parameters();
    let initial_radius = ui.random_sphere.radius;

    ui.base_scale = 1e9;
    ui.sync_scaled_object_input_parameters();
    assert!((ui.random_sphere.radius - initial_radius * 0.1).abs() < 1e-3);
    let expected_mass_min = 1e29 * 0.001;
    let mass_error = (ui.random_sphere.mass_range.0 - expected_mass_min).abs();
    assert!(mass_error / expected_mass_min < 1e-9, "mass_error={mass_error}");
}

#[test]
fn object_input_type_change_syncs_elliptical_orbit_parameters() {
    let mut ui = UiState::default();
    ui.base_scale = 42.0;
    ui.object_input_type = ObjectInputType::EllipticalOrbit;
    ui.apply_object_input_type_change(ObjectInputType::RandomSphere);
    let reference = ObjectInputType::EllipticalOrbit.default_base_scale();
    let factor = 42.0 / reference;
    let factor_cubed = factor * factor * factor;
    assert!((ui.elliptical_orbit.planetary_distance - 2.0e11 * factor).abs() < 1e-6);
    assert!((ui.elliptical_orbit.planetary_speed - 2.0e5 * factor).abs() < 1e-6);
    assert!((ui.elliptical_orbit.central_mass - 1.989e32 * factor_cubed).abs() < 1e-6);
    assert!((ui.elliptical_orbit.planetary_mass - 5.972e24 * factor_cubed).abs() < 1e-6);
}

#[test]
fn base_scale_edit_disables_add_until_reset() {
    let mut ui = UiState::default();
    assert!(ui.is_add_particles_enabled);
    ui.apply_base_scale_edit(2.0, false);
    assert!(!ui.is_add_particles_enabled);
    ui.is_add_particles_enabled = true;
    ui.apply_base_scale_edit(2.0, false);
    assert!(ui.is_add_particles_enabled);
}

#[test]
fn maybe_sync_skips_non_scaled_object_input_types() {
    let mut ui = UiState::default();
    ui.object_input_type = ObjectInputType::SolarSystem;
    let before = ui.random_sphere.radius;
    ui.base_scale = 99.0;
    ui.maybe_sync_scaled_object_input_parameters();
    assert_eq!(ui.random_sphere.radius, before);
}

#[test]
fn mpc_display_avoids_round_trip_artifacts() {
    let unit = BaseScaleUnit::Mpc;
    let meters = unit.to_meters(1.0);
    let display = unit.sanitize_display(unit.from_meters(meters));
    assert_eq!(display, 1.0);
    assert_eq!(unit.format_display(display), "1");
}

#[test]
fn pc_drag_values_stay_clean() {
    let unit = BaseScaleUnit::Pc;
    for steps in [1.0, 1.01, 1.1, 2.0] {
        let display = unit.sanitize_display(steps);
        let roundtrip = unit.sanitize_display(unit.from_meters(unit.to_meters(display)));
        assert_eq!(roundtrip, display, "steps={steps}");
        assert!(!unit.format_display(display).contains("0000000000"));
    }
}