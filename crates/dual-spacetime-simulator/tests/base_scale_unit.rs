use dual_spacetime_simulator::object_input::clamp_world_scale;
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