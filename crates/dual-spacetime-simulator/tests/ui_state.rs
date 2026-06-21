use dual_spacetime_simulator::object_input::ObjectInputType;
use dual_spacetime_simulator::settings::AppSettings;
use dual_spacetime_simulator::ui_state::{AppMode, PlacementMode, SimulationType, UiState};

#[test]
fn apply_settings_clamps_add_particle_count() {
    let mut ui = UiState::default();
    ui.add_particle_count = 10_000;
    let mut s = AppSettings::default();
    s.max_particle_count = 100;
    ui.apply_settings(&s);
    assert_eq!(ui.max_particle_count, 100);
    assert_eq!(ui.add_particle_count, 100);
}

#[test]
fn add_particle_count_range_matches_remaining_capacity() {
    assert_eq!(UiState::add_particle_count_range(0), None);
    assert_eq!(UiState::add_particle_count_range(1), Some(1..=1));
    assert_eq!(UiState::add_particle_count_range(2), Some(2..=2));
    assert_eq!(UiState::add_particle_count_range(500), Some(2..=500));
}

#[test]
fn clamp_add_particle_count_to_capacity_limits_batch_size() {
    let mut ui = UiState::default();
    ui.max_particle_count = 100;
    ui.add_particle_count = 80;
    ui.clamp_add_particle_count_to_capacity(95);
    assert_eq!(ui.add_particle_count, 5);

    ui.add_particle_count = 80;
    ui.clamp_add_particle_count_to_capacity(100);
    assert_eq!(ui.add_particle_count, 80);
}

#[test]
fn simulation_type_change_disables_add_until_reset() {
    let mut ui = UiState::default();
    assert!(ui.is_add_particles_enabled);

    ui.simulation_type = SimulationType::SpeedOfLightLimit;
    ui.apply_simulation_type_change(SimulationType::Normal);
    assert!(!ui.is_add_particles_enabled);

    ui.request_reset();
    assert!(ui.is_add_particles_enabled);

    ui.simulation_type = SimulationType::LorentzTransformation;
    ui.apply_simulation_type_change(SimulationType::SpeedOfLightLimit);
    assert!(!ui.is_add_particles_enabled);
}

#[test]
fn simulation_type_unchanged_keeps_add_enabled() {
    let mut ui = UiState::default();
    ui.apply_simulation_type_change(SimulationType::Normal);
    assert!(ui.is_add_particles_enabled);
}

#[test]
fn reset_timing_defaults_follow_placement_and_add_type() {
    let mut ui = UiState::default();
    ui.placement_mode = PlacementMode::SolarSystem;
    ui.apply_reset_timing_defaults();
    assert_eq!(ui.time_per_frame, 10_000.0);
    assert_eq!(ui.max_fps, 1000);
    assert_eq!(ui.skip, 10);

    ui.placement_mode = PlacementMode::Manual;
    ui.object_input_type = ObjectInputType::EllipticalOrbit;
    ui.apply_reset_timing_defaults();
    assert_eq!(ui.time_per_frame, 100_000.0);
    assert_eq!(ui.max_fps, 1000);
    assert_eq!(ui.skip, 0);

    ui.object_input_type = ObjectInputType::RandomSphere;
    ui.apply_reset_timing_defaults();
    assert_eq!(ui.time_per_frame, 10.0);
    assert_eq!(ui.max_fps, 60);
    assert_eq!(ui.skip, 0);
}

#[test]
fn placement_mode_change_disables_add_until_reset() {
    let mut ui = UiState::default();
    assert!(ui.is_add_particles_enabled);

    ui.placement_mode = PlacementMode::SolarSystem;
    ui.apply_placement_mode_change(PlacementMode::Manual);
    assert!(!ui.is_add_particles_enabled);

    ui.request_reset();
    assert!(ui.is_add_particles_enabled);

    ui.placement_mode = PlacementMode::SatelliteOrbit;
    ui.apply_placement_mode_change(PlacementMode::SolarSystem);
    assert!(!ui.is_add_particles_enabled);
}

#[test]
fn app_mode_change_resets_panels() {
    let mut ui = UiState::default();
    ui.is_simulation_panel_open = true;
    ui.is_graph3d_panel_open = false;
    ui.apply_panel_defaults_on_app_mode_change(AppMode::Simulation, AppMode::Graph3D);
    assert!(!ui.is_simulation_panel_open);
    assert!(ui.is_graph3d_panel_open);
}
