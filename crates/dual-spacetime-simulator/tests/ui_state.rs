use dual_spacetime_simulator::object_input::ObjectInputType;
use dual_spacetime_simulator::settings::AppSettings;
use dual_spacetime_simulator::ui_state::{
    AppMode, ComputingUnit, ParticleDisplayMode, PlacementMode, SimulationType, UiState,
};

#[test]
fn particle_display_mode_size_scale_factor() {
    assert_eq!(ParticleDisplayMode::Glow.size_scale_factor(), 1.0);
    assert!((ParticleDisplayMode::Sphere.size_scale_factor() - 0.7).abs() < f32::EPSILON);
}

#[test]
fn apply_settings_propagates_particle_display_mode() {
    let mut ui = UiState::default();
    let mut s = AppSettings::default();
    s.particle_display_mode = ParticleDisplayMode::Sphere;
    ui.apply_settings(&s);
    assert_eq!(ui.particle_display_mode, ParticleDisplayMode::Sphere);
}

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
fn computing_unit_change_held_until_reset() {
    let mut ui = UiState::default();
    assert!(ui.is_add_particles_enabled);
    assert!(!ui.uses_gpu_simulation());

    ui.computing_unit = ComputingUnit::Gpu;
    ui.apply_computing_unit_change(ComputingUnit::Cpu);
    assert_eq!(ui.computing_unit, ComputingUnit::Gpu);
    assert_eq!(ui.active_computing_unit, ComputingUnit::Cpu);
    assert!(!ui.uses_gpu_simulation());
    assert!(!ui.is_add_particles_enabled);

    ui.request_reset();
    assert_eq!(ui.active_computing_unit, ComputingUnit::Gpu);
    assert!(ui.uses_gpu_simulation());
    assert!(ui.is_add_particles_enabled);
}

#[test]
fn simulation_type_change_keeps_gpu_computing_unit() {
    let mut ui = UiState::default();
    ui.computing_unit = ComputingUnit::Gpu;
    ui.active_computing_unit = ComputingUnit::Gpu;

    ui.simulation_type = SimulationType::SpeedOfLightLimit;
    ui.apply_simulation_type_change(SimulationType::Normal);
    assert_eq!(ui.computing_unit, ComputingUnit::Gpu);
    assert_eq!(ui.active_computing_unit, ComputingUnit::Gpu);
}

#[test]
fn gpu_computing_available_for_all_types() {
    let mut ui = UiState::default();
    ui.simulation_type = SimulationType::Normal;
    ui.computing_unit = ComputingUnit::Gpu;
    ui.active_computing_unit = ComputingUnit::Gpu;
    assert!(ui.gpu_computing_available());
    assert!(ui.uses_gpu_simulation());

    ui.active_computing_unit = ComputingUnit::Cpu;
    assert!(!ui.uses_gpu_simulation());

    // Relativistic types now also support GPU compute.
    ui.simulation_type = SimulationType::SpeedOfLightLimit;
    ui.active_computing_unit = ComputingUnit::Gpu;
    assert!(ui.gpu_computing_available());
    assert!(ui.uses_gpu_simulation());

    ui.simulation_type = SimulationType::LorentzTransformation;
    assert!(ui.gpu_computing_available());
    assert!(ui.uses_gpu_simulation());
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

#[test]
fn can_start_simulation_requires_at_least_two_particles() {
    assert!(!UiState::can_start_simulation(0));
    assert!(!UiState::can_start_simulation(1));
    assert!(UiState::can_start_simulation(2));
}

#[test]
fn request_reset_stops_running_simulation() {
    let mut ui = UiState::default();
    ui.is_running = true;
    ui.request_reset();
    assert!(!ui.is_running);
    assert!(ui.is_reset_requested);
}
