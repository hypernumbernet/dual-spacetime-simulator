use dual_spacetime_simulator::object_input::ObjectInputType;
use dual_spacetime_simulator::settings::AppSettings;
use dual_spacetime_simulator::ui_state::{
    ComputingUnit, DEFAULT_ADD_PARTICLE_COUNT, DEFAULT_MAX_FPS, DEFAULT_SATELLITE_COUNT,
    DEFAULT_SCALE_UI, DEFAULT_SKIP_DRAWING_FRAMES, ParticleDisplayMode, PlacementMode,
    SimulationType, UiState,
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
    assert_eq!(ui.satellite_orbit.satellite_count, 99);
}

#[test]
fn clamp_satellite_count_respects_max_particle_count() {
    let mut ui = UiState::default();
    ui.max_particle_count = 50;
    ui.satellite_orbit.satellite_count = 999;
    ui.clamp_satellite_count();
    assert_eq!(ui.satellite_orbit.satellite_count, 49);
}

#[test]
fn add_particle_count_range_matches_remaining_capacity() {
    assert_eq!(UiState::add_particle_count_range(0), None);
    assert_eq!(UiState::add_particle_count_range(1), Some(1..=1));
    assert_eq!(UiState::add_particle_count_range(2), Some(1..=2));
    assert_eq!(UiState::add_particle_count_range(500), Some(1..=500));
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
fn request_reset_stops_running_simulation() {
    let mut ui = UiState::default();
    ui.is_running = true;
    ui.add_center = glam::DVec3::new(1.0, 2.0, 3.0);
    ui.request_reset();
    assert!(!ui.is_running);
    assert!(ui.is_reset_requested);
    assert_eq!(ui.add_center, glam::DVec3::ZERO);
}

#[test]
fn reset_repopulates_particles_by_placement_mode() {
    let mut ui = UiState::default();
    assert!(!ui.reset_repopulates_particles());

    ui.placement_mode = PlacementMode::SolarSystem;
    assert!(ui.reset_repopulates_particles());

    ui.placement_mode = PlacementMode::SatelliteOrbit;
    assert!(ui.reset_repopulates_particles());
}

#[test]
fn preset_placement_reset_repopulates_particles() {
    use dual_spacetime_simulator::object_input::ObjectInput;
    use dual_spacetime_simulator::simulation::SimulationManager;

    let mut ui = UiState::default();
    ui.placement_mode = PlacementMode::SolarSystem;
    assert!(matches!(
        ui.build_reset_object_input(),
        ObjectInput::SolarSystem { .. }
    ));

    ui.placement_mode = PlacementMode::SatelliteOrbit;
    let object_input = ui.build_reset_object_input();
    let mgr = SimulationManager::new();
    mgr.reset(
        object_input,
        ui.simulation_type,
        ui.add_particle_count,
        ui.base_scale,
    );
    assert_eq!(
        mgr.particle_count(),
        ui.satellite_orbit.total_particle_count()
    );
}

#[test]
fn solar_system_reset_opens_log_panel() {
    let mut ui = UiState::default();
    ui.placement_mode = PlacementMode::SolarSystem;
    ui.request_reset();
    assert!(ui.reset_log.is_open);
    assert!(ui.reset_log.in_progress);
    assert!(!ui.reset_abort_requested());
}

#[test]
fn manual_reset_does_not_open_log_panel() {
    let mut ui = UiState::default();
    ui.placement_mode = PlacementMode::Manual;
    ui.request_reset();
    assert!(!ui.reset_log.is_open);
    assert!(!ui.reset_log.in_progress);
}

#[test]
fn request_reset_abort_sets_flag() {
    let ui = UiState::default();
    ui.request_reset_abort();
    assert!(ui.reset_abort_requested());
}

#[test]
fn close_reset_log_panel_requires_idle_state() {
    let mut ui = UiState::default();
    ui.open_solar_system_reset_log();
    ui.close_reset_log_panel();
    assert!(ui.reset_log.is_open);

    ui.finish_reset_log();
    ui.close_reset_log_panel();
    assert!(!ui.reset_log.is_open);
}

#[test]
fn panel_slider_double_click_resets_to_defaults() {
    let mut ui = UiState::default();
    ui.base_scale = 42.0;
    ui.scale = 99.0;
    ui.scale_gauge = DEFAULT_SCALE_UI * 2.0;
    ui.max_fps = 999;
    ui.skip = 50;
    ui.add_particle_count = 1;
    ui.satellite_orbit.satellite_count = 1;

    ui.reset_scale_to_base();
    ui.reset_max_fps_to_default();
    ui.reset_skip_to_default();
    ui.reset_add_particle_count_to_default(0);
    ui.reset_satellite_count_to_default();

    assert_eq!(ui.scale, 42.0);
    assert_eq!(ui.scale_gauge, DEFAULT_SCALE_UI);
    assert_eq!(ui.max_fps, DEFAULT_MAX_FPS);
    assert_eq!(ui.skip, DEFAULT_SKIP_DRAWING_FRAMES);
    assert_eq!(ui.add_particle_count, DEFAULT_ADD_PARTICLE_COUNT);
    assert_eq!(ui.satellite_orbit.satellite_count, DEFAULT_SATELLITE_COUNT);
}
