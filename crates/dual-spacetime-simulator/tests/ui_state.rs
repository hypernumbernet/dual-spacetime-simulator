use dual_spacetime_simulator::settings::AppSettings;
use dual_spacetime_simulator::ui_state::{AppMode, UiState};

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
fn app_mode_change_resets_panels() {
    let mut ui = UiState::default();
    ui.is_simulation_panel_open = true;
    ui.is_graph3d_panel_open = false;
    ui.apply_panel_defaults_on_app_mode_change(AppMode::Simulation, AppMode::Graph3D);
    assert!(!ui.is_simulation_panel_open);
    assert!(ui.is_graph3d_panel_open);
}
