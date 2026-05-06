use dual_spacetime_simulator::settings::AppSettings;
use dual_spacetime_simulator::tree::TreeParams;
use dual_spacetime_simulator::ui_state::{AppMode, UiState};

#[test]
fn apply_settings_clamps_particle_count() {
    let mut ui = UiState::default();
    ui.particle_count = 10_000;
    let mut s = AppSettings::default();
    s.max_particle_count = 100;
    ui.apply_settings(&s);
    assert_eq!(ui.max_particle_count, 100);
    assert_eq!(ui.particle_count, 100);
}

#[test]
fn gpu_tree_fingerprint_changes_with_params() {
    let mut ui = UiState::default();
    let a = ui.gpu_tree_fingerprint();
    ui.gpu_tree_params.trunk_height += 0.01;
    let b = ui.gpu_tree_fingerprint();
    assert_ne!(a, b);
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
fn gpu_tree_fingerprint_stable_for_clone_params() {
    let mut ui = UiState::default();
    ui.gpu_tree_params = TreeParams {
        seed: 99,
        ..Default::default()
    };
    let a = ui.gpu_tree_fingerprint();
    ui.gpu_tree_params = TreeParams {
        seed: 99,
        ..Default::default()
    };
    let b = ui.gpu_tree_fingerprint();
    assert_eq!(a, b);
}
