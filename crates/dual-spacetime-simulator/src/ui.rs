use crate::object_input::{clamp_world_scale, ObjectInputType};
use crate::particle_snapshot::{
    ParticleSnapshot, SNAPSHOT_FILTER_EXT, SNAPSHOT_FILTER_NAME,
};
use crate::settings::AppSettings;
use crate::simulation::{AU, LY, MPC, PC, SimulationManager};
use crate::ui_state::*;
use crate::ui_styles::*;
use egui::{Checkbox, ComboBox, Slider};
use std::sync::{Arc, RwLock};
use winit::window::Window;

const MENU_POPUP_WIDTH: f32 = 180.0;
const PANEL_DEFAULT_X: f32 = 16.0;
const PANEL_MENU_OFFSET_Y: f32 = 16.0;

/// Draws the full control UI and applies user edits to shared UI/application state.
pub fn draw_ui(
    ui_state: &Arc<RwLock<UiState>>,
    simulation_manager: &Arc<RwLock<SimulationManager>>,
    settings: &mut AppSettings,
    ctx: &egui::Context,
) {
    let mut uis = ui_state.write().unwrap();
    let menu_bar_height = egui::TopBottomPanel::top("menu_bar")
        .show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                ui.menu_button("File", |ui| {
                    ui.set_min_width(MENU_POPUP_WIDTH);
                    if ui.button("Exit").clicked() {
                        uis.request_exit = true;
                        ui.close_menu();
                    }
                });

                ui.menu_button("Mode", |ui| {
                    ui.set_min_width(MENU_POPUP_WIDTH);
                    if ui
                        .selectable_label(
                            uis.app_mode == crate::ui_state::AppMode::Simulation,
                            "Simulation",
                        )
                        .clicked()
                    {
                        uis.app_mode = crate::ui_state::AppMode::Simulation;
                        ui.close_menu();
                    }
                    if ui
                        .selectable_label(
                            uis.app_mode == crate::ui_state::AppMode::Graph3D,
                            "3D Graph",
                        )
                        .clicked()
                    {
                        uis.app_mode = crate::ui_state::AppMode::Graph3D;
                        uis.is_running = false;
                        ui.close_menu();
                    }
                });

                ui.menu_button("Panel", |ui| {
                    ui.set_min_width(MENU_POPUP_WIDTH);
                    let available = uis.get_available_panels();
                    if available.contains(&PanelKind::Simulation) {
                        if ui
                            .checkbox(
                                &mut uis.is_simulation_panel_open,
                                PanelKind::Simulation.label(),
                            )
                            .clicked()
                        {
                            ui.close_menu();
                        }
                    }
                    if available.contains(&PanelKind::ObjectInput) {
                        if ui
                            .checkbox(
                                &mut uis.is_object_input_panel_open,
                                PanelKind::ObjectInput.label(),
                            )
                            .clicked()
                        {
                            ui.close_menu();
                        }
                    }
                    if available.contains(&PanelKind::Graph3D) {
                        if ui
                            .checkbox(&mut uis.is_graph3d_panel_open, PanelKind::Graph3D.label())
                            .clicked()
                        {
                            ui.close_menu();
                        }
                    }
                    if available.contains(&PanelKind::Settings) {
                        if ui
                            .checkbox(&mut uis.is_settings_panel_open, PanelKind::Settings.label())
                            .clicked()
                        {
                            ui.close_menu();
                        }
                    }
                });

                ui.menu_button("View", |ui| {
                    ui.set_min_width(MENU_POPUP_WIDTH);
                    if ui.checkbox(&mut uis.show_grid, "Show Grid").clicked() {
                        ui.close_menu();
                    }
                });

                if uis.app_mode == AppMode::Simulation {
                    ui.menu_button("Simulation", |ui| {
                        ui.set_min_width(MENU_POPUP_WIDTH);
                        if ui
                            .button(if uis.is_running { "Pause" } else { "Start" })
                            .clicked()
                        {
                            uis.is_running = !uis.is_running;
                            ui.close_menu();
                        }
                        if ui.button("Reset").clicked() {
                            uis.request_reset();
                            ui.close_menu();
                        }
                    });
                }

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(format!("Frame {}", uis.frame));
                    ui.separator();
                    ui.label(format!("FPS {}", uis.fps));
                });
            });
        })
        .response
        .rect
        .height();

    let from_mode = uis.last_app_mode_for_panel_sync;
    let to_mode = uis.app_mode;
    if from_mode != to_mode {
        uis.apply_panel_defaults_on_app_mode_change(from_mode, to_mode);
        uis.last_app_mode_for_panel_sync = to_mode;
    }

    if uis.app_mode == AppMode::Simulation && uis.is_simulation_panel_open {
        egui::Window::new("Simulation")
            .resizable(false)
            .collapsible(true)
            .default_pos(egui::pos2(
                PANEL_DEFAULT_X,
                menu_bar_height + PANEL_MENU_OFFSET_Y,
            ))
            .default_width(uis.input_panel_width)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    label_normal(ui, "FPS");
                    label_indicator(ui, &uis.fps.to_string());
                });
                ui.horizontal(|ui| {
                    label_normal(ui, "Frame");
                    label_indicator(ui, &uis.frame.to_string());
                });
                ui.horizontal(|ui| {
                    label_normal(ui, "Time");
                    label_indicator(ui, &format_simulation_time(uis.simulation_time));
                });
                ui.horizontal(|ui| {
                    label_normal(ui, "Particle Count");
                    let count = simulation_manager.read().unwrap().particle_count();
                    label_indicator(ui, &count.to_string());
                });
                ui.separator();
                if button_normal(ui, "Start/Pause").clicked() {
                    uis.is_running = !uis.is_running;
                }
                ui.separator();
                if button_normal(ui, "Object Input").clicked() {
                    uis.is_object_input_panel_open = !uis.is_object_input_panel_open;
                }
                ui.separator();
                dragvalue_normal(ui, &mut uis.time_per_frame, 1.0, "Time(sec)/Frame");
                ui.separator();
                ui.horizontal(|ui| {
                    label_normal(ui, "Scale");
                    label_indicator(ui, format_scale(uis.scale_gauge, uis.scale).as_str());
                });
                slider_pure(
                    ui,
                    &mut uis.scale_gauge,
                    DEFAULT_SCALE_UI * 0.2..=DEFAULT_SCALE_UI * 3.0,
                );
                ui.separator();
                ui.style_mut().spacing.slider_width = 160.0;
                label_normal(ui, "Max FPS");
                ui.add(Slider::new(&mut uis.max_fps, 1..=1000));
                ui.separator();
                label_normal(ui, "Skip drawing frames");
                ui.add(Slider::new(&mut uis.skip, 0..=1000));
                ui.separator();
                ui.horizontal(|ui| {
                    let mut v = uis.show_grid;
                    if ui.add(Checkbox::new(&mut v, "Show Grid")).changed() {
                        uis.show_grid = v;
                    }
                });
                ui.separator();
                let (save, load) = button_row_pair(ui, "Save", "Load");
                if save.clicked() {
                    uis.pending_snapshot_dialog = Some(PendingSnapshotDialog::Save);
                }
                if load.clicked() {
                    uis.pending_snapshot_dialog = Some(PendingSnapshotDialog::Load);
                }
            });
    }

    if uis.is_settings_panel_open {
        egui::Window::new("Settings")
            .resizable(false)
            .collapsible(true)
            .default_width(uis.input_panel_width)
            .show(ctx, |ui| {
                dragvalue_normal(ui, &mut uis.min_window_width, 1.0, "Min Window Width");
                dragvalue_normal(ui, &mut uis.min_window_height, 1.0, "Min Window Height");
                dragvalue_normal(ui, &mut uis.max_particle_count, 10.0, "Max Particle Count");
                ui.separator();
                ui.horizontal(|ui| {
                    let mut v = uis.start_maximized;
                    if ui.add(Checkbox::new(&mut v, "Start Maximized")).changed() {
                        uis.start_maximized = v;
                    }
                });
                ui.horizontal(|ui| {
                    let mut v = uis.link_point_size_to_scale;
                    if ui
                        .add(Checkbox::new(&mut v, "Link Point Size to Scale"))
                        .changed()
                    {
                        uis.link_point_size_to_scale = v;
                    }
                });
                ui.horizontal(|ui| {
                    let mut v = uis.lock_camera_up;
                    if ui
                        .add(Checkbox::new(&mut v, "Lock Camera Up/Down"))
                        .changed()
                    {
                        uis.lock_camera_up = v;
                    }
                });
                ui.horizontal(|ui| {
                    let mut v = uis.mailbox_present_mode;
                    if ui
                        .add(Checkbox::new(&mut v, "Mailbox Present Mode"))
                        .changed()
                    {
                        uis.mailbox_present_mode = v;
                    }
                });
                ui.separator();
                if button_normal(ui, "Save Settings").clicked() {
                    settings.window_min_width = uis.min_window_width;
                    settings.window_min_height = uis.min_window_height;
                    settings.max_particle_count = uis.max_particle_count;
                    settings.start_maximized = uis.start_maximized;
                    settings.link_point_size_to_scale = uis.link_point_size_to_scale;
                    settings.lock_camera_up = uis.lock_camera_up;
                    settings.mailbox_present_mode = uis.mailbox_present_mode;
                    if let Err(e) = settings.save() {
                        eprintln!("Failed to save settings: {}", e);
                    }
                }
            });
    }

    if uis.app_mode == AppMode::Simulation && uis.is_object_input_panel_open {
        egui::Window::new("Object Input")
            .resizable(false)
            .collapsible(true)
            .default_width(uis.input_panel_width)
            .max_width(uis.input_panel_width)
            .show(ctx, |ui| {
                ui.set_width(uis.input_panel_width);
                combobox_simulation_type(ui, &mut uis);
                base_scale_input(ui, &mut uis);
                combobox_placement_mode(ui, &mut uis);
                match uis.placement_mode {
                    PlacementMode::SolarSystem => {
                        condition_solar_system(ui, &mut uis);
                    }
                    PlacementMode::SatelliteOrbit => {
                        condition_satellite_orbit(ui, &mut uis);
                    }
                    PlacementMode::Manual => {}
                }
                ui.separator();
                combobox_object_input_type(ui, &mut uis);
                match uis.object_input_type {
                    ObjectInputType::RandomSphere => {
                        condition_random_sphere(ui, &mut uis);
                    }
                    ObjectInputType::RandomCube => {
                        condition_random_cube(ui, &mut uis);
                    }
                    ObjectInputType::SpiralDisk => {
                        condition_spiral_disk(ui, &mut uis);
                    }
                    ObjectInputType::EllipticalOrbit => {
                        condition_elliptical_orbit(ui, &mut uis);
                    }
                }
                ui.separator();
                let current_count = simulation_manager.read().unwrap().particle_count();
                slider_add_particle_count(ui, &mut uis, current_count);
                ui.separator();
                slider_add_center(ui, &mut uis);
                ui.horizontal(|ui| {
                    let mut v = uis.show_add_center_preview;
                    if ui
                        .add(Checkbox::new(&mut v, "Show Add Center Preview"))
                        .changed()
                    {
                        uis.show_add_center_preview = v;
                    }
                });
                button_add_particles(ui, &mut uis, current_count);
                ui.separator();
                if !uis.is_reset_requested {
                    button_reset(ui, &mut uis);
                } else {
                    label_normal(ui, "Resetting...");
                }
            });
    }

    if uis.is_resetting && uis.is_reset_requested {
        uis.is_resetting = false;
        uis.base_scale = clamp_world_scale(uis.base_scale);
        uis.object_input = uis.build_reset_object_input();
        uis.scale = uis.base_scale;
        if uis.placement_mode == PlacementMode::SolarSystem {
            uis.time_per_frame = 10_000.0;
            uis.max_fps = 1000;
            uis.skip = 10;
        } else if uis.placement_mode == PlacementMode::Manual
            && uis.object_input_type == ObjectInputType::EllipticalOrbit
        {
            uis.time_per_frame = 100_000.0;
            uis.max_fps = 1000;
            uis.skip = 0;
        } else {
            uis.time_per_frame = 10.0;
            uis.max_fps = 60;
            uis.skip = 0;
        }
        uis.scale_gauge = DEFAULT_SCALE_UI;
    }

    if uis.app_mode == AppMode::Graph3D && uis.is_graph3d_panel_open {
        egui::Window::new("3D Graph")
            .resizable(false)
            .collapsible(true)
            .default_pos(egui::pos2(
                PANEL_DEFAULT_X,
                menu_bar_height + PANEL_MENU_OFFSET_Y,
            ))
            .default_width(uis.input_panel_width)
            .show(ctx, |ui| {
                combobox_graph_type(ui, &mut uis);
                ui.separator();

                match uis.graph_type {
                    GraphType::SphericalFibonacciLattice => {
                        condition_spherical_fibonacci_lattice(ui, &mut uis);
                    }
                    GraphType::RapidityFieldMatrix => {
                        condition_rapidity_field_matrix(ui, &mut uis);
                    }
                    GraphType::RapidityFieldBiquaternion => {
                        condition_rapidity_field_biquaternion(ui, &mut uis);
                    }
                }

                ui.separator();
                label_normal(ui, "Sample Count");
                ui.add(Slider::new(&mut uis.graph_sample_count, 1..=5000).drag_value_speed(1.0));
            });
    }

}

/// Formats simulation time into a compact signed human-readable duration string.
fn format_simulation_time(simulation_time: f64) -> String {
    let sign = if simulation_time < 0.0 { "-" } else { "" };
    let total_seconds = simulation_time.abs();
    let days = (total_seconds / 86400.0).floor() as i64;
    let remaining_seconds = total_seconds % 86400.0;
    let hours = (remaining_seconds / 3600.0).floor() as i64;
    let minutes = ((remaining_seconds % 3600.0) / 60.0).floor() as i64;
    let seconds = (remaining_seconds % 60.0).floor() as i64;
    format!(
        "{}{} {:02}:{:02}:{:02}",
        sign, days, hours, minutes, seconds
    )
}

/// Formats current scale and gauge ratio for display in the simulation panel.
fn format_scale(scale_guage: f64, scale: f64) -> String {
    let scale_inv = DEFAULT_SCALE_UI / scale_guage;
    let pow10 = scale_inv.powi(4) * scale;
    if pow10 >= MPC {
        format!("{:.3e} Mpc", pow10 / MPC)
    } else if pow10 >= PC {
        format!("{:.3e} pc", pow10 / PC)
    } else if pow10 >= LY {
        format!("{:.3e} ly", pow10 / LY)
    } else if pow10 >= AU {
        format!("{:.3} au", pow10 / AU)
    } else if pow10 >= 1e9 {
        format!("{:.3e} km", pow10 / 1e3)
    } else if pow10 >= 1e3 {
        format!("{:.3} km", pow10 / 1e3)
    } else if pow10 < 1e-15 {
        format!("{:.6} fm", pow10 * 1e15)
    } else if pow10 < 1e-9 {
        format!("{:.6} nm", pow10 * 1e9)
    } else if pow10 < 1e-3 {
        format!("{:.6} mm", pow10 * 1e3)
    } else {
        format!("{:.3} m", pow10)
    }
}

/// Renders base-scale value input with selectable length units.
fn base_scale_input(ui: &mut egui::Ui, uis: &mut UiState) {
    label_normal(ui, "Base Scale");
    let previous_unit = uis.base_scale_unit;
    let unit = uis.base_scale_unit;
    let mut display = uis.base_scale_display_value();
    let format_display = |value: f64| unit.format_display(value);
    ui.horizontal(|ui| {
        dragvalue_positive_f64(
            ui,
            &mut display,
            BASE_SCALE_DRAG_SPEED,
            unit.min_display_value(),
            110.0,
            None,
            Some(&format_display),
        );
        display = unit.sanitize_display(display);
        let id = ui.make_persistent_id("base_scale_unit_combobox");
        ComboBox::from_id_salt(id)
            .selected_text(format!("{}", uis.base_scale_unit))
            .width(60.0)
            .show_ui(ui, |ui| {
                for unit in BaseScaleUnit::ALL {
                    selectable_value(ui, &mut uis.base_scale_unit, unit);
                }
            });
    });
    uis.apply_base_scale_edit(display, uis.base_scale_unit != previous_unit);
}

/// Renders the simulation-type combo box and updates dependent UI state.
fn combobox_simulation_type(ui: &mut egui::Ui, uis: &mut UiState) {
    label_normal(ui, "Simulation Type");
    let previous_type = uis.simulation_type;
    let id = ui.make_persistent_id("simulation_type_combobox");
    ComboBox::from_id_salt(id)
        .selected_text(format!("{}", uis.simulation_type))
        .width(ui.available_width())
        .show_ui(ui, |ui| {
            selectable_value(ui, &mut uis.simulation_type, SimulationType::Normal);
            selectable_value(
                ui,
                &mut uis.simulation_type,
                SimulationType::SpeedOfLightLimit,
            );
            selectable_value(
                ui,
                &mut uis.simulation_type,
                SimulationType::LorentzTransformation,
            );
        });
    uis.apply_simulation_type_change(previous_type);
}

/// Renders the placement-mode combo box and updates dependent UI state.
fn combobox_placement_mode(ui: &mut egui::Ui, uis: &mut UiState) {
    label_normal(ui, "Placement Mode");
    let previous_mode = uis.placement_mode;
    let id = ui.make_persistent_id("placement_mode_combobox");
    ComboBox::from_id_salt(id)
        .selected_text(format!("{}", uis.placement_mode))
        .width(ui.available_width())
        .show_ui(ui, |ui| {
            selectable_value(ui, &mut uis.placement_mode, PlacementMode::Manual);
            selectable_value(ui, &mut uis.placement_mode, PlacementMode::SolarSystem);
            selectable_value(ui, &mut uis.placement_mode, PlacementMode::SatelliteOrbit);
        });
    uis.apply_placement_mode_change(previous_mode);
}

/// Renders the object-input type combo box and syncs scaled parameters on change.
fn combobox_object_input_type(ui: &mut egui::Ui, uis: &mut UiState) {
    label_normal(ui, "Object Input Type");
    let previous_type = uis.object_input_type;
    let id = ui.make_persistent_id("object_input_type_combobox");
    ComboBox::from_id_salt(id)
        .selected_text(format!("{}", uis.object_input_type))
        .width(ui.available_width())
        .show_ui(ui, |ui| {
            selectable_value(
                ui,
                &mut uis.object_input_type,
                ObjectInputType::RandomSphere,
            );
            selectable_value(
                ui,
                &mut uis.object_input_type,
                ObjectInputType::RandomCube,
            );
            selectable_value(
                ui,
                &mut uis.object_input_type,
                ObjectInputType::SpiralDisk,
            );
            selectable_value(
                ui,
                &mut uis.object_input_type,
                ObjectInputType::EllipticalOrbit,
            );
        });
    uis.apply_object_input_type_change(previous_type);
}

/// Renders parameter controls for the random-sphere object input.
fn condition_random_sphere(ui: &mut egui::Ui, uis: &mut UiState) {
    dragvalue_normal(ui, &mut uis.random_sphere.radius, 1e9, "Sphere Radius (m)");
    dragvalue_normal(
        ui,
        &mut uis.random_sphere.mass_range.0,
        1e20,
        "Mass Min (kg)",
    );
    dragvalue_normal(
        ui,
        &mut uis.random_sphere.mass_range.1,
        1e20,
        "Mass Max (kg)",
    );
    dragvalue_normal(
        ui,
        &mut uis.random_sphere.velocity_std,
        1e3,
        "Velocity Std (m/s)",
    );
}

/// Renders parameter controls for the random-cube object input.
fn condition_random_cube(ui: &mut egui::Ui, uis: &mut UiState) {
    dragvalue_normal(ui, &mut uis.random_cube.cube_size, 1e3, "Cube Size (m)");
    dragvalue_normal(ui, &mut uis.random_cube.mass_range.0, 1e20, "Mass Min (kg)");
    dragvalue_normal(ui, &mut uis.random_cube.mass_range.1, 1e20, "Mass Max (kg)");
    dragvalue_normal(
        ui,
        &mut uis.random_cube.velocity_std,
        1e3,
        "Velocity Std (m/s)",
    );
}

/// Renders parameter controls for the spiral-disk object input.
fn condition_spiral_disk(ui: &mut egui::Ui, uis: &mut UiState) {
    dragvalue_normal(ui, &mut uis.spiral_disk.disk_radius, 1e7, "Disk Radius (m)");
    dragvalue_normal(ui, &mut uis.spiral_disk.mass_fixed, 1e20, "Mass Fixed (kg)");
}

/// Renders start-date controls for the solar-system object input.
fn condition_solar_system(ui: &mut egui::Ui, uis: &mut UiState) {
    dragvalue_normal(ui, &mut uis.solar_system.start_year, 1, "Year");
    dragvalue_normal(ui, &mut uis.solar_system.start_month, 1, "Month");
    dragvalue_normal(ui, &mut uis.solar_system.start_day, 1, "Day");
    dragvalue_normal(ui, &mut uis.solar_system.start_hour, 1, "Hour");
}

/// Renders parameter controls for the satellite-orbit object input.
fn condition_satellite_orbit(ui: &mut egui::Ui, uis: &mut UiState) {
    dragvalue_normal(
        ui,
        &mut uis.satellite_orbit.orbit_altitude_min,
        1e3,
        "Orbit Min (m)",
    );
    dragvalue_normal(
        ui,
        &mut uis.satellite_orbit.orbit_altitude_max,
        1e3,
        "Orbit Max (m)",
    );
    label_normal(ui, "Asteroid");
    dragvalue_normal(
        ui,
        &mut uis.satellite_orbit.asteroid_mass,
        1e10,
        "Mass (kg)",
    );
    dragvalue_normal(
        ui,
        &mut uis.satellite_orbit.asteroid_distance,
        1e3,
        "Distance (m)",
    );
    dragvalue_normal(
        ui,
        &mut uis.satellite_orbit.asteroid_speed,
        1e3,
        "Speed (m/s)",
    );
}

/// Renders parameter controls for the elliptical-orbit object input.
fn condition_elliptical_orbit(ui: &mut egui::Ui, uis: &mut UiState) {
    label_normal(ui, "Central Body");
    dragvalue_normal(
        ui,
        &mut uis.elliptical_orbit.central_mass,
        1e20,
        "Mass (kg)",
    );
    label_normal(ui, "Planetary Body");
    dragvalue_normal(
        ui,
        &mut uis.elliptical_orbit.planetary_mass,
        1e20,
        "Mass (kg)",
    );
    dragvalue_normal(
        ui,
        &mut uis.elliptical_orbit.planetary_speed,
        1e3,
        "Initial Speed (m/s)",
    );
    dragvalue_normal(
        ui,
        &mut uis.elliptical_orbit.planetary_distance,
        1e7,
        "Initial Distance (m)",
    );
}

const ADD_CENTER_SLIDER_RANGE: std::ops::RangeInclusive<f64> = -10.0..=10.0;
const ADD_CENTER_SLIDER_STEP: f64 = 0.01;

fn add_center_axis_slider(value: &mut f64) -> Slider<'_> {
    Slider::new(value, ADD_CENTER_SLIDER_RANGE.clone())
        .step_by(ADD_CENTER_SLIDER_STEP)
        .drag_value_speed(ADD_CENTER_SLIDER_STEP)
}

/// Renders X/Y/Z sliders as base-scale multipliers, converted via Correct.m like other inputs.
fn slider_add_center(ui: &mut egui::Ui, uis: &mut UiState) {
    ui.style_mut().spacing.slider_width = 140.0;
    label_normal(ui, "Add Center");
    ui.horizontal(|ui| {
        label_normal(ui, "X");
        ui.add(add_center_axis_slider(&mut uis.add_center.x));
    });
    ui.horizontal(|ui| {
        label_normal(ui, "Y");
        ui.add(add_center_axis_slider(&mut uis.add_center.y));
    });
    ui.horizontal(|ui| {
        label_normal(ui, "Z");
        ui.add(add_center_axis_slider(&mut uis.add_center.z));
    });
}

/// Draws add button and flags particle append when clicked.
fn button_add_particles(ui: &mut egui::Ui, uis: &mut UiState, current_count: u32) {
    let at_limit = uis.remaining_particle_capacity(current_count) == 0;
    if at_limit {
        label_normal(ui, "Particle limit reached");
    }
    ui.add_enabled_ui(
        !at_limit && !uis.is_add_particles_requested && uis.is_add_particles_enabled,
        |ui| {
            if button_normal(ui, "Add").clicked() {
                uis.base_scale = clamp_world_scale(uis.base_scale);
                uis.object_input = uis.build_object_input();
                uis.is_add_particles_requested = true;
            }
        },
    );
    if !uis.is_add_particles_enabled && !at_limit {
        label_normal(ui, "Reset required");
    }
    if uis.is_add_particles_requested {
        label_normal(ui, "Adding...");
    }
}

/// Renders add-particle-count slider capped by remaining particle capacity.
fn slider_add_particle_count(ui: &mut egui::Ui, uis: &mut UiState, current_count: u32) {
    ui.style_mut().spacing.slider_width = 150.0;
    label_normal(ui, "Add Particle Count");
    let remaining = uis.remaining_particle_capacity(current_count);
    uis.clamp_add_particle_count_to_capacity(current_count);
    if let Some(range) = UiState::add_particle_count_range(remaining) {
        ui.add(Slider::new(&mut uis.add_particle_count, range));
    }
}

/// Draws reset button and flags simulation reset when clicked.
fn button_reset(ui: &mut egui::Ui, uis: &mut UiState) {
    if button_normal(ui, "Reset").clicked() {
        uis.request_reset();
    }
}

/// Opens a deferred save/load dialog after the UI frame completes.
pub fn process_pending_snapshot_dialog(
    window: &Window,
    ui_state: &Arc<RwLock<UiState>>,
    simulation_manager: &Arc<RwLock<SimulationManager>>,
    need_redraw: &Arc<RwLock<bool>>,
) {
    let pending = ui_state.write().unwrap().pending_snapshot_dialog.take();
    let Some(pending) = pending else {
        return;
    };
    match pending {
        PendingSnapshotDialog::Save => {
            save_particles(window, ui_state, simulation_manager);
        }
        PendingSnapshotDialog::Load => {
            load_particles(window, ui_state, simulation_manager, need_redraw);
        }
    }
}

fn snapshot_file_dialog(parent: &Window) -> rfd::FileDialog {
    parent.focus_window();
    rfd::FileDialog::new()
        .add_filter(SNAPSHOT_FILTER_NAME, &[SNAPSHOT_FILTER_EXT])
        .set_parent(parent)
}

/// Saves all current particles to a zip snapshot via a native file dialog.
fn save_particles(
    window: &Window,
    ui_state: &Arc<RwLock<UiState>>,
    simulation_manager: &Arc<RwLock<SimulationManager>>,
) {
    let Some(path) = snapshot_file_dialog(window)
        .set_file_name("particles.zip")
        .save_file()
    else {
        return;
    };
    let uis = ui_state.read().unwrap();
    let particles = simulation_manager.read().unwrap().particles();
    let snapshot = ParticleSnapshot::new(uis.simulation_type, uis.scale, particles);
    if let Err(e) = snapshot.save(&path) {
        eprintln!("Failed to save particles: {}", e);
    }
}

/// Loads particles from a zip snapshot and restores them as the initial state.
fn load_particles(
    window: &Window,
    ui_state: &Arc<RwLock<UiState>>,
    simulation_manager: &Arc<RwLock<SimulationManager>>,
    need_redraw: &Arc<RwLock<bool>>,
) {
    let Some(path) = snapshot_file_dialog(window).pick_file() else {
        return;
    };
    let snapshot = match ParticleSnapshot::load(&path) {
        Ok(snapshot) => snapshot,
        Err(e) => {
            eprintln!("Failed to load particles: {}", e);
            return;
        }
    };
    let mut uis = ui_state.write().unwrap();
    if snapshot.particles.len() > uis.max_particle_count as usize {
        eprintln!(
            "Particle count {} exceeds maximum {}",
            snapshot.particles.len(),
            uis.max_particle_count
        );
        return;
    }
    uis.simulation_type = snapshot.simulation_type;
    let scale = clamp_world_scale(snapshot.scale);
    uis.scale = scale;
    uis.apply_external_base_scale(scale);
    uis.frame = 1;
    uis.simulation_time = 0.0;
    uis.is_running = false;
    simulation_manager.write().unwrap().load_from_snapshot(snapshot);
    *need_redraw.write().unwrap() = true;
}

/// Renders graph-type combo box for Graph3D mode.
fn combobox_graph_type(ui: &mut egui::Ui, uis: &mut UiState) {
    label_normal(ui, "Graph Type");
    let id = ui.make_persistent_id("graph_type_combobox");
    ComboBox::from_id_salt(id)
        .selected_text(format!("{}", uis.graph_type))
        .width(ui.available_width())
        .show_ui(ui, |ui| {
            selectable_value(
                ui,
                &mut uis.graph_type,
                GraphType::SphericalFibonacciLattice,
            );
            selectable_value(ui, &mut uis.graph_type, GraphType::RapidityFieldMatrix);
            selectable_value(
                ui,
                &mut uis.graph_type,
                GraphType::RapidityFieldBiquaternion,
            );
        });
}

/// Renders controls specific to spherical Fibonacci lattice graph mode.
fn condition_spherical_fibonacci_lattice(ui: &mut egui::Ui, uis: &mut UiState) {
    label_normal(ui, "Spherical Fibonacci Lattice");
    dragvalue_normal(ui, &mut uis.graph_radius, 0.01, "Radius");
    label_normal(
        ui,
        "Deterministic spherical sampling using a Fibonacci lattice.",
    );
    ui.separator();
    label_normal(
        ui,
        "Uses golden-angle azimuth progression with near-equal-area latitude spacing.",
    );
    label_normal(
        ui,
        "Generates quasi-uniform points on the sphere, then scales by radius.",
    );
}

/// Renders controls specific to the rapidity-field graph mode by matrix.
fn condition_rapidity_field_matrix(ui: &mut egui::Ui, uis: &mut UiState) {
    label_normal(ui, "Rapidity Vector Field by matrix");
    dragvalue_normal(ui, &mut uis.graph_velocity_scale, 0.01, "Velocity Scale");
    ui.separator();
    label_normal(
        ui,
        "Lorentz boost using matrices (standard 4x4 representation)",
    );
}

/// Renders controls specific to the rapidity-field graph mode by biquaternion.
fn condition_rapidity_field_biquaternion(ui: &mut egui::Ui, uis: &mut UiState) {
    label_normal(ui, "Rapidity Vector Field by biquaternion");
    dragvalue_normal(ui, &mut uis.graph_velocity_scale, 0.01, "Velocity Scale");
    label_normal(ui, "Calculation of Lorentz boost using biquaternions.");
}
