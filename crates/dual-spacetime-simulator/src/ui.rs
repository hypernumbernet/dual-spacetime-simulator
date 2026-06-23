use crate::object_input::{ObjectInputType, ParticleBasicColor, clamp_world_scale};
use crate::particle_snapshot::{ParticleSnapshot, SNAPSHOT_FILTER_EXT, SNAPSHOT_FILTER_NAME};
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
            egui::MenuBar::new().ui(ui, |ui| {
                ui.menu_button("File", |ui| {
                    ui.set_min_width(MENU_POPUP_WIDTH);
                    if ui.button("Exit").clicked() {
                        uis.request_exit = true;
                        ui.close_kind(egui::UiKind::Menu);
                    }
                });

                ui.menu_button("Panel", |ui| {
                    ui.set_min_width(MENU_POPUP_WIDTH);
                    for panel in PANELS {
                        if ui
                            .checkbox(uis.panel_open_mut(*panel), panel.label())
                            .clicked()
                        {
                            ui.close_kind(egui::UiKind::Menu);
                        }
                    }
                });

                ui.menu_button("View", |ui| {
                    ui.set_min_width(MENU_POPUP_WIDTH);
                    if ui.checkbox(&mut uis.show_grid, "Show Grid").clicked() {
                        ui.close_kind(egui::UiKind::Menu);
                    }
                });

                ui.menu_button("Simulation", |ui| {
                    ui.set_min_width(MENU_POPUP_WIDTH);
                    let particle_count = simulation_manager.read().unwrap().particle_count();
                    let can_start = UiState::can_start_simulation(particle_count);
                    ui.add_enabled_ui(can_start || uis.is_running, |ui| {
                        if ui
                            .button(if uis.is_running { "Pause" } else { "Start" })
                            .clicked()
                        {
                            uis.is_running = !uis.is_running;
                            ui.close_kind(egui::UiKind::Menu);
                        }
                    });
                    if ui.button("Reset").clicked() {
                        uis.request_reset();
                        ui.close_kind(egui::UiKind::Menu);
                    }
                });

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

    let input_panel_width = uis.input_panel_width;

    uis.is_simulation_panel_open = show_closable_window(
        ctx,
        "Simulation",
        uis.is_simulation_panel_open,
        true,
        |window| {
            window
                .resizable(false)
                .collapsible(true)
                .default_pos(egui::pos2(
                    PANEL_DEFAULT_X,
                    menu_bar_height + PANEL_MENU_OFFSET_Y,
                ))
                .default_width(input_panel_width)
        },
        |ui| {
            lock_panel_content_width(ui);
            let particle_count = simulation_manager.read().unwrap().particle_count();
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
                label_indicator(ui, &particle_count.to_string());
            });
            ui.separator();
            let can_start = UiState::can_start_simulation(particle_count);
            ui.add_enabled_ui(can_start || uis.is_running, |ui| {
                if button_normal(
                    ui,
                    if uis.is_running { "Pause" } else { "Start" },
                    uis.is_running,
                )
                .clicked()
                {
                    uis.is_running = !uis.is_running;
                }
            });
            if !can_start && !uis.is_running {
                label_normal(ui, "Need at least 2 particles");
            }
            ui.separator();
            if button_normal(ui, "Object Input", false).clicked() {
                uis.is_object_input_panel_open = !uis.is_object_input_panel_open;
            }
            ui.separator();
            dragvalue_normal(ui, &mut uis.time_per_frame, 1.0, "Time(sec)/Frame");
            ui.separator();
            let dbl_click = primary_double_click_pos(ui);
            ui.horizontal(|ui| {
                label_normal(ui, "Scale");
                label_indicator(ui, format_scale(uis.scale_gauge, uis.scale).as_str());
            });
            let scale_slider = slider_pure(
                ui,
                &mut uis.scale_gauge,
                DEFAULT_SCALE_UI * 0.2..=DEFAULT_SCALE_UI * 3.0,
            );
            apply_slider_double_click_reset_with_pos(&scale_slider, dbl_click, || {
                uis.reset_scale_to_base();
            });
            ui.separator();
            ui.style_mut().spacing.slider_width = 160.0;
            ui.horizontal(|ui| {
                label_normal(ui, "Max FPS");
                ui.checkbox(&mut uis.max_fps_unlimited, "Unlimited");
            });
            let max_fps_slider = ui.add_enabled(
                !uis.max_fps_unlimited,
                Slider::new(&mut uis.max_fps, 1..=1000),
            );
            apply_slider_double_click_reset_with_pos(&max_fps_slider, dbl_click, || {
                uis.reset_max_fps_to_default();
            });
            ui.separator();
            label_normal(ui, "Skip drawing frames");
            let skip_slider = ui.add(Slider::new(&mut uis.skip, 0..=1000));
            apply_slider_double_click_reset_with_pos(&skip_slider, dbl_click, || {
                uis.reset_skip_to_default();
            });
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
        },
    );

    uis.is_settings_panel_open = show_closable_window(
        ctx,
        "Settings",
        uis.is_settings_panel_open,
        true,
        |window| {
            window
                .resizable(false)
                .collapsible(true)
                .default_width(input_panel_width)
        },
        |ui| {
            dragvalue_normal(ui, &mut uis.min_window_width, 1.0, "Min Window Width");
            dragvalue_normal(ui, &mut uis.min_window_height, 1.0, "Min Window Height");
            dragvalue_normal(ui, &mut uis.max_particle_count, 10.0, "Max Particle Count");
            combobox_particle_display_mode(ui, &mut uis);
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
            if button_normal(ui, "Save Settings", false).clicked() {
                settings.window_min_width = uis.min_window_width;
                settings.window_min_height = uis.min_window_height;
                settings.max_particle_count = uis.max_particle_count;
                settings.start_maximized = uis.start_maximized;
                settings.link_point_size_to_scale = uis.link_point_size_to_scale;
                settings.lock_camera_up = uis.lock_camera_up;
                settings.mailbox_present_mode = uis.mailbox_present_mode;
                settings.particle_display_mode = uis.particle_display_mode;
                if let Err(e) = settings.save() {
                    eprintln!("Failed to save settings: {}", e);
                }
            }
        },
    );

    uis.is_object_input_panel_open = show_closable_window(
        ctx,
        "Object Input",
        uis.is_object_input_panel_open,
        true,
        |window| {
            window
                .resizable(false)
                .collapsible(true)
                .default_width(input_panel_width)
        },
        |ui| {
            lock_panel_content_width(ui);
            combobox_simulation_type(ui, &mut uis);
            ui.separator();
            combobox_computing_unit(ui, &mut uis);
            ui.separator();
            base_scale_input(ui, &mut uis);
            ui.separator();
            combobox_placement_mode(ui, &mut uis);
            placement_mode_conditions(ui, &mut uis);
            if !uis.is_reset_requested {
                button_reset(ui, &mut uis);
            } else {
                label_normal(ui, "Resetting...");
            }
            ui.separator();
            combobox_object_input_type(ui, &mut uis);
            object_input_type_conditions(ui, &mut uis);
            let current_count = simulation_manager.read().unwrap().particle_count();
            if uis.object_input_type.uses_add_particle_count() {
                slider_add_particle_count(ui, &mut uis, current_count);
            }
            ui.separator();
            slider_add_center(ui, &mut uis);
            ui.horizontal(|ui| {
                let mut v = uis.show_add_center_preview;
                if ui
                    .add(Checkbox::new(&mut v, "Show Add Center Pointer"))
                    .changed()
                {
                    uis.show_add_center_preview = v;
                }
            });
            button_add_particles(ui, &mut uis, current_count);
        },
    );

    if uis.is_resetting && uis.is_reset_requested {
        uis.is_resetting = false;
        uis.base_scale = clamp_world_scale(uis.base_scale);
        uis.object_input = uis.build_reset_object_input();
        uis.reset_scale_to_base();
        uis.apply_reset_timing_defaults();
    }

    if uis.reset_log.is_open {
        solar_system_reset_log_window(ctx, &mut uis);
    }
}

const RESET_LOG_MONO_SIZE: f32 = 12.0;
const RESET_LOG_ROW_HEIGHT: f32 = 14.0;

fn solar_system_reset_log_window(ctx: &egui::Context, uis: &mut UiState) {
    let in_progress = uis.reset_log.in_progress;
    uis.reset_log.is_open = show_closable_window(
        ctx,
        "Solar System Reset",
        uis.reset_log.is_open,
        !in_progress,
        |window| {
            window
                .resizable(true)
                .collapsible(true)
                .default_size([480.0, 320.0])
        },
        |ui| {
            ui.set_min_width(320.0);
            let line_count = uis.reset_log.lines.len();
            egui::ScrollArea::vertical()
                .id_salt("reset_log_scroll")
                .max_height(240.0)
                .stick_to_bottom(true)
                .show_rows(ui, RESET_LOG_ROW_HEIGHT, line_count, |ui, row_range| {
                    ui.set_width(ui.available_width());
                    for row in row_range {
                        ui.add(
                            egui::Label::new(
                                egui::RichText::new(&uis.reset_log.lines[row])
                                    .monospace()
                                    .size(RESET_LOG_MONO_SIZE),
                            )
                            .selectable(true),
                        );
                    }
                });
            ui.separator();
            let (close, abort) = button_row_close_abort(ui, !in_progress, in_progress);
            if close.clicked() {
                uis.close_reset_log_panel();
            }
            if abort.clicked() {
                uis.request_reset_abort();
                if uis.reset_log.in_progress {
                    uis.append_reset_log("Abort requested...");
                }
            }
        },
    );
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

/// Renders the computing-unit combo box and updates dependent UI state.
fn combobox_computing_unit(ui: &mut egui::Ui, uis: &mut UiState) {
    label_normal(ui, "Computing Unit");
    let previous_unit = uis.computing_unit;
    let id = ui.make_persistent_id("computing_unit_combobox");
    ComboBox::from_id_salt(id)
        .selected_text(format!("{}", uis.computing_unit))
        .width(ui.available_width())
        .show_ui(ui, |ui| {
            selectable_value(ui, &mut uis.computing_unit, ComputingUnit::Cpu);
            selectable_value(ui, &mut uis.computing_unit, ComputingUnit::Gpu);
        });
    uis.apply_computing_unit_change(previous_unit);
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
            for mode in PlacementMode::ALL {
                selectable_value(ui, &mut uis.placement_mode, mode);
            }
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
            for ty in ObjectInputType::ALL {
                selectable_value(ui, &mut uis.object_input_type, ty);
            }
        });
    uis.apply_object_input_type_change(previous_type);
}

/// Renders parameter controls for the active placement mode.
fn placement_mode_conditions(ui: &mut egui::Ui, uis: &mut UiState) {
    match uis.placement_mode {
        PlacementMode::SolarSystem => condition_solar_system(ui, uis),
        PlacementMode::SatelliteOrbit => condition_satellite_orbit(ui, uis),
        PlacementMode::Manual => {}
    }
}

/// Renders parameter controls for the active object-input type.
fn object_input_type_conditions(ui: &mut egui::Ui, uis: &mut UiState) {
    match uis.object_input_type {
        ObjectInputType::RandomSphere => condition_random_sphere(ui, uis),
        ObjectInputType::RandomCube => condition_random_cube(ui, uis),
        ObjectInputType::SpiralDisk => condition_spiral_disk(ui, uis),
        ObjectInputType::EllipticalOrbit => condition_elliptical_orbit(ui, uis),
        ObjectInputType::SingleParticle => condition_single_particle(ui, uis),
    }
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
    if let Some(range) = uis.satellite_count_slider() {
        let response = slider_labeled_u32(
            ui,
            "Satellite Count",
            &mut uis.satellite_orbit.satellite_count,
            range,
        );
        apply_slider_double_click_reset(ui, &response, || {
            uis.reset_satellite_count_to_default();
        });
    }
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

/// Renders parameter controls for the single-particle object input.
fn condition_single_particle(ui: &mut egui::Ui, uis: &mut UiState) {
    dragvalue_normal(ui, &mut uis.single_particle.mass, 1e20, "Mass (kg)");
    label_normal(ui, "Position (m)");
    dragvalue_normal(ui, &mut uis.single_particle.position.x, 1e9, "X");
    dragvalue_normal(ui, &mut uis.single_particle.position.y, 1e9, "Y");
    dragvalue_normal(ui, &mut uis.single_particle.position.z, 1e9, "Z");
    label_normal(ui, "Velocity (m/s)");
    dragvalue_normal(ui, &mut uis.single_particle.velocity.x, 1e3, "X");
    dragvalue_normal(ui, &mut uis.single_particle.velocity.y, 1e3, "Y");
    dragvalue_normal(ui, &mut uis.single_particle.velocity.z, 1e3, "Z");
    ui.horizontal(|ui| {
        label_normal(ui, "Color");
        let id = ui.make_persistent_id("single_particle_color_combobox");
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ComboBox::from_id_salt(id)
                .selected_text(format!("{}", uis.single_particle.color))
                .width(90.0)
                .show_ui(ui, |ui| {
                    for color in ParticleBasicColor::ALL {
                        selectable_value(ui, &mut uis.single_particle.color, color);
                    }
                });
        });
    });
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
    label_normal(ui, "Add Center");
    let row_width = ui.available_width();
    let dbl_click = primary_double_click_pos(ui);
    for (label, component) in [
        ("X", &mut uis.add_center.x),
        ("Y", &mut uis.add_center.y),
        ("Z", &mut uis.add_center.z),
    ] {
        let response = slider_axis_row(ui, row_width, label, add_center_axis_slider(component));
        apply_slider_double_click_reset_with_pos(&response, dbl_click, || {
            *component = 0.0;
        });
    }
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
            if button_normal(ui, "Add", false).clicked() {
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
    let remaining = uis.remaining_particle_capacity(current_count);
    uis.clamp_add_particle_count_to_capacity(current_count);
    if let Some(range) = UiState::add_particle_count_range(remaining) {
        let response =
            slider_labeled_u32(ui, "Add Particle Count", &mut uis.add_particle_count, range);
        apply_slider_double_click_reset(ui, &response, || {
            uis.reset_add_particle_count_to_default(current_count);
        });
    }
}

/// Draws reset button and flags simulation reset when clicked.
fn button_reset(ui: &mut egui::Ui, uis: &mut UiState) {
    if button_normal(ui, "Reset", false).clicked() {
        uis.request_reset();
    }
}

/// Opens a deferred save/load dialog after the UI frame completes.
pub fn process_pending_snapshot_dialog(
    window: &Window,
    ui_state: &Arc<RwLock<UiState>>,
    simulation_manager: &Arc<RwLock<SimulationManager>>,
    render_pipeline: Option<&crate::pipeline::ParticleRenderPipeline>,
    need_redraw: &Arc<RwLock<bool>>,
) {
    let pending = ui_state.write().unwrap().pending_snapshot_dialog.take();
    let Some(pending) = pending else {
        return;
    };
    match pending {
        PendingSnapshotDialog::Save => {
            save_particles(window, ui_state, simulation_manager, render_pipeline);
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
    render_pipeline: Option<&crate::pipeline::ParticleRenderPipeline>,
) {
    let Some(path) = snapshot_file_dialog(window)
        .set_file_name("particles.zip")
        .save_file()
    else {
        return;
    };
    let uis = ui_state.read().unwrap();
    let particles = if uis.uses_gpu_simulation() {
        render_pipeline
            .map(crate::pipeline::ParticleRenderPipeline::readback_particles)
            .unwrap_or_else(|| simulation_manager.read().unwrap().particles())
    } else {
        simulation_manager.read().unwrap().particles()
    };
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
    simulation_manager
        .write()
        .unwrap()
        .load_from_snapshot(snapshot);
    uis.request_particle_buffer_reload();
    *need_redraw.write().unwrap() = true;
}

/// Renders particle display mode combo box in the Settings panel.
fn combobox_particle_display_mode(ui: &mut egui::Ui, uis: &mut UiState) {
    ui.horizontal(|ui| {
        label_normal(ui, "Particle Display");
        let id = ui.make_persistent_id("particle_display_mode_combobox");
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ComboBox::from_id_salt(id)
                .selected_text(format!("{}", uis.particle_display_mode))
                .width(90.0)
                .show_ui(ui, |ui| {
                    for mode in ParticleDisplayMode::ALL {
                        selectable_value(ui, &mut uis.particle_display_mode, mode);
                    }
                });
        });
    });
}
