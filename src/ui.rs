use crate::initial_condition::InitialCondition;
use crate::simulation::AU;
use crate::ui_state::*;
use crate::ui_styles::*;
use egui::{Button, ComboBox, DragValue, Label, Slider, vec2};
use glam::DVec3;
use std::sync::{Arc, RwLock};

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

fn format_scale(scale_guage: f64, scale: f64) -> String {
    let scale_inv = DEFAULT_SCALE_UI / scale_guage;
    let pow10 = scale_inv.powi(4) * scale;
    if pow10 >= AU * 1e6 {
        format!("{:.3e} au", pow10 / AU)
    } else if pow10 >= AU {
        format!("{:.3} au", pow10 / AU)
    } else if pow10 >= 1e9 {
        format!("{:.3e} km", pow10 / 1e3)
    } else if pow10 >= 1e3 {
        format!("{:.3} km", pow10 / 1e3)
    } else if pow10 < 1e-9 {
        format!("{:.6} nm", pow10 * 1e9)
    } else if pow10 < 1e-3 {
        format!("{:.6} mm", pow10 * 1e3)
    } else {
        format!("{:.3} m", pow10)
    }
}

pub fn draw_ui(ui_state: &Arc<RwLock<UiState>>, ctx: &egui::Context) {
    let mut ui_state_guard = ui_state.write().unwrap();
    egui::Window::new("Control Panel")
        .resizable(false)
        .collapsible(true)
        .default_width(ui_state_guard.input_panel_width)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                label_normal(ui, "FPS");
                label_indicator(ui, &ui_state_guard.fps.to_string());
            });
            ui.horizontal(|ui| {
                label_normal(ui, "Frame");
                label_indicator(ui, &ui_state_guard.frame.to_string());
            });
            ui.horizontal(|ui| {
                label_normal(ui, "Time");
                label_indicator(ui, &format_simulation_time(ui_state_guard.simulation_time));
            });
            ui.separator();
            let button_width = ui.available_width();
            let button_height = ui.spacing().interact_size.y * 1.5;
            let button_size = vec2(button_width, button_height);
            if ui
                .add_sized(button_size, Button::new("Start/Pause"))
                .clicked()
            {
                ui_state_guard.is_running = !ui_state_guard.is_running;
            }
            ui.separator();
            ui.add(Label::new("Initial Condition:"));
            let id_salt = ui.make_persistent_id("initial_condition_combobox");
            ComboBox::from_id_salt(id_salt)
                .selected_text(format!("{}", ui_state_guard.selected_initial_condition))
                .show_ui(ui, |ui| {
                    ui.selectable_value(
                        &mut ui_state_guard.selected_initial_condition,
                        InitialCondition::default(),
                        "Random Sphere",
                    );
                    ui.selectable_value(
                        &mut ui_state_guard.selected_initial_condition,
                        InitialCondition::RandomCube {
                            scale: 1e10,
                            cube_size: 2e10,
                            mass_range: (1e29, 1e31),
                            velocity_std: 1e6,
                        },
                        "Random Cube",
                    );
                    ui.selectable_value(
                        &mut ui_state_guard.selected_initial_condition,
                        InitialCondition::TwoSpheres {
                            scale: 1.0,
                            sphere1_center: DVec3::new(-1.0, 0.0, 0.0),
                            sphere1_radius: 0.5,
                            sphere2_center: DVec3::new(1.0, 0.0, 0.0),
                            sphere2_radius: 0.5,
                            mass_fixed: 1e-1,
                        },
                        "Two Spheres",
                    );
                    ui.selectable_value(
                        &mut ui_state_guard.selected_initial_condition,
                        InitialCondition::SpiralDisk {
                            scale: 1e7,
                            disk_radius: 1.5e7,
                            mass_fixed: 1e20,
                        },
                        "Spiral Disk",
                    );
                    ui.selectable_value(
                        &mut ui_state_guard.selected_initial_condition,
                        InitialCondition::SolarSystem,
                        "Solar System",
                    );
                    ui.selectable_value(
                        &mut ui_state_guard.selected_initial_condition,
                        InitialCondition::SatelliteOrbit {
                            earth_mass: 5.972e24,
                        },
                        "Satellite Orbit",
                    );
                });
            ui.add(Label::new("Simulation Type:"));
            let id_salt = ui.make_persistent_id("simulation_type_combobox");
            ComboBox::from_id_salt(id_salt)
                .selected_text(format!("{:?}", ui_state_guard.simulation_type))
                .show_ui(ui, |ui| {
                    ui.selectable_value(
                        &mut ui_state_guard.simulation_type,
                        SimulationType::Normal,
                        "Normal",
                    );
                    ui.selectable_value(
                        &mut ui_state_guard.simulation_type,
                        SimulationType::Special,
                        "Special",
                    );
                });
            ui.style_mut().spacing.slider_width = 150.0;
            ui.add(Label::new("Particle Count:"));
            let max_particle_count = ui_state_guard.max_particle_count;
            ui.add(Slider::new(
                &mut ui_state_guard.particle_count,
                2..=max_particle_count as u32,
            ));
            if ui.add_sized(button_size, Button::new("Reset")).clicked() {
                ui_state_guard.is_reset_requested = true;
            }
            ui.separator();
            ui.add(
                DragValue::new(&mut ui_state_guard.time_per_frame)
                    .speed(0.1)
                    .prefix("Time(sec)/Frame: "),
            );
            ui.separator();
            ui.horizontal(|ui| {
                label_normal(ui, "Scale (m):");
                label_indicator(
                    ui,
                    format_scale(ui_state_guard.scale_gauge, ui_state_guard.scale).as_str(),
                );
            });
            slider_pure(
                ui,
                &mut ui_state_guard.scale_gauge,
                DEFAULT_SCALE_UI * 0.4..=DEFAULT_SCALE_UI * 3.0,
            );
            ui.separator();
            ui.style_mut().spacing.slider_width = 160.0;
            label_normal(ui, "Max FPS:");
            ui.add(Slider::new(&mut ui_state_guard.max_fps, 1..=1000));
            label_normal(ui, "Skip drawing:");
            ui.add(Slider::new(&mut ui_state_guard.skip, 0..=1000));
        });
}
