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
    let mut uis = ui_state.write().unwrap();
    egui::Window::new("Control Panel")
        .resizable(false)
        .collapsible(true)
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
            ui.separator();
            let button_width = ui.available_width();
            let button_height = ui.spacing().interact_size.y * 1.5;
            let button_size = vec2(button_width, button_height);
            if ui
                .add_sized(button_size, Button::new("Start/Pause"))
                .clicked()
            {
                uis.is_running = !uis.is_running;
            }
            ui.separator();
            if ui
                .add_sized(button_size, Button::new("Initial Condition"))
                .clicked()
            {
                uis.is_initial_condition_window_open = !uis.is_initial_condition_window_open;
            }
            ui.separator();
            ui.add(
                DragValue::new(&mut uis.time_per_frame)
                    .speed(0.1)
                    .prefix("Time(sec)/Frame: "),
            );
            ui.separator();
            ui.horizontal(|ui| {
                label_normal(ui, "Scale (m):");
                label_indicator(ui, format_scale(uis.scale_gauge, uis.scale).as_str());
            });
            slider_pure(
                ui,
                &mut uis.scale_gauge,
                DEFAULT_SCALE_UI * 0.2..=DEFAULT_SCALE_UI * 3.0,
            );
            ui.separator();
            ui.style_mut().spacing.slider_width = 160.0;
            label_normal(ui, "Max FPS:");
            ui.add(Slider::new(&mut uis.max_fps, 1..=1000));
            label_normal(ui, "Skip drawing:");
            ui.add(Slider::new(&mut uis.skip, 0..=1000));
        });

    if uis.is_initial_condition_window_open {
        egui::Window::new("Initial Condition")
            .resizable(false)
            .collapsible(true)
            .default_width(uis.input_panel_width)
            .show(ctx, |ui| {
                combobox_simulation_type(ui, &mut uis);
                combobox_basic_presets(ui, &mut uis);
                ui.style_mut().spacing.slider_width = 150.0;
                ui.add(Label::new("Particle Count:"));
                let max_particle_count = uis.max_particle_count;
                ui.add(Slider::new(
                    &mut uis.particle_count,
                    2..=max_particle_count as u32,
                ));
                let button_width = ui.available_width();
                let button_height = ui.spacing().interact_size.y * 1.5;
                let button_size = vec2(button_width, button_height);
                if ui.add_sized(button_size, Button::new("Reset")).clicked() {
                    uis.is_reset_requested = true;
                }
            });
    }
    if uis.is_reset_requested {
        uis.scale = uis.selected_initial_condition.get_scale();
        if uis.selected_initial_condition == InitialCondition::SolarSystem {
            uis.time_per_frame = 10000.0;
            uis.max_fps = 1000;
            uis.skip = 10;
        }
        uis.scale_gauge = DEFAULT_SCALE_UI;
    }
}

fn combobox_simulation_type(ui: &mut egui::Ui, state: &mut UiState) {
    ui.add(Label::new("Simulation Type:"));
    let id = ui.make_persistent_id("simulation_type_combobox");
    ComboBox::from_id_salt(id)
        .selected_text(format!("{}", state.simulation_type))
        .width(ui.available_width())
        .show_ui(ui, |ui| {
            selectable_value(ui, &mut state.simulation_type, SimulationType::Normal);
            selectable_value(
                ui,
                &mut state.simulation_type,
                SimulationType::SpeedOfLightLimit,
            );
            selectable_value(
                ui,
                &mut state.simulation_type,
                SimulationType::LorentzTransformation,
            );
        });
}

fn combobox_basic_presets(ui: &mut egui::Ui, state: &mut UiState) {
    ui.add(Label::new("Basic Presets:"));
    let id = ui.make_persistent_id("basic_presets_combobox");
    ComboBox::from_id_salt(id)
        .selected_text(format!("{}", state.selected_initial_condition))
        .width(ui.available_width())
        .show_ui(ui, |ui| {
            selectable_value(
                ui,
                &mut state.selected_initial_condition,
                InitialCondition::default(),
            );
            selectable_value(
                ui,
                &mut state.selected_initial_condition,
                InitialCondition::RandomCube {
                    scale: 1e10,
                    cube_size: 2e10,
                    mass_range: (1e29, 1e31),
                    velocity_std: 1e6,
                },
            );
            selectable_value(
                ui,
                &mut state.selected_initial_condition,
                InitialCondition::TwoSpheres {
                    scale: 1.0,
                    sphere1_center: DVec3::new(-1.0, 0.0, 0.0),
                    sphere1_radius: 0.5,
                    sphere2_center: DVec3::new(1.0, 0.0, 0.0),
                    sphere2_radius: 0.5,
                    mass_fixed: 1e-1,
                },
            );
            selectable_value(
                ui,
                &mut state.selected_initial_condition,
                InitialCondition::SpiralDisk {
                    scale: 1e7,
                    disk_radius: 1.5e7,
                    mass_fixed: 1e20,
                },
            );
            selectable_value(
                ui,
                &mut state.selected_initial_condition,
                InitialCondition::SolarSystem,
            );
            selectable_value(
                ui,
                &mut state.selected_initial_condition,
                InitialCondition::SatelliteOrbit {
                    earth_mass: 5.972e24,
                },
            );
            selectable_value(
                ui,
                &mut state.selected_initial_condition,
                InitialCondition::EllipticalOrbit {
                    central_mass: 1.989e30,
                    planetary_mass: 5.972e24,
                    planetary_speed: 2.0e4,
                    planetary_distance: 2.0e11,
                    scale: 1.5e11,
                },
            );
        });
}
