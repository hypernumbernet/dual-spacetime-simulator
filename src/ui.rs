use crate::initial_condition::{InitialCondition, InitialConditionType};
use crate::simulation::AU;
use crate::ui_state::*;
use crate::ui_styles::*;
use egui::{Button, ComboBox, Label, Slider, vec2};
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
        });

    if uis.is_initial_condition_window_open {
        egui::Window::new("Initial Condition")
            .resizable(false)
            .collapsible(true)
            .default_width(uis.input_panel_width)
            .show(ctx, |ui| {
                combobox_simulation_type(ui, &mut uis);
                ui.separator();
                combobox_initial_condition_type(ui, &mut uis);
                match uis.initial_condition_type {
                    InitialConditionType::RandomSphere => {
                        condition_random_sphere(ui, &mut uis);
                    }
                    InitialConditionType::RandomCube => {
                        condition_random_cube(ui, &mut uis);
                    }
                    InitialConditionType::TwoSpheres => {}
                    InitialConditionType::SpiralDisk => {}
                    InitialConditionType::SolarSystem => {}
                    InitialConditionType::SatelliteOrbit => {}
                    InitialConditionType::EllipticalOrbit => {}
                }
                ui.separator();
                slider_perticle_count(ui, &mut uis);
                ui.separator();
                button_reset(ui, &mut uis);
            });
    }
    if uis.initial_condition_type != uis.previous_initial_condition_type {
        uis.initial_condition = match uis.initial_condition_type {
            InitialConditionType::RandomSphere => InitialCondition::RandomSphere {
                scale: uis.random_sphere.scale,
                radius: uis.random_sphere.radius,
                mass_range: uis.random_sphere.mass_range,
                velocity_std: uis.random_sphere.velocity_std,
            },
            InitialConditionType::RandomCube => InitialCondition::RandomCube {
                scale: uis.random_cube.scale,
                cube_size: uis.random_cube.cube_size,
                mass_range: uis.random_cube.mass_range,
                velocity_std: uis.random_cube.velocity_std,
            },
            InitialConditionType::TwoSpheres => InitialCondition::default(),
            InitialConditionType::SpiralDisk => InitialCondition::default(),
            InitialConditionType::SolarSystem => InitialCondition::SolarSystem,
            InitialConditionType::SatelliteOrbit => InitialCondition::default(),
            InitialConditionType::EllipticalOrbit => InitialCondition::default(),
        };
        uis.previous_initial_condition_type = uis.initial_condition_type.clone();
    }
    if uis.is_reset_requested {
        uis.scale = uis.initial_condition.get_scale();
        if uis.initial_condition == InitialCondition::SolarSystem {
            uis.time_per_frame = 10000.0;
            uis.max_fps = 1000;
            uis.skip = 10;
        }
        uis.scale_gauge = DEFAULT_SCALE_UI;
    }
}

fn combobox_simulation_type(ui: &mut egui::Ui, uis: &mut UiState) {
    label_normal(ui, "Simulation Type");
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
}

fn combobox_initial_condition_type(ui: &mut egui::Ui, uis: &mut UiState) {
    label_normal(ui, "Initial Condition Type");
    let id = ui.make_persistent_id("initial_condition_type_combobox");
    ComboBox::from_id_salt(id)
        .selected_text(format!("{}", uis.initial_condition_type))
        .width(ui.available_width())
        .show_ui(ui, |ui| {
            selectable_value(
                ui,
                &mut uis.initial_condition_type,
                InitialConditionType::RandomSphere,
            );
            selectable_value(
                ui,
                &mut uis.initial_condition_type,
                InitialConditionType::RandomCube,
            );
            selectable_value(
                ui,
                &mut uis.initial_condition_type,
                InitialConditionType::TwoSpheres,
            );
            selectable_value(
                ui,
                &mut uis.initial_condition_type,
                InitialConditionType::SpiralDisk,
            );
            selectable_value(
                ui,
                &mut uis.initial_condition_type,
                InitialConditionType::SolarSystem,
            );
            selectable_value(
                ui,
                &mut uis.initial_condition_type,
                InitialConditionType::SatelliteOrbit,
            );
            selectable_value(
                ui,
                &mut uis.initial_condition_type,
                InitialConditionType::EllipticalOrbit,
            );
        });
}

fn condition_random_sphere(ui: &mut egui::Ui, uis: &mut UiState) {
    dragvalue_normal(ui, &mut uis.random_sphere.scale, 1e9, "Scale (m): ");
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

fn condition_random_cube(ui: &mut egui::Ui, uis: &mut UiState) {
    dragvalue_normal(ui, &mut uis.random_cube.scale, 1e3, "Scale (m)");
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

fn combobox_basic_presets(ui: &mut egui::Ui, uis: &mut UiState) {
    ui.add(Label::new("Basic Presets:"));
    let id = ui.make_persistent_id("basic_presets_combobox");
    ComboBox::from_id_salt(id)
        .selected_text(format!("{}", uis.initial_condition))
        .width(ui.available_width())
        .show_ui(ui, |ui| {
            selectable_value(ui, &mut uis.initial_condition, InitialCondition::default());
            selectable_value(
                ui,
                &mut uis.initial_condition,
                InitialCondition::RandomCube {
                    scale: 1e10,
                    cube_size: 2e10,
                    mass_range: (1e29, 1e31),
                    velocity_std: 1e6,
                },
            );
            selectable_value(
                ui,
                &mut uis.initial_condition,
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
                &mut uis.initial_condition,
                InitialCondition::SpiralDisk {
                    scale: 1e7,
                    disk_radius: 1.5e7,
                    mass_fixed: 1e20,
                },
            );
            selectable_value(
                ui,
                &mut uis.initial_condition,
                InitialCondition::SolarSystem,
            );
            selectable_value(
                ui,
                &mut uis.initial_condition,
                InitialCondition::SatelliteOrbit {
                    earth_mass: 5.972e24,
                },
            );
            selectable_value(
                ui,
                &mut uis.initial_condition,
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

fn slider_perticle_count(ui: &mut egui::Ui, uis: &mut UiState) {
    ui.style_mut().spacing.slider_width = 150.0;
    label_normal(ui, "Particle Count");
    let max_particle_count = uis.max_particle_count;
    ui.add(Slider::new(
        &mut uis.particle_count,
        2..=max_particle_count as u32,
    ));
}

fn button_reset(ui: &mut egui::Ui, uis: &mut UiState) {
    let button_width = ui.available_width();
    let button_height = ui.spacing().interact_size.y * 1.5;
    let button_size = vec2(button_width, button_height);
    if ui.add_sized(button_size, Button::new("Reset")).clicked() {
        uis.is_reset_requested = true;
    }
}
