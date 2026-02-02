use crate::initial_condition::{InitialCondition, InitialConditionType};
use crate::simulation::AU;
use crate::ui_state::*;
use crate::ui_styles::*;
use egui::{Button, ComboBox, Slider, vec2};
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
                    InitialConditionType::TwoSpheres => {
                        condition_two_spheres(ui, &mut uis);
                    }
                    InitialConditionType::SpiralDisk => {
                        condition_spiral_disk(ui, &mut uis);
                    }
                    InitialConditionType::SolarSystem => {
                        condition_solar_system(ui, &mut uis);
                    }
                    InitialConditionType::SatelliteOrbit => {
                        condition_satellite_orbit(ui, &mut uis);
                    }
                    InitialConditionType::EllipticalOrbit => {}
                }
                ui.separator();
                slider_perticle_count(ui, &mut uis);
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

            InitialConditionType::TwoSpheres => InitialCondition::TwoSpheres {
                scale: uis.two_spheres.scale,
                sphere1_center: uis.two_spheres.sphere1_center,
                sphere1_radius: uis.two_spheres.sphere1_radius,
                sphere2_center: uis.two_spheres.sphere2_center,
                sphere2_radius: uis.two_spheres.sphere2_radius,
                mass_fixed: uis.two_spheres.mass_fixed,
            },
            InitialConditionType::SpiralDisk => InitialCondition::SpiralDisk {
                scale: uis.spiral_disk.scale,
                disk_radius: uis.spiral_disk.disk_radius,
                mass_fixed: uis.spiral_disk.mass_fixed,
            },
            InitialConditionType::SolarSystem => InitialCondition::SolarSystem {
                start_year: uis.solar_system.start_year,
                start_month: uis.solar_system.start_month,
                start_day: uis.solar_system.start_day,
                start_hour: uis.solar_system.start_hour,
            },
            InitialConditionType::SatelliteOrbit => InitialCondition::SatelliteOrbit {
                orbit_altitude_min: uis.satellite_orbit.orbit_altitude_min,
                orbit_altitude_max: uis.satellite_orbit.orbit_altitude_max,
                asteroid_mass: uis.satellite_orbit.asteroid_mass,
                asteroid_distance: uis.satellite_orbit.asteroid_distance,
                asteroid_speed: uis.satellite_orbit.asteroid_speed,
            },
            InitialConditionType::EllipticalOrbit => InitialCondition::default(),
        };
        uis.scale = uis.initial_condition.get_scale();
        if uis.initial_condition_type == InitialConditionType::SolarSystem {
            uis.time_per_frame = 10000.0;
            uis.max_fps = 1000;
            uis.skip = 10;
        } else {
            uis.time_per_frame = 10.0;
            uis.max_fps = 60;
            uis.skip = 0;
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

fn condition_two_spheres(ui: &mut egui::Ui, uis: &mut UiState) {
    dragvalue_normal(ui, &mut uis.two_spheres.scale, 1e9, "Scale (m)");
    label_normal(ui, "Sphere 1 Center");
    dragvalue_normal(ui, &mut uis.two_spheres.sphere1_center.x, 1.0, "X");
    dragvalue_normal(ui, &mut uis.two_spheres.sphere1_center.y, 1.0, "Y");
    dragvalue_normal(ui, &mut uis.two_spheres.sphere1_center.z, 1.0, "Z");
    dragvalue_normal(
        ui,
        &mut uis.two_spheres.sphere1_radius,
        1e8,
        "Sphere 1 Radius",
    );
    label_normal(ui, "Sphere 2 Center");
    dragvalue_normal(ui, &mut uis.two_spheres.sphere2_center.x, 1.0, "X");
    dragvalue_normal(ui, &mut uis.two_spheres.sphere2_center.y, 1.0, "Y");
    dragvalue_normal(ui, &mut uis.two_spheres.sphere2_center.z, 1.0, "Z");
    dragvalue_normal(
        ui,
        &mut uis.two_spheres.sphere2_radius,
        1e8,
        "Sphere 2 Radius",
    );
    dragvalue_normal(ui, &mut uis.two_spheres.mass_fixed, 1e20, "Mass Fixed (kg)");
}

fn condition_spiral_disk(ui: &mut egui::Ui, uis: &mut UiState) {
    dragvalue_normal(ui, &mut uis.spiral_disk.scale, 1e7, "Scale (m)");
    dragvalue_normal(ui, &mut uis.spiral_disk.disk_radius, 1e7, "Disk Radius (m)");
    dragvalue_normal(ui, &mut uis.spiral_disk.mass_fixed, 1e20, "Mass Fixed (kg)");
}

fn condition_solar_system(ui: &mut egui::Ui, uis: &mut UiState) {
    dragvalue_normal(ui, &mut uis.solar_system.start_year, 1, "Year");
    dragvalue_normal(ui, &mut uis.solar_system.start_month, 1, "Month");
    dragvalue_normal(ui, &mut uis.solar_system.start_day, 1, "Day");
    dragvalue_normal(ui, &mut uis.solar_system.start_hour, 1, "Hour");
}

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
        uis.is_resetting = true;
    }
}
