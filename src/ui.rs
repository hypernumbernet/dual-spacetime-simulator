use crate::initial_condition::{InitialCondition, InitialConditionType};
use crate::settings::AppSettings;
use crate::simulation::AU;
use crate::ui_state::*;
use crate::ui_styles::*;
use egui::{Checkbox, ComboBox, Slider};
use std::sync::{Arc, RwLock};

const MENU_POPUP_WIDTH: f32 = 180.0;

pub fn draw_ui(ui_state: &Arc<RwLock<UiState>>, settings: &mut AppSettings, ctx: &egui::Context) {
    let mut uis = ui_state.write().unwrap();
    egui::TopBottomPanel::top("menu_bar").show(ctx, |ui| {
        egui::menu::bar(ui, |ui| {
            ui.menu_button("File", |ui| {
                ui.set_min_width(MENU_POPUP_WIDTH);
                if ui.button("Exit").clicked() {
                    uis.request_exit = true;
                    ui.close_menu();
                }
            });

            ui.menu_button("Panel", |ui| {
                ui.set_min_width(MENU_POPUP_WIDTH);
                if ui
                    .checkbox(&mut uis.is_simulation_panel_open, "Simulation")
                    .clicked()
                {
                    ui.close_menu();
                }
                if ui
                    .checkbox(
                        &mut uis.is_initial_condition_panel_open,
                        "Initial Condition",
                    )
                    .clicked()
                {
                    ui.close_menu();
                }
                if ui
                    .checkbox(&mut uis.is_settings_panel_open, "Settings")
                    .clicked()
                {
                    ui.close_menu();
                }
                if ui
                    .checkbox(&mut uis.is_math_graph_panel_open, "Math 3D Graph")
                    .clicked()
                {
                    ui.close_menu();
                }
            });

            ui.menu_button("View", |ui| {
                ui.set_min_width(MENU_POPUP_WIDTH);
                if ui.checkbox(&mut uis.show_grid, "Show Grid").clicked() {
                    ui.close_menu();
                }
            });

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
                    uis.is_reset_requested = true;
                    uis.is_resetting = true;
                    ui.close_menu();
                }
            });

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(format!("Frame {}", uis.frame));
                ui.separator();
                ui.label(format!("FPS {}", uis.fps));
            });
        });
    });

    if uis.is_simulation_panel_open {
        egui::Window::new("Simulation")
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
                if button_normal(ui, "Start/Pause").clicked() {
                    uis.is_running = !uis.is_running;
                }
                ui.separator();
                if button_normal(ui, "Initial Condition").clicked() {
                    uis.is_initial_condition_panel_open = !uis.is_initial_condition_panel_open;
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
                ui.separator();
                if button_normal(ui, "Save Settings").clicked() {
                    settings.window_min_width = uis.min_window_width;
                    settings.window_min_height = uis.min_window_height;
                    settings.max_particle_count = uis.max_particle_count;
                    settings.start_maximized = uis.start_maximized;
                    settings.link_point_size_to_scale = uis.link_point_size_to_scale;
                    settings.lock_camera_up = uis.lock_camera_up;
                    if let Err(e) = settings.save() {
                        eprintln!("Failed to save settings: {}", e);
                    }
                }
            });
    }

    if uis.is_initial_condition_panel_open {
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
                    InitialConditionType::EllipticalOrbit => {
                        condition_elliptical_orbit(ui, &mut uis);
                    }
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

    if uis.is_math_graph_panel_open {
        egui::Window::new("Math 3D Graph")
            .resizable(false)
            .collapsible(true)
            .default_width(uis.input_panel_width)
            .show(ctx, |ui| {
                label_normal(ui, "Graph Type");
                let id = ui.make_persistent_id("math_graph_type_combobox");
                ComboBox::from_id_salt(id)
                    .selected_text(format!("{}", uis.math_graph.graph_type))
                    .width(ui.available_width())
                    .show_ui(ui, |ui| {
                        selectable_value(
                            ui,
                            &mut uis.math_graph.graph_type,
                            MathGraphType::Surface,
                        );
                        selectable_value(
                            ui,
                            &mut uis.math_graph.graph_type,
                            MathGraphType::PointCloud,
                        );
                        selectable_value(
                            ui,
                            &mut uis.math_graph.graph_type,
                            MathGraphType::VectorField,
                        );
                    });

                ui.separator();

                if matches!(uis.math_graph.graph_type, MathGraphType::Surface) {
                    label_normal(ui, "Surface Function");
                    let id = ui.make_persistent_id("math_graph_surface_function_combobox");
                    ComboBox::from_id_salt(id)
                        .selected_text(format!("{}", uis.math_graph.surface_function))
                        .width(ui.available_width())
                        .show_ui(ui, |ui| {
                            selectable_value(
                                ui,
                                &mut uis.math_graph.surface_function,
                                MathGraphSurfaceFunction::SinCos,
                            );
                            selectable_value(
                                ui,
                                &mut uis.math_graph.surface_function,
                                MathGraphSurfaceFunction::Paraboloid,
                            );
                        });
                }

                ui.separator();

                label_normal(ui, "X Range");
                dragvalue_normal(ui, &mut uis.math_graph.x_min, 0.5, "Min");
                dragvalue_normal(ui, &mut uis.math_graph.x_max, 0.5, "Max");

                label_normal(ui, "Y Range");
                dragvalue_normal(ui, &mut uis.math_graph.y_min, 0.5, "Min");
                dragvalue_normal(ui, &mut uis.math_graph.y_max, 0.5, "Max");

                ui.separator();

                ui.horizontal(|ui| {
                    label_normal(ui, "Grid Resolution");
                    let mut res = uis.math_graph.grid_resolution as i32;
                    ui.add(Slider::new(&mut res, 8..=256));
                    uis.math_graph.grid_resolution = res.max(8) as u32;
                });
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
            InitialConditionType::EllipticalOrbit => InitialCondition::EllipticalOrbit {
                scale: uis.elliptical_orbit.scale,
                central_mass: uis.elliptical_orbit.central_mass,
                planetary_mass: uis.elliptical_orbit.planetary_mass,
                planetary_speed: uis.elliptical_orbit.planetary_speed,
                planetary_distance: uis.elliptical_orbit.planetary_distance,
            },
        };
        uis.scale = uis.initial_condition.get_scale();
        if uis.initial_condition_type == InitialConditionType::SolarSystem {
            uis.time_per_frame = 10_000.0;
            uis.max_fps = 1000;
            uis.skip = 10;
        } else if uis.initial_condition_type == InitialConditionType::EllipticalOrbit {
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
}

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

fn condition_elliptical_orbit(ui: &mut egui::Ui, uis: &mut UiState) {
    dragvalue_normal(ui, &mut uis.elliptical_orbit.scale, 1e7, "Scale (m)");
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
    if button_normal(ui, "Reset").clicked() {
        uis.is_reset_requested = true;
        uis.is_resetting = true;
    }
}
