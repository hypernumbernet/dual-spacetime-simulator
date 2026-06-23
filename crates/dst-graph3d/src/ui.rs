use crate::graph3d::GraphType;
use crate::settings::AppSettings;
use crate::ui_state::*;
use crate::ui_styles::*;
use egui::{Checkbox, ComboBox, Slider};
use std::sync::{Arc, RwLock};

const MENU_POPUP_WIDTH: f32 = 180.0;
const PANEL_DEFAULT_X: f32 = 16.0;
const PANEL_MENU_OFFSET_Y: f32 = 16.0;

/// Draws the full control UI and applies user edits to shared UI state.
pub fn draw_ui(
    ui_state: &Arc<RwLock<UiState>>,
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

                ui.menu_button("Panel", |ui| {
                    ui.set_min_width(MENU_POPUP_WIDTH);
                    let available = uis.get_available_panels();
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

    if uis.is_settings_panel_open {
        egui::Window::new("Settings")
            .resizable(false)
            .collapsible(true)
            .default_width(uis.input_panel_width)
            .show(ctx, |ui| {
                dragvalue_normal(ui, &mut uis.min_window_width, 1.0, "Min Window Width");
                dragvalue_normal(ui, &mut uis.min_window_height, 1.0, "Min Window Height");
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
                    settings.start_maximized = uis.start_maximized;
                    settings.link_point_size_to_scale = uis.link_point_size_to_scale;
                    settings.lock_camera_up = uis.lock_camera_up;
                    settings.mailbox_present_mode = uis.mailbox_present_mode;
                    settings.particle_display_mode = uis.particle_display_mode;
                    if let Err(e) = settings.save() {
                        eprintln!("Failed to save settings: {}", e);
                    }
                }
            });
    }

    if uis.is_graph3d_panel_open {
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
