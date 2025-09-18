use crate::types::UiState;
use crate::ui_styles::styles::*;
use egui::{Align, DragValue, Grid, Label, Layout, RichText, SidePanel, Slider};

pub fn draw_ui(ui_state: &mut UiState, ctx: &egui::Context) {
    SidePanel::right("input_panel")
        .resizable(false)
        .default_width(ui_state.input_panel_width)
        .show(ctx, |ui| {
            ui.heading("Input Panel");
            ui.horizontal(|ui| {
                ui.label("FPS");
                label_indicator(ui, "1000");
            });
            Grid::new("particle_grid").striped(true).show(ui, |ui| {
                ui.label("Parameter");
                ui.with_layout(Layout::right_to_left(Align::RIGHT), |ui| {
                    ui.label(
                        RichText::new("200.03")
                            .color(egui::Color32::LIGHT_BLUE)
                            .size(16.0),
                    );
                });
                ui.end_row();
                ui.label("Mass");
                ui.with_layout(Layout::right_to_left(Align::RIGHT), |ui| {
                    ui.label("1");
                });
                ui.end_row();
                ui.label("Force");
                ui.with_layout(Layout::right_to_left(Align::RIGHT), |ui| {
                    ui.label("2");
                });
                ui.end_row();
            });
            ui.horizontal(|ui| {
                ui.label("Frames");
                ui.with_layout(Layout::right_to_left(Align::RIGHT), |ui| {
                    ui.label("0");
                });
            });
            ui.add(Label::new("Particle Count:"));
            ui.add(Slider::new(
                &mut ui_state.particle_count,
                2..=ui_state.max_particle_count as u32,
            ));
            ui.add(
                DragValue::new(&mut ui_state.gravity)
                    .speed(0.1)
                    .prefix("Gravity: "),
            );
        });
}
