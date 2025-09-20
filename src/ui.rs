use crate::types::UiState;
use crate::ui_styles::*;
use egui::{DragValue, Label, SidePanel, Slider};

pub fn draw_ui(ui_state: &mut UiState, ctx: &egui::Context) {
    SidePanel::right("input_panel")
        .resizable(false)
        //.default_width(ui_state.input_panel_width)
        .exact_width(ui_state.input_panel_width)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                label_normal(ui, "FPS");
                label_indicator(ui, &ui_state.fps.to_string());
            });
            ui.horizontal(|ui| {
                label_normal(ui, "Frame");
                label_indicator(ui, &ui_state.frame.to_string());
            });
            ui.horizontal(|ui| {
                label_normal(ui, "Time (day)");
                label_indicator_short(ui, &ui_state.frame.to_string());
            });
            ui.horizontal(|ui| {
                label_normal(ui, "Time/Frame(s)");
                label_indicator_short(ui, &ui_state.frame.to_string());
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
