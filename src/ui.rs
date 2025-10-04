use crate::types::UiState;
use crate::ui_styles::*;
use egui::{Button, DragValue, Label, Slider, vec2};

pub fn draw_ui(ui_state: &mut UiState, ctx: &egui::Context) {
    egui::Window::new("Control Panel")
        .resizable(false)
        .collapsible(true)
        .default_width(ui_state.input_panel_width)
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
                label_indicator_short(ui, &ui_state.simulation_time.to_string());
            });
            ui.horizontal(|ui| {
                label_normal(ui, "Time/Frame(s)");
                label_indicator_short(ui, &ui_state.time_per_frame.to_string());
            });
            ui.separator();
            let button_width = ui.available_width();
            let button_height = ui.spacing().interact_size.y * 1.5;
            let button_size = vec2(button_width, button_height);
            if ui
                .add_sized(button_size, Button::new("Start/Pause"))
                .clicked()
            {
                ui_state.is_running = !ui_state.is_running;
            }
            if ui.add_sized(button_size, Button::new("Reset")).clicked() {}
            ui.separator();
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
            ui.add(Label::new("Max FPS:"));
            ui.horizontal(|ui| {
                ui.checkbox(&mut ui_state.unlimited_fps, "∞");
                if !ui_state.unlimited_fps {
                    ui.add(Slider::new(&mut ui_state.max_fps, 1..=120));
                }
            });
        });
}
