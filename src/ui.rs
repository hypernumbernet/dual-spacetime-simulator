use crate::types::UiState;
use egui::SidePanel;

pub fn draw_ui(ui_state: &mut UiState, ctx: &egui::Context) {
    SidePanel::right("input_panel")
        .resizable(false)
        .default_width(200.0)
        .show(ctx, |ui| {
            ui.heading("Input Panel");
            ui.add(
                egui::Slider::new(&mut ui_state.particle_count, 0..=1000).text("Particle Count"),
            );
            ui.add(
                egui::DragValue::new(&mut ui_state.gravity)
                    .speed(0.1)
                    .prefix("Gravity: "),
            );
        });
}
