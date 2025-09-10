use crate::types::UiState;
use egui::SidePanel;

pub fn draw_ui(ui_state: &mut UiState, ctx: &egui::Context) {
    SidePanel::right("input_panel")
        .resizable(false)
        .exact_width(ui_state.input_panel_width)
        //.default_width(ui_state.input_panel_width)
        .show(ctx, |ui| {
            ui.heading("Input Panel");
            ui.add(egui::Label::new("Particle Count:"));
            ui.add(
                egui::Slider::new(&mut ui_state.particle_count, 2..=ui_state.max_particle_count as u32)
            );
            ui.add(
                egui::DragValue::new(&mut ui_state.gravity)
                    .speed(0.1)
                    .prefix("Gravity: "),
            );
        });
}
