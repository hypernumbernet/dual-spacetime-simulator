use crate::ui_state::*;
use crate::ui_styles::*;
use egui::{Button, DragValue, Label, Slider, vec2};
use std::sync::{Arc, RwLock};
use crate::simulation::AU;

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

fn format_scale(scale: f64) -> String {
    let scale_inv = DEFAULT_SCALE_UI / scale;
    let pow10 = scale_inv.powi(4) * 1e10;
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
    let mut ui_state_guard = ui_state.write().unwrap();
    egui::Window::new("Control Panel")
        .resizable(false)
        .collapsible(true)
        .default_width(ui_state_guard.input_panel_width)
        .show(ctx, |ui| {
            ui.style_mut().spacing.slider_width = 140.0;
            ui.horizontal(|ui| {
                label_normal(ui, "FPS");
                label_indicator(ui, &ui_state_guard.fps.to_string());
            });
            ui.horizontal(|ui| {
                label_normal(ui, "Frame");
                label_indicator(ui, &ui_state_guard.frame.to_string());
            });
            ui.horizontal(|ui| {
                label_normal(ui, "Time");
                label_indicator(ui, &format_simulation_time(ui_state_guard.simulation_time));
            });
            ui.separator();
            let button_width = ui.available_width();
            let button_height = ui.spacing().interact_size.y * 1.5;
            let button_size = vec2(button_width, button_height);
            if ui
                .add_sized(button_size, Button::new("Start/Pause"))
                .clicked()
            {
                ui_state_guard.is_running = !ui_state_guard.is_running;
            }
            if ui.add_sized(button_size, Button::new("Reset")).clicked() {
                ui_state_guard.is_reset_requested = true;
            }
            ui.separator();
            ui.add(Label::new("Particle Count:"));
            let max_particle_count = ui_state_guard.max_particle_count;
            ui.add(Slider::new(
                &mut ui_state_guard.particle_count,
                2..=max_particle_count as u32,
            ));
            ui.add(
                DragValue::new(&mut ui_state_guard.time_per_frame)
                    .speed(0.1)
                    .prefix("Time(sec)/Frame: "),
            );
            ui.separator();
            ui.horizontal(|ui| {
                label_normal(ui, "Scale (m):");
                label_indicator(ui, format_scale(ui_state_guard.scale).as_str());
            });
            slider_pure(ui, &mut ui_state_guard.scale, DEFAULT_SCALE_UI * 0.4..=DEFAULT_SCALE_UI * 3.0);
            ui.separator();
            ui.style_mut().spacing.slider_width = 140.0;
            label_normal(ui, "Max FPS:");
            ui.checkbox(&mut ui_state_guard.unlimited_fps, "∞");
            if !ui_state_guard.unlimited_fps {
                ui.add(Slider::new(&mut ui_state_guard.max_fps, 1..=120));
            }
        });
}
