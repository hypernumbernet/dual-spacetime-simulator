//! Fixed-width left docked parameter panel (egui SidePanel).

use crate::landing::LandingAutopilot;
use crate::sim::{GRAVITY, RocketState};
use crate::target_landing::TargetLandingAutopilot;
use egui::{RichText, ScrollArea};

/// Fixed width of the left parameter panel (logical points).
/// 70% of the previous 280 pt so more of the 3D view stays visible.
pub const PANEL_WIDTH: f32 = 196.0;

/// Draws the left docked parameter panel with live simulation telemetry.
pub fn draw_params_panel(
    ctx: &egui::Context,
    rocket: &RocketState,
    landing: &LandingAutopilot,
    target_landing: &TargetLandingAutopilot,
    fps: f32,
    cam_yaw: f32,
    cam_pitch: f32,
    cam_distance: f32,
    target_xz: [f32; 2],
) {
    egui::SidePanel::left("params")
        .exact_width(PANEL_WIDTH)
        .resizable(false)
        .show(ctx, |ui| {
            ui.heading("PGA Rocket");
            ui.separator();

            ScrollArea::vertical()
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    flight_section(ui, rocket, target_xz);
                    ui.separator();
                    control_section(ui, rocket, landing, target_landing);
                    ui.separator();
                    vehicle_section(ui, rocket);
                    ui.separator();
                    camera_section(ui, fps, cam_yaw, cam_pitch, cam_distance);
                    ui.separator();
                    help_section(ui);
                });
        });
}

fn flight_section(ui: &mut egui::Ui, rocket: &RocketState, target_xz: [f32; 2]) {
    ui.label(RichText::new("Flight").strong());
    let p = rocket.position();
    let v = rocket.velocity;
    let w = rocket.omega;
    let range = ((p[0] as f32 - target_xz[0]).powi(2) + (p[2] as f32 - target_xz[1]).powi(2))
        .sqrt();
    kv(ui, "Alt", format!("{:.2} m", p[1]));
    kv(ui, "CoM X", format!("{:.2} m", p[0]));
    kv(ui, "CoM Y", format!("{:.2} m", p[1]));
    kv(ui, "CoM Z", format!("{:.2} m", p[2]));
    kv(ui, "Target X", format!("{:.1} m", target_xz[0]));
    kv(ui, "Target Z", format!("{:.1} m", target_xz[1]));
    kv(ui, "Range", format!("{:.1} m", range));
    kv(ui, "Vel X", format!("{:.2} m/s", v[0]));
    kv(ui, "Vel Y", format!("{:.2} m/s", v[1]));
    kv(ui, "Vel Z", format!("{:.2} m/s", v[2]));
    kv(ui, "ω X", format!("{:.3} rad/s", w[0]));
    kv(ui, "ω Y", format!("{:.3} rad/s", w[1]));
    kv(ui, "ω Z", format!("{:.3} rad/s", w[2]));
    kv(
        ui,
        "Contact",
        if rocket.destroyed {
            format!("DESTROYED\n{:.1} m/s", rocket.last_impact_speed)
        } else if rocket.contacting {
            if rocket.body_contacting {
                "yes body".to_string()
            } else {
                "yes feet".to_string()
            }
        } else {
            "no".to_string()
        },
    );
    kv(ui, "Foot Y", format!("{:.2} m", rocket.lowest_foot_y()));
    kv(ui, "Probe Y", format!("{:.2} m", rocket.lowest_probe_y()));
}

fn control_section(
    ui: &mut egui::Ui,
    rocket: &RocketState,
    landing: &LandingAutopilot,
    target_landing: &TargetLandingAutopilot,
) {
    ui.label(RichText::new("Control").strong());
    kv(ui, "Land (L)", landing.status_label().to_string());
    kv(ui, "Target (T)", target_landing.status_label().to_string());
    let cmd = rocket.command;
    let thrust = rocket.thrust_newtons();
    let weight = rocket.params.mass * GRAVITY;
    let tw = if weight > 0.0 { thrust / weight } else { 0.0 };
    let (gp, gy) = rocket.gimbal_angles();
    let eng = rocket.engine_wrench_body();
    let rcs = rocket.rcs_wrench_body();
    kv(ui, "Throttle", format!("{:.0}%", cmd.throttle * 100.0));
    kv(ui, "Pitch cmd", format!("{:.2}", cmd.pitch));
    kv(ui, "Yaw cmd", format!("{:.2}", cmd.yaw));
    kv(ui, "Roll cmd", format!("{:.2}", cmd.roll));
    kv(
        ui,
        "Gimbal",
        format!("{:.1}°/{:.1}°", gp.to_degrees(), gy.to_degrees()),
    );
    kv(ui, "Thrust", format!("{:.0} N", thrust));
    kv(
        ui,
        "Max thrust",
        format!("{:.0} N", rocket.params.max_thrust),
    );
    kv(ui, "T/W", format!("{:.2}", tw));
    kv(
        ui,
        "Eng τ",
        format!(
            "{:.0},{:.0},{:.0}",
            eng.torque[0], eng.torque[1], eng.torque[2]
        ),
    );
    kv(ui, "RCS τ_y", format!("{:.0} N·m", rcs.torque[1]));
}

fn vehicle_section(ui: &mut egui::Ui, rocket: &RocketState) {
    ui.label(RichText::new("Vehicle").strong());
    let p = &rocket.params;
    kv(ui, "Mass", format!("{:.0} kg", p.mass));
    kv(ui, "Half-H", format!("{:.2} m", p.body_half_height));
    kv(ui, "Radius", format!("{:.2} m", p.body_radius));
    kv(ui, "Nozzle", format!("{:.2} m", p.nozzle_length));
    kv(ui, "Leg clr", format!("{:.2} m", p.leg_clearance));
    kv(
        ui,
        "Inertia",
        format!(
            "{:.0},{:.0},{:.0}",
            p.inertia[0], p.inertia[1], p.inertia[2]
        ),
    );
    kv(
        ui,
        "Max gimb",
        format!("{:.1}°", p.max_gimbal_angle.to_degrees()),
    );
    kv(ui, "Exit Y", format!("{:.2} m", p.nozzle_exit_y()));
    kv(ui, "RCS thr", format!("{:.0} N/ea", p.rcs_thrust));
    kv(ui, "RCS radius", format!("{:.2} m", p.rcs_radius));
    kv(ui, "Contact k", format!("{:.0}", p.contact_stiffness));
    kv(ui, "Contact c", format!("{:.0}", p.contact_damping));
    kv(ui, "Foot μ", format!("{:.2}", p.friction_mu));
    kv(ui, "Body μ", format!("{:.2}", p.body_friction_mu));
    kv(ui, "Restit.", format!("{:.2}", p.restitution));
    kv(ui, "Crash v", format!("{:.1} m/s", p.crash_impact_speed));
    kv(ui, "Slip eps", format!("{:.3} m/s", p.friction_slip_eps));
}

fn camera_section(
    ui: &mut egui::Ui,
    fps: f32,
    cam_yaw: f32,
    cam_pitch: f32,
    cam_distance: f32,
) {
    ui.label(RichText::new("Camera").strong());
    kv(ui, "FPS", format!("{:.0}", fps));
    kv(ui, "Yaw", format!("{:.2} rad", cam_yaw));
    kv(ui, "Pitch", format!("{:.2} rad", cam_pitch));
    kv(ui, "Distance", format!("{:.1} m", cam_distance));
}

fn help_section(ui: &mut egui::Ui) {
    ui.label(RichText::new("Controls").strong());
    ui.label("Space / Ctrl: throttle");
    ui.label("W/S: pitch (needs thr)");
    ui.label("Q/E: yaw (needs thr)");
    ui.label("A/D: roll RCS");
    ui.label("L: auto-land");
    ui.label("T: land at T mark");
    ui.label("R: reset");
    ui.label("LMB/RMB drag: orbit");
    ui.label("Wheel/PgUp-Dn: zoom");
    ui.label("Arrows: orbit · Esc: quit");
}

/// Label + monospace value: one row when it fits, otherwise value on the next line.
fn kv(ui: &mut egui::Ui, key: &str, value: String) {
    if value.contains('\n') {
        ui.label(key);
        for line in value.lines() {
            ui.horizontal(|ui| {
                ui.add_space(8.0);
                ui.monospace(line);
            });
        }
        return;
    }

    let mono = egui::TextStyle::Monospace.resolve(ui.style());
    let body = egui::TextStyle::Body.resolve(ui.style());
    let value_width = ui
        .painter()
        .layout_no_wrap(value.clone(), mono, egui::Color32::WHITE)
        .size()
        .x;
    let key_width = ui
        .painter()
        .layout_no_wrap(key.to_owned(), body, egui::Color32::WHITE)
        .size()
        .x;
    // Reserve a small gap between label and value.
    let gap = 6.0;
    let fits_one_line = key_width + gap + value_width <= ui.available_width();

    if fits_one_line {
        ui.horizontal(|ui| {
            ui.label(key);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.monospace(value);
            });
        });
    } else {
        ui.label(key);
        ui.horizontal(|ui| {
            ui.add_space(8.0);
            ui.monospace(value);
        });
    }
}
