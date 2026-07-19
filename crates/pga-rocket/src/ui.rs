//! Fixed-width left docked parameter panel (egui SidePanel).

use crate::landing::LandingAutopilot;
use crate::sim::{GRAVITY, RocketState};
use crate::target_landing::TargetLandingAutopilot;
use egui::{RichText, ScrollArea};

/// Fixed width of the left parameter panel (logical points).
pub const PANEL_WIDTH: f32 = 196.0;

/// 3D content region to the right of the left UI panel.
///
/// Aspect and left inset stay in one place so the camera frustum and the
/// Vulkan viewport always describe the same rectangle.
#[derive(Clone, Copy, Debug)]
pub struct ContentRegion {
    /// Physical-pixel inset from the left edge of the framebuffer.
    pub left_inset_px: f32,
    /// `(framebuffer_width - left_inset) / framebuffer_height`.
    pub aspect: f32,
}

impl ContentRegion {
    /// Compute the drawable region beside the panel for a framebuffer size.
    pub fn from_framebuffer(width_px: f32, height_px: f32, scale_factor: f32) -> Self {
        let max_inset = (width_px - 1.0).max(0.0);
        let left_inset_px = (PANEL_WIDTH * scale_factor.max(0.0)).clamp(0.0, max_inset);
        let content_w = (width_px - left_inset_px).max(1.0);
        let aspect = content_w / height_px.max(1.0);
        Self {
            left_inset_px,
            aspect,
        }
    }
}

/// Draws the left docked parameter panel with live simulation telemetry.
pub fn draw_params_panel(
    ctx: &egui::Context,
    rocket: &mut RocketState,
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
            ui.checkbox(&mut rocket.moon_mode, "Moon mode");
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
    section(ui, "Flight");
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
    kv(ui, "Speed", format!("{:.2} m/s", rocket.speed()));
    kv(ui, "ω X", format!("{:.3} rad/s", w[0]));
    kv(ui, "ω Y", format!("{:.3} rad/s", w[1]));
    kv(ui, "ω Z", format!("{:.3} rad/s", w[2]));
    kv(ui, "Contact", contact_label(rocket));
    kv(ui, "Foot Y", format!("{:.2} m", rocket.lowest_foot_y()));
    kv(ui, "Probe Y", format!("{:.2} m", rocket.lowest_probe_y()));
}

fn contact_label(rocket: &RocketState) -> String {
    if rocket.destroyed {
        format!("DESTROYED\n{:.1} m/s", rocket.last_impact_speed)
    } else if rocket.contacting {
        if rocket.body_contacting {
            "yes body".into()
        } else {
            "yes feet".into()
        }
    } else {
        "no".into()
    }
}

fn control_section(
    ui: &mut egui::Ui,
    rocket: &RocketState,
    landing: &LandingAutopilot,
    target_landing: &TargetLandingAutopilot,
) {
    section(ui, "Control");
    kv(ui, "Land (L)", landing.status_label());
    kv(ui, "Target (T)", target_landing.status_label());
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
    kv(ui, "Max thrust", format!("{:.0} N", rocket.params.max_thrust));
    kv(ui, "T/W", format!("{:.2}", tw));
    kv(ui, "Eng τ", fmt_xyz0(eng.torque));
    kv(ui, "RCS τ_y", format!("{:.0} N·m", rcs.torque[1]));
}

fn vehicle_section(ui: &mut egui::Ui, rocket: &RocketState) {
    section(ui, "Vehicle");
    let p = &rocket.params;
    kv(ui, "Mass", format!("{:.0} kg", p.mass));
    kv(ui, "Half-H", format!("{:.2} m", p.body_half_height));
    kv(ui, "Radius", format!("{:.2} m", p.body_radius));
    kv(ui, "Nozzle", format!("{:.2} m", p.nozzle_length));
    kv(ui, "Leg clr", format!("{:.2} m", p.leg_clearance));
    kv(ui, "Inertia", fmt_xyz0(p.inertia));
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
    section(ui, "Camera");
    kv(ui, "FPS", format!("{:.0}", fps));
    kv(ui, "Yaw", format!("{:.2} rad", cam_yaw));
    kv(ui, "Pitch", format!("{:.2} rad", cam_pitch));
    kv(ui, "Distance", format!("{:.1} m", cam_distance));
}

fn help_section(ui: &mut egui::Ui) {
    section(ui, "Controls");
    for line in [
        "Space / Ctrl: hold throttle",
        "F: full (0.2s latch)",
        "C: cut (0.2s latch)",
        "W/S: pitch (needs thr)",
        "Q/E: yaw (needs thr)",
        "A/D: roll RCS",
        "L: auto-land",
        "T: land at T mark",
        "R: reset",
        "LMB/RMB drag: orbit",
        "Wheel/PgUp-Dn: zoom",
        "Arrows: orbit · Esc: quit",
    ] {
        ui.label(line);
    }
}

fn section(ui: &mut egui::Ui, title: &str) {
    ui.label(RichText::new(title).strong());
}

fn fmt_xyz0(v: [f64; 3]) -> String {
    format!("{:.0},{:.0},{:.0}", v[0], v[1], v[2])
}

/// Label + monospace value: one row when it fits, otherwise value under the key.
fn kv(ui: &mut egui::Ui, key: &str, value: impl AsRef<str>) {
    let value = value.as_ref();
    if fits_one_line(ui, key, value) {
        ui.horizontal(|ui| {
            ui.label(key);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.monospace(value);
            });
        });
    } else {
        ui.label(key);
        for line in value.lines() {
            ui.horizontal(|ui| {
                ui.add_space(8.0);
                ui.monospace(line);
            });
        }
    }
}

fn fits_one_line(ui: &egui::Ui, key: &str, value: &str) -> bool {
    if value.contains('\n') {
        return false;
    }
    let mono = egui::TextStyle::Monospace.resolve(ui.style());
    let body = egui::TextStyle::Body.resolve(ui.style());
    let value_w = ui
        .painter()
        .layout_no_wrap(value.to_owned(), mono, egui::Color32::WHITE)
        .size()
        .x;
    let key_w = ui
        .painter()
        .layout_no_wrap(key.to_owned(), body, egui::Color32::WHITE)
        .size()
        .x;
    const GAP: f32 = 6.0;
    key_w + GAP + value_w <= ui.available_width()
}
