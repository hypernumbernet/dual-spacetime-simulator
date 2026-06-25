use crate::ui_state::trim_trailing_zeros;
use egui::Response;
use egui::{Align, Color32, FontId, Frame, Layout, Margin, RichText, Shape, Stroke, TextStyle, Ui};
use egui::{Button, vec2};

#[derive(Default, Clone)]
struct LabelStyle {
    font_size: f32,
    text_color: Option<Color32>,
    bg_color: Option<Color32>,
    border_color: Option<Color32>,
    border_width: f32,
    width: f32,
    height: f32,
    padding: Option<Margin>,
}

fn draw_label_with_style(ui: &mut Ui, text: &str, style: &LabelStyle) {
    let stroke = if style.border_width > 0.0 {
        Some(Stroke::new(
            style.border_width,
            style.border_color.unwrap_or(Color32::WHITE),
        ))
    } else {
        None
    };
    let frame = Frame::default();
    let frame = if let Some(bg_color) = style.bg_color {
        frame.fill(bg_color)
    } else {
        frame
    };
    let frame = if let Some(padding) = style.padding {
        frame.inner_margin(padding)
    } else {
        frame
    };
    let frame = if let Some(stroke) = stroke {
        frame.stroke(stroke)
    } else {
        frame
    };
    let rich_text = RichText::new(text);
    let rich_text = if let Some(color) = style.text_color {
        rich_text.color(color)
    } else {
        rich_text
    };
    let rich_text = if style.font_size > 0.0 {
        rich_text.size(style.font_size)
    } else {
        rich_text
    };
    frame.show(ui, |ui| {
        if style.width > 0.0 {
            ui.set_min_width(style.width);
        }
        if style.height > 0.0 {
            ui.set_min_height(style.height);
        }
        ui.label(rich_text);
    });
}

fn format_drag_value(value: f64) -> String {
    if !value.is_finite() {
        return format!("{value}");
    }
    if value == 0.0 {
        return "0".to_string();
    }
    let abs = value.abs();
    if abs >= 1e6 || abs <= 1e-4 {
        return format!("{:.3e}", value);
    }
    let exponent = abs.log10().floor() as i32;
    let decimals = (5 - exponent).clamp(0, 12) as usize;
    trim_trailing_zeros(&format!("{:.*}", decimals, value))
}

/// Draws a styled drag-value row with label, custom parsing, and scientific fallback formatting.
pub fn dragvalue_normal<T: egui::emath::Numeric>(
    ui: &mut Ui,
    value: &mut T,
    speed: impl Into<f64>,
    prefix: &str,
) {
    ui.horizontal(|ui| {
        label_normal(ui, prefix);
        ui.scope(|ui| {
            let visuals = ui.visuals_mut();
            visuals.widgets.inactive.weak_bg_fill = Color32::BLACK;
            visuals.widgets.inactive.bg_stroke = Stroke::new(1.0, Color32::LIGHT_BLUE);
            visuals.widgets.hovered.weak_bg_fill = Color32::from_rgb(60, 0, 0);
            visuals.widgets.hovered.bg_stroke = Stroke::new(1.0, Color32::RED);
            visuals.widgets.active.weak_bg_fill = Color32::from_rgb(60, 60, 0);
            visuals.widgets.active.bg_stroke = Stroke::new(1.0, Color32::YELLOW);
            visuals.extreme_bg_color = Color32::from_rgb(0, 60, 0);
            visuals.override_text_color = Some(Color32::WHITE);
            visuals.selection.stroke = Stroke::new(1.0, Color32::GREEN);

            let style = ui.style_mut();
            style
                .text_styles
                .insert(TextStyle::Body, FontId::proportional(14.0));
            style.drag_value_text_style = TextStyle::Body;
            ui.spacing_mut().button_padding = egui::vec2(6.0, 4.0);
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                ui.add_sized(
                    [90.0, 18.0],
                    egui::DragValue::new(value)
                        .speed(speed)
                        .custom_parser(|s| s.trim().parse::<f64>().ok())
                        .custom_formatter(|value, _range| format_drag_value(value)),
                );
            });
        });
    });
}

/// Draws a standard body-style label used throughout the control panels.
pub fn label_normal(ui: &mut Ui, text: &str) {
    let style = LabelStyle {
        font_size: 12.0,
        text_color: Some(Color32::from_rgb(220, 220, 220)),
        padding: Some(Margin::same(2)),
        height: 20.0,
        ..Default::default()
    };
    draw_label_with_style(ui, text, &style);
}

/// Binds a selectable UI item by display string to a typed selected value.
pub fn selectable_value<T: PartialEq + std::fmt::Display>(
    ui: &mut Ui,
    current: &mut T,
    selected: T,
) {
    let display_text = format!("{}", selected);
    ui.selectable_value(current, selected, display_text.as_str());
}

fn panel_button_height(ui: &Ui) -> f32 {
    ui.spacing().interact_size.y * 1.5
}

/// Draws a full-width normal button with enlarged interactive height.
pub fn button_normal(ui: &mut Ui, text: &str, inverted: bool) -> Response {
    let button_size = vec2(ui.available_width(), panel_button_height(ui));
    let button = if inverted {
        Button::new(RichText::new(text).color(Color32::BLACK))
            .fill(Color32::WHITE)
            .stroke(Stroke::new(1.0, Color32::BLACK))
    } else {
        Button::new(text)
    };
    ui.add_sized(button_size, button)
}

/// Shows a closable window when `is_open` is true and returns the updated open state.
pub fn show_closable_window(
    ctx: &egui::Context,
    title: &'static str,
    is_open: bool,
    sync_close: bool,
    configure: impl FnOnce(egui::Window) -> egui::Window,
    add_contents: impl FnOnce(&mut Ui),
) -> bool {
    if !is_open {
        return is_open;
    }
    let mut panel_open = is_open;
    configure(egui::Window::new(title).open(&mut panel_open)).show(ctx, add_contents);
    if sync_close {
        panel_open
    } else {
        is_open
    }
}

const STEER_MARKER_RADIUS: f32 = 70.0;
const STEER_MARKER_STROKE: f32 = 10.0;
const STEER_MARKER_LAYER: &str = "spacecraft_steer_marker";

/// Draws the spacecraft-mode steer anchor (⊕) fixed at a screen position.
pub fn draw_spacecraft_steer_marker(ctx: &egui::Context, anchor: [f64; 2]) {
    let pixels_per_point = ctx.pixels_per_point();
    let center = egui::pos2(
        (anchor[0] as f32) / pixels_per_point,
        (anchor[1] as f32) / pixels_per_point,
    );
    let painter = ctx.layer_painter(egui::LayerId::new(
        egui::Order::Foreground,
        egui::Id::new(STEER_MARKER_LAYER),
    ));
    let stroke = egui::Stroke::new(STEER_MARKER_STROKE, egui::Color32::from_rgb(220, 220, 220));
    painter.circle_stroke(center, STEER_MARKER_RADIUS, stroke);
    let half = STEER_MARKER_RADIUS * 0.55;
    painter.line_segment(
        [center - egui::vec2(half, 0.0), center + egui::vec2(half, 0.0)],
        stroke,
    );
    painter.line_segment(
        [center - egui::vec2(0.0, half), center + egui::vec2(0.0, half)],
        stroke,
    );
}

const YAW_STEER_MARKER_LAYER: &str = "spacecraft_yaw_steer_marker";
const YAW_MARKER_HALF_WIDTH: f32 = 96.0;
const YAW_MARKER_HEAD_LENGTH: f32 = 30.0;
const YAW_MARKER_HEAD_HALF_HEIGHT: f32 = 24.0;

fn draw_yaw_steer_glyph(painter: &egui::Painter, center: egui::Pos2, stroke: egui::Stroke) {
    let color = stroke.color;
    let half_shaft = stroke.width * 0.5;
    let left_tip = center - egui::vec2(YAW_MARKER_HALF_WIDTH, 0.0);
    let right_tip = center + egui::vec2(YAW_MARKER_HALF_WIDTH, 0.0);
    let left_base_x = left_tip.x + YAW_MARKER_HEAD_LENGTH;
    let right_base_x = right_tip.x - YAW_MARKER_HEAD_LENGTH;

    painter.add(Shape::convex_polygon(
        vec![
            left_tip,
            egui::pos2(left_base_x, center.y - YAW_MARKER_HEAD_HALF_HEIGHT),
            egui::pos2(left_base_x, center.y + YAW_MARKER_HEAD_HALF_HEIGHT),
        ],
        color,
        Stroke::NONE,
    ));
    painter.add(Shape::convex_polygon(
        vec![
            right_tip,
            egui::pos2(right_base_x, center.y - YAW_MARKER_HEAD_HALF_HEIGHT),
            egui::pos2(right_base_x, center.y + YAW_MARKER_HEAD_HALF_HEIGHT),
        ],
        color,
        Stroke::NONE,
    ));
    painter.add(Shape::convex_polygon(
        vec![
            egui::pos2(left_base_x, center.y - half_shaft),
            egui::pos2(right_base_x, center.y - half_shaft),
            egui::pos2(right_base_x, center.y + half_shaft),
            egui::pos2(left_base_x, center.y + half_shaft),
        ],
        color,
        Stroke::NONE,
    ));
}

/// Draws the spacecraft-mode yaw steer anchor (↔) fixed at a screen position.
pub fn draw_spacecraft_yaw_steer_marker(ctx: &egui::Context, anchor: [f64; 2]) {
    let pixels_per_point = ctx.pixels_per_point();
    let center = egui::pos2(
        (anchor[0] as f32) / pixels_per_point,
        (anchor[1] as f32) / pixels_per_point,
    );
    let painter = ctx.layer_painter(egui::LayerId::new(
        egui::Order::Foreground,
        egui::Id::new(YAW_STEER_MARKER_LAYER),
    ));
    let stroke = egui::Stroke::new(STEER_MARKER_STROKE, egui::Color32::from_rgb(220, 220, 220));
    draw_yaw_steer_glyph(&painter, center, stroke);
}
