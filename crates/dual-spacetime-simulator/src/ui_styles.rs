use egui::{Align, Color32, FontId, Frame, Layout, Margin, RichText, Stroke, TextStyle, Ui};
use egui::{Button, vec2};
use egui::Response;

#[derive(Default, Clone)]
pub struct LabelStyle {
    pub font_size: f32,
    pub text_color: Option<Color32>,
    pub bg_color: Option<Color32>,
    pub border_color: Option<Color32>,
    pub border_width: f32,
    pub width: f32,
    pub height: f32,
    pub padding: Option<Margin>,
}

/// Draws a label with optional frame, text, and sizing overrides.
pub fn draw_label_with_style(ui: &mut Ui, text: &str, style: &LabelStyle) {
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

const POSITIVE_DRAG_VALUE_HEIGHT: f32 = 18.0;

fn apply_positive_dragvalue_style(ui: &mut Ui) {
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
}

fn format_positive_drag_value(value: f64) -> String {
    if value.abs() >= 1e6 || value.abs() <= 1e-4 && value != 0.0 {
        format!("{:.3e}", value)
    } else {
        format!("{:}", value)
    }
}

fn positive_drag_value(
    ui: &mut Ui,
    value: &mut f64,
    speed: f64,
    min: f64,
    width: f32,
    formatter: Option<&dyn Fn(f64) -> String>,
) {
    apply_positive_dragvalue_style(ui);
    ui.add_sized(
        [width, POSITIVE_DRAG_VALUE_HEIGHT],
        egui::DragValue::new(value)
            .speed(speed)
            .range(min..=f64::MAX)
            .custom_parser(|s| {
                s.trim()
                    .parse::<f64>()
                    .ok()
                    .filter(|&v| v.is_finite() && v > min)
            })
            .custom_formatter(|value, _range| {
                formatter
                    .map(|format| format(value))
                    .unwrap_or_else(|| format_positive_drag_value(value))
            }),
    );
    *value = value.max(min);
}

/// Draws a positive-only f64 drag value clamped to `min`..=f64::MAX.
pub fn dragvalue_positive_f64(
    ui: &mut Ui,
    value: &mut f64,
    speed: impl Into<f64>,
    min: f64,
    width: f32,
    label: Option<&str>,
    formatter: Option<&dyn Fn(f64) -> String>,
) {
    let speed = speed.into();
    if let Some(prefix) = label {
        ui.horizontal(|ui| {
            label_normal(ui, prefix);
            ui.scope(|ui| positive_drag_value(ui, value, speed, min, width, formatter));
        });
    } else {
        ui.scope(|ui| positive_drag_value(ui, value, speed, min, width, formatter));
    }
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
                        .custom_formatter(|value, _range| {
                            if value.abs() >= 1e6 || value.abs() <= 1e-4 && value != 0.0 {
                                format!("{:.3e}", value)
                            } else {
                                format!("{:}", value)
                            }
                        }),
                );
            });
        });
    });
}

/// Draws a right-aligned highlighted indicator label for compact status display.
pub fn label_indicator(ui: &mut Ui, text: &str) {
    let style = LabelStyle {
        font_size: 14.0,
        bg_color: Some(Color32::from_rgb(50, 50, 150)),
        text_color: Some(Color32::WHITE),
        border_color: Some(Color32::LIGHT_BLUE),
        border_width: 1.0,
        width: 120.0,
        padding: Some(Margin::same(2)),
        ..Default::default()
    };
    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
        draw_label_with_style(ui, text, &style);
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

/// Draws a full-width slider without rendering its numeric value text.
pub fn slider_pure(ui: &mut Ui, value: &mut f64, range: std::ops::RangeInclusive<f64>) {
    ui.add_space(4.0);
    ui.style_mut().spacing.slider_width = ui.available_width();
    ui.add(egui::Slider::new(value, range).show_value(false));
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
/// When `inverted` is true, uses white background + black text (and border) to indicate running state.
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

/// Draws two equal-width buttons side by side without expanding the parent width.
pub fn button_row_pair(ui: &mut Ui, left: &str, right: &str) -> (Response, Response) {
    let total_width = ui.available_width();
    let spacing = ui.spacing().item_spacing.x;
    let half_width = (total_width - spacing) * 0.5;
    let button_height = panel_button_height(ui);
    ui.horizontal(|ui| {
        ui.set_max_width(total_width);
        let left = ui.add_sized(vec2(half_width, button_height), Button::new(left));
        let right = ui.add_sized(vec2(half_width, button_height), Button::new(right));
        (left, right)
    })
    .inner
}
