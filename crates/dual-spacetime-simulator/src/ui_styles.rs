use crate::ui_state::trim_trailing_zeros;
use egui::Response;
use egui::{Align, Color32, FontId, Frame, Layout, Margin, RichText, Stroke, TextStyle, Ui};
use egui::{Button, Slider, vec2};

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

/// Formats a drag-value number with up to 6 significant figures, falling back to
/// scientific notation for very large or very small magnitudes so the rendered
/// text stays narrow enough to never overflow the input field and hide its label.
pub fn format_drag_value(value: f64) -> String {
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

fn format_positive_drag_value(value: f64) -> String {
    format_drag_value(value)
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
                        .custom_formatter(|value, _range| format_drag_value(value)),
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
pub fn slider_pure(
    ui: &mut Ui,
    value: &mut f64,
    range: std::ops::RangeInclusive<f64>,
) -> egui::Response {
    ui.add_space(4.0);
    ui.style_mut().spacing.slider_width = ui.available_width();
    ui.add(Slider::new(value, range).show_value(false))
}

const PANEL_SLIDER_MIN_TRACK_WIDTH: f32 = 40.0;
const SLIDER_VALUE_CHAR_WIDTH: f32 = 8.0;
const SLIDER_VALUE_PADDING: f32 = 12.0;
const SLIDER_VALUE_MIN_RESERVE: f32 = 36.0;
/// Width reserved for f64 panel sliders with two decimal places (e.g. "-10.00").
const SLIDER_F64_VALUE_RESERVE_WIDTH: f32 = 48.0;

/// Locks panel column width so later widgets cannot widen separators or buttons above.
pub fn lock_panel_content_width(ui: &mut Ui) {
    ui.set_max_width(ui.available_width());
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

fn panel_slider_track_width(row_width: f32, value_reserve: f32, spacing: f32) -> f32 {
    (row_width - value_reserve - spacing).max(PANEL_SLIDER_MIN_TRACK_WIDTH)
}

fn set_panel_slider_track_width(ui: &mut Ui, row_width: f32, value_reserve: f32) {
    let spacing = ui.spacing().item_spacing.x;
    ui.style_mut().spacing.slider_width =
        panel_slider_track_width(row_width, value_reserve, spacing);
}

fn slider_u32_value_reserve_width(max_value: u32) -> f32 {
    let digits = max_value.to_string().len();
    (digits as f32 * SLIDER_VALUE_CHAR_WIDTH + SLIDER_VALUE_PADDING).max(SLIDER_VALUE_MIN_RESERVE)
}

/// Renders a labeled u32 slider that fits within the panel column width.
pub fn slider_labeled_u32(
    ui: &mut Ui,
    label: &str,
    value: &mut u32,
    range: std::ops::RangeInclusive<u32>,
) -> Response {
    label_normal(ui, label);
    let row_width = ui.available_width();
    let value_reserve = slider_u32_value_reserve_width(*range.end());
    ui.horizontal(|ui| {
        ui.set_max_width(row_width);
        ui.set_min_width(row_width);
        set_panel_slider_track_width(ui, row_width, value_reserve);
        ui.add(Slider::new(value, range))
    })
    .inner
}

/// Renders a short inline label plus slider row that fits within the panel column width.
fn slider_inline_row(
    ui: &mut Ui,
    row_width: f32,
    label: &str,
    value_reserve: f32,
    add_slider: impl FnOnce(&mut Ui) -> Response,
) -> Response {
    ui.horizontal(|ui| {
        ui.set_max_width(row_width);
        label_normal(ui, label);
        set_panel_slider_track_width(ui, ui.available_width(), value_reserve);
        add_slider(ui)
    })
    .inner
}

/// Renders an axis label (X/Y/Z) plus f64 slider row for the input panels.
pub fn slider_axis_row(ui: &mut Ui, row_width: f32, label: &str, slider: Slider<'_>) -> Response {
    slider_inline_row(ui, row_width, label, SLIDER_F64_VALUE_RESERVE_WIDTH, |ui| {
        ui.add(slider)
    })
}

/// Primary-button double-click position this frame, if any.
pub fn primary_double_click_pos(ui: &Ui) -> Option<egui::Pos2> {
    ui.input(|i| {
        if i.pointer
            .button_double_clicked(egui::PointerButton::Primary)
        {
            i.pointer.interact_pos()
        } else {
            None
        }
    })
}

/// Returns true when `pos` lies inside `response`'s rect.
pub fn response_contains_double_click(response: &Response, pos: Option<egui::Pos2>) -> bool {
    pos.is_some_and(|p| response.rect.contains(p))
}

/// Returns true when a primary double-click occurred over `response`'s rect.
///
/// egui sliders use drag sense only, so [`egui::Response::double_clicked`] never fires on them.
pub fn response_double_clicked(ui: &Ui, response: &Response) -> bool {
    response_contains_double_click(response, primary_double_click_pos(ui))
}

/// Invokes `reset` when `response` received a primary double-click.
pub fn apply_slider_double_click_reset(ui: &Ui, response: &Response, reset: impl FnOnce()) {
    if response_double_clicked(ui, response) {
        reset();
    }
}

/// Invokes `reset` when `response` contains the cached primary double-click position.
pub fn apply_slider_double_click_reset_with_pos(
    response: &Response,
    pos: Option<egui::Pos2>,
    reset: impl FnOnce(),
) {
    if response_contains_double_click(response, pos) {
        reset();
    }
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

/// Draws Close and Abort buttons side by side for the reset log panel.
pub fn button_row_close_abort(
    ui: &mut Ui,
    close_enabled: bool,
    abort_enabled: bool,
) -> (Response, Response) {
    let total_width = ui.available_width();
    let spacing = ui.spacing().item_spacing.x;
    let half_width = (total_width - spacing) * 0.5;
    let button_height = panel_button_height(ui);
    ui.horizontal(|ui| {
        ui.set_max_width(total_width);
        let close = ui.add_enabled_ui(close_enabled, |ui| {
            ui.add_sized(vec2(half_width, button_height), Button::new("Close"))
        });
        let abort = ui.add_enabled_ui(abort_enabled, |ui| {
            ui.add_sized(
                vec2(half_width, button_height),
                Button::new(RichText::new("Abort").color(Color32::WHITE))
                    .fill(Color32::from_rgb(140, 45, 45)),
            )
        });
        (close.inner, abort.inner)
    })
    .inner
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn u32_value_reserve_grows_with_digit_count() {
        assert!(slider_u32_value_reserve_width(999) < slider_u32_value_reserve_width(10_000));
    }

    #[test]
    fn track_width_clamps_to_minimum() {
        assert_eq!(
            panel_slider_track_width(10.0, 48.0, 4.0),
            PANEL_SLIDER_MIN_TRACK_WIDTH,
        );
    }

    #[test]
    fn track_width_fits_panel_row_with_five_digit_value() {
        let row_width = 184.0;
        let value_reserve = slider_u32_value_reserve_width(19_999);
        let spacing = 4.0;
        let track = panel_slider_track_width(row_width, value_reserve, spacing);
        assert!(track + value_reserve + spacing <= row_width);
    }
}
