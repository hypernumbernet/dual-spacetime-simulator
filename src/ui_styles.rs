use egui::{Align, Color32, FontId, Frame, Layout, Margin, RichText, Stroke, TextStyle, Ui};

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

pub fn dragvalue_normal<T: egui::emath::Numeric>(
    ui: &mut Ui,
    value: &mut T,
    speed: impl Into<f64>,
    prefix: &str,
) {
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
        ui.add(egui::DragValue::new(value).speed(speed).prefix(prefix));
    });
}

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
    ui.with_layout(Layout::right_to_left(Align::RIGHT), |ui| {
        draw_label_with_style(ui, text, &style);
    });
}

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

pub fn slider_pure(ui: &mut Ui, value: &mut f64, range: std::ops::RangeInclusive<f64>) {
    ui.add_space(4.0);
    ui.style_mut().spacing.slider_width = ui.available_width();
    ui.add(egui::Slider::new(value, range).show_value(false));
}

pub fn selectable_value<T: PartialEq + std::fmt::Display>(
    ui: &mut Ui,
    current: &mut T,
    selected: T,
) {
    let display_text = format!("{}", selected);
    ui.selectable_value(current, selected, display_text.as_str());
}
