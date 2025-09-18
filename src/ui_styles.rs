use egui::{Align, Color32, Frame, Layout, Margin, RichText, Stroke, Ui};

pub mod styles {
    use super::*;

    #[derive(Default, Clone)]
    pub struct LabelStyle {
        pub font_size: Option<f32>,
        pub text_color: Option<Color32>,
        pub bg_color: Option<Color32>,
        pub border_color: Option<Color32>,
        pub border_width: Option<f32>,
        pub width: Option<f32>,
        pub height: Option<f32>,
        pub padding: Option<Margin>,
    }

    pub fn draw_colored_label_with_border(ui: &mut Ui, text: &str, style: LabelStyle) {
        let stroke = if style.border_width.is_some() {
            Some(Stroke::new(
                style.border_width.unwrap(),
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
        let rich_text = if let Some(size) = style.font_size {
            rich_text.size(size)
        } else {
            rich_text
        };
        frame.show(ui, |ui| {
            if let Some(width) = style.width {
                ui.set_min_width(width);
            }
            if let Some(height) = style.height {
                ui.set_min_height(height);
            }
            ui.label(rich_text);
        });
    }

    pub fn label_indicator(ui: &mut Ui, text: &str) {
        let style = LabelStyle {
            font_size: Some(16.0),
            bg_color: Some(Color32::from_rgb(50, 50, 150)),
            text_color: Some(Color32::WHITE),
            border_color: Some(Color32::LIGHT_BLUE),
            border_width: Some(1.0),
            width: Some(100.0),
            //height: Some(30.0),
            padding: Some(Margin::same(2)),
            ..Default::default()
        };
        ui.with_layout(Layout::right_to_left(Align::RIGHT), |ui| {
            draw_colored_label_with_border(ui, text, style);
        });
    }
}
