use egui::{Color32, Context, LayerId, Order, Painter, Pos2, Shape, Stroke};

const STEER_MARKER_RADIUS: f32 = 70.0;
const STEER_MARKER_STROKE: f32 = 10.0;
const STEER_MARKER_COLOR: Color32 = Color32::from_rgb(220, 220, 220);
const STEER_MARKER_LAYER: &str = "spacecraft_steer_marker";
const YAW_STEER_MARKER_LAYER: &str = "spacecraft_yaw_steer_marker";
const YAW_MARKER_HALF_WIDTH: f32 = 96.0;
const YAW_MARKER_HEAD_LENGTH: f32 = 30.0;
const YAW_MARKER_HEAD_HALF_HEIGHT: f32 = 24.0;

fn steer_anchor_center(ctx: &Context, anchor: [f64; 2]) -> Pos2 {
    let pixels_per_point = ctx.pixels_per_point();
    Pos2::new(
        (anchor[0] as f32) / pixels_per_point,
        (anchor[1] as f32) / pixels_per_point,
    )
}

fn steer_marker_painter(ctx: &Context, layer: &str) -> Painter {
    ctx.layer_painter(LayerId::new(
        Order::Foreground,
        egui::Id::new(layer),
    ))
}

fn steer_marker_stroke() -> Stroke {
    Stroke::new(STEER_MARKER_STROKE, STEER_MARKER_COLOR)
}

fn draw_yaw_steer_glyph(painter: &Painter, center: Pos2, stroke: Stroke) {
    let color = stroke.color;
    let half_shaft = stroke.width * 0.5;
    let left_tip = center - egui::vec2(YAW_MARKER_HALF_WIDTH, 0.0);
    let right_tip = center + egui::vec2(YAW_MARKER_HALF_WIDTH, 0.0);
    let left_base_x = left_tip.x + YAW_MARKER_HEAD_LENGTH;
    let right_base_x = right_tip.x - YAW_MARKER_HEAD_LENGTH;

    painter.add(Shape::convex_polygon(
        vec![
            left_tip,
            Pos2::new(left_base_x, center.y - YAW_MARKER_HEAD_HALF_HEIGHT),
            Pos2::new(left_base_x, center.y + YAW_MARKER_HEAD_HALF_HEIGHT),
        ],
        color,
        Stroke::NONE,
    ));
    painter.add(Shape::convex_polygon(
        vec![
            right_tip,
            Pos2::new(right_base_x, center.y - YAW_MARKER_HEAD_HALF_HEIGHT),
            Pos2::new(right_base_x, center.y + YAW_MARKER_HEAD_HALF_HEIGHT),
        ],
        color,
        Stroke::NONE,
    ));
    painter.add(Shape::convex_polygon(
        vec![
            Pos2::new(left_base_x, center.y - half_shaft),
            Pos2::new(right_base_x, center.y - half_shaft),
            Pos2::new(right_base_x, center.y + half_shaft),
            Pos2::new(left_base_x, center.y + half_shaft),
        ],
        color,
        Stroke::NONE,
    ));
}

/// Draws the spacecraft-mode steer anchor (⊕) fixed at a screen position.
pub fn draw_spacecraft_steer_marker(ctx: &Context, anchor: [f64; 2]) {
    let center = steer_anchor_center(ctx, anchor);
    let painter = steer_marker_painter(ctx, STEER_MARKER_LAYER);
    let stroke = steer_marker_stroke();
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

/// Draws the spacecraft-mode yaw steer anchor (↔) fixed at a screen position.
pub fn draw_spacecraft_yaw_steer_marker(ctx: &Context, anchor: [f64; 2]) {
    let center = steer_anchor_center(ctx, anchor);
    let painter = steer_marker_painter(ctx, YAW_STEER_MARKER_LAYER);
    draw_yaw_steer_glyph(&painter, center, steer_marker_stroke());
}
