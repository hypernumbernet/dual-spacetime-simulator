use egui::Pos2;

/// Bracket half-size as a fraction of particle diameter (`size_scale / w`).
/// Full bracket span is `2 * this` times the particle diameter (0.75 → 1.5×).
pub const BRACKET_RADIUS_RATIO: f32 = 0.75;
pub const MIN_HALF_SIZE_PX: f32 = 12.0;
pub const CORNER_ARM_RATIO: f32 = 0.35;
pub const SELECTION_MARKER_VERTEX_COUNT: u32 = 16;

/// Encodes a particle index for the selection-marker shader push constants.
pub fn selection_index_bits(index: i32) -> f32 {
    f32::from_bits(index as u32)
}

/// Computes bracket half-size in window pixels from clip-space view depth (`w`)
/// and the same `size_scale` used for particle point sprites.
///
/// Result is floored at [`MIN_HALF_SIZE_PX`].
pub fn compute_bracket_half_size(view_depth: f32, size_scale: f32) -> f32 {
    if !view_depth.is_finite()
        || view_depth <= 0.0
        || !size_scale.is_finite()
        || size_scale <= 0.0
    {
        return MIN_HALF_SIZE_PX;
    }
    (size_scale / view_depth * BRACKET_RADIUS_RATIO).max(MIN_HALF_SIZE_PX)
}

/// Builds the eight line segments forming a square bracket with edge-center gaps.
pub fn bracket_line_segments(center: Pos2, half_size: f32) -> [[Pos2; 2]; 8] {
    let arm = half_size * CORNER_ARM_RATIO;
    let left = center.x - half_size;
    let right = center.x + half_size;
    let top = center.y - half_size;
    let bottom = center.y + half_size;

    [
        [Pos2::new(left, top), Pos2::new(left + arm, top)],
        [Pos2::new(right - arm, top), Pos2::new(right, top)],
        [Pos2::new(right, top), Pos2::new(right, top + arm)],
        [Pos2::new(right, bottom - arm), Pos2::new(right, bottom)],
        [Pos2::new(right, bottom), Pos2::new(right - arm, bottom)],
        [Pos2::new(left + arm, bottom), Pos2::new(left, bottom)],
        [Pos2::new(left, bottom), Pos2::new(left, bottom - arm)],
        [Pos2::new(left, top + arm), Pos2::new(left, top)],
    ]
}
