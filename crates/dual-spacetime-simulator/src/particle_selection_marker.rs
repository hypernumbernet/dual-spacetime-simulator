/// Bracket half-size as a fraction of particle diameter (`size_scale / w`).
/// Full bracket span is `2 * this` times the particle diameter (0.75 → 1.5×).
pub const BRACKET_RADIUS_RATIO: f32 = 0.75;
pub const MIN_HALF_SIZE_PX: f32 = 12.0;
pub const SELECTION_MARKER_VERTEX_COUNT: u32 = 16;

/// Encodes a particle index for the selection-marker shader push constants.
#[inline]
pub fn selection_index_bits(index: i32) -> f32 {
    f32::from_bits(index as u32)
}

/// Computes bracket half-size in window pixels from clip-space view depth (`w`)
/// and the same `size_scale` used for particle point sprites.
///
/// Result is floored at [`MIN_HALF_SIZE_PX`]. Mirrors the GPU sizing formula.
#[inline]
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
