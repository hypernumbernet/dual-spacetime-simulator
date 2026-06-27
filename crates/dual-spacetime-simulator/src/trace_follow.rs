use crate::ui_state::ParticleDisplayMode;

/// Point sprite diameter as a fraction of framebuffer height (matches particle shader sizing).
pub const PARTICLE_SIZE_RATIO: f32 = 0.06;

/// At minimum trace follow distance, the particle diameter occupies this fraction of screen height.
pub const MAX_TRACE_PARTICLE_SCREEN_FRACTION: f32 = 0.3;

/// At maximum trace follow distance, the particle diameter occupies this fraction of screen height.
pub const MIN_TRACE_PARTICLE_SCREEN_FRACTION: f32 = 0.003;

/// Returns world-space min/max trace follow distances for the given rendering context.
///
/// Limits are derived from [`MAX_TRACE_PARTICLE_SCREEN_FRACTION`] and
/// [`MIN_TRACE_PARTICLE_SCREEN_FRACTION`], using the same sizing formula as the particle shader
/// (`gl_PointSize = size_scale / gl_Position.w`).
pub fn compute_trace_follow_distance_limits(
    visual_scale: f32,
    link_point_size_to_scale: bool,
    mode: ParticleDisplayMode,
) -> (f32, f32) {
    let point_scale_factor = if link_point_size_to_scale {
        visual_scale
    } else {
        1.0
    };
    let numerator = PARTICLE_SIZE_RATIO * point_scale_factor * mode.size_scale_factor();
    let min = numerator / (MAX_TRACE_PARTICLE_SCREEN_FRACTION * visual_scale);
    let max = numerator / (MIN_TRACE_PARTICLE_SCREEN_FRACTION * visual_scale);
    (min, max)
}

/// Returns the particle diameter as a fraction of screen height at a world-space trace distance.
pub fn compute_trace_particle_screen_fraction(
    world_distance: f32,
    visual_scale: f32,
    link_point_size_to_scale: bool,
    mode: ParticleDisplayMode,
) -> f32 {
    if !world_distance.is_finite()
        || world_distance <= 0.0
        || !visual_scale.is_finite()
        || visual_scale <= 0.0
    {
        return 0.0;
    }
    let point_scale_factor = if link_point_size_to_scale {
        visual_scale
    } else {
        1.0
    };
    PARTICLE_SIZE_RATIO * point_scale_factor * mode.size_scale_factor()
        / (world_distance * visual_scale)
}
