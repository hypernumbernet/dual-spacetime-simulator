use dual_spacetime_simulator::trace_follow::{
    compute_trace_follow_distance_limits, compute_trace_particle_screen_fraction,
    MAX_TRACE_PARTICLE_SCREEN_FRACTION, MIN_TRACE_PARTICLE_SCREEN_FRACTION,
    PARTICLE_SIZE_RATIO,
};
use dual_spacetime_simulator::ui_state::ParticleDisplayMode;

#[test]
fn trace_follow_distance_limits_match_default_at_unit_scale() {
    let (min, max) = compute_trace_follow_distance_limits(1.0, true, ParticleDisplayMode::Glow);
    assert!((min - 0.1).abs() < 1e-6);
    assert!((max - 100.0).abs() < 1e-4);
}

#[test]
fn trace_follow_distance_limits_halve_when_visual_scale_doubles_unlinked() {
    let base = compute_trace_follow_distance_limits(1.0, false, ParticleDisplayMode::Glow);
    let scaled = compute_trace_follow_distance_limits(2.0, false, ParticleDisplayMode::Glow);
    assert!((scaled.0 - base.0 * 0.5).abs() < 1e-6);
    assert!((scaled.1 - base.1 * 0.5).abs() < 1e-4);
}

#[test]
fn trace_follow_distance_limits_ignore_visual_scale_when_linked() {
    let base = compute_trace_follow_distance_limits(1.0, true, ParticleDisplayMode::Glow);
    let scaled = compute_trace_follow_distance_limits(4.0, true, ParticleDisplayMode::Glow);
    assert!((scaled.0 - base.0).abs() < 1e-6);
    assert!((scaled.1 - base.1).abs() < 1e-4);
}

#[test]
fn trace_follow_distance_limits_scale_with_sphere_mode() {
    let glow = compute_trace_follow_distance_limits(1.0, true, ParticleDisplayMode::Glow);
    let sphere = compute_trace_follow_distance_limits(1.0, true, ParticleDisplayMode::Sphere);
    let ratio = ParticleDisplayMode::Sphere.size_scale_factor();
    assert!((sphere.0 - glow.0 * ratio).abs() < 1e-6);
    assert!((sphere.1 - glow.1 * ratio).abs() < 1e-4);
}

#[test]
fn trace_particle_screen_fraction_round_trips_limits() {
    let visual_scale = 3.0;
    let link = false;
    let mode = ParticleDisplayMode::Glow;
    let (min, max) = compute_trace_follow_distance_limits(visual_scale, link, mode);

    let min_fraction = compute_trace_particle_screen_fraction(min, visual_scale, link, mode);
    let max_fraction = compute_trace_particle_screen_fraction(max, visual_scale, link, mode);
    assert!((min_fraction - MAX_TRACE_PARTICLE_SCREEN_FRACTION).abs() < 1e-6);
    assert!((max_fraction - MIN_TRACE_PARTICLE_SCREEN_FRACTION).abs() < 1e-6);
}

#[test]
fn trace_particle_screen_fraction_uses_particle_size_ratio() {
    let fraction = compute_trace_particle_screen_fraction(1.0, 1.0, true, ParticleDisplayMode::Glow);
    assert!((fraction - PARTICLE_SIZE_RATIO).abs() < 1e-6);
}
