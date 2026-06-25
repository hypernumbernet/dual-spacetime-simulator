use dual_spacetime_simulator::settings::AppSettings;
use dual_spacetime_simulator::ui_state::ParticleDisplayMode;

#[test]
fn app_settings_json_roundtrip() {
    let s = AppSettings {
        max_particle_count: 1234,
        window_min_width: 512.0,
        window_min_height: 384.0,
        start_maximized: true,
        link_point_size_to_scale: false,
        mailbox_present_mode: true,
        particle_display_mode: ParticleDisplayMode::Sphere,
    };
    let json = serde_json::to_string_pretty(&s).unwrap();
    let back: AppSettings = serde_json::from_str(&json).unwrap();
    assert_eq!(s.max_particle_count, back.max_particle_count);
    assert!((s.window_min_width - back.window_min_width).abs() < f32::EPSILON);
    assert_eq!(s.start_maximized, back.start_maximized);
    assert_eq!(s.particle_display_mode, back.particle_display_mode);
}
