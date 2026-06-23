use dual_spacetime_simulator::ui_styles::format_drag_value;

#[test]
fn long_decimal_is_rounded_to_six_significant_figures() {
    assert_eq!(format_drag_value(13333.3333333333334), "13333.3");
}

#[test]
fn integers_are_displayed_without_decimals() {
    assert_eq!(format_drag_value(2000.0), "2000");
    assert_eq!(format_drag_value(1.0), "1");
    assert_eq!(format_drag_value(20000.0), "20000");
}

#[test]
fn short_decimals_are_preserved() {
    assert_eq!(format_drag_value(1.5), "1.5");
    assert_eq!(format_drag_value(0.25), "0.25");
}

#[test]
fn zero_is_plain_zero() {
    assert_eq!(format_drag_value(0.0), "0");
}

#[test]
fn negative_long_decimal_is_compact() {
    assert_eq!(format_drag_value(-13333.3333333333334), "-13333.3");
}

#[test]
fn large_magnitudes_use_scientific_notation() {
    assert!(format_drag_value(1.0e6).contains('e'));
    assert!(format_drag_value(2_500_000.0).contains('e'));
}

#[test]
fn small_magnitudes_use_scientific_notation() {
    assert!(format_drag_value(1.0e-5).contains('e'));
    // Boundary: `abs <= 1e-4` is treated as "very small".
    assert!(format_drag_value(1.0e-4).contains('e'));
}

#[test]
fn mid_range_output_length_stays_bounded() {
    let samples = [
        13333.3333333333334,
        -13333.3333333333334,
        0.00012345678,
        999_999.9,
        123.456789012,
        1.0 / 3.0,
        -1.0 / 7.0,
    ];
    for v in samples {
        let formatted = format_drag_value(v);
        assert!(
            formatted.len() <= 12,
            "formatted value too long: {formatted:?} (len {})",
            formatted.len()
        );
    }
}

mod particle_info_format {
    use dual_spacetime_simulator::ui_styles::format_particle_info_value;

    #[test]
    fn zero_is_zero_padded_to_ten_decimals() {
        assert_eq!(format_particle_info_value(0.0), "0.0000000000");
    }

    #[test]
    fn short_decimals_are_zero_padded() {
        assert_eq!(format_particle_info_value(1.5), "1.5000000000");
        assert_eq!(format_particle_info_value(-0.25), "-0.2500000000");
    }

    #[test]
    fn extra_decimals_are_truncated_not_rounded() {
        assert_eq!(
            format_particle_info_value(1.99999999999),
            "1.9999999999"
        );
        assert_eq!(
            format_particle_info_value(-3.141592653589),
            "-3.1415926535"
        );
    }

    #[test]
    fn integers_keep_ten_decimal_places() {
        assert_eq!(format_particle_info_value(2000.0), "2000.0000000000");
    }
}
