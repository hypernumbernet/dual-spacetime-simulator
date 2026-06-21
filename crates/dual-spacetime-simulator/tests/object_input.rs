use dual_spacetime_simulator::object_input::{
    clamp_world_scale, ObjectInput, ObjectInputType, MIN_WORLD_SCALE, SATELLITE_ORBIT_SCALE,
    SOLAR_SYSTEM_SCALE,
};

#[test]
fn clamp_world_scale_rejects_non_positive_values() {
    assert_eq!(clamp_world_scale(-1.0), MIN_WORLD_SCALE);
    assert_eq!(clamp_world_scale(0.0), MIN_WORLD_SCALE);
    assert_eq!(clamp_world_scale(1e-18), MIN_WORLD_SCALE);
    assert_eq!(clamp_world_scale(1e-17), 1e-17);
    assert_eq!(clamp_world_scale(1e-12), 1e-12);
    assert_eq!(clamp_world_scale(f64::NAN), MIN_WORLD_SCALE);
    assert_eq!(clamp_world_scale(f64::INFINITY), MIN_WORLD_SCALE);
    assert_eq!(clamp_world_scale(1e7), 1e7);
}

#[test]
fn get_scale_positive_for_all_variants() {
    for ty in [
        ObjectInputType::RandomSphere,
        ObjectInputType::RandomCube,
        ObjectInputType::SpiralDisk,
        ObjectInputType::EllipticalOrbit,
    ] {
        let scale = ty.default_base_scale();
        let ic = ty.to_object_input(scale);
        assert!(ic.get_scale() > 0.0, "{ty}");
        assert_eq!(ic.get_scale(), scale, "{ty}");
    }
}

#[test]
fn uses_scaled_parameters_matches_expected_types() {
    assert!(ObjectInputType::RandomSphere.uses_scaled_parameters());
    assert!(ObjectInputType::RandomCube.uses_scaled_parameters());
    assert!(ObjectInputType::SpiralDisk.uses_scaled_parameters());
    assert!(ObjectInputType::EllipticalOrbit.uses_scaled_parameters());
}

#[test]
fn default_base_scale_matches_type_presets() {
    assert_eq!(ObjectInputType::RandomSphere.default_base_scale(), 1e10);
    assert_eq!(ObjectInputType::RandomCube.default_base_scale(), 1e10);
    assert_eq!(ObjectInputType::SpiralDisk.default_base_scale(), 1e7);
    assert_eq!(ObjectInputType::EllipticalOrbit.default_base_scale(), 1.5e11);
}

#[test]
fn generate_particle_count_matches_for_simple_types() {
    let n = 64u32;
    for ty in [
        ObjectInputType::RandomSphere,
        ObjectInputType::RandomCube,
        ObjectInputType::SpiralDisk,
    ] {
        let sim = ty
            .to_object_input(ty.default_base_scale())
            .generate_particles(n);
        assert_eq!(sim.particles.len() as u32, n, "{ty}");
    }
}

#[test]
fn elliptical_orbit_always_two_bodies() {
    let ic = ObjectInputType::EllipticalOrbit.to_object_input(1.5e11);
    let sim = ic.generate_particles(999);
    assert_eq!(sim.particles.len(), 2);
}

#[test]
fn satellite_orbit_adds_satellites_beyond_two_bodies() {
    let ic = ObjectInput::SatelliteOrbit {
        scale: SATELLITE_ORBIT_SCALE,
        orbit_altitude_min: 300e3,
        orbit_altitude_max: 800e3,
        asteroid_mass: 1e24,
        asteroid_distance: 2e7,
        asteroid_speed: 3e3,
    };
    let sim = ic.generate_particles(10);
    // Earth + asteroid + (n-1) satellites
    assert_eq!(sim.particles.len(), 11);
}

#[test]
fn solar_system_scale_from_variant() {
    let ic = ObjectInput::SolarSystem {
        scale: SOLAR_SYSTEM_SCALE,
        start_year: 2000,
        start_month: 1,
        start_day: 1,
        start_hour: 12,
    };
    assert_eq!(ic.get_scale(), SOLAR_SYSTEM_SCALE);
}

#[test]
fn to_object_input_scales_random_sphere_parameters_with_base_scale() {
    let scale = 42.0;
    let reference = ObjectInputType::RandomSphere.default_base_scale();
    let factor = scale / reference;
    let factor_cubed = factor * factor * factor;
    let input = ObjectInputType::RandomSphere.to_object_input(scale);
    if let ObjectInput::RandomSphere {
        radius,
        mass_range,
        velocity_std,
        ..
    } = input
    {
        assert!((radius - scale).abs() < 1e-6);
        assert!((mass_range.0 - 1e29 * factor_cubed).abs() < 1e-6);
        assert!((mass_range.1 - 1e31 * factor_cubed).abs() < 1e-6);
        assert!((velocity_std - 1e6 * factor).abs() < 1e-6);
        assert!((input.preview_group_extent() - 1.0).abs() < 1e-6);
    } else {
        panic!("expected RandomSphere");
    }
}

#[test]
fn to_object_input_scales_random_cube_parameters_with_base_scale() {
    let scale = 1e7;
    let reference = ObjectInputType::RandomCube.default_base_scale();
    let factor = scale / reference;
    let input = ObjectInputType::RandomCube.to_object_input(scale);
    if let ObjectInput::RandomCube {
        cube_size,
        mass_range,
        velocity_std,
        ..
    } = input
    {
        assert!((cube_size - 2e10 * factor).abs() < 1e-6);
        assert!((mass_range.0 - 1e29 * factor * factor * factor).abs() < 1e-6);
        assert!((mass_range.1 - 1e31 * factor * factor * factor).abs() < 1e-6);
        assert!((velocity_std - 1e6 * factor).abs() < 1e-6);
        assert!((input.preview_group_extent() - 1.0).abs() < 1e-6);
    } else {
        panic!("expected RandomCube");
    }
}

#[test]
fn to_object_input_scales_spiral_disk_parameters_with_base_scale() {
    let scale = 2.5e8;
    let reference = ObjectInputType::SpiralDisk.default_base_scale();
    let factor = scale / reference;
    let factor_cubed = factor * factor * factor;
    let input = ObjectInputType::SpiralDisk.to_object_input(scale);
    if let ObjectInput::SpiralDisk {
        disk_radius,
        mass_fixed,
        ..
    } = input
    {
        assert!((disk_radius - 1.5e7 * factor).abs() < 1e-6);
        assert!((mass_fixed - 1e20 * factor_cubed).abs() < 1e-6);
        assert!((input.preview_group_extent() - 1.5).abs() < 1e-6);
    } else {
        panic!("expected SpiralDisk");
    }
}

#[test]
fn to_object_input_scales_elliptical_orbit_parameters_with_base_scale() {
    let scale = 42.0;
    let reference = ObjectInputType::EllipticalOrbit.default_base_scale();
    let factor = scale / reference;
    let factor_cubed = factor * factor * factor;
    let input = ObjectInputType::EllipticalOrbit.to_object_input(scale);
    if let ObjectInput::EllipticalOrbit {
        central_mass,
        planetary_mass,
        planetary_speed,
        planetary_distance,
        ..
    } = input
    {
        assert!((planetary_distance - 2.0e11 * factor).abs() < 1e-6);
        assert!((planetary_speed - 2.0e5 * factor).abs() < 1e-6);
        assert!((central_mass - 1.989e32 * factor_cubed).abs() < 1e-6);
        assert!((planetary_mass - 5.972e24 * factor_cubed).abs() < 1e-6);
        assert!((input.preview_group_extent() - 2.0e11 * factor / scale).abs() < 1e-6);
    } else {
        panic!("expected EllipticalOrbit");
    }
}

#[test]
fn get_scale_clamps_negative_input() {
    let ic = ObjectInput::RandomSphere {
        scale: -5.0,
        radius: 1e10,
        mass_range: (1e29, 1e31),
        velocity_std: 1e6,
    };
    assert_eq!(ic.get_scale(), MIN_WORLD_SCALE);
}
