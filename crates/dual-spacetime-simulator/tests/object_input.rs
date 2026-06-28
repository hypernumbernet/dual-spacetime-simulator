use dual_spacetime_simulator::object_input::{
    MIN_WORLD_SCALE, ObjectInput, ObjectInputType, ParticleBasicColor, SATELLITE_ORBIT_SCALE,
    SOLAR_SYSTEM_SCALE, clamp_world_scale,
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
    for ty in ObjectInputType::ALL {
        let scale = ty.default_base_scale();
        let ic = ty.to_object_input(scale);
        assert!(ic.get_scale() > 0.0, "{ty}");
        assert_eq!(ic.get_scale(), scale, "{ty}");
    }
}

#[test]
fn default_base_scale_matches_type_presets() {
    assert_eq!(ObjectInputType::RandomSphere.default_base_scale(), 1e10);
    assert_eq!(ObjectInputType::RandomCube.default_base_scale(), 1e10);
    assert_eq!(ObjectInputType::SpiralDisk.default_base_scale(), 1e7);
    assert_eq!(
        ObjectInputType::EllipticalOrbit.default_base_scale(),
        1.5e11
    );
    assert_eq!(ObjectInputType::SingleParticle.default_base_scale(), 1e10);
}

#[test]
fn uses_add_particle_count_matches_generation_behavior() {
    for ty in [
        ObjectInputType::RandomSphere,
        ObjectInputType::RandomCube,
        ObjectInputType::SpiralDisk,
    ] {
        assert!(ty.uses_add_particle_count(), "{ty}");
    }
    assert!(!ObjectInputType::TorsionStar.uses_add_particle_count());
    assert!(!ObjectInputType::EllipticalOrbit.uses_add_particle_count());
    assert!(!ObjectInputType::SingleParticle.uses_add_particle_count());
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
fn single_particle_always_one_body() {
    let ic = ObjectInputType::SingleParticle.to_object_input(1e10);
    let sim = ic.generate_particles(999);
    assert_eq!(sim.particles.len(), 1);
}

#[test]
fn satellite_orbit_particle_count_includes_earth() {
    let ic = ObjectInput::SatelliteOrbit {
        scale: SATELLITE_ORBIT_SCALE,
        orbit_altitude_min: 300e3,
        orbit_altitude_max: 800e3,
        satellite_count: 9,
    };
    let sim = ic.generate_particles(999);
    // Earth + satellite_count satellites; external count is ignored
    assert_eq!(sim.particles.len(), 10);
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
fn to_object_input_scales_single_particle_parameters_with_base_scale() {
    let scale = 42.0;
    let reference = ObjectInputType::SingleParticle.default_base_scale();
    let factor = scale / reference;
    let factor_cubed = factor * factor * factor;
    let input = ObjectInputType::SingleParticle.to_object_input(scale);
    if let ObjectInput::SingleParticle {
        mass,
        position,
        velocity,
        ..
    } = input
    {
        assert!((mass - 5.972e24 * factor_cubed).abs() < 1e-6);
        assert!((position.x - 1e10 * factor).abs() < 1e-6);
        assert_eq!(position.y, 0.0);
        assert_eq!(position.z, 0.0);
        assert_eq!(velocity.x, 0.0);
        assert_eq!(velocity.y, 0.0);
        assert!((velocity.z - 1e6 * factor).abs() < 1e-6);
        assert!((input.preview_group_extent() - 1.0).abs() < 1e-6);
    } else {
        panic!("expected SingleParticle");
    }
}

#[test]
fn generate_particles_uses_specified_single_particle_state() {
    let scale = 1e10;
    let mass = 1e24;
    let position = glam::DVec3::new(2e10, 3e10, 4e10);
    let velocity = glam::DVec3::new(1e5, 2e5, 3e5);
    let input = ObjectInput::SingleParticle {
        scale,
        mass,
        position,
        velocity,
        color: ParticleBasicColor::Blue,
    };
    let sim = input.generate_particles(1);
    assert_eq!(sim.particles.len(), 1);
    let p = &sim.particles[0];
    assert!((p.mass - mass / (scale * scale * scale)).abs() < 1e-6);
    assert!((p.position.x - position.x / scale).abs() < 1e-6);
    assert!((p.position.y - position.y / scale).abs() < 1e-6);
    assert!((p.position.z - position.z / scale).abs() < 1e-6);
    assert!((p.velocity.x - velocity.x / scale).abs() < 1e-6);
    assert!((p.velocity.y - velocity.y / scale).abs() < 1e-6);
    assert!((p.velocity.z - velocity.z / scale).abs() < 1e-6);
    assert_eq!(p.color, [0.2, 0.5, 1.0, 1.0]);
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

#[test]
fn spiral_disk_initial_speed_scales_with_enclosed_mass() {
    use dual_spacetime_simulator::simulation::G;

    let scale = ObjectInputType::SpiralDisk.default_base_scale();
    let input = ObjectInputType::SpiralDisk.to_object_input(scale);
    let particle_count = 128u32;
    let sim = input.generate_particles(particle_count);

    let ObjectInput::SpiralDisk {
        disk_radius,
        mass_fixed,
        ..
    } = input
    else {
        panic!("expected SpiralDisk");
    };

    let radius = disk_radius / scale;
    let mass = mass_fixed / (scale * scale * scale);
    let total_mass = particle_count as f64 * mass;
    let edge_speed = (G * total_mass / radius).sqrt();

    let mut min_speed = f64::INFINITY;
    let mut max_speed = 0.0_f64;

    for particle in &sim.particles {
        let r = (particle.position.x * particle.position.x
            + particle.position.z * particle.position.z)
            .sqrt();
        assert!(r >= radius * 0.1 - 1e-9 && r <= radius + 1e-9);

        let speed = particle.velocity.length();
        let expected = edge_speed * (r / radius).sqrt();
        assert!(
            (speed - expected).abs() < 1e-9 * edge_speed.max(1.0),
            "speed {speed} != expected {expected} at r={r}"
        );

        min_speed = min_speed.min(speed);
        max_speed = max_speed.max(speed);
    }

    assert!(min_speed < max_speed);
    let inner_expected = edge_speed * (0.1_f64).sqrt();
    let outer_expected = edge_speed;
    assert!((min_speed - inner_expected).abs() < 0.05 * edge_speed);
    assert!((max_speed - outer_expected).abs() < 0.05 * edge_speed);
}
