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
        ObjectInputType::SolarSystem,
        ObjectInputType::SatelliteOrbit,
        ObjectInputType::EllipticalOrbit,
    ] {
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
        ObjectInputType::SolarSystem.default_base_scale(),
        SOLAR_SYSTEM_SCALE
    );
    assert_eq!(
        ObjectInputType::SatelliteOrbit.default_base_scale(),
        SATELLITE_ORBIT_SCALE
    );
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
    let ic = ObjectInputType::SatelliteOrbit.to_object_input(SATELLITE_ORBIT_SCALE);
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
fn get_scale_clamps_negative_input() {
    let ic = ObjectInput::RandomSphere {
        scale: -5.0,
        radius: 1e10,
        mass_range: (1e29, 1e31),
        velocity_std: 1e6,
    };
    assert_eq!(ic.get_scale(), MIN_WORLD_SCALE);
}
