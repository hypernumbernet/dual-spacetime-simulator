use dual_spacetime_simulator::initial_condition::{
    InitialCondition, InitialConditionType, SOLAR_SYSTEM_SCALE,
};

#[test]
fn get_scale_positive_for_all_variants() {
    for ty in [
        InitialConditionType::RandomSphere,
        InitialConditionType::RandomCube,
        InitialConditionType::TwoSpheres,
        InitialConditionType::SpiralDisk,
        InitialConditionType::SolarSystem,
        InitialConditionType::SatelliteOrbit,
        InitialConditionType::EllipticalOrbit,
    ] {
        let ic = ty.to_initial_condition();
        assert!(ic.get_scale() > 0.0, "{ty}");
    }
}

#[test]
fn generate_particle_count_matches_for_simple_types() {
    let n = 64u32;
    for ty in [
        InitialConditionType::RandomSphere,
        InitialConditionType::RandomCube,
        InitialConditionType::TwoSpheres,
        InitialConditionType::SpiralDisk,
    ] {
        let sim = ty.to_initial_condition().generate_particles(n);
        assert_eq!(sim.particles.len() as u32, n, "{ty}");
    }
}

#[test]
fn elliptical_orbit_always_two_bodies() {
    let ic = InitialConditionType::EllipticalOrbit.to_initial_condition();
    let sim = ic.generate_particles(999);
    assert_eq!(sim.particles.len(), 2);
}

#[test]
fn satellite_orbit_adds_satellites_beyond_two_bodies() {
    let ic = InitialConditionType::SatelliteOrbit.to_initial_condition();
    let sim = ic.generate_particles(10);
    // Earth + asteroid + (n-1) satellites
    assert_eq!(sim.particles.len(), 11);
}

#[test]
fn solar_system_scale_constant() {
    let ic = InitialCondition::SolarSystem {
        start_year: 2000,
        start_month: 1,
        start_day: 1,
        start_hour: 12,
    };
    assert_eq!(ic.get_scale(), SOLAR_SYSTEM_SCALE);
}

#[test]
fn two_spheres_masses_positive() {
    let ic = InitialConditionType::TwoSpheres.to_initial_condition();
    let sim = ic.generate_particles(20);
    assert!(sim.particles.iter().all(|p| p.mass > 0.0));
}
