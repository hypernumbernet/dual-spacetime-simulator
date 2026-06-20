use dual_spacetime_simulator::object_input::{
    ObjectInput, ObjectInputType, SOLAR_SYSTEM_SCALE,
};

#[test]
fn get_scale_positive_for_all_variants() {
    for ty in [
        ObjectInputType::RandomSphere,
        ObjectInputType::RandomCube,
        ObjectInputType::TwoSpheres,
        ObjectInputType::SpiralDisk,
        ObjectInputType::SolarSystem,
        ObjectInputType::SatelliteOrbit,
        ObjectInputType::EllipticalOrbit,
    ] {
        let ic = ty.to_object_input();
        assert!(ic.get_scale() > 0.0, "{ty}");
    }
}

#[test]
fn generate_particle_count_matches_for_simple_types() {
    let n = 64u32;
    for ty in [
        ObjectInputType::RandomSphere,
        ObjectInputType::RandomCube,
        ObjectInputType::TwoSpheres,
        ObjectInputType::SpiralDisk,
    ] {
        let sim = ty.to_object_input().generate_particles(n);
        assert_eq!(sim.particles.len() as u32, n, "{ty}");
    }
}

#[test]
fn elliptical_orbit_always_two_bodies() {
    let ic = ObjectInputType::EllipticalOrbit.to_object_input();
    let sim = ic.generate_particles(999);
    assert_eq!(sim.particles.len(), 2);
}

#[test]
fn satellite_orbit_adds_satellites_beyond_two_bodies() {
    let ic = ObjectInputType::SatelliteOrbit.to_object_input();
    let sim = ic.generate_particles(10);
    // Earth + asteroid + (n-1) satellites
    assert_eq!(sim.particles.len(), 11);
}

#[test]
fn solar_system_scale_constant() {
    let ic = ObjectInput::SolarSystem {
        start_year: 2000,
        start_month: 1,
        start_day: 1,
        start_hour: 12,
    };
    assert_eq!(ic.get_scale(), SOLAR_SYSTEM_SCALE);
}

#[test]
fn two_spheres_masses_positive() {
    let ic = ObjectInputType::TwoSpheres.to_object_input();
    let sim = ic.generate_particles(20);
    assert!(sim.particles.iter().all(|p| p.mass > 0.0));
}
