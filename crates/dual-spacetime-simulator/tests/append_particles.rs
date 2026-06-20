use dual_spacetime_simulator::object_input::{ObjectInput, ObjectInputType};
use dual_spacetime_simulator::simulation::{
    Particle, SimulationManager, SimulationNormal, SimulationState,
};
use dual_spacetime_simulator::ui_state::SimulationType as UiSimType;
use glam::DVec3;
use std::sync::{Arc, RwLock};

fn random_sphere_input(scale: f64) -> ObjectInput {
    ObjectInputType::RandomSphere.to_object_input(scale)
}

fn manager_with_particles(particles: Vec<Particle>) -> SimulationManager {
    SimulationManager {
        state: Arc::new(RwLock::new(SimulationState::Normal(SimulationNormal {
            particles,
        }))),
    }
}

#[test]
fn append_particles_increases_count() {
    let scale = 1e10;
    let mgr = manager_with_particles(vec![Particle {
        position: DVec3::new(100.0, 0.0, 0.0),
        velocity: DVec3::new(1.0, 2.0, 3.0),
        mass: 1.0,
        color: [1.0, 0.0, 0.0, 1.0],
    }]);
    let added = mgr.append_particles(
        random_sphere_input(scale),
        UiSimType::Normal,
        16,
        scale,
        DVec3::ZERO,
        scale,
        20_000,
    );
    assert_eq!(added, 16);
    assert_eq!(mgr.particles().len(), 17);
}

#[test]
fn append_particles_preserves_existing_particles() {
    let scale = 1e10;
    let existing = Particle {
        position: DVec3::new(100.0, 0.0, 0.0),
        velocity: DVec3::new(1.0, 2.0, 3.0),
        mass: 1.0,
        color: [1.0, 0.0, 0.0, 1.0],
    };
    let mgr = manager_with_particles(vec![existing]);
    mgr.append_particles(
        random_sphere_input(scale),
        UiSimType::Normal,
        8,
        scale,
        DVec3::ZERO,
        scale,
        20_000,
    );
    let first = &mgr.particles()[0];
    assert_eq!(first.position, existing.position);
    assert_eq!(first.velocity, existing.velocity);
    assert_eq!(first.mass, existing.mass);
}

#[test]
fn append_particles_offsets_positions_by_base_scale() {
    let base_scale = 1e10;
    let mgr = manager_with_particles(vec![]);
    let object_input = ObjectInput::RandomSphere {
        scale: base_scale,
        radius: base_scale,
        mass_range: (1e20, 1e21),
        velocity_std: 1e3,
    };
    let center = DVec3::new(2.0, 3.0, 4.0);
    mgr.append_particles(
        object_input,
        UiSimType::Normal,
        32,
        base_scale,
        center,
        base_scale,
        20_000,
    );
    let offset = ObjectInput::add_center_world_position(center, base_scale);
    let radius = 1.0;
    for particle in mgr.particles() {
        assert!(particle.position.x >= offset.x - radius);
        assert!(particle.position.x <= offset.x + radius);
        assert!(particle.position.y >= offset.y - radius);
        assert!(particle.position.y <= offset.y + radius);
        assert!(particle.position.z >= offset.z - radius);
        assert!(particle.position.z <= offset.z + radius);
    }
}

#[test]
fn append_particles_respects_max_count() {
    let scale = 1e10;
    let existing: Vec<Particle> = (0..5)
        .map(|i| Particle {
            position: DVec3::new(i as f64, 0.0, 0.0),
            velocity: DVec3::ZERO,
            mass: 1.0,
            color: [1.0, 1.0, 1.0, 1.0],
        })
        .collect();
    let mgr = manager_with_particles(existing);
    let added = mgr.append_particles(
        random_sphere_input(scale),
        UiSimType::Normal,
        10,
        scale,
        DVec3::ZERO,
        scale,
        8,
    );
    assert_eq!(added, 3);
    assert_eq!(mgr.particles().len(), 8);
}

#[test]
fn append_particles_lorentz_mode() {
    let scale = 1e10;
    let mgr = SimulationManager {
        state: Arc::new(RwLock::new(
            dual_spacetime_simulator::simulation::SimulationState::LorentzTransformation(
                dual_spacetime_simulator::simulation::SimulationLorentzTransformation {
                    particles: vec![],
                    scale,
                },
            ),
        )),
    };
    let added = mgr.append_particles(
        random_sphere_input(scale),
        UiSimType::LorentzTransformation,
        4,
        scale,
        DVec3::ZERO,
        scale,
        20_000,
    );
    assert_eq!(added, 4);
    for particle in mgr.particles() {
        assert!(particle.velocity.x.is_finite());
        assert!(particle.velocity.y.is_finite());
        assert!(particle.velocity.z.is_finite());
    }
}