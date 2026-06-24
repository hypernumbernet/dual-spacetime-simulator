use dual_spacetime_simulator::object_input::ObjectInput;
use dual_spacetime_simulator::simulation::{G, Particle, SimulationManager};
use dual_spacetime_simulator::ui_state::SimulationType as UiSimType;
use glam::DVec3;

fn total_energy(particles: &[Particle]) -> f64 {
    let mut ke = 0.0f64;
    let mut pe = 0.0f64;
    let n = particles.len();
    for i in 0..n {
        let pi = &particles[i];
        ke += 0.5 * pi.mass * pi.velocity.length_squared();
        for j in (i + 1)..n {
            let pj = &particles[j];
            let diff = pj.position - pi.position;
            let r2 = diff.length_squared().max(1e-20);
            pe -= G * pi.mass * pj.mass / r2.sqrt();
        }
    }
    ke + pe
}

#[test]
fn elliptical_two_body_energy_approximately_conserved_short_run() {
    let ic = ObjectInput::EllipticalOrbit {
        scale: 1.5e11,
        central_mass: 1.989e32,
        planetary_mass: 5.972e24,
        planetary_speed: 2.0e5,
        planetary_distance: 2.0e11,
    };
    let state = SimulationManager::create_simulation(ic, UiSimType::Normal, 2, 1e10);
    let mgr = SimulationManager {
        state: std::sync::Arc::new(std::sync::RwLock::new(state)),
    };
    let e0 = {
        let g = mgr.state.read().unwrap();
        total_energy(match &*g {
            dual_spacetime_simulator::simulation::SimulationState::Normal(s) => &s.particles,
            _ => panic!("expected Normal"),
        })
    };
    for _ in 0..40 {
        mgr.advance(3600.0 * 24.0);
    }
    let e1 = {
        let g = mgr.state.read().unwrap();
        total_energy(match &*g {
            dual_spacetime_simulator::simulation::SimulationState::Normal(s) => &s.particles,
            _ => panic!("expected Normal"),
        })
    };
    let rel = ((e1 - e0) / e0.abs().max(1.0)).abs();
    assert!(rel < 0.12, "relative energy drift {rel} e0={e0} e1={e1}");
}

#[test]
fn convert_to_lorentz_finite() {
    // `rapidity_vector` uses `(|v|^2 * scale / c).atanh()` — keep |v|^2 * scale / c < 1.
    let p = Particle {
        position: DVec3::new(1.0, 0.0, 0.0),
        velocity: DVec3::new(10.0, -20.0, 5.0),
        mass: 1e24,
        color: [1.0, 1.0, 1.0, 1.0],
    };
    let scale = 1e3;
    let out = SimulationManager::convert_to_lorentz(vec![p], scale);
    assert!(out[0].velocity.x.is_finite());
    assert!(out[0].velocity.y.is_finite());
    assert!(out[0].velocity.z.is_finite());
}

#[test]
fn speed_of_light_limit_advance_stays_finite() {
    let ic = ObjectInput::RandomSphere {
        scale: 1e10,
        radius: 1e9,
        mass_range: (1e28, 1e29),
        velocity_std: 1e5,
    };
    let state = SimulationManager::create_simulation(ic, UiSimType::SpeedOfLightLimit, 8, 1e10);
    let mgr = SimulationManager {
        state: std::sync::Arc::new(std::sync::RwLock::new(state)),
    };
    for _ in 0..20 {
        mgr.advance(1e3);
    }
    for p in mgr.particles() {
        assert!(p.position.x.is_finite());
        assert!(p.velocity.length_squared().is_finite());
    }
}

#[test]
fn clear_removes_all_particles() {
    let ic = ObjectInput::RandomSphere {
        scale: 1e10,
        radius: 1e9,
        mass_range: (1e28, 1e29),
        velocity_std: 1e5,
    };
    let state = SimulationManager::create_simulation(ic, UiSimType::Normal, 10, 1e10);
    let mgr = SimulationManager {
        state: std::sync::Arc::new(std::sync::RwLock::new(state)),
    };
    assert_eq!(mgr.particle_count(), 10);
    mgr.clear(UiSimType::Normal, 1e10);
    assert_eq!(mgr.particle_count(), 0);
}

#[test]
fn remove_particle_at_deletes_index_and_shifts_remaining() {
    let ic = ObjectInput::RandomSphere {
        scale: 1e10,
        radius: 1e9,
        mass_range: (1e28, 1e29),
        velocity_std: 1e5,
    };
    let state = SimulationManager::create_simulation(ic, UiSimType::Normal, 3, 1e10);
    let mgr = SimulationManager {
        state: std::sync::Arc::new(std::sync::RwLock::new(state)),
    };
    assert_eq!(mgr.particle_count(), 3);
    assert!(mgr.remove_particle_at(1));
    assert_eq!(mgr.particle_count(), 2);
    assert!(!mgr.remove_particle_at(2));
    assert!(mgr.remove_particle_at(0));
    assert_eq!(mgr.particle_count(), 1);
    assert!(mgr.remove_particle_at(0));
    assert_eq!(mgr.particle_count(), 0);
}

#[test]
fn advance_with_zero_particles_is_noop_for_all_simulation_types() {
    let scale = 1e10;
    for sim_type in [
        UiSimType::Normal,
        UiSimType::SpeedOfLightLimit,
        UiSimType::LorentzTransformation,
    ] {
        let mgr = SimulationManager::new();
        mgr.clear(sim_type, scale);
        assert_eq!(mgr.particle_count(), 0);
        for _ in 0..50 {
            mgr.advance(1e3);
        }
        assert_eq!(mgr.particle_count(), 0);
    }
}
