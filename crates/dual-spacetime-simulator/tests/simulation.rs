use dual_spacetime_simulator::object_input::ObjectInput;
use dual_spacetime_simulator::simulation::{
    G, LIGHT_SPEED, Particle, SimulationManager, clamp_scalar_speed_m_s, clamp_velocity_m_s,
    max_subluminal_speed_m_s,
};
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
fn clamp_scalar_speed_m_s_leaves_subluminal_unchanged() {
    let sub = LIGHT_SPEED * 0.5;
    assert_eq!(clamp_scalar_speed_m_s(sub), sub);
}

#[test]
fn clamp_scalar_speed_m_s_caps_at_subluminal_fraction() {
    assert_eq!(clamp_scalar_speed_m_s(LIGHT_SPEED), max_subluminal_speed_m_s());
    assert_eq!(
        clamp_scalar_speed_m_s(LIGHT_SPEED * 2.0),
        max_subluminal_speed_m_s()
    );
}

#[test]
fn clamp_velocity_m_s_preserves_direction() {
    let v = DVec3::new(LIGHT_SPEED, LIGHT_SPEED, 0.0);
    let clamped = clamp_velocity_m_s(v);
    let expected_speed = max_subluminal_speed_m_s();
    assert!((clamped.length() - expected_speed).abs() < expected_speed * 1e-12);
    let dir = v.normalize();
    assert!((clamped.normalize() - dir).length() < 1e-12);
}

#[test]
fn create_simulation_lorentz_with_superluminal_velocity_std_stays_finite() {
    let ic = ObjectInput::RandomSphere {
        scale: 1e10,
        radius: 1e9,
        mass_range: (1e28, 1e29),
        velocity_std: LIGHT_SPEED,
    };
    let state = SimulationManager::create_simulation(ic, UiSimType::LorentzTransformation, 16, 1e10);
    let particles = match state {
        dual_spacetime_simulator::simulation::SimulationState::LorentzTransformation(s) => {
            s.particles
        }
        _ => panic!("expected LorentzTransformation"),
    };
    for p in &particles {
        assert!(p.velocity.x.is_finite());
        assert!(p.velocity.y.is_finite());
        assert!(p.velocity.z.is_finite());
    }
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
    let p = Particle::from_kinematics(
        DVec3::new(1.0, 0.0, 0.0),
        DVec3::new(10.0, -20.0, 5.0),
        1e24,
        [1.0, 1.0, 1.0, 1.0],
    );
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
        assert!(p.momentum.length_squared().is_finite());
    }
}

#[test]
fn speed_of_light_limit_huge_momentum_stays_finite() {
    let scale = 1e10;
    let mut particle = SimulationManager::convert_to_momentum(
        vec![Particle::from_kinematics(
            DVec3::ZERO,
            DVec3::new(0.01, 0.0, 0.0),
            1e24,
            [1.0, 1.0, 1.0, 1.0],
        )],
        scale,
    )[0];
    particle.momentum = DVec3::new(1e30, 0.0, 0.0);
    let state = dual_spacetime_simulator::simulation::SimulationState::SpeedOfLightLimit(
        dual_spacetime_simulator::simulation::SimulationSpeedOfLightLimit {
            particles: vec![particle],
            scale,
        },
    );
    let mgr = SimulationManager {
        state: std::sync::Arc::new(std::sync::RwLock::new(state)),
    };
    for _ in 0..10 {
        mgr.advance(1e3);
    }
    let p = &mgr.particles()[0];
    assert!(p.position.is_finite());
    assert!(p.velocity.is_finite());
    assert!(p.momentum.is_finite());
    let ls = dual_spacetime_simulator::simulation::LIGHT_SPEED / scale;
    assert!(p.velocity.length() <= ls * 1.0001);
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
    let scale = dual_spacetime_simulator::simulation::DEFAULT_WORLD_SCALE;
    for sim_type in UiSimType::ALL {
        let mgr = SimulationManager::new();
        mgr.clear(sim_type, scale);
        assert_eq!(mgr.particle_count(), 0);
        for _ in 0..50 {
            mgr.advance(1e3);
        }
        assert_eq!(mgr.particle_count(), 0);
    }
}

#[test]
fn dst_gravity_two_body_short_run_stays_finite() {
    let scale = 1e10;
    let particles = SimulationManager::convert_to_dst_gravity(
        vec![
            Particle::from_kinematics(
                DVec3::ZERO,
                DVec3::ZERO,
                1.0e3,
                [1.0, 1.0, 1.0, 1.0],
            ),
            Particle::from_kinematics(
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(0.0, 0.01, 0.0),
                1.0e2,
                [0.8, 0.8, 1.0, 1.0],
            ),
        ],
        scale,
    );
    let mgr = SimulationManager {
        state: std::sync::Arc::new(std::sync::RwLock::new(
            dual_spacetime_simulator::simulation::SimulationState::DstGravity(
                dual_spacetime_simulator::simulation::SimulationDstGravity {
                    particles,
                    scale,
                },
            ),
        )),
    };
    for _ in 0..50 {
        mgr.advance(1.0);
    }
    for p in mgr.particles() {
        assert!(p.position.x.is_finite());
        assert!(p.position.y.is_finite());
        assert!(p.position.z.is_finite());
        assert!(p.velocity.x.is_finite());
        assert!(p.velocity.y.is_finite());
        assert!(p.velocity.z.is_finite());
        assert!(p.dual_rotor.x.is_finite());
        assert!(p.dual_rotor.y.is_finite());
        assert!(p.dual_rotor.z.is_finite());
        assert!(p.proper_time.is_finite());
    }
}

#[test]
fn dst_gravity_elliptical_orbit_repro_divergence() {
    let scale = 1.5e11;
    let ic = ObjectInput::EllipticalOrbit {
        scale,
        central_mass: 1.989e32,
        planetary_mass: 5.972e24,
        planetary_speed: 2.0e5,
        planetary_distance: 2.0e11,
    };
    let state = SimulationManager::create_simulation(ic, UiSimType::DstGravity, 2, scale);
    let mgr = SimulationManager {
        state: std::sync::Arc::new(std::sync::RwLock::new(state)),
    };
    for frame in 1..=25 {
        mgr.advance(10.0);
        let particles = mgr.particles();
        for p in &particles {
            assert!(p.position.x.is_finite(), "diverged frame {frame} pos={:?}", p.position);
            assert!(p.position.length() < 1e6, "exploded frame {frame} pos_len={}", p.position.length());
        }
    }
}

#[test]
fn dst_gravity_random_sphere_stays_finite_short_run() {
    use dual_spacetime_simulator::simulation::DEFAULT_WORLD_SCALE;
    let scale = DEFAULT_WORLD_SCALE;
    let ic = ObjectInput::RandomSphere {
        scale,
        radius: 1e10,
        mass_range: (1e29, 1e31),
        velocity_std: 1e6,
    };
    let state = SimulationManager::create_simulation(ic, UiSimType::DstGravity, 32, scale);
    let mgr = SimulationManager {
        state: std::sync::Arc::new(std::sync::RwLock::new(state)),
    };
    for frame in 1..=25 {
        mgr.advance(10.0);
        for p in mgr.particles() {
            assert!(p.position.x.is_finite(), "diverged frame {frame} pos={:?}", p.position);
            assert!(
                p.position.length() < 1e8,
                "exploded frame {frame} pos_len={} phi={:?} phi_vel={:?}",
                p.position.length(),
                p.dual_rotor,
                p.dual_rotor_vel
            );
        }
    }
}

#[test]
fn torsion_star_shell_expands_under_central_mass() {
    let scale = 1e10;
    let ic = ObjectInput::TorsionStar {
        scale,
        central_mass: 1.989e32,
        shell_radius: 5.0e9,
        shell_particle_count: 8,
        shell_mass: 1.0e28,
        shell_phi_magnitude: 1.0,
    };
    let state = SimulationManager::create_simulation(ic, UiSimType::DstGravity, 9, scale);
    let mgr = SimulationManager {
        state: std::sync::Arc::new(std::sync::RwLock::new(state)),
    };
    let r0 = {
        let p = &mgr.particles()[1];
        p.position.length()
    };
    for _ in 0..30 {
        mgr.advance(100.0);
    }
    let r1 = {
        let p = &mgr.particles()[1];
        p.position.length()
    };
    assert!(
        r1 > r0 * 1.05,
        "repulsion shell should expand: r0={r0} r1={r1}"
    );
}

#[test]
fn dst_gravity_random_sphere_large_dt_stays_finite() {
    use dual_spacetime_simulator::simulation::DEFAULT_WORLD_SCALE;
    let scale = DEFAULT_WORLD_SCALE;
    let ic = ObjectInput::RandomSphere {
        scale,
        radius: 1e10,
        mass_range: (1e29, 1e31),
        velocity_std: 1e6,
    };
    let state = SimulationManager::create_simulation(ic, UiSimType::DstGravity, 64, scale);
    let mgr = SimulationManager {
        state: std::sync::Arc::new(std::sync::RwLock::new(state)),
    };
    for frame in 1..=100 {
        mgr.advance(10_000.0);
        for p in mgr.particles() {
            assert!(p.position.x.is_finite(), "diverged frame {frame} pos={:?}", p.position);
            assert!(
                p.position.length() < 1e12,
                "exploded frame {frame} pos_len={}",
                p.position.length()
            );
        }
    }
}
