use dual_spacetime_simulator::object_input::ObjectInput;
use dual_spacetime_simulator::simulation::{G, LY, Particle, SimulationManager};
use dual_spacetime_simulator::ui_state::SimulationType as UiSimType;
use dst_math::s3_galaxy::{GALAXY_RADIUS_LY, galaxy_radius_sim};
use glam::DVec3;

/// Galaxy-scale baryon sphere: DST galaxy runs excite disk structure from
/// sphere/cube initial conditions, so no disk-shaped preset exists.
fn galaxy_sphere_input(scale: f64) -> ObjectInput {
    ObjectInput::RandomSphere {
        scale,
        radius: GALAXY_RADIUS_LY * LY * 0.9,
        mass_range: (1e35, 1e36),
        velocity_std: 1.0,
    }
}

#[test]
fn dst_galaxy_particles_move_after_advance() {
    let scale = 1e20;
    let ic = galaxy_sphere_input(scale);
    let mgr = SimulationManager {
        state: std::sync::Arc::new(std::sync::RwLock::new(
            SimulationManager::create_simulation(ic, UiSimType::DstGalaxy, 32, scale),
        )),
    };
    let before = mgr.particles();
    assert!(!before.is_empty());
    mgr.advance(86400.0 * 365.25 * 1e6);
    let after = mgr.particles();
    let moved = before.iter().zip(after.iter()).any(|(a, b)| {
        let o_diff = (b.orientation.x - a.orientation.x).abs()
            + (b.orientation.y - a.orientation.y).abs()
            + (b.orientation.z - a.orientation.z).abs()
            + (b.orientation.w - a.orientation.w).abs();
        o_diff > 1e-20 || (b.position - a.position).length_squared() > 1e-20
    });
    assert!(moved, "DstGalaxy particles should evolve after advance");
}

#[test]
fn dst_galaxy_sphere_input_derives_orientations() {
    // Non-disk initial conditions must get their S³ coordinate derived from
    // the 3D position when the DstGalaxy simulation is built.
    let scale = 1e20;
    let ic = galaxy_sphere_input(scale);
    let state = SimulationManager::create_simulation(ic, UiSimType::DstGalaxy, 16, scale);
    let particles = match state {
        dual_spacetime_simulator::simulation::SimulationState::DstGalaxy(s) => s.particles,
        _ => panic!("expected DstGalaxy"),
    };
    assert!(!particles.is_empty());
    for p in &particles {
        assert!(p.orientation.w.is_finite());
        let has_rotation =
            p.orientation.x != 0.0 || p.orientation.y != 0.0 || p.orientation.z != 0.0;
        assert!(has_rotation, "orientation must be derived from position");
    }
}

pub fn measure_rotation_curve(particles: &[Particle], r_galaxy: f64) -> Vec<(f64, f64)> {
    let mut bins: Vec<(f64, f64, u32)> = Vec::new();
    let bin_count = 8usize;
    for p in particles {
        let r = (p.position.x * p.position.x + p.position.y * p.position.y).sqrt();
        if r <= 0.0 {
            continue;
        }
        let bin = ((r / r_galaxy) * bin_count as f64).floor() as usize;
        let bin = bin.min(bin_count - 1);
        let v = (p.velocity.x * p.position.y - p.velocity.y * p.position.x).abs() / r;
        while bins.len() <= bin {
            bins.push((0.0, 0.0, 0));
        }
        bins[bin].0 += v;
        bins[bin].2 += 1;
    }
    bins.into_iter()
        .enumerate()
        .filter(|(_, (_, _, n))| *n > 0)
        .map(|(i, (sum, _, n))| {
            let r_mid = (i as f64 + 0.5) / bin_count as f64 * r_galaxy;
            (r_mid, sum / n as f64)
        })
        .collect()
}

#[test]
fn rotation_curve_smoke_after_short_evolution() {
    let scale = 1e20;
    let ic = galaxy_sphere_input(scale);
    let mgr = SimulationManager {
        state: std::sync::Arc::new(std::sync::RwLock::new(
            SimulationManager::create_simulation(ic, UiSimType::DstGalaxy, 64, scale),
        )),
    };
    for _ in 0..20 {
        mgr.advance(86400.0 * 365.25 * 1e5);
    }
    let r_galaxy = galaxy_radius_sim(scale);
    let curve = measure_rotation_curve(&mgr.particles(), r_galaxy);
    assert!(
        curve.len() >= 2,
        "expected at least two radial bins in rotation curve"
    );
}

#[test]
fn near_field_two_body_matches_newtonian_simulation() {
    // Scene size (tens of sim units) is ~1e-9 of the galaxy radius at this scale,
    // so the S³ model must reproduce the Newtonian two-body trajectory.
    let scale = 1e10;
    let central_mass = 1e12;
    let orbit_radius = 10.0;
    let v_orb = (G * central_mass / orbit_radius).sqrt();
    let make_particles = || {
        vec![
            Particle::from_kinematics(DVec3::ZERO, DVec3::ZERO, central_mass, [1.0; 4]),
            Particle::from_kinematics(
                DVec3::new(orbit_radius, 0.0, 0.0),
                DVec3::new(0.0, v_orb, 0.0),
                1e-3,
                [1.0; 4],
            ),
        ]
    };
    let newton = SimulationManager::new();
    newton.reset_from_particles(make_particles(), UiSimType::Normal, scale);
    let galaxy = SimulationManager::new();
    galaxy.reset_from_particles(make_particles(), UiSimType::DstGalaxy, scale);

    let dt = 0.01;
    for _ in 0..1000 {
        newton.advance(dt);
        galaxy.advance(dt);
    }

    let p_newton = newton.particles()[1].position;
    let p_galaxy = galaxy.particles()[1].position;
    assert!(
        (p_newton - p_galaxy).length() < 1e-6 * orbit_radius,
        "DstGalaxy near-field diverged from Newtonian: newton={p_newton:?} galaxy={p_galaxy:?}"
    );
    let travelled = (p_newton - DVec3::new(orbit_radius, 0.0, 0.0)).length();
    assert!(
        travelled > 0.1 * orbit_radius,
        "orbiter barely moved ({travelled}); comparison is not meaningful"
    );
}

#[test]
fn dst_galaxy_zero_particles_advance_is_noop() {
    let mgr = SimulationManager {
        state: std::sync::Arc::new(std::sync::RwLock::new(
            SimulationManager::create_simulation(
                galaxy_sphere_input(1e20),
                UiSimType::DstGalaxy,
                0,
                1e20,
            ),
        )),
    };
    mgr.advance(100.0);
    assert_eq!(mgr.particle_count(), 0);
}
