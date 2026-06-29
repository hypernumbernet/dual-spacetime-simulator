mod common;

use dual_spacetime_simulator::gpu_simulation::{
    GpuParticle, GpuParticleSimulation, create_particle_descriptor_set_layout,
};
use dual_spacetime_simulator::simulation::{EPSILON, G, LIGHT_SPEED, Particle, SimulationManager};
use dual_spacetime_simulator::ui_state::SimulationType;
use dst_math::gravity::{dst_gravity_velocity_delta, schwarzschild_radius};
use glam::DVec3;
use std::sync::Arc;

#[test]
fn gpu_particle_matches_std430_vec4_layout() {
    assert_eq!(std::mem::size_of::<GpuParticle>(), 80);
}

#[test]
fn gpu_particle_roundtrip_preserves_values() {
    let particle = Particle::from_kinematics(
        DVec3::new(1.0e6, -2.0e6, 3.5e5),
        DVec3::new(1.0e3, -500.0, 0.0),
        5.0e6,
        [1.0, 0.5, 0.25, 1.0],
    );
    let gpu = GpuParticle::from_cpu(&particle, SimulationType::Normal);
    let restored = gpu.to_cpu(SimulationType::Normal, 1e10);
    assert!((particle.position - restored.position).length() < 1.0);
    assert!((particle.velocity - restored.velocity).length() < 1e-3);
    assert!((particle.mass - restored.mass).abs() < 1e-3);
    assert_eq!(particle.color, restored.color);
}

#[test]
fn gpu_particle_speed_of_light_limit_stores_momentum_in_velocity_slot() {
    let scale = 1e10;
    let particle = dual_spacetime_simulator::simulation::SimulationManager::convert_to_momentum(
        vec![Particle::from_kinematics(
            DVec3::ZERO,
            DVec3::new(0.01, 0.005, -0.002),
            1.0,
            [1.0, 1.0, 1.0, 1.0],
        )],
        scale,
    )[0];
    let gpu = GpuParticle::from_cpu(&particle, SimulationType::SpeedOfLightLimit);
    assert!((gpu.velocity[0] as f64 - particle.momentum.x).abs() < 1e-6);
    let restored = gpu.to_cpu(SimulationType::SpeedOfLightLimit, scale);
    assert!((restored.momentum - particle.momentum).length() < 1e-6);
    assert!((restored.velocity - particle.velocity).length() < 1e-9);
}

#[test]
fn gpu_particle_from_display_sets_position_and_color_only() {
    let gpu = GpuParticle::from_display([1.0, 2.0, 3.0], [0.2, 0.4, 0.6, 1.0]);
    assert_eq!(gpu.position[..3], [1.0, 2.0, 3.0]);
    assert_eq!(gpu.velocity, [0.0; 4]);
    assert_eq!(gpu.attrs, [0.0; 4]);
    assert_eq!(gpu.dual_state, [0.0; 4]);
    assert_eq!(gpu.color, [0.2, 0.4, 0.6, 1.0]);
}

#[test]
fn gpu_particle_dst_gravity_roundtrip_preserves_dual_state() {
    let scale = 1e10;
    let particle = dual_spacetime_simulator::simulation::SimulationManager::convert_to_dst_gravity(
        vec![Particle::from_kinematics(
            DVec3::new(1.0e6, 2.0e6, 3.0e6),
            DVec3::new(0.01, -0.005, 0.002),
            5.0e6,
            [1.0, 0.5, 0.25, 1.0],
        )],
        scale,
    )[0];
    let gpu = GpuParticle::from_cpu(&particle, SimulationType::DstGravity);
    let restored = gpu.to_cpu(SimulationType::DstGravity, scale);
    assert!((particle.position - restored.position).length() < 1.0);
    assert!((particle.velocity - restored.velocity).length() < 1e-3);
    assert!((particle.dual_rotor - restored.dual_rotor).length() < 1e-6);
    assert!((particle.dual_rotor_vel - restored.dual_rotor_vel).length() < 1e-9);
    assert!((particle.proper_time - restored.proper_time).abs() < 1e-9);
    assert!((particle.mass - restored.mass).abs() < 1e-3);
    assert_eq!(particle.color, restored.color);
}

#[test]
fn gpu_dst_gravity_velocity_update_inverts_inside_horizon() {
    let Some(v) = common::try_create_headless_vulkan() else {
        panic!("Vulkan initialization failed (no loader or no graphics queue)");
    };
    let allocator = v.allocator.as_ref().expect("allocator");
    let mass = 1.0e30;
    let scale = 1.0;
    let light_speed_sim = LIGHT_SPEED / scale;
    let rs = schwarzschild_radius(mass, mass, G, light_speed_sim);
    let diff = DVec3::new(0.4 * rs, 0.0, 0.0);
    let particles = SimulationManager::convert_to_dst_gravity(
        vec![
            Particle::from_kinematics(DVec3::ZERO, DVec3::ZERO, mass, [1.0; 4]),
            Particle::from_kinematics(diff, DVec3::ZERO, mass, [1.0; 4]),
        ],
        scale,
    );
    let expected = dst_gravity_velocity_delta(
        mass,
        mass,
        diff,
        G,
        light_speed_sim,
        1.0,
        EPSILON,
    );
    assert!(expected.x.is_finite() && expected.x < 0.0);

    let set_layout = create_particle_descriptor_set_layout(&v.device);
    let mut gpu_sim = GpuParticleSimulation::new(
        v.device.clone(),
        Arc::clone(allocator),
        set_layout,
        &particles,
    );
    gpu_sim.upload_from_cpu(&particles, SimulationType::DstGravity);
    let v0 = particles[0].velocity;

    common::submit_graphics(&v, |cmd| {
        gpu_sim.dispatch(cmd, SimulationType::DstGravity, 1.0, scale, 1);
    });

    let restored = gpu_sim.readback_to_cpu(SimulationType::DstGravity, scale);
    let dv_gpu = restored[0].velocity - v0;
    eprintln!(
        "gpu_dst_gravity_inversion: scale={scale:.6e} rs={rs:.6e} dv_gpu.x={:.6e} expected.x={:.6e} |dv_gpu|={:.6e}",
        dv_gpu.x,
        expected.x,
        dv_gpu.length()
    );
    assert!(dv_gpu.x < 0.0, "GPU DstGravity should repel inside horizon: dv={dv_gpu:?}");
    assert!(dv_gpu.length() > 0.0);
    assert!(dv_gpu.is_finite());
    let tol = 1e-3 * expected.length().max(1.0);
    assert!(
        (dv_gpu - expected).length() < tol,
        "GPU dv {dv_gpu:?} != CPU expected {expected:?}"
    );
}

#[test]
fn gpu_dst_gravity_velocity_update_attracts_in_weak_field() {
    let Some(v) = common::try_create_headless_vulkan() else {
        panic!("Vulkan initialization failed (no loader or no graphics queue)");
    };
    let allocator = v.allocator.as_ref().expect("allocator");
    let mass = 1.0e30;
    let scale = 1.0;
    let light_speed_sim = LIGHT_SPEED / scale;
    let rs = schwarzschild_radius(mass, mass, G, light_speed_sim);
    let diff = DVec3::new(10.0 * rs, 0.0, 0.0);
    let particles = SimulationManager::convert_to_dst_gravity(
        vec![
            Particle::from_kinematics(DVec3::ZERO, DVec3::ZERO, mass, [1.0; 4]),
            Particle::from_kinematics(diff, DVec3::ZERO, mass, [1.0; 4]),
        ],
        scale,
    );
    let expected = dst_gravity_velocity_delta(
        mass,
        mass,
        diff,
        G,
        light_speed_sim,
        1.0,
        EPSILON,
    );
    assert!(expected.x.is_finite() && expected.x > 0.0);

    let set_layout = create_particle_descriptor_set_layout(&v.device);
    let mut gpu_sim = GpuParticleSimulation::new(
        v.device.clone(),
        Arc::clone(allocator),
        set_layout,
        &particles,
    );
    gpu_sim.upload_from_cpu(&particles, SimulationType::DstGravity);
    let v0 = particles[0].velocity;

    common::submit_graphics(&v, |cmd| {
        gpu_sim.dispatch(cmd, SimulationType::DstGravity, 1.0, scale, 1);
    });

    let restored = gpu_sim.readback_to_cpu(SimulationType::DstGravity, scale);
    let dv_gpu = restored[0].velocity - v0;
    eprintln!(
        "gpu_dst_gravity_weak_field: dv_gpu.x={:.6e} expected.x={:.6e} |dv_gpu|={:.6e}",
        dv_gpu.x,
        expected.x,
        dv_gpu.length()
    );
    assert!(dv_gpu.x > 0.0, "GPU DstGravity should attract in weak field: dv={dv_gpu:?}");
    assert!(dv_gpu.length() > 0.0);
}
