use dual_spacetime_simulator::gpu_simulation::GpuParticle;
use dual_spacetime_simulator::simulation::Particle;
use dual_spacetime_simulator::ui_state::SimulationType;
use glam::DVec3;

#[test]
fn gpu_particle_matches_std430_vec4_layout() {
    assert_eq!(std::mem::size_of::<GpuParticle>(), 64);
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
    assert_eq!(gpu.color, [0.2, 0.4, 0.6, 1.0]);
}
