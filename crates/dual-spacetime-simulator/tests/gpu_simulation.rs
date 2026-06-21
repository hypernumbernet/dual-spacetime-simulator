use dual_spacetime_simulator::gpu_simulation::GpuParticle;
use dual_spacetime_simulator::simulation::Particle;
use glam::DVec3;

#[test]
fn gpu_particle_matches_std430_vec4_layout() {
    assert_eq!(std::mem::size_of::<GpuParticle>(), 64);
}

#[test]
fn gpu_particle_roundtrip_preserves_values() {
    let particle = Particle {
        position: DVec3::new(1.0e6, -2.0e6, 3.5e5),
        velocity: DVec3::new(1.0e3, -500.0, 0.0),
        mass: 5.0e6,
        color: [1.0, 0.5, 0.25, 1.0],
    };
    let gpu = GpuParticle::from_cpu(&particle);
    let restored = gpu.to_cpu();
    assert!((particle.position - restored.position).length() < 1.0);
    assert!((particle.velocity - restored.velocity).length() < 1e-3);
    assert!((particle.mass - restored.mass).abs() < 1e-3);
    assert_eq!(particle.color, restored.color);
}
