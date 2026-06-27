use dual_spacetime_simulator::object_input::ObjectInput;
use dual_spacetime_simulator::particle_snapshot::ParticleSnapshot;
use dual_spacetime_simulator::simulation::SimulationManager;
use dual_spacetime_simulator::ui_state::SimulationType;
use glam::DVec3;

fn assert_dvec3_approx(a: DVec3, b: DVec3) {
    assert!((a.x - b.x).abs() < 1e-12);
    assert!((a.y - b.y).abs() < 1e-12);
    assert!((a.z - b.z).abs() < 1e-12);
}

#[test]
fn particle_snapshot_json_roundtrip() {
    let normal = ObjectInput::default().generate_particles(4);
    let snapshot = ParticleSnapshot::new(SimulationType::Normal, 1e10, normal.particles);
    let json = serde_json::to_string_pretty(&snapshot).unwrap();
    let back: ParticleSnapshot = serde_json::from_str(&json).unwrap();
    assert_eq!(snapshot.version, back.version);
    assert_eq!(snapshot.simulation_type, back.simulation_type);
    assert!((snapshot.scale - back.scale).abs() < f64::EPSILON);
    assert_eq!(snapshot.particles.len(), back.particles.len());
    for (a, b) in snapshot.particles.iter().zip(back.particles.iter()) {
        assert_dvec3_approx(a.position, b.position);
        assert_dvec3_approx(a.velocity, b.velocity);
        assert_dvec3_approx(a.momentum, b.momentum);
        assert!((a.mass - b.mass).abs() < 1e-12);
        assert_eq!(a.color, b.color);
    }
}

#[test]
fn load_from_snapshot_restores_particles() {
    let normal = ObjectInput::default().generate_particles(8);
    let particles = normal.particles;
    let expected_position = particles[3].position;

    let snapshot = ParticleSnapshot::new(SimulationType::SpeedOfLightLimit, 5e9, particles);
    let manager = SimulationManager::default();
    manager.load_from_snapshot(snapshot);

    let loaded = manager.particles();
    assert_eq!(loaded.len(), 8);
    assert_eq!(loaded[3].position, expected_position);
}

#[test]
fn particle_snapshot_file_roundtrip() {
    let particles = vec![dual_spacetime_simulator::simulation::Particle::from_kinematics(
        DVec3::new(1.0, 2.0, 3.0),
        DVec3::new(4.0, 5.0, 6.0),
        7.0,
        [0.1, 0.2, 0.3, 1.0],
    )];

    let dir = std::env::temp_dir().join("dual-spacetime-simulator-test");
    for (i, sim_type) in SimulationType::ALL.into_iter().enumerate() {
        let snapshot = ParticleSnapshot::new(sim_type, 3e9 + i as f64, particles.clone());
        let path = dir.join(format!("particles_{i}.zip"));
        snapshot.save(&path).unwrap();
        let bytes = std::fs::read(&path).unwrap();
        assert!(bytes.starts_with(b"PK"));
        let loaded = ParticleSnapshot::load(&path).unwrap();
        assert_eq!(snapshot, loaded);
        let _ = std::fs::remove_file(&path);
    }
}

#[test]
fn particle_snapshot_loads_legacy_v1_json_without_momentum() {
    let json = r#"{
        "version": 1,
        "simulation_type": "Normal",
        "scale": 1e10,
        "particles": [{
            "position": [0.0, 0.0, 0.0],
            "velocity": [1.0, 2.0, 3.0],
            "mass": 5.0,
            "color": [1.0, 1.0, 1.0, 1.0]
        }]
    }"#;

    let dir = std::env::temp_dir().join("dual-spacetime-simulator-test");
    let path = dir.join("particles_legacy_v1.json");
    std::fs::write(&path, json).unwrap();
    let loaded = ParticleSnapshot::load(&path).unwrap();
    assert_eq!(loaded.version, 1);
    assert_eq!(loaded.particles[0].momentum, DVec3::ZERO);
    let _ = std::fs::remove_file(&path);
}

#[test]
fn particle_snapshot_loads_legacy_json() {
    let particles = vec![dual_spacetime_simulator::simulation::Particle::from_kinematics(
        DVec3::ZERO,
        DVec3::ZERO,
        1.0,
        [1.0, 1.0, 1.0, 1.0],
    )];
    let snapshot = ParticleSnapshot::new(SimulationType::Normal, 1e10, particles);
    let json = serde_json::to_string_pretty(&snapshot).unwrap();

    let dir = std::env::temp_dir().join("dual-spacetime-simulator-test");
    let path = dir.join("particles_legacy.json");
    std::fs::write(&path, json).unwrap();
    let loaded = ParticleSnapshot::load(&path).unwrap();
    assert_eq!(snapshot, loaded);
    let _ = std::fs::remove_file(&path);
}
