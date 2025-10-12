use crate::simulation::{AU, Particle, SimulationState};
use glam::DVec3;
use rand::Rng;
use rand_distr::Distribution;
use rand_distr::Normal;
use rand_distr::Uniform;

#[derive(Clone, PartialEq, Debug)]
pub enum InitialCondition {
    RandomCube {
        scale: f64,
        cube_size: f64,
        mass_range: (f64, f64),
        velocity_std: f64,
    },
    TwoSpheres {
        num_particles: usize,
        sphere1_center: DVec3,
        sphere1_radius: f64,
        sphere2_center: DVec3,
        sphere2_radius: f64,
        mass_fixed: f64,
    },
    SpiralDisk {
        num_particles: usize,
        disk_radius: f64,
        spiral_strength: f64,
        mass_fixed: f64,
    },
    SolarSystem,
    SatelliteOrbit {
        num_satellites: usize,
        earth_mass: f64,
        orbit_radius: f64,
    },
}

impl std::fmt::Display for InitialCondition {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            InitialCondition::RandomCube { .. } => write!(f, "Random Cube"),
            InitialCondition::TwoSpheres { .. } => write!(f, "Two Spheres"),
            InitialCondition::SpiralDisk { .. } => write!(f, "Spiral Disk"),
            InitialCondition::SolarSystem => write!(f, "Solar System"),
            InitialCondition::SatelliteOrbit { .. } => write!(f, "Satellite Orbit"),
        }
    }
}

impl InitialCondition {
    pub fn generate_particles(&self, particle_count: u32, dt: f64) -> SimulationState {
        let mut rng = rand::rng();
        let sim = match self {
            InitialCondition::RandomCube {
                scale,
                cube_size,
                mass_range,
                velocity_std,
            } => {
                let correct = Correct::new(*scale);
                let pos_max = cube_size * 0.5 * correct.m;
                let speed_max = velocity_std * correct.m;
                let particles = (0..particle_count)
                    .map(|i| {
                        let pos = DVec3 {
                            x: rng.random_range(-pos_max..pos_max),
                            y: rng.random_range(-pos_max..pos_max),
                            z: rng.random_range(-pos_max..pos_max),
                        };
                        let vel = DVec3 {
                            x: rng.random_range(-speed_max..speed_max),
                            y: rng.random_range(-speed_max..speed_max),
                            z: rng.random_range(-speed_max..speed_max),
                        };
                        let mass =
                            rng.random_range(mass_range.0 * correct.kg..mass_range.1 * correct.kg);
                        let color = match i % 5 {
                            0 => [1.0, 0.3, 0.2, 1.0], // Reddish color
                            1 => [0.2, 0.5, 1.0, 1.0], // Bluish color
                            2 => [1.0, 0.8, 0.2, 1.0], // Yellowish color
                            3 => [0.9, 0.4, 1.0, 1.0], // Purplish color
                            4 => [0.6, 1.0, 0.8, 1.0], // Cyanish color
                            _ => unreachable!(),
                        };
                        Particle {
                            position: pos,
                            velocity: vel,
                            mass,
                            color,
                        }
                    })
                    .collect();
                SimulationState {
                    particles,
                    scale: *scale,
                    dt,
                }
            }
            InitialCondition::TwoSpheres {
                num_particles,
                sphere1_center,
                sphere1_radius,
                sphere2_center,
                sphere2_radius,
                mass_fixed,
            } => {
                let mut particles = Vec::with_capacity(*num_particles);
                for _ in 0..(*num_particles / 2) {
                    particles.push(Self::random_in_sphere(
                        *sphere1_center,
                        *sphere1_radius,
                        *mass_fixed,
                        &mut rng,
                    ));
                }
                for _ in 0..(*num_particles / 2) {
                    particles.push(Self::random_in_sphere(
                        *sphere2_center,
                        *sphere2_radius,
                        *mass_fixed,
                        &mut rng,
                    ));
                }
                SimulationState {
                    particles,
                    scale: 1e10,
                    dt,
                }
            }
            InitialCondition::SpiralDisk {
                num_particles,
                disk_radius,
                spiral_strength,
                mass_fixed,
            } => {
                let particles = (0..*num_particles)
                    .map(|i| {
                        let theta =
                            (i as f64) * 2.0 * std::f64::consts::PI / (*num_particles as f64);
                        let r = rng.random_range(0.0..*disk_radius);
                        let pos = DVec3 {
                            x: r * theta.cos() + *spiral_strength * theta,
                            y: r * theta.sin() + *spiral_strength * theta,
                            z: 0.0,
                        };
                        let vel = DVec3 {
                            x: -r * theta.sin(),
                            y: r * theta.cos(),
                            z: 0.0,
                        };
                        let color = match i % 5 {
                            0 => [1.0, 0.3, 0.2, 1.0], // Reddish color
                            1 => [0.2, 0.5, 1.0, 1.0], // Bluish color
                            2 => [1.0, 0.8, 0.2, 1.0], // Yellowish color
                            3 => [0.9, 0.4, 1.0, 1.0], // Purplish color
                            4 => [0.6, 1.0, 0.8, 1.0], // Cyanish color
                            _ => unreachable!(),
                        };
                        Particle {
                            position: pos,
                            velocity: vel,
                            mass: *mass_fixed,
                            color,
                        }
                    })
                    .collect();
                SimulationState {
                    particles,
                    scale: 1e10,
                    dt,
                }
            }
            InitialCondition::SolarSystem => {
                let particles = vec![
                    // Sun
                    Particle {
                        position: DVec3 {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                        },
                        velocity: DVec3 {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                        },
                        mass: 1.989e30,
                        color: [1.0, 1.0, 0.0, 1.0], // Yellow
                    },
                    // Earth
                    Particle {
                        position: DVec3 {
                            x: 1.496e11,
                            y: 0.0,
                            z: 0.0,
                        },
                        velocity: DVec3 {
                            x: 0.0,
                            y: 29780.0,
                            z: 0.0,
                        },
                        mass: 5.972e24,
                        color: [0.2, 0.5, 1.0, 1.0], // Blue
                    },
                ];
                SimulationState {
                    particles,
                    scale: AU,
                    dt,
                }
            }
            InitialCondition::SatelliteOrbit {
                num_satellites,
                earth_mass,
                orbit_radius,
            } => {
                let mut particles = vec![
                    // Earth
                    Particle {
                        position: DVec3 {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                        },
                        velocity: DVec3 {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                        },
                        mass: *earth_mass,
                        color: [0.2, 0.5, 1.0, 1.0], // Blue
                    },
                ];
                for i in 0..*num_satellites {
                    let theta = (i as f64) * 2.0 * std::f64::consts::PI / (*num_satellites as f64);
                    let pos = DVec3 {
                        x: *orbit_radius * theta.cos(),
                        y: *orbit_radius * theta.sin(),
                        z: 0.0,
                    };
                    let vel_speed = (6.67430e-11 * *earth_mass / *orbit_radius).sqrt();
                    let vel = DVec3 {
                        x: -vel_speed * theta.sin(),
                        y: vel_speed * theta.cos(),
                        z: 0.0,
                    };
                    particles.push(Particle {
                        position: pos,
                        velocity: vel,
                        mass: 1000.0,
                        color: [1.0, 0.0, 0.0, 1.0],
                    });
                }
                SimulationState {
                    particles,
                    scale: 1e7,
                    dt,
                }
            }
        };
        sim
    }

    fn random_in_sphere(center: DVec3, radius: f64, mass: f64, rng: &mut impl Rng) -> Particle {
        loop {
            let pos = DVec3 {
                x: center.x + rng.random_range(-radius..radius),
                y: center.y + rng.random_range(-radius..radius),
                z: center.z + rng.random_range(-radius..radius),
            };
            let dist = (pos.x.powi(2) + pos.y.powi(2) + pos.z.powi(2)).sqrt();
            if dist <= radius {
                return Particle {
                    position: pos,
                    velocity: DVec3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    mass,
                    color: [0.5, 0.5, 0.5, 1.0],
                };
            }
        }
    }
}

impl Default for InitialCondition {
    fn default() -> Self {
        InitialCondition::RandomCube {
            scale: 1e10,
            cube_size: 2e10,
            mass_range: (1e31, 1e33),
            velocity_std: 1e6,
        }
    }
}

struct Correct {
    m: f64,
    kg: f64,
}

impl Correct {
    fn new(scale: f64) -> Self {
        let m = 1.0 / scale; // Scale-corrected length
        let kg = m * m * m; // Scale-corrected mass
        Self { m, kg }
    }
}
