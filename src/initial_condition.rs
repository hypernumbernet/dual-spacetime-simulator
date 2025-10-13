use crate::simulation::{Particle, SimulationState};
use glam::DVec3;
use rand::Rng;
use rand_distr::Distribution;
use std::f64::consts::*;
use vulkano::half;

#[derive(Clone, PartialEq, Debug)]
pub enum InitialCondition {
    RandomCube {
        scale: f64,
        cube_size: f64,
        mass_range: (f64, f64),
        velocity_std: f64,
    },
    TwoSpheres {
        scale: f64,
        sphere1_center: DVec3,
        sphere1_radius: f64,
        sphere2_center: DVec3,
        sphere2_radius: f64,
        mass_fixed: f64,
    },
    SpiralDisk {
        scale: f64,
        disk_radius: f64,
        mass_fixed: f64,
    },
    SolarSystem,
    SatelliteOrbit {
        earth_mass: f64,
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
                            0 => [1.0, 0.3, 0.2, 1.0], // Red
                            1 => [0.2, 0.5, 1.0, 1.0], // Blue
                            2 => [1.0, 0.8, 0.2, 1.0], // Yellow
                            3 => [0.9, 0.4, 1.0, 1.0], // Purple
                            4 => [0.6, 1.0, 0.8, 1.0], // Cyan
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
                scale,
                sphere1_center,
                sphere1_radius,
                sphere2_center,
                sphere2_radius,
                mass_fixed,
            } => {
                let correct = Correct::new(*scale);
                let sphere1_center = *sphere1_center * correct.m;
                let sphere1_radius = *sphere1_radius * correct.m;
                let sphere2_center = *sphere2_center * correct.m;
                let sphere2_radius = *sphere2_radius * correct.m;
                let mass = *mass_fixed * correct.kg;
                let mut particles = Vec::with_capacity(particle_count as usize);
                let half = particle_count / 2;
                for _ in 0..half {
                    particles.push(Self::random_in_sphere(
                        sphere1_center,
                        sphere1_radius,
                        mass,
                        &mut rng,
                    ));
                }
                for _ in half..particle_count {
                    particles.push(Self::random_in_sphere(
                        sphere2_center,
                        sphere2_radius,
                        mass,
                        &mut rng,
                    ));
                }
                SimulationState {
                    particles,
                    scale: *scale,
                    dt,
                }
            }
            InitialCondition::SpiralDisk {
                scale,
                disk_radius,
                mass_fixed,
            } => {
                let correct = Correct::new(*scale);
                let radius = *disk_radius * correct.m;
                let mass = *mass_fixed * correct.kg;
                let total_mass = particle_count as f64 * mass;
                let normal = rand_distr::Normal::new(0.0, radius * 0.05).unwrap();
                let particles = (0..particle_count)
                    .map(|i| {
                        let theta = (i as f64) * TAU / (particle_count as f64);
                        let r = rng.random_range(radius * 0.1..radius);
                        let speed_rate =
                            (crate::simulation::G * total_mass * (r / radius) / r).sqrt();
                        let y_thickness = normal.sample(&mut rng);
                        let pos = DVec3 {
                            x: r * theta.cos(),
                            y: y_thickness,
                            z: r * theta.sin(),
                        };
                        let vel = DVec3 {
                            x: -theta.sin() * speed_rate,
                            y: 0.0,
                            z: theta.cos() * speed_rate,
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
            InitialCondition::SolarSystem => {
                let scale = 1.5e11;
                let correct = Correct::new(scale);
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
                        mass: 1.989e30 * correct.kg,
                        color: [1.0, 1.0, 0.0, 1.0], // Yellow
                    },
                    // Earth
                    Particle {
                        position: DVec3 {
                            x: 1.496e11 * correct.m,
                            y: 0.0,
                            z: 0.0,
                        },
                        velocity: DVec3 {
                            x: 0.0,
                            y: 0.0,
                            z: 29780.0 * correct.m,
                        },
                        mass: 5.972e24 * correct.kg,
                        color: [0.2, 0.5, 1.0, 1.0], // Blue
                    },
                    // Mars
                    Particle {
                        position: DVec3 {
                            x: 2.279e11 * correct.m,
                            y: 0.0,
                            z: 0.0,
                        },
                        velocity: DVec3 {
                            x: 0.0,
                            y: 0.0,
                            z: 24070.0 * correct.m,
                        },
                        mass: 6.39e23 * correct.kg,
                        color: [1.0, 0.3, 0.2, 1.0], // Reddish color
                    },
                    // Venus
                    Particle {
                        position: DVec3 {
                            x: 1.082e11 * correct.m,
                            y: 0.0,
                            z: 0.0,
                        },
                        velocity: DVec3 {
                            x: 0.0,
                            y: 0.0,
                            z: 35020.0 * correct.m,
                        },
                        mass: 4.867e24 * correct.kg,
                        color: [1.0, 0.8, 0.2, 1.0], // Yellowish color
                    },
                    // Mercury
                    Particle {
                        position: DVec3 {
                            x: 5.791e10 * correct.m,
                            y: 0.0,
                            z: 0.0,
                        },
                        velocity: DVec3 {
                            x: 0.0,
                            y: 0.0,
                            z: 47360.0 * correct.m,
                        },
                        mass: 3.285e23 * correct.kg,
                        color: [0.5, 0.5, 0.5, 1.0], // Grayish color
                    },
                ];
                SimulationState {
                    particles,
                    scale,
                    dt,
                }
            }
            InitialCondition::SatelliteOrbit { earth_mass } => {
                let scale = 12_756e3 * 0.5;
                let correct = Correct::new(scale);
                let mass = earth_mass * correct.kg;
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
                        mass,
                        color: [0.2, 0.5, 1.0, 1.0], // Blue
                    },
                ];
                for _ in 0..particle_count {
                    let orbit_radius = (scale + rng.random_range(100e3..500e3)) * correct.m;
                    let cos_theta = rng.random::<f64>() * 2.0 - 1.0;
                    let sin_theta = (1.0 - cos_theta * cos_theta).sqrt();
                    let phi = rng.random::<f64>() * TAU;
                    let pos = DVec3 {
                        x: orbit_radius * sin_theta * phi.cos(),
                        y: orbit_radius * sin_theta * phi.sin(),
                        z: orbit_radius * cos_theta,
                    };
                    let vel_speed = (crate::simulation::G * mass / orbit_radius).sqrt();
                    let vel = Self::random_perpendicular_unit_vector(pos, &mut rng);
                    let vel = vel * vel_speed;
                    particles.push(Particle {
                        position: pos,
                        velocity: vel,
                        mass: 1000.0 * correct.kg,
                        color: [1.0, 1.0, 1.0, 1.0],
                    });
                }
                SimulationState {
                    particles,
                    scale,
                    dt,
                }
            }
        };
        sim
    }

    fn random_in_sphere(center: DVec3, radius: f64, mass: f64, rng: &mut impl Rng) -> Particle {
        Particle {
            position: Self::position_in_sphere(center, radius, rng),
            velocity: DVec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            mass,
            color: [0.5, 0.5, 0.5, 1.0],
        }
    }

    fn position_in_sphere(center: DVec3, radius: f64, rng: &mut impl Rng) -> DVec3 {
        let r = radius * rng.random::<f64>().cbrt();
        let cos_theta = rng.random::<f64>() * 2.0 - 1.0;
        let sin_theta = (1.0 - cos_theta * cos_theta).sqrt();
        let phi = rng.random::<f64>() * TAU;
        DVec3 {
            x: center.x + r * sin_theta * phi.cos(),
            y: center.y + r * sin_theta * phi.sin(),
            z: center.z + r * cos_theta,
        }
    }

    fn random_perpendicular_unit_vector(x: DVec3, rng: &mut impl Rng) -> DVec3 {
        let n = x.normalize();
        let a = if n.x.abs() > 0.9 { DVec3::Y } else { DVec3::X };
        let u = n.cross(a).normalize();
        let v = n.cross(u).normalize();
        let theta = rng.random_range(0.0..std::f64::consts::TAU);
        u * theta.cos() + v * theta.sin()
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
