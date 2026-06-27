use crate::simulation::{Particle, SimulationNormal};
use crate::solar_system_data::{UpdateDataError, update_datafiles_with_log};
use glam::DVec3;
use rand::Rng;
use rand_distr::Distribution;
use satkit::{Instant, SolarSystem, jplephem};
use std::f64::consts::*;
use std::sync::atomic::{AtomicBool, Ordering};

pub const MASS_SUN: f64 = 1.988475e30;
pub const MASS_EARTH: f64 = 5.97217e24;
//pub const MASS_MOON: f64 = 7.3458e22;
pub const MASS_MERCURY: f64 = 3.3011e23;
pub const MASS_VENUS: f64 = 4.8673e24;
pub const MASS_MARS: f64 = 6.4171e23;
pub const MASS_JUPITER: f64 = 1.898125e27;
pub const MASS_SATURN: f64 = 5.68317e26;
pub const MASS_URANUS: f64 = 8.68099e25;
pub const MASS_NEPTUNE: f64 = 1.024092e26;
pub const MASS_PLUTO: f64 = 1.3025e22;
pub const SOLAR_SYSTEM_SCALE: f64 = 2.50e12;
pub const SATELLITE_ORBIT_SCALE: f64 = 12_756e3 * 0.5;
pub const EARTH_RADIUS: f64 = 6.371e6;
/// Minimum allowed world scale in meters (0.01 fm; values at or below this are clamped).
pub const MIN_WORLD_SCALE: f64 = 1e-17;

/// Default particle colors used by random batch generators (Red, Blue, Yellow, Purple, Cyan).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum ParticleBasicColor {
    #[default]
    Red,
    Blue,
    Yellow,
    Purple,
    Cyan,
}

impl ParticleBasicColor {
    /// All basic colors in UI display order.
    pub const ALL: [Self; 5] = [
        Self::Red,
        Self::Blue,
        Self::Yellow,
        Self::Purple,
        Self::Cyan,
    ];

    /// Returns RGBA components for rendering.
    pub fn rgba(self) -> [f32; 4] {
        match self {
            Self::Red => [1.0, 0.3, 0.2, 1.0],
            Self::Blue => [0.2, 0.5, 1.0, 1.0],
            Self::Yellow => [1.0, 0.8, 0.2, 1.0],
            Self::Purple => [0.9, 0.4, 1.0, 1.0],
            Self::Cyan => [0.6, 1.0, 0.8, 1.0],
        }
    }
}

impl std::fmt::Display for ParticleBasicColor {
    /// Formats basic particle color names for UI selection controls.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Red => write!(f, "Red"),
            Self::Blue => write!(f, "Blue"),
            Self::Yellow => write!(f, "Yellow"),
            Self::Purple => write!(f, "Purple"),
            Self::Cyan => write!(f, "Cyan"),
        }
    }
}

/// Clamps a world-scale value to a finite positive minimum.
pub fn clamp_world_scale(scale: f64) -> f64 {
    if scale.is_finite() && scale > MIN_WORLD_SCALE {
        scale
    } else {
        MIN_WORLD_SCALE
    }
}

#[derive(Clone, PartialEq, Debug)]
pub enum ObjectInput {
    RandomSphere {
        scale: f64,
        radius: f64,
        mass_range: (f64, f64),
        velocity_std: f64,
    },
    RandomCube {
        scale: f64,
        cube_size: f64,
        mass_range: (f64, f64),
        velocity_std: f64,
    },
    SpiralDisk {
        scale: f64,
        disk_radius: f64,
        mass_fixed: f64,
    },
    SolarSystem {
        scale: f64,
        start_year: i32,
        start_month: i32,
        start_day: i32,
        start_hour: i32,
    },
    SatelliteOrbit {
        scale: f64,
        orbit_altitude_min: f64,
        orbit_altitude_max: f64,
        satellite_count: u32,
    },
    EllipticalOrbit {
        scale: f64,
        central_mass: f64,
        planetary_mass: f64,
        planetary_speed: f64,
        planetary_distance: f64,
    },
    SingleParticle {
        scale: f64,
        mass: f64,
        position: DVec3,
        velocity: DVec3,
        color: ParticleBasicColor,
    },
}

impl std::fmt::Display for ObjectInput {
    /// Formats each object-input variant into a human-readable label.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ObjectInput::RandomSphere { .. } => write!(f, "Random Sphere"),
            ObjectInput::RandomCube { .. } => write!(f, "Random Cube"),
            ObjectInput::SpiralDisk { .. } => write!(f, "Spiral Disk"),
            ObjectInput::SolarSystem { .. } => write!(f, "Solar System"),
            ObjectInput::SatelliteOrbit { .. } => write!(f, "Satellite Orbit"),
            ObjectInput::EllipticalOrbit { .. } => write!(f, "Elliptical Orbit"),
            ObjectInput::SingleParticle { .. } => write!(f, "Single Particle"),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ObjectInputType {
    RandomSphere,
    RandomCube,
    SpiralDisk,
    EllipticalOrbit,
    SingleParticle,
}

impl Default for ObjectInputType {
    /// Selects random-sphere as the default object-input type.
    fn default() -> Self {
        ObjectInputType::RandomSphere
    }
}

impl std::fmt::Display for ObjectInputType {
    /// Formats each object-input type into a human-readable label.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ObjectInputType::RandomSphere => write!(f, "Random Sphere"),
            ObjectInputType::RandomCube => write!(f, "Random Cube"),
            ObjectInputType::SpiralDisk => write!(f, "Spiral Disk"),
            ObjectInputType::EllipticalOrbit => write!(f, "Elliptical Orbit"),
            ObjectInputType::SingleParticle => write!(f, "Single Particle"),
        }
    }
}

impl ObjectInputType {
    /// All add-type variants in UI display order.
    pub const ALL: [Self; 5] = [
        Self::RandomSphere,
        Self::RandomCube,
        Self::SpiralDisk,
        Self::EllipticalOrbit,
        Self::SingleParticle,
    ];

    /// Returns whether the add-particle-count slider applies to this type.
    pub fn uses_add_particle_count(self) -> bool {
        matches!(
            self,
            Self::RandomSphere | Self::RandomCube | Self::SpiralDisk
        )
    }

    /// Returns the recommended world scale for this object-input type.
    pub fn default_base_scale(self) -> f64 {
        match self {
            ObjectInputType::RandomSphere => 1e10,
            ObjectInputType::RandomCube => 1e10,
            ObjectInputType::SpiralDisk => 1e7,
            ObjectInputType::EllipticalOrbit => 1.5e11,
            ObjectInputType::SingleParticle => 1e10,
        }
    }

    /// Expands a condition type into concrete object-input parameters scaled for `scale`.
    pub fn to_object_input(self, scale: f64) -> ObjectInput {
        let scale = clamp_world_scale(scale);
        let reference = self.default_base_scale();
        let factor = scale / reference;
        let factor_cubed = factor * factor * factor;
        match self {
            ObjectInputType::RandomSphere => ObjectInput::RandomSphere {
                scale,
                radius: 1e10 * factor,
                mass_range: (1e29 * factor_cubed, 1e31 * factor_cubed),
                velocity_std: 1e6 * factor,
            },
            ObjectInputType::RandomCube => ObjectInput::RandomCube {
                scale,
                cube_size: 2e10 * factor,
                mass_range: (1e29 * factor_cubed, 1e31 * factor_cubed),
                velocity_std: 1e6 * factor,
            },
            ObjectInputType::SpiralDisk => ObjectInput::SpiralDisk {
                scale,
                disk_radius: 1.5e7 * factor,
                mass_fixed: 1e20 * factor_cubed,
            },
            ObjectInputType::EllipticalOrbit => ObjectInput::EllipticalOrbit {
                scale,
                central_mass: 1.989e32 * factor_cubed,
                planetary_mass: 5.972e24 * factor_cubed,
                planetary_speed: 2.0e5 * factor,
                planetary_distance: 2.0e11 * factor,
            },
            ObjectInputType::SingleParticle => ObjectInput::SingleParticle {
                scale,
                mass: 5.972e24 * factor_cubed,
                position: DVec3::new(1e10 * factor, 0.0, 0.0),
                velocity: DVec3::new(0.0, 0.0, 1e6 * factor),
                color: ParticleBasicColor::default(),
            },
        }
    }
}

/// Error returned when Solar System particle generation is aborted.
#[derive(Debug, PartialEq, Eq)]
pub enum SolarSystemBuildError {
    Aborted,
}

/// Builds Solar System particles with progress logging and cooperative abort.
pub fn build_solar_system_particles(
    scale: f64,
    start_year: i32,
    start_month: i32,
    start_day: i32,
    start_hour: i32,
    log: &impl Fn(&str),
    abort: &AtomicBool,
) -> Result<Vec<Particle>, SolarSystemBuildError> {
    let correct = Correct::new(scale);
    match update_datafiles_with_log(log, abort) {
        Ok(()) => {}
        Err(UpdateDataError::Aborted) => return Err(SolarSystemBuildError::Aborted),
        Err(err) => {
            log(&format!(
                "Failed to update data files: {err}; using fallback particles"
            ));
            return Ok(get_solar_system_fallback_particles(&correct));
        }
    }
    let time = Instant::from_datetime(start_year, start_month, start_day, start_hour, 0, 0.0)
        .unwrap_or_else(|_| Instant::from_datetime(2000, 1, 1, 12, 0, 0.0).unwrap());
    let mut particles: Vec<Particle> = vec![];
    let bodies = vec![
        SolarSystem::Mercury,
        SolarSystem::Venus,
        SolarSystem::EMB,
        SolarSystem::Mars,
        SolarSystem::Jupiter,
        SolarSystem::Saturn,
        SolarSystem::Uranus,
        SolarSystem::Neptune,
        SolarSystem::Pluto,
        SolarSystem::Sun,
    ];
    for body in bodies {
        if abort.load(Ordering::Acquire) {
            return Err(SolarSystemBuildError::Aborted);
        }
        match jplephem::barycentric_state(body, &time) {
            Ok((position, velocity)) => {
                let pos_dvec3 = DVec3 {
                    x: position.x(),
                    y: position.y(),
                    z: position.z(),
                };
                let vel_dvec3 = DVec3 {
                    x: velocity.x(),
                    y: velocity.y(),
                    z: velocity.z(),
                };
                particles.push(Particle::from_kinematics(
                    pos_dvec3 * correct.m,
                    vel_dvec3 * correct.m,
                    match body {
                        SolarSystem::Mercury => MASS_MERCURY * correct.kg,
                        SolarSystem::Venus => MASS_VENUS * correct.kg,
                        SolarSystem::EMB => MASS_EARTH * correct.kg,
                        SolarSystem::Mars => MASS_MARS * correct.kg,
                        SolarSystem::Jupiter => MASS_JUPITER * correct.kg,
                        SolarSystem::Saturn => MASS_SATURN * correct.kg,
                        SolarSystem::Uranus => MASS_URANUS * correct.kg,
                        SolarSystem::Neptune => MASS_NEPTUNE * correct.kg,
                        SolarSystem::Pluto => MASS_PLUTO * correct.kg,
                        SolarSystem::Sun => MASS_SUN * correct.kg,
                        _ => 1.0 * correct.kg,
                    },
                    match body {
                        SolarSystem::Mercury => [0.5, 0.5, 0.5, 1.0],
                        SolarSystem::Venus => [1.0, 0.8, 0.2, 1.0],
                        SolarSystem::EMB => [0.2, 0.5, 1.0, 1.0],
                        SolarSystem::Mars => [1.0, 0.3, 0.2, 1.0],
                        SolarSystem::Jupiter => [1.0, 0.9, 0.6, 1.0],
                        SolarSystem::Saturn => [1.0, 1.0, 0.6, 1.0],
                        SolarSystem::Uranus => [0.5, 1.0, 1.0, 1.0],
                        SolarSystem::Neptune => [0.2, 0.4, 1.0, 1.0],
                        SolarSystem::Pluto => [0.8, 0.7, 0.6, 1.0],
                        SolarSystem::Sun => [1.0, 1.0, 0.0, 1.0],
                        _ => [1.0, 1.0, 1.0, 1.0],
                    },
                ));
            }
            Err(e) => {
                log(&format!("Error for {:?}: {}", body, e));
            }
        }
    }
    Ok(particles)
}

/// Provides a small deterministic solar-system particle set when ephemeris data is unavailable.
fn get_solar_system_fallback_particles(correct: &Correct) -> Vec<Particle> {
    vec![
        Particle::from_kinematics(
            DVec3::ZERO,
            DVec3::ZERO,
            MASS_SUN * correct.kg,
            [1.0, 1.0, 0.0, 1.0], // Yellow
        ),
        // Earth
        Particle::from_kinematics(
            DVec3::new(1.496e11 * correct.m, 0.0, 0.0),
            DVec3::new(0.0, 0.0, 29780.0 * correct.m),
            MASS_EARTH * correct.kg,
            [0.2, 0.5, 1.0, 1.0], // Blue
        ),
        // Mars
        Particle::from_kinematics(
            DVec3::new(2.279e11 * correct.m, 0.0, 0.0),
            DVec3::new(0.0, 0.0, 24070.0 * correct.m),
            MASS_MARS * correct.kg,
            [1.0, 0.3, 0.2, 1.0], // Reddish color
        ),
        // Venus
        Particle::from_kinematics(
            DVec3::new(1.082e11 * correct.m, 0.0, 0.0),
            DVec3::new(0.0, 0.0, 35020.0 * correct.m),
            MASS_VENUS * correct.kg,
            [1.0, 0.8, 0.2, 1.0], // Yellowish color
        ),
        // Mercury
        Particle::from_kinematics(
            DVec3::new(5.791e10 * correct.m, 0.0, 0.0),
            DVec3::new(0.0, 0.0, 47360.0 * correct.m),
            MASS_MERCURY * correct.kg,
            [0.5, 0.5, 0.5, 1.0], // Grayish color
        ),
    ]
}

impl ObjectInput {
    /// Returns the canonical world scale associated with this object input.
    pub fn get_scale(&self) -> f64 {
        clamp_world_scale(match self {
            ObjectInput::RandomSphere { scale, .. } => *scale,
            ObjectInput::RandomCube { scale, .. } => *scale,
            ObjectInput::SpiralDisk { scale, .. } => *scale,
            ObjectInput::SolarSystem { scale, .. } => *scale,
            ObjectInput::SatelliteOrbit { scale, .. } => *scale,
            ObjectInput::EllipticalOrbit { scale, .. } => *scale,
            ObjectInput::SingleParticle { scale, .. } => *scale,
        })
    }

    /// Returns a characteristic world-space size for the current object-input preset.
    pub fn preview_group_extent(&self) -> f64 {
        let scale = self.get_scale();
        let correct = Correct::new(scale);
        match self {
            ObjectInput::RandomSphere { radius, .. } => radius * correct.m,
            ObjectInput::RandomCube { cube_size, .. } => cube_size * 0.5 * correct.m,
            ObjectInput::SpiralDisk { disk_radius, .. } => disk_radius * correct.m,
            ObjectInput::SolarSystem { .. } => crate::simulation::AU * correct.m,
            ObjectInput::SatelliteOrbit {
                orbit_altitude_max, ..
            } => (EARTH_RADIUS + orbit_altitude_max) * correct.m,
            ObjectInput::EllipticalOrbit {
                planetary_distance, ..
            } => planetary_distance * correct.m,
            ObjectInput::SingleParticle { position, .. } => position.length() * correct.m,
        }
    }

    /// Interprets add-center slider values; positive Y slider moves center in -Y world direction.
    pub fn add_center_effective(center: DVec3) -> DVec3 {
        DVec3::new(center.x, -center.y, center.z)
    }

    /// Converts add-center slider values into simulation-world coordinates.
    pub fn add_center_world_position(center: DVec3, base_scale: f64) -> DVec3 {
        let (scale, m) = Self::add_center_scale_factor(base_scale);
        Self::add_center_effective(center) * scale * m
    }

    /// Returns add-center octahedron arm half-length: `(0.15 * base_scale) * Correct.m`.
    pub fn add_center_marker_half_extent(base_scale: f64) -> f32 {
        let (scale, m) = Self::add_center_scale_factor(base_scale);
        (0.15 * scale * m) as f32
    }

    /// Returns marker center and arm half-length for preview rendering.
    pub fn add_center_marker_geometry(center: DVec3, base_scale: f64) -> ([f32; 3], f32) {
        let (scale, m) = Self::add_center_scale_factor(base_scale);
        let world = Self::add_center_effective(center) * scale * m;
        (
            [world.x as f32, world.y as f32, world.z as f32],
            (0.15 * scale * m) as f32,
        )
    }

    /// Preview marker geometry in axes space (grid-aligned, constant size).
    pub fn add_center_marker_preview_geometry(
        center: DVec3,
        base_scale: f64,
        visual_scale_factor: f32,
    ) -> ([f32; 3], f32) {
        let world = Self::add_center_world_position(center, base_scale);
        let scaled = [
            (world.x * visual_scale_factor as f64) as f32,
            (world.y * visual_scale_factor as f64) as f32,
            (world.z * visual_scale_factor as f64) as f32,
        ];
        (scaled, Self::add_center_marker_half_extent(base_scale))
    }

    fn add_center_scale_factor(base_scale: f64) -> (f64, f64) {
        let scale = clamp_world_scale(base_scale);
        (scale, Correct::new(scale).m)
    }

    /// Generates particles offset so the group is centered at the effective add-center position.
    pub fn generate_particles_at_center(
        &self,
        particle_count: u32,
        center: DVec3,
        base_scale: f64,
    ) -> SimulationNormal {
        let mut sim = self.generate_particles(particle_count);
        let offset = Self::add_center_world_position(center, base_scale);
        for particle in &mut sim.particles {
            particle.position += offset;
        }
        sim
    }

    /// Generates particles according to the selected object-input variant and settings.
    pub fn generate_particles(&self, particle_count: u32) -> SimulationNormal {
        let mut rng = rand::rng();
        let sim = match self {
            ObjectInput::RandomSphere {
                scale,
                radius,
                mass_range,
                velocity_std,
            } => {
                let correct = Correct::new(*scale);
                let pos_max = radius * correct.m;
                let speed_max = velocity_std * correct.m;
                let mass_lower = mass_range.0 * correct.kg;
                let mass_upper = if mass_lower >= mass_range.1 * correct.kg {
                    mass_lower * 1.01
                } else {
                    mass_range.1 * correct.kg
                };
                let particles = (0..particle_count)
                    .map(|i| {
                        let pos = Self::position_in_sphere(DVec3::ZERO, pos_max, &mut rng);
                        let vel = DVec3 {
                            x: rng.random_range(-speed_max..speed_max),
                            y: rng.random_range(-speed_max..speed_max),
                            z: rng.random_range(-speed_max..speed_max),
                        };
                        let mass = rng.random_range(mass_lower..mass_upper);
                        let color = Self::basic_particle_color(i);
                        Particle::from_kinematics(pos, vel, mass, color)
                    })
                    .collect();
                SimulationNormal { particles }
            }
            ObjectInput::RandomCube {
                scale,
                cube_size,
                mass_range,
                velocity_std,
            } => {
                let correct = Correct::new(*scale);
                let pos_max = cube_size * 0.5 * correct.m;
                let speed_max = velocity_std * correct.m;
                let mass_lower = mass_range.0 * correct.kg;
                let mass_upper = if mass_lower >= mass_range.1 * correct.kg {
                    mass_lower * 1.01
                } else {
                    mass_range.1 * correct.kg
                };
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
                        let mass = rng.random_range(mass_lower..mass_upper);
                        let color = Self::basic_particle_color(i);
                        Particle::from_kinematics(pos, vel, mass, color)
                    })
                    .collect();
                SimulationNormal { particles }
            }
            ObjectInput::SpiralDisk {
                scale,
                disk_radius,
                mass_fixed,
            } => {
                let correct = Correct::new(*scale);
                let radius = (*disk_radius).abs() * correct.m;
                let radius = if radius <= 0.1 { 0.1 } else { radius };
                let mass = *mass_fixed * correct.kg;
                let total_mass = particle_count as f64 * mass;
                let normal = rand_distr::Normal::new(0.0, radius * 0.05).unwrap();
                let particles = (0..particle_count)
                    .map(|i| {
                        let theta = (i as f64) * TAU / (particle_count as f64);
                        let r = rng.random_range(radius * 0.1..radius);
                        let r_over_radius = r / radius;
                        let enclosed_fraction = r_over_radius * r_over_radius;
                        let speed_rate =
                            (crate::simulation::G * total_mass * enclosed_fraction / r).sqrt();
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
                        let color = Self::basic_particle_color(i);
                        Particle::from_kinematics(pos, vel, mass, color)
                    })
                    .collect();
                SimulationNormal { particles }
            }
            ObjectInput::SolarSystem {
                scale,
                start_year,
                start_month,
                start_day,
                start_hour,
            } => {
                static NO_ABORT: AtomicBool = AtomicBool::new(false);
                let particles = build_solar_system_particles(
                    *scale,
                    *start_year,
                    *start_month,
                    *start_day,
                    *start_hour,
                    &|line| println!("{}", line),
                    &NO_ABORT,
                )
                .unwrap_or_else(|_| get_solar_system_fallback_particles(&Correct::new(*scale)));
                SimulationNormal { particles }
            }
            ObjectInput::SatelliteOrbit {
                scale,
                orbit_altitude_min,
                orbit_altitude_max,
                satellite_count,
            } => {
                let correct = Correct::new(*scale);
                let earth_mass = MASS_EARTH * correct.kg;
                let gm_earth = crate::simulation::G * earth_mass;
                let radius_scale = correct.m;
                let alt_min = *orbit_altitude_min;
                let alt_max = *orbit_altitude_max;
                let mass_min = 500.0 * correct.kg;
                let mass_max = 1000.0 * correct.kg;

                let mut particles = Vec::with_capacity(1 + *satellite_count as usize);
                particles.push(Particle::from_kinematics(
                    DVec3::ZERO,
                    DVec3::ZERO,
                    earth_mass,
                    [0.2, 0.5, 1.0, 1.0], // Blue
                ));
                for _ in 0..*satellite_count {
                    let orbit_radius =
                        (EARTH_RADIUS + rng.random_range(alt_min..alt_max)) * radius_scale;
                    let cos_theta = rng.random::<f64>() * 2.0 - 1.0;
                    let sin_theta = (1.0 - cos_theta * cos_theta).sqrt();
                    let phi = rng.random::<f64>() * TAU;
                    let pos = DVec3 {
                        x: orbit_radius * sin_theta * phi.cos(),
                        y: orbit_radius * sin_theta * phi.sin(),
                        z: orbit_radius * cos_theta,
                    };
                    let vel_speed = (gm_earth / orbit_radius).sqrt();
                    particles.push(Particle::from_kinematics(
                        pos,
                        Self::random_perpendicular_unit_vector(pos, &mut rng) * vel_speed,
                        rng.random_range(mass_min..mass_max),
                        [1.0, 1.0, 1.0, 1.0],
                    ));
                }
                SimulationNormal { particles }
            }
            ObjectInput::EllipticalOrbit {
                scale,
                central_mass,
                planetary_mass,
                planetary_speed,
                planetary_distance,
            } => {
                let correct = Correct::new(*scale);
                let central_mass = *central_mass * correct.kg;
                let planetary_mass = *planetary_mass * correct.kg;
                let planetary_distance = *planetary_distance * correct.m;
                let planetary_speed = *planetary_speed * correct.m;
                let particles = vec![
                    Particle::from_kinematics(
                        DVec3::ZERO,
                        DVec3::ZERO,
                        central_mass,
                        [1.0, 1.0, 0.0, 1.0], // Yellow
                    ),
                    Particle::from_kinematics(
                        DVec3::new(planetary_distance, 0.0, 0.0),
                        DVec3::new(0.0, 0.0, planetary_speed),
                        planetary_mass,
                        [0.2, 0.5, 1.0, 1.0], // Blue
                    ),
                ];
                SimulationNormal { particles }
            }
            ObjectInput::SingleParticle {
                scale,
                mass,
                position,
                velocity,
                color,
            } => {
                let correct = Correct::new(*scale);
                let particles = vec![Particle::from_kinematics(
                    *position * correct.m,
                    *velocity * correct.m,
                    *mass * correct.kg,
                    color.rgba(),
                )];
                SimulationNormal { particles }
            }
        };
        sim
    }

    /// Returns one of the basic particle colors by index.
    fn basic_particle_color(index: u32) -> [f32; 4] {
        ParticleBasicColor::ALL[(index as usize) % ParticleBasicColor::ALL.len()].rgba()
    }

    /// Samples a uniformly distributed position inside a sphere around the given center.
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

    /// Samples a random unit vector orthogonal to the provided direction vector.
    fn random_perpendicular_unit_vector(x: DVec3, rng: &mut impl Rng) -> DVec3 {
        let n = x.normalize();
        let a = if n.x.abs() > 0.9 { DVec3::Y } else { DVec3::X };
        let u = n.cross(a).normalize();
        let v = n.cross(u).normalize();
        let theta = rng.random_range(0.0..std::f64::consts::TAU);
        u * theta.cos() + v * theta.sin()
    }
}

impl Default for ObjectInput {
    /// Creates the default object-input instance from the default type preset.
    fn default() -> Self {
        ObjectInputType::RandomSphere.to_object_input(1e10)
    }
}

struct Correct {
    m: f64,
    kg: f64,
}

impl Correct {
    /// Builds unit-conversion factors from world scale into simulation units.
    fn new(scale: f64) -> Self {
        let scale = clamp_world_scale(scale);
        let m = 1.0 / scale; // Scale-corrected length
        let kg = m * m * m; // Scale-corrected mass
        Self { m, kg }
    }
}
