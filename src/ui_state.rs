use crate::initial_condition::{InitialCondition, InitialConditionType};
use glam::DVec3;

pub const DEFAULT_SCALE_UI: f64 = 5000.0;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum SimulationType {
    Normal,
    SpeedOfLightLimit,
    LorentzTransformation,
}

impl std::fmt::Display for SimulationType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let text = match self {
            SimulationType::Normal => "Normal",
            SimulationType::SpeedOfLightLimit => "Speed of Light Limit",
            SimulationType::LorentzTransformation => "Lorentz Transformation",
        };
        write!(f, "{}", text)
    }
}

pub struct UiState {
    pub input_panel_width: f32,
    pub min_window_width: f32,
    pub min_window_height: f32,
    pub particle_count: u32,
    pub max_particle_count: u32,
    pub fps: i64,
    pub frame: i64,
    pub simulation_time: f64,
    pub time_per_frame: f64,
    pub scale: f64,
    pub scale_gauge: f64,
    pub is_running: bool,
    pub max_fps: u32,
    pub is_reset_requested: bool,
    pub is_resetting: bool,
    pub skip: u32,
    pub initial_condition_type: InitialConditionType,
    pub initial_condition: InitialCondition,
    pub simulation_type: SimulationType,
    pub is_initial_condition_window_open: bool,
    pub random_sphere: RandomSphereParameters,
    pub random_cube: RandomCubeParameters,
    pub two_spheres: TwoSpheresParameters,
    pub spiral_disk: SpiralDiskParameters,
    pub solar_system: SolarSystemParameters,
    pub satellite_orbit: SatelliteOrbitParameters,
    pub elliptical_orbit: EllipticalOrbitParameters,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            input_panel_width: 200.0,
            min_window_width: 400.0,
            min_window_height: 300.0,
            particle_count: 1000,
            max_particle_count: 20000,
            fps: 0,
            frame: 1,
            simulation_time: 0.0,
            time_per_frame: 10.0,
            scale: 1e10,
            scale_gauge: DEFAULT_SCALE_UI,
            is_running: false,
            max_fps: 60,
            is_reset_requested: false,
            is_resetting: false,
            skip: 0,
            initial_condition_type: InitialConditionType::default(),
            initial_condition: InitialCondition::default(),
            simulation_type: SimulationType::Normal,
            is_initial_condition_window_open: false,
            random_sphere: RandomSphereParameters::default(),
            random_cube: RandomCubeParameters::default(),
            two_spheres: TwoSpheresParameters::default(),
            spiral_disk: SpiralDiskParameters::default(),
            solar_system: SolarSystemParameters::default(),
            satellite_orbit: SatelliteOrbitParameters::default(),
            elliptical_orbit: EllipticalOrbitParameters::default(),
        }
    }
}

pub struct RandomSphereParameters {
    pub scale: f64,
    pub radius: f64,
    pub mass_range: (f64, f64),
    pub velocity_std: f64,
}

impl Default for RandomSphereParameters {
    fn default() -> Self {
        if let InitialCondition::RandomSphere {
            scale,
            radius,
            mass_range,
            velocity_std,
        } = InitialConditionType::RandomSphere.to_initial_condition()
        {
            Self {
                scale,
                radius,
                mass_range,
                velocity_std,
            }
        } else {
            panic!();
        }
    }
}

pub struct RandomCubeParameters {
    pub scale: f64,
    pub cube_size: f64,
    pub mass_range: (f64, f64),
    pub velocity_std: f64,
}

impl Default for RandomCubeParameters {
    fn default() -> Self {
        if let InitialCondition::RandomCube {
            scale,
            cube_size,
            mass_range,
            velocity_std,
        } = InitialConditionType::RandomCube.to_initial_condition()
        {
            Self {
                scale,
                cube_size,
                mass_range,
                velocity_std,
            }
        } else {
            panic!();
        }
    }
}

pub struct TwoSpheresParameters {
    pub scale: f64,
    pub sphere1_center: DVec3,
    pub sphere1_radius: f64,
    pub sphere2_center: DVec3,
    pub sphere2_radius: f64,
    pub mass_fixed: f64,
}

impl Default for TwoSpheresParameters {
    fn default() -> Self {
        if let InitialCondition::TwoSpheres {
            scale,
            sphere1_center,
            sphere1_radius,
            sphere2_center,
            sphere2_radius,
            mass_fixed,
        } = InitialConditionType::TwoSpheres.to_initial_condition()
        {
            Self {
                scale,
                sphere1_center,
                sphere1_radius,
                sphere2_center,
                sphere2_radius,
                mass_fixed,
            }
        } else {
            panic!();
        }
    }
}

pub struct SpiralDiskParameters {
    pub scale: f64,
    pub disk_radius: f64,
    pub mass_fixed: f64,
}

impl Default for SpiralDiskParameters {
    fn default() -> Self {
        if let InitialCondition::SpiralDisk {
            scale,
            disk_radius,
            mass_fixed,
        } = InitialConditionType::SpiralDisk.to_initial_condition()
        {
            Self {
                scale,
                disk_radius,
                mass_fixed,
            }
        } else {
            panic!();
        }
    }
}

pub struct SolarSystemParameters {
    pub start_year: i32,
    pub start_month: i32,
    pub start_day: i32,
    pub start_hour: i32,
}

impl Default for SolarSystemParameters {
    fn default() -> Self {
        if let InitialCondition::SolarSystem {
            start_year,
            start_month,
            start_day,
            start_hour,
        } = InitialConditionType::SolarSystem.to_initial_condition()
        {
            Self {
                start_year,
                start_month,
                start_day,
                start_hour,
            }
        } else {
            panic!();
        }
    }
}

pub struct SatelliteOrbitParameters {
    pub orbit_altitude_min: f64,
    pub orbit_altitude_max: f64,
    pub asteroid_mass: f64,
    pub asteroid_distance: f64,
    pub asteroid_speed: f64,
}

impl Default for SatelliteOrbitParameters {
    fn default() -> Self {
        if let InitialCondition::SatelliteOrbit {
            orbit_altitude_min,
            orbit_altitude_max,
            asteroid_mass,
            asteroid_distance,
            asteroid_speed,
        } = InitialConditionType::SatelliteOrbit.to_initial_condition()
        {
            Self {
                orbit_altitude_min,
                orbit_altitude_max,
                asteroid_mass,
                asteroid_distance,
                asteroid_speed,
            }
        } else {
            panic!();
        }
    }
}

pub struct EllipticalOrbitParameters {
    pub scale: f64,
    pub central_mass: f64,
    pub planetary_mass: f64,
    pub planetary_speed: f64,
    pub planetary_distance: f64,
}

impl Default for EllipticalOrbitParameters {
    fn default() -> Self {
        if let InitialCondition::EllipticalOrbit {
            scale,
            central_mass,
            planetary_mass,
            planetary_speed,
            planetary_distance,
        } = InitialConditionType::EllipticalOrbit.to_initial_condition()
        {
            Self {
                scale,
                central_mass,
                planetary_mass,
                planetary_speed,
                planetary_distance,
            }
        } else {
            panic!();
        }
    }
}
