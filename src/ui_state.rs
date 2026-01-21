use crate::initial_condition::{InitialCondition, InitialConditionType};

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
    pub skip: u32,
    pub initial_condition_type: InitialConditionType,
    pub previous_initial_condition_type: InitialConditionType,
    pub initial_condition: InitialCondition,
    pub simulation_type: SimulationType,
    pub is_initial_condition_window_open: bool,
    pub random_sphere: RandomSphereParameters,
    pub random_cube: RandomCubeParameters,
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
            skip: 0,
            initial_condition_type: InitialConditionType::default(),
            previous_initial_condition_type: InitialConditionType::default(),
            initial_condition: InitialCondition::default(),
            simulation_type: SimulationType::Normal,
            is_initial_condition_window_open: false,
            random_sphere: RandomSphereParameters::default(),
            random_cube: RandomCubeParameters::default(),
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
        Self {
            scale: 1e10,
            radius: 1e10,
            mass_range: (1e29, 1e31),
            velocity_std: 1e6,
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
        Self {
            scale: 1e10,
            cube_size: 2e10,
            mass_range: (1e29, 1e31),
            velocity_std: 1e6,
        }
    }
}
