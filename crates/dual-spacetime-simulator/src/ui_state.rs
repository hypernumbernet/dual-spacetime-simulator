use crate::object_input::{
    clamp_world_scale, ObjectInput, ObjectInputType, MIN_WORLD_SCALE, SATELLITE_ORBIT_SCALE,
    SOLAR_SYSTEM_SCALE,
};
use crate::settings::AppSettings;
use crate::simulation::{AU, LY, MPC, PC};
use glam::DVec3;

pub const DEFAULT_SCALE_UI: f64 = 5000.0;
pub const BASE_SCALE_DRAG_SPEED: f64 = 0.01;

#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub enum BaseScaleUnit {
    Mpc,
    Pc,
    Ly,
    Au,
    #[default]
    Km,
    M,
    Mm,
    Nm,
    Fm,
}

impl std::fmt::Display for BaseScaleUnit {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let text = match self {
            BaseScaleUnit::Mpc => "Mpc",
            BaseScaleUnit::Pc => "pc",
            BaseScaleUnit::Ly => "ly",
            BaseScaleUnit::Au => "au",
            BaseScaleUnit::Km => "km",
            BaseScaleUnit::M => "m",
            BaseScaleUnit::Mm => "mm",
            BaseScaleUnit::Nm => "nm",
            BaseScaleUnit::Fm => "fm",
        };
        write!(f, "{}", text)
    }
}

impl BaseScaleUnit {
    /// All units in descending size order (largest first).
    pub const ALL: [Self; 9] = [
        Self::Mpc,
        Self::Pc,
        Self::Ly,
        Self::Au,
        Self::Km,
        Self::M,
        Self::Mm,
        Self::Nm,
        Self::Fm,
    ];

    /// Returns how many meters one unit of this scale represents.
    pub fn meters_per_unit(self) -> f64 {
        match self {
            BaseScaleUnit::Mpc => MPC,
            BaseScaleUnit::Pc => PC,
            BaseScaleUnit::Ly => LY,
            BaseScaleUnit::Au => AU,
            BaseScaleUnit::Km => 1e3,
            BaseScaleUnit::M => 1.0,
            BaseScaleUnit::Mm => 1e-3,
            BaseScaleUnit::Nm => 1e-9,
            BaseScaleUnit::Fm => 1e-15,
        }
    }

    /// Converts a display value in this unit to meters.
    pub fn to_meters(self, display: f64) -> f64 {
        display * self.meters_per_unit()
    }

    /// Converts a meter value to this unit for display.
    pub fn from_meters(self, meters: f64) -> f64 {
        meters / self.meters_per_unit()
    }

    /// Decimal places used when rounding display values for this unit.
    pub fn display_decimal_places(self) -> i32 {
        match self {
            BaseScaleUnit::Mpc | BaseScaleUnit::Pc | BaseScaleUnit::Ly | BaseScaleUnit::Au => 6,
            BaseScaleUnit::Km | BaseScaleUnit::M => 3,
            BaseScaleUnit::Mm => 6,
            BaseScaleUnit::Nm | BaseScaleUnit::Fm => 2,
        }
    }

    /// Rounds a display value to a unit-appropriate precision.
    pub fn sanitize_display(self, display: f64) -> f64 {
        if !display.is_finite() {
            return display;
        }
        let factor = 10f64.powi(self.display_decimal_places());
        (display * factor).round() / factor
    }

    /// Formats a display value without floating-point noise.
    pub fn format_display(self, display: f64) -> String {
        let value = self.sanitize_display(display);
        if value.abs() >= 1e6 || value.abs() < 1e-3 && value != 0.0 {
            return format!("{:.6e}", value);
        }
        let places = self.display_decimal_places().max(0) as usize;
        trim_trailing_zeros(&format!("{:.*}", places, value))
    }

    /// Converts meters to a canonical value free of unit round-trip artifacts.
    pub fn canonical_meters(self, meters: f64) -> f64 {
        let display = self.sanitize_display(self.from_meters(meters));
        clamp_world_scale(self.to_meters(display))
    }

    /// Minimum allowed display value for this unit.
    pub fn min_display_value(self) -> f64 {
        let precision_min = 10f64.powi(-self.display_decimal_places().max(0));
        let physical_min = self.sanitize_display(MIN_WORLD_SCALE / self.meters_per_unit());
        precision_min.max(physical_min)
    }
}

fn trim_trailing_zeros(formatted: &str) -> String {
    if !formatted.contains('.') {
        return formatted.to_string();
    }
    formatted
        .trim_end_matches('0')
        .trim_end_matches('.')
        .to_string()
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum AppMode {
    Simulation,
    Graph3D,
}

impl Default for AppMode {
    /// Selects simulation mode as the default application startup mode.
    fn default() -> Self {
        AppMode::Simulation
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PanelKind {
    Simulation,
    ObjectInput,
    Settings,
    Graph3D,
}

impl PanelKind {
    /// Returns the UI label used to represent each panel kind.
    pub const fn label(self) -> &'static str {
        match self {
            PanelKind::Simulation => "Simulation",
            PanelKind::ObjectInput => "Object Input",
            PanelKind::Settings => "Settings",
            PanelKind::Graph3D => "3D Graph",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DragOwner {
    None,
    Ui,
    PendingSceneLeft,
    PendingSceneRight,
    PendingSceneMiddle,
    SceneLeft,
    SceneRight,
    SceneMiddle,
}

impl DragOwner {
    /// Promotes a pending scene drag to a concrete owner based on UI capture state.
    pub(crate) fn promote_from_pending(self, ui_blocks_scene: bool) -> Option<Self> {
        Some(match self {
            Self::PendingSceneLeft => {
                if ui_blocks_scene {
                    Self::Ui
                } else {
                    Self::SceneLeft
                }
            }
            Self::PendingSceneRight => {
                if ui_blocks_scene {
                    Self::Ui
                } else {
                    Self::SceneRight
                }
            }
            Self::PendingSceneMiddle => {
                if ui_blocks_scene {
                    Self::Ui
                } else {
                    Self::SceneMiddle
                }
            }
            _ => return None,
        })
    }
}

const PANELS_SIMULATION: &[PanelKind] = &[
    PanelKind::Simulation,
    PanelKind::ObjectInput,
    PanelKind::Settings,
];

const PANELS_GRAPH3D: &[PanelKind] = &[PanelKind::Graph3D, PanelKind::Settings];

#[derive(Clone, Copy, PartialEq, Debug, serde::Serialize, serde::Deserialize)]
pub enum SimulationType {
    Normal,
    SpeedOfLightLimit,
    LorentzTransformation,
}

impl std::fmt::Display for SimulationType {
    /// Formats simulation type for combo-box and labels.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let text = match self {
            SimulationType::Normal => "Normal",
            SimulationType::SpeedOfLightLimit => "Speed of Light Limit",
            SimulationType::LorentzTransformation => "Lorentz Transformation",
        };
        write!(f, "{}", text)
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PendingSnapshotDialog {
    Save,
    Load,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum GraphType {
    SphericalFibonacciLattice,
    RapidityFieldMatrix,
    RapidityFieldBiquaternion,
}

impl std::fmt::Display for GraphType {
    /// Formats graph type names for UI selection controls.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let text = match self {
            GraphType::SphericalFibonacciLattice => "Spherical Fibonacci Lattice",
            GraphType::RapidityFieldMatrix => "Rapidity Field by matrix",
            GraphType::RapidityFieldBiquaternion => "Rapidity Field by biquaternion",
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
    pub add_center: DVec3,
    pub show_add_center_preview: bool,
    pub is_add_particles_requested: bool,
    pub skip: u32,
    pub app_mode: AppMode,
    pub last_app_mode_for_panel_sync: AppMode,
    pub object_input_type: ObjectInputType,
    pub object_input: ObjectInput,
    pub simulation_type: SimulationType,
    pub base_scale: f64,
    pub base_scale_unit: BaseScaleUnit,
    pub random_sphere: RandomSphereParameters,
    pub random_cube: RandomCubeParameters,
    pub two_spheres: TwoSpheresParameters,
    pub spiral_disk: SpiralDiskParameters,
    pub solar_system: SolarSystemParameters,
    pub satellite_orbit: SatelliteOrbitParameters,
    pub elliptical_orbit: EllipticalOrbitParameters,
    pub is_simulation_panel_open: bool,
    pub is_object_input_panel_open: bool,
    pub is_settings_panel_open: bool,
    pub is_graph3d_panel_open: bool,
    pub start_maximized: bool,
    pub link_point_size_to_scale: bool,
    pub lock_camera_up: bool,
    pub mailbox_present_mode: bool,
    pub show_grid: bool,
    pub request_exit: bool,
    pub pending_snapshot_dialog: Option<PendingSnapshotDialog>,
    pub graph_type: GraphType,
    pub graph_sample_count: u32,
    pub graph_radius: f64,
    pub graph_velocity_scale: f64,
}

impl Default for UiState {
    /// Initializes UI state with startup defaults for simulation and panels.
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
            add_center: DVec3::ZERO,
            show_add_center_preview: true,
            is_add_particles_requested: false,
            skip: 0,
            app_mode: AppMode::default(),
            last_app_mode_for_panel_sync: AppMode::default(),
            object_input_type: ObjectInputType::default(),
            object_input: ObjectInput::default(),
            simulation_type: SimulationType::Normal,
            base_scale: ObjectInputType::default().default_base_scale(),
            base_scale_unit: BaseScaleUnit::default(),
            random_sphere: RandomSphereParameters::default(),
            random_cube: RandomCubeParameters::default(),
            two_spheres: TwoSpheresParameters::default(),
            spiral_disk: SpiralDiskParameters::default(),
            solar_system: SolarSystemParameters::default(),
            satellite_orbit: SatelliteOrbitParameters::default(),
            elliptical_orbit: EllipticalOrbitParameters::default(),
            is_simulation_panel_open: true,
            is_object_input_panel_open: false,
            is_settings_panel_open: false,
            is_graph3d_panel_open: false,
            start_maximized: false,
            link_point_size_to_scale: true,
            lock_camera_up: true,
            mailbox_present_mode: false,
            show_grid: true,
            request_exit: false,
            pending_snapshot_dialog: None,
            graph_type: GraphType::SphericalFibonacciLattice,
            graph_sample_count: 1000,
            graph_radius: 1.0,
            graph_velocity_scale: 1.0,
        }
    }
}

impl UiState {
    /// Applies persisted app settings and clamps runtime values to new limits.
    pub fn apply_settings(&mut self, settings: &AppSettings) {
        self.max_particle_count = settings.max_particle_count;
        self.min_window_width = settings.window_min_width;
        self.min_window_height = settings.window_min_height;
        self.start_maximized = settings.start_maximized;
        self.link_point_size_to_scale = settings.link_point_size_to_scale;
        self.lock_camera_up = settings.lock_camera_up;
        self.mailbox_present_mode = settings.mailbox_present_mode;
        if self.particle_count > self.max_particle_count {
            self.particle_count = self.max_particle_count;
        }
    }

    /// Resets all 3D graph parameters back to their default values.
    pub fn reset_graph_params(&mut self) {
        self.graph_type = GraphType::SphericalFibonacciLattice;
        self.graph_sample_count = 1000;
        self.graph_radius = 1.0;
        self.graph_velocity_scale = 1.0;
    }

    /// Applies panel open-state defaults when switching between major app modes.
    pub fn apply_panel_defaults_on_app_mode_change(&mut self, from: AppMode, to: AppMode) {
        if from == to {
            return;
        }
        match to {
            AppMode::Graph3D => {
                self.is_simulation_panel_open = false;
                self.is_object_input_panel_open = false;
                self.is_graph3d_panel_open = true;
                self.reset_graph_params();
            }
            AppMode::Simulation => {
                self.is_graph3d_panel_open = false;
                self.is_simulation_panel_open = true;
            }
        }
    }

    /// Returns the panel set available for the currently selected app mode.
    pub fn get_available_panels(&self) -> &'static [PanelKind] {
        match self.app_mode {
            AppMode::Simulation => PANELS_SIMULATION,
            AppMode::Graph3D => PANELS_GRAPH3D,
        }
    }

    /// Returns the current base scale as a value in the selected display unit.
    pub fn base_scale_display_value(&self) -> f64 {
        self.base_scale_unit
            .sanitize_display(self.base_scale_unit.from_meters(self.base_scale))
    }

    /// Updates stored base scale from UI display input or a unit change.
    pub fn apply_base_scale_edit(&mut self, display: f64, unit_changed: bool) {
        let unit = self.base_scale_unit;
        let display = if unit_changed {
            1.0
        } else {
            unit.sanitize_display(display)
        };
        self.base_scale = unit.canonical_meters(unit.to_meters(display));
    }

    /// Builds an object-input snapshot from the current panel state.
    pub fn build_object_input(&self) -> ObjectInput {
        let scale = self.base_scale;
        match self.object_input_type {
            ObjectInputType::RandomSphere => ObjectInput::RandomSphere {
                scale,
                radius: self.random_sphere.radius,
                mass_range: self.random_sphere.mass_range,
                velocity_std: self.random_sphere.velocity_std,
            },
            ObjectInputType::RandomCube => ObjectInput::RandomCube {
                scale,
                cube_size: self.random_cube.cube_size,
                mass_range: self.random_cube.mass_range,
                velocity_std: self.random_cube.velocity_std,
            },
            ObjectInputType::TwoSpheres => ObjectInput::TwoSpheres {
                scale,
                sphere1_center: self.two_spheres.sphere1_center,
                sphere1_radius: self.two_spheres.sphere1_radius,
                sphere2_center: self.two_spheres.sphere2_center,
                sphere2_radius: self.two_spheres.sphere2_radius,
                mass_fixed: self.two_spheres.mass_fixed,
            },
            ObjectInputType::SpiralDisk => ObjectInput::SpiralDisk {
                scale,
                disk_radius: self.spiral_disk.disk_radius,
                mass_fixed: self.spiral_disk.mass_fixed,
            },
            ObjectInputType::SolarSystem => ObjectInput::SolarSystem {
                scale,
                start_year: self.solar_system.start_year,
                start_month: self.solar_system.start_month,
                start_day: self.solar_system.start_day,
                start_hour: self.solar_system.start_hour,
            },
            ObjectInputType::SatelliteOrbit => ObjectInput::SatelliteOrbit {
                scale,
                orbit_altitude_min: self.satellite_orbit.orbit_altitude_min,
                orbit_altitude_max: self.satellite_orbit.orbit_altitude_max,
                asteroid_mass: self.satellite_orbit.asteroid_mass,
                asteroid_distance: self.satellite_orbit.asteroid_distance,
                asteroid_speed: self.satellite_orbit.asteroid_speed,
            },
            ObjectInputType::EllipticalOrbit => ObjectInput::EllipticalOrbit {
                scale,
                central_mass: self.elliptical_orbit.central_mass,
                planetary_mass: self.elliptical_orbit.planetary_mass,
                planetary_speed: self.elliptical_orbit.planetary_speed,
                planetary_distance: self.elliptical_orbit.planetary_distance,
            },
        }
    }
}

pub struct RandomSphereParameters {
    pub radius: f64,
    pub mass_range: (f64, f64),
    pub velocity_std: f64,
}

impl Default for RandomSphereParameters {
    /// Loads default random-sphere parameter values from object-input presets.
    fn default() -> Self {
        if let ObjectInput::RandomSphere {
            radius,
            mass_range,
            velocity_std,
            ..
        } = ObjectInputType::RandomSphere.to_object_input(1e10)
        {
            Self {
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
    pub cube_size: f64,
    pub mass_range: (f64, f64),
    pub velocity_std: f64,
}

impl Default for RandomCubeParameters {
    /// Loads default random-cube parameter values from object-input presets.
    fn default() -> Self {
        if let ObjectInput::RandomCube {
            cube_size,
            mass_range,
            velocity_std,
            ..
        } = ObjectInputType::RandomCube.to_object_input(1e10)
        {
            Self {
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
    pub sphere1_center: DVec3,
    pub sphere1_radius: f64,
    pub sphere2_center: DVec3,
    pub sphere2_radius: f64,
    pub mass_fixed: f64,
}

impl Default for TwoSpheresParameters {
    /// Loads default two-spheres parameter values from object-input presets.
    fn default() -> Self {
        if let ObjectInput::TwoSpheres {
            sphere1_center,
            sphere1_radius,
            sphere2_center,
            sphere2_radius,
            mass_fixed,
            ..
        } = ObjectInputType::TwoSpheres.to_object_input(1.0)
        {
            Self {
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
    pub disk_radius: f64,
    pub mass_fixed: f64,
}

impl Default for SpiralDiskParameters {
    /// Loads default spiral-disk parameter values from object-input presets.
    fn default() -> Self {
        if let ObjectInput::SpiralDisk {
            disk_radius,
            mass_fixed,
            ..
        } = ObjectInputType::SpiralDisk.to_object_input(1e7)
        {
            Self {
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
    /// Loads default solar-system start-time values from object-input presets.
    fn default() -> Self {
        if let ObjectInput::SolarSystem {
            start_year,
            start_month,
            start_day,
            start_hour,
            ..
        } = ObjectInputType::SolarSystem.to_object_input(SOLAR_SYSTEM_SCALE)
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
    /// Loads default satellite-orbit parameter values from object-input presets.
    fn default() -> Self {
        if let ObjectInput::SatelliteOrbit {
            orbit_altitude_min,
            orbit_altitude_max,
            asteroid_mass,
            asteroid_distance,
            asteroid_speed,
            ..
        } = ObjectInputType::SatelliteOrbit.to_object_input(SATELLITE_ORBIT_SCALE)
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
    pub central_mass: f64,
    pub planetary_mass: f64,
    pub planetary_speed: f64,
    pub planetary_distance: f64,
}

impl Default for EllipticalOrbitParameters {
    /// Loads default elliptical-orbit parameter values from object-input presets.
    fn default() -> Self {
        if let ObjectInput::EllipticalOrbit {
            central_mass,
            planetary_mass,
            planetary_speed,
            planetary_distance,
            ..
        } = ObjectInputType::EllipticalOrbit.to_object_input(1.5e11)
        {
            Self {
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
