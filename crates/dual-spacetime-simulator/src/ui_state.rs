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

#[derive(Clone, Copy, PartialEq, Debug, Default, serde::Serialize, serde::Deserialize)]
pub enum ComputingUnit {
    #[default]
    Cpu,
    Gpu,
}

impl std::fmt::Display for ComputingUnit {
    /// Formats computing unit for combo-box and labels.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let text = match self {
            ComputingUnit::Cpu => "CPU",
            ComputingUnit::Gpu => "GPU",
        };
        write!(f, "{}", text)
    }
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

#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub enum PlacementMode {
    #[default]
    Manual,
    SolarSystem,
    SatelliteOrbit,
}

impl std::fmt::Display for PlacementMode {
    /// Formats placement mode for combo-box and labels.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let text = match self {
            PlacementMode::Manual => "Manual",
            PlacementMode::SolarSystem => "Solar System",
            PlacementMode::SatelliteOrbit => "Satellite Orbit",
        };
        write!(f, "{}", text)
    }
}

impl PlacementMode {
    /// All placement modes in UI display order.
    pub const ALL: [Self; 3] = [Self::Manual, Self::SolarSystem, Self::SatelliteOrbit];

    /// Returns the recommended base scale for preset placement modes.
    pub fn default_base_scale(self) -> Option<f64> {
        match self {
            PlacementMode::Manual => None,
            PlacementMode::SolarSystem => Some(SOLAR_SYSTEM_SCALE),
            PlacementMode::SatelliteOrbit => Some(SATELLITE_ORBIT_SCALE),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PendingSnapshotDialog {
    Save,
    Load,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default, serde::Serialize, serde::Deserialize)]
pub enum ParticleDisplayMode {
    #[default]
    Glow = 0,
    Sphere = 1,
}

impl ParticleDisplayMode {
    pub const ALL: [Self; 2] = [Self::Glow, Self::Sphere];
    const SPHERE_SIZE_SCALE: f32 = 0.7;

    /// Returns the particle pipeline slot for this display mode.
    pub const fn pipeline_index(self) -> usize {
        self as usize
    }

    /// Returns the multiplier applied to point sprite size for this mode.
    pub const fn size_scale_factor(self) -> f32 {
        match self {
            Self::Glow => 1.0,
            Self::Sphere => Self::SPHERE_SIZE_SCALE,
        }
    }
}

impl std::fmt::Display for ParticleDisplayMode {
    /// Formats particle display mode names for UI selection controls.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let text = match self {
            ParticleDisplayMode::Glow => "Glow",
            ParticleDisplayMode::Sphere => "Sphere",
        };
        write!(f, "{}", text)
    }
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
    pub add_particle_count: u32,
    pub max_particle_count: u32,
    pub fps: i64,
    pub frame: i64,
    pub simulation_time: f64,
    pub time_per_frame: f64,
    pub scale: f64,
    pub scale_gauge: f64,
    pub is_running: bool,
    pub max_fps: u32,
    pub max_fps_unlimited: bool,
    pub is_reset_requested: bool,
    pub is_resetting: bool,
    pub add_center: DVec3,
    pub show_add_center_preview: bool,
    pub is_add_particles_requested: bool,
    pub is_add_particles_enabled: bool,
    pub skip: u32,
    pub app_mode: AppMode,
    pub last_app_mode_for_panel_sync: AppMode,
    pub object_input_type: ObjectInputType,
    pub object_input: ObjectInput,
    pub placement_mode: PlacementMode,
    pub simulation_type: SimulationType,
    pub computing_unit: ComputingUnit,
    pub active_computing_unit: ComputingUnit,
    pub base_scale: f64,
    pub base_scale_unit: BaseScaleUnit,
    pub random_sphere: RandomSphereParameters,
    pub random_cube: RandomCubeParameters,
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
    pub particle_display_mode: ParticleDisplayMode,
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
            add_particle_count: 1000,
            max_particle_count: 20000,
            fps: 0,
            frame: 1,
            simulation_time: 0.0,
            time_per_frame: 10.0,
            scale: 1e10,
            scale_gauge: DEFAULT_SCALE_UI,
            is_running: false,
            max_fps: 60,
            max_fps_unlimited: false,
            is_reset_requested: false,
            is_resetting: false,
            add_center: DVec3::ZERO,
            show_add_center_preview: true,
            is_add_particles_requested: false,
            is_add_particles_enabled: true,
            skip: 0,
            app_mode: AppMode::default(),
            last_app_mode_for_panel_sync: AppMode::default(),
            object_input_type: ObjectInputType::default(),
            object_input: ObjectInput::default(),
            placement_mode: PlacementMode::default(),
            simulation_type: SimulationType::Normal,
            computing_unit: ComputingUnit::default(),
            active_computing_unit: ComputingUnit::default(),
            base_scale: ObjectInputType::default().default_base_scale(),
            base_scale_unit: BaseScaleUnit::default(),
            random_sphere: RandomSphereParameters::default(),
            random_cube: RandomCubeParameters::default(),
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
            particle_display_mode: ParticleDisplayMode::default(),
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
    /// Returns how many more particles can be added before hitting the configured maximum.
    pub fn remaining_particle_capacity(&self, current_count: u32) -> u32 {
        self.max_particle_count.saturating_sub(current_count)
    }

    /// Returns the valid add-count slider range for the given remaining capacity.
    pub fn add_particle_count_range(remaining: u32) -> Option<std::ops::RangeInclusive<u32>> {
        match remaining {
            0 => None,
            1 => Some(1..=1),
            n => Some(2..=n),
        }
    }

    /// Clamps the add batch size to the current remaining particle capacity.
    pub fn clamp_add_particle_count_to_capacity(&mut self, current_count: u32) {
        let remaining = self.remaining_particle_capacity(current_count);
        if let Some(range) = Self::add_particle_count_range(remaining) {
            self.add_particle_count = self.add_particle_count.clamp(*range.start(), *range.end());
        }
    }

    /// Applies persisted app settings and clamps runtime values to new limits.
    pub fn apply_settings(&mut self, settings: &AppSettings) {
        self.max_particle_count = settings.max_particle_count;
        self.min_window_width = settings.window_min_width;
        self.min_window_height = settings.window_min_height;
        self.start_maximized = settings.start_maximized;
        self.link_point_size_to_scale = settings.link_point_size_to_scale;
        self.lock_camera_up = settings.lock_camera_up;
        self.mailbox_present_mode = settings.mailbox_present_mode;
        self.particle_display_mode = settings.particle_display_mode;
        if self.add_particle_count > self.max_particle_count {
            self.add_particle_count = self.max_particle_count;
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
        self.set_base_scale(unit.canonical_meters(unit.to_meters(display)));
    }

    /// Returns whether GPU particle simulation is available for the current settings.
    pub fn gpu_computing_available(&self) -> bool {
        self.simulation_type == SimulationType::Normal
    }

    /// Returns whether GPU compute should drive the active Normal simulation.
    pub fn uses_gpu_simulation(&self) -> bool {
        matches!(
            (self.simulation_type, self.active_computing_unit),
            (SimulationType::Normal, ComputingUnit::Gpu)
        )
    }

    /// Disables particle append when simulation type changes until the next reset.
    pub fn apply_simulation_type_change(&mut self, previous_type: SimulationType) {
        if self.simulation_type != previous_type {
            if !self.gpu_computing_available() {
                self.force_cpu_computing_units();
            }
            self.disable_add_until_reset();
        }
    }

    /// Disables particle append when computing unit changes until the next reset.
    pub fn apply_computing_unit_change(&mut self, previous_unit: ComputingUnit) {
        if self.computing_unit != previous_unit {
            self.disable_add_until_reset();
        }
    }

    /// Disables particle append when placement mode changes until the next reset.
    pub fn apply_placement_mode_change(&mut self, previous_mode: PlacementMode) {
        if self.placement_mode == previous_mode {
            return;
        }
        self.disable_add_until_reset();
        if let Some(scale) = self.placement_mode.default_base_scale() {
            self.set_base_scale(scale);
        }
    }

    /// Syncs scaled object-input parameters when the add type changes.
    pub fn apply_object_input_type_change(&mut self, previous_type: ObjectInputType) {
        if self.object_input_type == previous_type {
            return;
        }
        self.sync_scaled_object_input_parameters();
    }

    /// Updates base scale from an external source such as snapshot load.
    pub fn apply_external_base_scale(&mut self, scale: f64) {
        self.set_base_scale(scale);
    }

    /// Flags a simulation reset and re-enables particle append.
    pub fn request_reset(&mut self) {
        self.commit_active_computing_unit();
        self.is_reset_requested = true;
        self.is_resetting = true;
        self.is_add_particles_enabled = true;
    }

    fn commit_active_computing_unit(&mut self) {
        if !self.gpu_computing_available() {
            self.force_cpu_computing_units();
            return;
        }
        self.active_computing_unit = self.computing_unit;
    }

    fn force_cpu_computing_units(&mut self) {
        self.computing_unit = ComputingUnit::Cpu;
        self.active_computing_unit = ComputingUnit::Cpu;
    }

    fn disable_add_until_reset(&mut self) {
        self.is_add_particles_enabled = false;
    }

    fn set_base_scale(&mut self, scale: f64) {
        let previous_scale = self.base_scale;
        self.base_scale = clamp_world_scale(scale);
        if !Self::base_scale_value_changed(previous_scale, self.base_scale) {
            return;
        }
        self.disable_add_until_reset();
        self.sync_scaled_object_input_parameters();
    }

    fn base_scale_value_changed(previous: f64, next: f64) -> bool {
        clamp_world_scale(previous).to_bits() != clamp_world_scale(next).to_bits()
    }

    /// Overwrites scaled object-input panel parameters from the current base scale.
    pub fn sync_scaled_object_input_parameters(&mut self) {
        match self.object_input_type.to_object_input(self.base_scale) {
            ObjectInput::RandomSphere {
                radius,
                mass_range,
                velocity_std,
                ..
            } => {
                self.random_sphere = RandomSphereParameters {
                    radius,
                    mass_range,
                    velocity_std,
                };
            }
            ObjectInput::RandomCube {
                cube_size,
                mass_range,
                velocity_std,
                ..
            } => {
                self.random_cube = RandomCubeParameters {
                    cube_size,
                    mass_range,
                    velocity_std,
                };
            }
            ObjectInput::SpiralDisk {
                disk_radius,
                mass_fixed,
                ..
            } => {
                self.spiral_disk = SpiralDiskParameters {
                    disk_radius,
                    mass_fixed,
                };
            }
            ObjectInput::EllipticalOrbit {
                central_mass,
                planetary_mass,
                planetary_speed,
                planetary_distance,
                ..
            } => {
                self.elliptical_orbit = EllipticalOrbitParameters {
                    central_mass,
                    planetary_mass,
                    planetary_speed,
                    planetary_distance,
                };
            }
            ObjectInput::SolarSystem { .. } | ObjectInput::SatelliteOrbit { .. } => unreachable!(),
        }
    }

    /// Builds object input for particle append from the current add-type panel state.
    pub fn build_object_input(&self) -> ObjectInput {
        let scale = self.base_scale;
        match self.object_input_type {
            ObjectInputType::RandomSphere => self.random_sphere.to_object_input(scale),
            ObjectInputType::RandomCube => self.random_cube.to_object_input(scale),
            ObjectInputType::SpiralDisk => self.spiral_disk.to_object_input(scale),
            ObjectInputType::EllipticalOrbit => self.elliptical_orbit.to_object_input(scale),
        }
    }

    /// Builds object input for simulation reset from the current placement mode.
    pub fn build_reset_object_input(&self) -> ObjectInput {
        let scale = self.base_scale;
        match self.placement_mode {
            PlacementMode::Manual => self.build_object_input(),
            PlacementMode::SolarSystem => self.solar_system.to_object_input(scale),
            PlacementMode::SatelliteOrbit => self.satellite_orbit.to_object_input(scale),
        }
    }

    /// Applies time-step defaults after a simulation reset completes.
    pub fn apply_reset_timing_defaults(&mut self) {
        if self.placement_mode == PlacementMode::SolarSystem {
            self.time_per_frame = 10_000.0;
            self.max_fps = 1000;
            self.skip = 10;
        } else if self.placement_mode == PlacementMode::Manual
            && self.object_input_type == ObjectInputType::EllipticalOrbit
        {
            self.time_per_frame = 100_000.0;
            self.max_fps = 1000;
            self.skip = 0;
        } else {
            self.time_per_frame = 10.0;
            self.max_fps = 60;
            self.skip = 0;
        }
    }
}

pub struct RandomSphereParameters {
    pub radius: f64,
    pub mass_range: (f64, f64),
    pub velocity_std: f64,
}

impl RandomSphereParameters {
    /// Builds a random-sphere object input from panel parameters.
    pub fn to_object_input(&self, scale: f64) -> ObjectInput {
        ObjectInput::RandomSphere {
            scale,
            radius: self.radius,
            mass_range: self.mass_range,
            velocity_std: self.velocity_std,
        }
    }
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

impl RandomCubeParameters {
    /// Builds a random-cube object input from panel parameters.
    pub fn to_object_input(&self, scale: f64) -> ObjectInput {
        ObjectInput::RandomCube {
            scale,
            cube_size: self.cube_size,
            mass_range: self.mass_range,
            velocity_std: self.velocity_std,
        }
    }
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

pub struct SpiralDiskParameters {
    pub disk_radius: f64,
    pub mass_fixed: f64,
}

impl SpiralDiskParameters {
    /// Builds a spiral-disk object input from panel parameters.
    pub fn to_object_input(&self, scale: f64) -> ObjectInput {
        ObjectInput::SpiralDisk {
            scale,
            disk_radius: self.disk_radius,
            mass_fixed: self.mass_fixed,
        }
    }
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

impl SolarSystemParameters {
    /// Builds a solar-system object input from panel parameters.
    pub fn to_object_input(&self, scale: f64) -> ObjectInput {
        ObjectInput::SolarSystem {
            scale,
            start_year: self.start_year,
            start_month: self.start_month,
            start_day: self.start_day,
            start_hour: self.start_hour,
        }
    }
}

impl Default for SolarSystemParameters {
    /// Loads default solar-system start-time values.
    fn default() -> Self {
        Self {
            start_year: 2000,
            start_month: 1,
            start_day: 1,
            start_hour: 12,
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

impl SatelliteOrbitParameters {
    /// Builds a satellite-orbit object input from panel parameters.
    pub fn to_object_input(&self, scale: f64) -> ObjectInput {
        ObjectInput::SatelliteOrbit {
            scale,
            orbit_altitude_min: self.orbit_altitude_min,
            orbit_altitude_max: self.orbit_altitude_max,
            asteroid_mass: self.asteroid_mass,
            asteroid_distance: self.asteroid_distance,
            asteroid_speed: self.asteroid_speed,
        }
    }
}

impl Default for SatelliteOrbitParameters {
    /// Loads default satellite-orbit parameter values.
    fn default() -> Self {
        Self {
            orbit_altitude_min: 300e3,
            orbit_altitude_max: 800e3,
            asteroid_mass: 1e24,
            asteroid_distance: 2e7,
            asteroid_speed: 3e3,
        }
    }
}

pub struct EllipticalOrbitParameters {
    pub central_mass: f64,
    pub planetary_mass: f64,
    pub planetary_speed: f64,
    pub planetary_distance: f64,
}

impl EllipticalOrbitParameters {
    /// Builds an elliptical-orbit object input from panel parameters.
    pub fn to_object_input(&self, scale: f64) -> ObjectInput {
        ObjectInput::EllipticalOrbit {
            scale,
            central_mass: self.central_mass,
            planetary_mass: self.planetary_mass,
            planetary_speed: self.planetary_speed,
            planetary_distance: self.planetary_distance,
        }
    }
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
