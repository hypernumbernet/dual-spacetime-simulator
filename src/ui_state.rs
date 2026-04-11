use crate::initial_condition::{InitialCondition, InitialConditionType};
use crate::settings::AppSettings;
use crate::tree::TreeParams;
use glam::DVec3;

pub const DEFAULT_SCALE_UI: f64 = 5000.0;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum AppMode {
    Simulation,
    Graph3D,
    /// Real-Time GPU Tree Generation (HPG 2025) 向け。GPU 上で木を生成・描画するモード（実装は段階的に追加）。
    GpuTree,
}

impl Default for AppMode {
    fn default() -> Self {
        AppMode::Simulation
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PanelKind {
    Simulation,
    InitialCondition,
    Settings,
    Graph3D,
    GpuTree,
}

impl PanelKind {
    pub const fn label(self) -> &'static str {
        match self {
            PanelKind::Simulation => "Simulation",
            PanelKind::InitialCondition => "Initial Condition",
            PanelKind::Settings => "Settings",
            PanelKind::Graph3D => "3D Graph",
            PanelKind::GpuTree => "GPU Tree",
        }
    }
}

const PANELS_SIMULATION: &[PanelKind] = &[
    PanelKind::Simulation,
    PanelKind::InitialCondition,
    PanelKind::Settings,
];

const PANELS_GRAPH3D: &[PanelKind] = &[PanelKind::Graph3D, PanelKind::Settings];

const PANELS_GPU_TREE: &[PanelKind] = &[PanelKind::GpuTree, PanelKind::Settings];

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

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum GraphType {
    LightCone,
    RapidityField,
    BoostExponent,
    BivectorVisualization,
    QuaternionProjection,
}

/// GpuTree 表示レイアウト: シングル木 または xzグリッド交点すべてに木を生やす
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GpuTreeLayout {
    Single,
    ForestOnGrid,
}

impl std::fmt::Display for GpuTreeLayout {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let text = match self {
            GpuTreeLayout::Single => "Single Tree",
            GpuTreeLayout::ForestOnGrid => "Forest on XZ Grid",
        };
        write!(f, "{}", text)
    }
}

/// GpuTree の描画モード: 線分 (LineList) または ポリゴンチューブ (TriangleList with normals/lighting)
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GpuTreeRenderMode {
    Lines,
    Polygons,
}

impl std::fmt::Display for GpuTreeRenderMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let text = match self {
            GpuTreeRenderMode::Lines => "Lines",
            GpuTreeRenderMode::Polygons => "Polygons",
        };
        write!(f, "{}", text)
    }
}

/// GpuTree の計算バックエンド: CPU (既存Tree::generate) または GPU (Computeシェーダ)
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GpuTreeComputeMode {
    CPU,
    GPU,
}

impl std::fmt::Display for GpuTreeComputeMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let text = match self {
            GpuTreeComputeMode::CPU => "CPU",
            GpuTreeComputeMode::GPU => "GPU (Compute)",
        };
        write!(f, "{}", text)
    }
}

impl std::fmt::Display for GraphType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let text = match self {
            GraphType::LightCone => "Light Cone Slice",
            GraphType::RapidityField => "Rapidity Field",
            GraphType::BoostExponent => "Boost Exponent",
            GraphType::BivectorVisualization => "Bivector Visualization",
            GraphType::QuaternionProjection => "Quaternion Projection",
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
    pub app_mode: AppMode,
    pub last_app_mode_for_panel_sync: AppMode,
    pub initial_condition_type: InitialConditionType,
    pub initial_condition: InitialCondition,
    pub simulation_type: SimulationType,
    pub random_sphere: RandomSphereParameters,
    pub random_cube: RandomCubeParameters,
    pub two_spheres: TwoSpheresParameters,
    pub spiral_disk: SpiralDiskParameters,
    pub solar_system: SolarSystemParameters,
    pub satellite_orbit: SatelliteOrbitParameters,
    pub elliptical_orbit: EllipticalOrbitParameters,
    pub is_simulation_panel_open: bool,
    pub is_initial_condition_panel_open: bool,
    pub is_settings_panel_open: bool,
    pub is_graph3d_panel_open: bool,
    pub is_gpu_tree_panel_open: bool,
    pub start_maximized: bool,
    pub link_point_size_to_scale: bool,
    pub lock_camera_up: bool,
    pub show_grid: bool,
    pub request_exit: bool,
    pub graph_type: GraphType,
    pub graph_sample_count: u32,
    pub graph_t_slice: f64,
    pub graph_velocity_scale: f64,
    pub graph_phi: f64,
    pub gpu_tree_layout: GpuTreeLayout,
    pub gpu_tree_render_mode: GpuTreeRenderMode,
    pub gpu_tree_compute_mode: GpuTreeComputeMode,
    pub gpu_tree_params: TreeParams,
    pub last_gpu_tree_fingerprint: u64,
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
            app_mode: AppMode::default(),
            last_app_mode_for_panel_sync: AppMode::default(),
            initial_condition_type: InitialConditionType::default(),
            initial_condition: InitialCondition::default(),
            simulation_type: SimulationType::Normal,
            random_sphere: RandomSphereParameters::default(),
            random_cube: RandomCubeParameters::default(),
            two_spheres: TwoSpheresParameters::default(),
            spiral_disk: SpiralDiskParameters::default(),
            solar_system: SolarSystemParameters::default(),
            satellite_orbit: SatelliteOrbitParameters::default(),
            elliptical_orbit: EllipticalOrbitParameters::default(),
            is_simulation_panel_open: true,
            is_initial_condition_panel_open: false,
            is_settings_panel_open: false,
            is_graph3d_panel_open: false,
            is_gpu_tree_panel_open: false,
            start_maximized: false,
            link_point_size_to_scale: true,
            lock_camera_up: true,
            show_grid: true,
            request_exit: false,
            graph_type: GraphType::LightCone,
            graph_sample_count: 1000,
            graph_t_slice: 0.0,
            graph_velocity_scale: 1.0,
            graph_phi: 1.0,
            gpu_tree_layout: GpuTreeLayout::Single,
            gpu_tree_render_mode: GpuTreeRenderMode::Polygons,
            gpu_tree_compute_mode: GpuTreeComputeMode::CPU,
            gpu_tree_params: TreeParams::default(),
            last_gpu_tree_fingerprint: 0,
        }
    }
}

impl UiState {
    pub fn apply_settings(&mut self, settings: &AppSettings) {
        self.max_particle_count = settings.max_particle_count;
        self.min_window_width = settings.window_min_width;
        self.min_window_height = settings.window_min_height;
        self.start_maximized = settings.start_maximized;
        self.link_point_size_to_scale = settings.link_point_size_to_scale;
        self.lock_camera_up = settings.lock_camera_up;
        if self.particle_count > self.max_particle_count {
            self.particle_count = self.max_particle_count;
        }
    }

    pub fn reset_graph_params(&mut self) {
        self.graph_type = GraphType::LightCone;
        self.graph_sample_count = 1000;
        self.graph_t_slice = 0.0;
        self.graph_velocity_scale = 1.0;
        self.graph_phi = 1.0;
    }

    pub fn reset_gpu_tree_params(&mut self) {
        self.gpu_tree_layout = GpuTreeLayout::Single;
        self.gpu_tree_render_mode = GpuTreeRenderMode::Polygons;
        self.gpu_tree_compute_mode = GpuTreeComputeMode::CPU;
        self.gpu_tree_params = TreeParams::default();
        self.last_gpu_tree_fingerprint = 0;
    }

    /// GpuTree のパラメータ変化を検知するための簡易 fingerprint (hash-like)
    pub fn gpu_tree_fingerprint(&self) -> u64 {
        let mut hash = 0u64;
        hash = hash.wrapping_add(self.gpu_tree_layout as u64);
        hash = hash.wrapping_add((self.gpu_tree_render_mode as u64) * 17);
        hash = hash.wrapping_add((self.gpu_tree_compute_mode as u64) * 31); // compute modeも検知
        // TreeParams の主要フィールドを簡易ハッシュ化
        hash = hash
            .wrapping_mul(31)
            .wrapping_add(self.gpu_tree_params.seed as u64);
        hash = hash
            .wrapping_mul(31)
            .wrapping_add((self.gpu_tree_params.trunk_height * 100.0) as u64);
        hash = hash
            .wrapping_mul(31)
            .wrapping_add((self.gpu_tree_params.trunk_radius_base * 1000.0) as u64);
        hash = hash
            .wrapping_mul(31)
            .wrapping_add(self.gpu_tree_params.max_depth as u64);
        hash = hash
            .wrapping_mul(31)
            .wrapping_add(self.gpu_tree_params.branch_factor as u64);
        hash = hash
            .wrapping_mul(31)
            .wrapping_add((self.gpu_tree_params.branch_angle * 100.0) as u64);
        hash = hash
            .wrapping_mul(31)
            .wrapping_add((self.gpu_tree_params.tropism * 100.0) as u64);
        hash
    }

    pub fn apply_panel_defaults_on_app_mode_change(&mut self, from: AppMode, to: AppMode) {
        if from == to {
            return;
        }
        match to {
            AppMode::Graph3D => {
                self.is_simulation_panel_open = false;
                self.is_initial_condition_panel_open = false;
                self.is_gpu_tree_panel_open = false;
                self.is_graph3d_panel_open = true;
                self.reset_graph_params();
            }
            AppMode::GpuTree => {
                self.is_simulation_panel_open = false;
                self.is_initial_condition_panel_open = false;
                self.is_graph3d_panel_open = false;
                self.is_gpu_tree_panel_open = true;
                self.reset_gpu_tree_params();
            }
            AppMode::Simulation => {
                self.is_graph3d_panel_open = false;
                self.is_gpu_tree_panel_open = false;
                self.is_simulation_panel_open = true;
            }
        }
    }

    pub fn get_available_panels(&self) -> &'static [PanelKind] {
        match self.app_mode {
            AppMode::Simulation => PANELS_SIMULATION,
            AppMode::Graph3D => PANELS_GRAPH3D,
            AppMode::GpuTree => PANELS_GPU_TREE,
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
