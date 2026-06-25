use crate::graph3d::GraphType;
use crate::settings::AppSettings;

pub(crate) fn trim_trailing_zeros(formatted: &str) -> String {
    if !formatted.contains('.') {
        return formatted.to_string();
    }
    formatted
        .trim_end_matches('0')
        .trim_end_matches('.')
        .to_string()
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PanelKind {
    Graph3D,
    Settings,
}

impl PanelKind {
    /// Returns the UI label used to represent each panel kind.
    pub const fn label(self) -> &'static str {
        match self {
            PanelKind::Graph3D => "3D Graph",
            PanelKind::Settings => "Settings",
        }
    }
}

pub const PANELS: &[PanelKind] = &[PanelKind::Graph3D, PanelKind::Settings];

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

    /// Returns the combobox label for this display mode.
    pub const fn combobox_label(self) -> &'static str {
        match self {
            Self::Glow => "Glow",
            Self::Sphere => "Sphere",
        }
    }

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
        f.write_str(self.combobox_label())
    }
}

pub struct UiState {
    pub input_panel_width: f32,
    pub min_window_width: f32,
    pub min_window_height: f32,
    pub is_graph3d_panel_open: bool,
    pub is_settings_panel_open: bool,
    pub start_maximized: bool,
    pub lock_camera_up: bool,
    /// Screen position of the spacecraft steer anchor (⊕ marker), when active.
    pub spacecraft_steer_anchor: Option<[f64; 2]>,
    /// Screen position of the spacecraft yaw steer anchor (⇔ marker), while RMB is held.
    pub spacecraft_yaw_steer_anchor: Option<[f64; 2]>,
    pub mailbox_present_mode: bool,
    pub show_grid: bool,
    pub particle_display_mode: ParticleDisplayMode,
    pub request_exit: bool,
    pub graph_type: GraphType,
    pub graph_sample_count: u32,
    pub graph_radius: f64,
    pub graph_velocity_scale: f64,
}

impl Default for UiState {
    /// Initializes UI state with startup defaults for the graph viewer.
    fn default() -> Self {
        Self {
            input_panel_width: 200.0,
            min_window_width: 400.0,
            min_window_height: 300.0,
            is_graph3d_panel_open: true,
            is_settings_panel_open: false,
            start_maximized: false,
            lock_camera_up: true,
            spacecraft_steer_anchor: None,
            spacecraft_yaw_steer_anchor: None,
            mailbox_present_mode: false,
            show_grid: true,
            particle_display_mode: ParticleDisplayMode::default(),
            request_exit: false,
            graph_type: GraphType::SphericalFibonacciLattice,
            graph_sample_count: 1000,
            graph_radius: 1.0,
            graph_velocity_scale: 1.0,
        }
    }
}

impl UiState {
    /// Returns the open-state flag for the given panel kind.
    pub fn panel_open_mut(&mut self, panel: PanelKind) -> &mut bool {
        match panel {
            PanelKind::Graph3D => &mut self.is_graph3d_panel_open,
            PanelKind::Settings => &mut self.is_settings_panel_open,
        }
    }

    /// Applies persisted app settings to runtime UI state.
    pub fn apply_settings(&mut self, settings: &AppSettings) {
        self.min_window_width = settings.window_min_width;
        self.min_window_height = settings.window_min_height;
        self.start_maximized = settings.start_maximized;
        self.lock_camera_up = settings.lock_camera_up;
        self.mailbox_present_mode = settings.mailbox_present_mode;
        self.particle_display_mode = settings.particle_display_mode;
    }
}
