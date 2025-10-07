#[derive(Clone)]
pub struct UiState {
    pub input_panel_width: f32,
    pub min_window_width: f32,
    pub min_window_height: f32,
    pub particle_count: u32,
    pub max_particle_count: u32,
    pub gravity: f32,
    pub fps: u64,
    pub frame: u64,
    pub simulation_time: f64,
    pub time_per_frame: f64,
    pub is_running: bool,
    pub max_fps: u32,
    pub unlimited_fps: bool,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            input_panel_width: 200.0,
            min_window_width: 400.0,
            min_window_height: 300.0,
            particle_count: 1000,
            max_particle_count: 20000,
            gravity: 9.81,
            fps: 0,
            frame: 1,
            simulation_time: 0.0,
            time_per_frame: 1.0,
            is_running: false,
            max_fps: 60,
            unlimited_fps: false,
        }
    }
}
