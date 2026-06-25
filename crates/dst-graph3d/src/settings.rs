use crate::ui_state::ParticleDisplayMode;
use serde::{Deserialize, Serialize};
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(default)]
pub struct AppSettings {
    pub window_min_width: f32,
    pub window_min_height: f32,
    pub start_maximized: bool,
    pub mailbox_present_mode: bool,
    #[serde(default)]
    pub particle_display_mode: ParticleDisplayMode,
}

impl Default for AppSettings {
    /// Returns default application settings for first launch and fallback loading.
    fn default() -> Self {
        Self {
            window_min_width: 400.0,
            window_min_height: 300.0,
            start_maximized: false,
            mailbox_present_mode: false,
            particle_display_mode: ParticleDisplayMode::default(),
        }
    }
}

impl AppSettings {
    /// Resolves the filesystem path used to load and save persisted settings.
    fn config_path() -> io::Result<PathBuf> {
        let exe_path = std::env::current_exe()?;
        let dir = exe_path.parent().unwrap_or_else(|| Path::new("."));
        Ok(dir.join("dst_graph3d_settings.json"))
    }

    /// Loads settings from disk and falls back to defaults on any read or parse failure.
    pub fn load() -> Self {
        if let Ok(path) = Self::config_path() {
            if let Ok(text) = fs::read_to_string(&path) {
                if let Ok(settings) = serde_json::from_str::<AppSettings>(&text) {
                    return settings;
                }
            }
        }
        Self::default()
    }

    /// Persists current settings to disk as pretty-printed JSON.
    pub fn save(&self) -> io::Result<()> {
        let path = Self::config_path()?;
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }
        let text = serde_json::to_string_pretty(self)
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e.to_string()))?;
        fs::write(path, text)
    }
}
