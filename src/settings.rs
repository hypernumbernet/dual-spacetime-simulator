use serde::{Deserialize, Serialize};
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct AppSettings {
    pub max_particle_count: u32,
    pub window_min_width: f32,
    pub window_min_height: f32,
}

impl Default for AppSettings {
    fn default() -> Self {
        Self {
            max_particle_count: 20_000,
            window_min_width: 400.0,
            window_min_height: 300.0,
        }
    }
}

impl AppSettings {
    fn config_path() -> io::Result<PathBuf> {
        let exe_path = std::env::current_exe()?;
        let dir = exe_path.parent().unwrap_or_else(|| Path::new("."));
        Ok(dir.join("setting.config"))
    }

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

