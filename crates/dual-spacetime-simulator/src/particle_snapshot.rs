use serde::{Deserialize, Serialize};
use std::fs::{self, File};
use std::io::{self, BufReader, Read, Seek, SeekFrom};
use std::path::Path;
use zip::write::SimpleFileOptions;
use zip::{ZipArchive, ZipWriter};

use crate::simulation::Particle;
use crate::ui_state::SimulationType;

pub const SNAPSHOT_VERSION: u32 = 1;
pub const SNAPSHOT_FILTER_NAME: &str = "Particle Snapshot";
pub const SNAPSHOT_FILTER_EXT: &str = "zip";
pub const SNAPSHOT_ENTRY_NAME: &str = "particles.json";

const ZIP_MAGIC: [u8; 2] = [b'P', b'K'];

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ParticleSnapshot {
    pub version: u32,
    pub simulation_type: SimulationType,
    pub scale: f64,
    pub particles: Vec<Particle>,
}

impl ParticleSnapshot {
    /// Builds a snapshot from current simulation metadata and particle data.
    pub fn new(simulation_type: SimulationType, scale: f64, particles: Vec<Particle>) -> Self {
        Self {
            version: SNAPSHOT_VERSION,
            simulation_type,
            scale,
            particles,
        }
    }

    /// Loads a particle snapshot from a zip archive (or legacy plain JSON file).
    pub fn load(path: &Path) -> io::Result<Self> {
        let file = File::open(path)?;
        let mut reader = BufReader::new(file);
        let mut magic = [0u8; 2];
        reader.read_exact(&mut magic)?;
        if magic == ZIP_MAGIC {
            reader.seek(SeekFrom::Start(0))?;
            Self::load_from_zip_reader(reader)
        } else {
            let mut bytes = magic.to_vec();
            reader.read_to_end(&mut bytes)?;
            let text = String::from_utf8(bytes)
                .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e.to_string()))?;
            Self::from_json_str(&text)
        }
    }

    /// Persists this snapshot as a deflate-compressed zip archive.
    pub fn save(&self, path: &Path) -> io::Result<()> {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }
        let file = File::create(path)?;
        let mut zip = ZipWriter::new(file);
        let options =
            SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);
        zip.start_file(SNAPSHOT_ENTRY_NAME, options)?;
        serde_json::to_writer(&mut zip, self)
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e.to_string()))?;
        zip.finish()?;
        Ok(())
    }

    fn from_json_str(text: &str) -> io::Result<Self> {
        let snapshot = serde_json::from_str::<Self>(text)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e.to_string()))?;
        Self::validate_version(snapshot)
    }

    fn from_json_reader<R: Read>(reader: R) -> io::Result<Self> {
        let snapshot = serde_json::from_reader(reader)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e.to_string()))?;
        Self::validate_version(snapshot)
    }

    fn validate_version(snapshot: Self) -> io::Result<Self> {
        if snapshot.version != SNAPSHOT_VERSION {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "Unsupported snapshot version: {} (expected {})",
                    snapshot.version, SNAPSHOT_VERSION
                ),
            ));
        }
        Ok(snapshot)
    }

    fn load_from_zip_reader<R: Read + Seek>(reader: R) -> io::Result<Self> {
        let mut archive =
            ZipArchive::new(reader).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
        let entry = archive.by_name(SNAPSHOT_ENTRY_NAME).map_err(|e| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Missing entry '{}': {}", SNAPSHOT_ENTRY_NAME, e),
            )
        })?;
        Self::from_json_reader(entry)
    }
}
