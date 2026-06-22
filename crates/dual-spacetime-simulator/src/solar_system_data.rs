use satkit::utils::datadir;
use serde_json::Value;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

const BASE_URL: &str = "https://storage.googleapis.com/astrokit-astro-data";
const FILES_REFRESH_URL: &str =
    "https://storage.googleapis.com/astrokit-astro-data/files_refresh.json";
const HTTP_POLL_INTERVAL: Duration = Duration::from_millis(200);
const COPY_BUFFER_SIZE: usize = 8192;
const DATA_REFRESH_INTERVAL: Duration = Duration::from_secs(30 * 24 * 60 * 60);
const LAST_REFRESH_FILENAME: &str = ".last_data_refresh";
const EOP_FILENAME: &str = "EOP-All.csv";
const SW_FILENAME: &str = "SW-All.csv";
const SECONDS_PER_DAY: u64 = 24 * 60 * 60;

#[derive(Debug, PartialEq, Eq)]
pub enum UpdateDataError {
    Aborted,
    Other(String),
}

impl std::fmt::Display for UpdateDataError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Aborted => write!(f, "aborted"),
            Self::Other(message) => write!(f, "{message}"),
        }
    }
}

fn is_jplephem_filename(name: &str) -> bool {
    if !(name.starts_with("linux_p") || name.starts_with("lnxp")) {
        return false;
    }
    let Some(ext) = name.rsplit('.').next() else {
        return false;
    };
    ext.len() == 3 && ext.starts_with('4') && ext.chars().all(|c| c.is_ascii_digit())
}

fn has_jplephem_data(downloaddir: &Path) -> bool {
    if let Ok(filename) = std::env::var("SATKIT_JPLEPHEM_FILE") {
        let path = PathBuf::from(&filename);
        let resolved = if path.is_absolute() || filename.contains(std::path::MAIN_SEPARATOR) {
            path
        } else {
            downloaddir.join(filename)
        };
        return resolved.is_file();
    }

    let Ok(entries) = std::fs::read_dir(downloaddir) else {
        return false;
    };
    entries.flatten().any(|entry| {
        entry.path().is_file() && entry.file_name().to_str().is_some_and(is_jplephem_filename)
    })
}

fn has_refresh_data(downloaddir: &Path) -> bool {
    downloaddir.join(EOP_FILENAME).is_file() && downloaddir.join(SW_FILENAME).is_file()
}

fn is_refresh_due(downloaddir: &Path) -> bool {
    read_last_refresh(downloaddir)
        .and_then(|last_refresh| last_refresh.elapsed().ok())
        .is_some_and(|elapsed| elapsed >= DATA_REFRESH_INTERVAL)
}

fn last_refresh_path(downloaddir: &Path) -> PathBuf {
    downloaddir.join(LAST_REFRESH_FILENAME)
}

fn read_last_refresh(downloaddir: &Path) -> Option<SystemTime> {
    let content = std::fs::read_to_string(last_refresh_path(downloaddir)).ok()?;
    let seconds = content.trim().parse::<u64>().ok()?;
    Some(UNIX_EPOCH + Duration::from_secs(seconds))
}

fn write_last_refresh(downloaddir: &Path) -> Result<(), UpdateDataError> {
    let seconds = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|err| UpdateDataError::Other(err.to_string()))?
        .as_secs();
    std::fs::write(last_refresh_path(downloaddir), seconds.to_string())
        .map_err(|err| UpdateDataError::Other(err.to_string()))
}

fn should_skip_refresh(downloaddir: &Path) -> bool {
    if !has_jplephem_data(downloaddir) {
        return false;
    }
    match read_last_refresh(downloaddir) {
        Some(last_refresh) => last_refresh
            .elapsed()
            .is_ok_and(|elapsed| elapsed < DATA_REFRESH_INTERVAL),
        None => has_refresh_data(downloaddir),
    }
}

fn format_cached_data_log(downloaddir: &Path) -> String {
    let Some(last_refresh) = read_last_refresh(downloaddir) else {
        return "Using cached satkit data".to_string();
    };
    let Ok(elapsed) = last_refresh.elapsed() else {
        return "Using cached satkit data".to_string();
    };
    let days = elapsed.as_secs() / SECONDS_PER_DAY;
    format!("Using cached satkit data (last updated {days} days ago)")
}

fn check_abort(abort: &AtomicBool) -> Result<(), UpdateDataError> {
    if abort.load(Ordering::Acquire) {
        Err(UpdateDataError::Aborted)
    } else {
        Ok(())
    }
}

fn http_agent() -> ureq::Agent {
    ureq::Agent::config_builder()
        .timeout_global(Some(HTTP_POLL_INTERVAL))
        .build()
        .into()
}

fn http_get(url: &str, abort: &AtomicBool) -> Result<ureq::Body, UpdateDataError> {
    let agent = http_agent();
    loop {
        check_abort(abort)?;
        match agent.get(url).call() {
            Ok(response) => return Ok(response.into_body()),
            Err(ureq::Error::Timeout(_)) => continue,
            Err(err) => return Err(UpdateDataError::Other(err.to_string())),
        }
    }
}

fn read_to_string_with_abort(
    reader: &mut impl Read,
    abort: &AtomicBool,
) -> Result<String, UpdateDataError> {
    let mut buffer = [0u8; COPY_BUFFER_SIZE];
    let mut body = Vec::new();
    loop {
        check_abort(abort)?;
        let read = reader
            .read(&mut buffer)
            .map_err(|err| UpdateDataError::Other(err.to_string()))?;
        if read == 0 {
            break;
        }
        body.extend_from_slice(&buffer[..read]);
    }
    String::from_utf8(body).map_err(|err| UpdateDataError::Other(err.to_string()))
}

fn copy_with_abort(
    source: &mut impl Read,
    dest: &mut impl Write,
    abort: &AtomicBool,
) -> Result<(), UpdateDataError> {
    let mut buffer = [0u8; COPY_BUFFER_SIZE];
    loop {
        check_abort(abort)?;
        let read = source
            .read(&mut buffer)
            .map_err(|err| UpdateDataError::Other(err.to_string()))?;
        if read == 0 {
            break;
        }
        dest.write_all(&buffer[..read])
            .map_err(|err| UpdateDataError::Other(err.to_string()))?;
    }
    Ok(())
}

fn download_to_string(url: &str, abort: &AtomicBool) -> Result<String, UpdateDataError> {
    let mut body = http_get(url, abort)?;
    read_to_string_with_abort(&mut body.as_reader(), abort)
}

fn download_file(
    url: &str,
    downloaddir: &Path,
    overwrite_if_exists: bool,
    log: &impl Fn(&str),
    abort: &AtomicBool,
) -> Result<bool, UpdateDataError> {
    check_abort(abort)?;
    let fname = Path::new(url)
        .file_name()
        .and_then(|name| name.to_str())
        .ok_or_else(|| UpdateDataError::Other("invalid download url".into()))?;
    let fullpath = downloaddir.join(fname);
    if fullpath.exists() && !overwrite_if_exists {
        log(&format!("File {fname} exists; skipping download"));
        return Ok(false);
    }
    log(&format!("Downloading {fname}"));
    let mut body = http_get(url, abort)?;
    check_abort(abort)?;
    let mut dest =
        std::fs::File::create(&fullpath).map_err(|err| UpdateDataError::Other(err.to_string()))?;
    if let Err(err) = copy_with_abort(&mut body.as_reader(), &mut dest, abort) {
        drop(dest);
        let _ = std::fs::remove_file(fullpath);
        return Err(err);
    }
    Ok(true)
}

fn download_from_json(
    value: &Value,
    basedir: PathBuf,
    baseurl: String,
    overwrite: bool,
    log: &impl Fn(&str),
    abort: &AtomicBool,
) -> Result<(), UpdateDataError> {
    match value {
        Value::Object(entries) => {
            for (name, child) in entries {
                check_abort(abort)?;
                let child_dir = basedir.join(name);
                if !child_dir.is_dir() {
                    std::fs::create_dir_all(&child_dir)
                        .map_err(|err| UpdateDataError::Other(err.to_string()))?;
                }
                let mut child_url = baseurl.clone();
                child_url.push('/');
                child_url.push_str(name);
                download_from_json(child, child_dir, child_url, overwrite, log, abort)?;
            }
            Ok(())
        }
        Value::Array(entries) => {
            for child in entries {
                check_abort(abort)?;
                download_from_json(
                    child,
                    basedir.clone(),
                    baseurl.clone(),
                    overwrite,
                    log,
                    abort,
                )?;
            }
            Ok(())
        }
        Value::String(file_name) => {
            check_abort(abort)?;
            let mut file_url = baseurl;
            file_url.push('/');
            file_url.push_str(file_name);
            download_file(&file_url, &basedir, overwrite, log, abort)?;
            Ok(())
        }
        _ => Err(UpdateDataError::Other(
            "invalid json for downloading files??!!".into(),
        )),
    }
}

fn download_datadir(
    basedir: PathBuf,
    baseurl: String,
    overwrite: bool,
    log: &impl Fn(&str),
    abort: &AtomicBool,
) -> Result<(), UpdateDataError> {
    check_abort(abort)?;
    if !basedir.is_dir() {
        std::fs::create_dir_all(&basedir).map_err(|err| UpdateDataError::Other(err.to_string()))?;
    }
    let mut fileurl = baseurl.clone();
    fileurl.push_str("/files.json");
    let json_text = download_to_string(&fileurl, abort)?;
    check_abort(abort)?;
    let json_base: Value =
        serde_json::from_str(&json_text).map_err(|err| UpdateDataError::Other(err.to_string()))?;
    download_from_json(&json_base, basedir, baseurl, overwrite, log, abort)
}

fn download_from_url_json(
    json_url: &str,
    basedir: &Path,
    overwrite_if_exists: bool,
    log: &impl Fn(&str),
    abort: &AtomicBool,
) -> Result<(), UpdateDataError> {
    check_abort(abort)?;
    let json_text = download_to_string(json_url, abort)?;
    check_abort(abort)?;
    let json_base: Value =
        serde_json::from_str(&json_text).map_err(|err| UpdateDataError::Other(err.to_string()))?;
    if let Value::Array(entries) = json_base {
        for entry in entries {
            check_abort(abort)?;
            if let Value::String(url) = entry {
                download_file(&url, basedir, overwrite_if_exists, log, abort)?;
            } else {
                return Err(UpdateDataError::Other("invalid refresh json entry".into()));
            }
        }
    }
    Ok(())
}

/// Downloads satkit data files with progress logging and cooperative abort checks.
///
/// This function performs HTTP requests and is intended for interactive app use only.
/// Unit and integration tests must not call it (see `docs/design_overview.md`).
pub fn update_datafiles_with_log(
    log: &impl Fn(&str),
    abort: &AtomicBool,
) -> Result<(), UpdateDataError> {
    check_abort(abort)?;
    let downloaddir = datadir().map_err(|err| UpdateDataError::Other(err.to_string()))?;
    let metadata = downloaddir
        .metadata()
        .map_err(|err| UpdateDataError::Other(err.to_string()))?;
    if metadata.permissions().readonly() {
        return Err(UpdateDataError::Other(
            r#"
            Data directory is read-only.
            Try setting SATKIT_DATA environment
            variable to a writeable directory and re-starting
            "#
            .into(),
        ));
    }

    if should_skip_refresh(&downloaddir) {
        log(&format_cached_data_log(&downloaddir));
        if read_last_refresh(&downloaddir).is_none() {
            write_last_refresh(&downloaddir)?;
        }
        return Ok(());
    }

    log(&format!(
        "Downloading data files to {}",
        downloaddir.to_str().unwrap_or("<unknown>")
    ));
    download_datadir(downloaddir.clone(), BASE_URL.to_string(), false, log, abort)?;

    log("Now downloading files that are regularly updated:");
    log("  Space Weather & Earth Orientation Parameters");
    let force_refresh = is_refresh_due(&downloaddir);
    download_from_url_json(FILES_REFRESH_URL, &downloaddir, force_refresh, log, abort)?;
    write_last_refresh(&downloaddir)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn temp_data_dir(test_name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "dual-spacetime-simulator-solar-system-data-{test_name}-{}",
            std::process::id()
        ));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).expect("create temp data dir");
        dir
    }

    fn write_refresh_timestamp(downloaddir: &Path, at: SystemTime) {
        let seconds = at
            .duration_since(UNIX_EPOCH)
            .expect("timestamp before unix epoch")
            .as_secs();
        fs::write(last_refresh_path(downloaddir), seconds.to_string())
            .expect("write refresh timestamp");
    }

    #[test]
    fn update_data_error_display_and_equality() {
        assert_eq!(UpdateDataError::Aborted, UpdateDataError::Aborted);
        assert_eq!(UpdateDataError::Aborted.to_string(), "aborted");
        assert_eq!(
            UpdateDataError::Other("network failure".into()).to_string(),
            "network failure"
        );
    }

    #[test]
    fn check_abort_returns_error_when_flag_is_set() {
        let abort = AtomicBool::new(true);
        assert_eq!(check_abort(&abort), Err(UpdateDataError::Aborted));

        abort.store(false, Ordering::Release);
        assert_eq!(check_abort(&abort), Ok(()));
    }

    #[test]
    fn is_jplephem_filename_matches_de4xx_layout() {
        assert!(is_jplephem_filename("linux_p1550p2650.440"));
        assert!(is_jplephem_filename("lnxp1550p2650.421"));
        assert!(!is_jplephem_filename("EOP-All.csv"));
        assert!(!is_jplephem_filename("linux_p1550p2650.440.bak"));
    }

    #[test]
    fn has_jplephem_data_detects_local_ephemeris_file() {
        let dir = temp_data_dir("has-jplephem-data");
        assert!(!has_jplephem_data(&dir));

        fs::write(dir.join("linux_p1550p2650.440"), b"stub").expect("write ephemeris stub");
        assert!(has_jplephem_data(&dir));

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn read_and_write_last_refresh_round_trip() {
        let dir = temp_data_dir("refresh-round-trip");
        let at = UNIX_EPOCH + Duration::from_secs(1_700_000_000);

        write_refresh_timestamp(&dir, at);
        assert_eq!(read_last_refresh(&dir), Some(at));

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn should_skip_refresh_when_data_is_recent() {
        let dir = temp_data_dir("skip-recent");
        fs::write(dir.join("linux_p1550p2650.440"), b"stub").expect("write ephemeris stub");
        write_refresh_timestamp(&dir, SystemTime::now());

        assert!(should_skip_refresh(&dir));

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn should_not_skip_refresh_when_data_is_stale() {
        let dir = temp_data_dir("skip-stale");
        fs::write(dir.join("linux_p1550p2650.440"), b"stub").expect("write ephemeris stub");
        write_refresh_timestamp(
            &dir,
            SystemTime::now() - DATA_REFRESH_INTERVAL - Duration::from_secs(1),
        );

        assert!(!should_skip_refresh(&dir));

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn should_not_skip_refresh_when_ephemeris_file_is_missing() {
        let dir = temp_data_dir("skip-missing-ephemeris");
        write_refresh_timestamp(&dir, SystemTime::now());

        assert!(!should_skip_refresh(&dir));

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn has_refresh_data_detects_eop_and_sw_files() {
        let dir = temp_data_dir("has-refresh-data");
        assert!(!has_refresh_data(&dir));

        fs::write(dir.join(EOP_FILENAME), b"eop").expect("write eop stub");
        assert!(!has_refresh_data(&dir));

        fs::write(dir.join(SW_FILENAME), b"sw").expect("write sw stub");
        assert!(has_refresh_data(&dir));

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn should_skip_refresh_when_timestamp_is_missing_but_refresh_files_exist() {
        let dir = temp_data_dir("skip-missing-timestamp-with-refresh");
        fs::write(dir.join("linux_p1550p2650.440"), b"stub").expect("write ephemeris stub");
        fs::write(dir.join(EOP_FILENAME), b"eop").expect("write eop stub");
        fs::write(dir.join(SW_FILENAME), b"sw").expect("write sw stub");

        assert!(should_skip_refresh(&dir));

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn should_not_skip_refresh_when_timestamp_and_refresh_files_are_missing() {
        let dir = temp_data_dir("skip-missing-timestamp");
        fs::write(dir.join("linux_p1550p2650.440"), b"stub").expect("write ephemeris stub");

        assert!(!should_skip_refresh(&dir));

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn is_refresh_due_when_timestamp_is_older_than_interval() {
        let dir = temp_data_dir("refresh-due");
        write_refresh_timestamp(
            &dir,
            SystemTime::now() - DATA_REFRESH_INTERVAL - Duration::from_secs(1),
        );

        assert!(is_refresh_due(&dir));

        let _ = fs::remove_dir_all(&dir);
    }
}
