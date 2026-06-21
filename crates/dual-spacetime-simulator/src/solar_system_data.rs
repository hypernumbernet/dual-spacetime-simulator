use satkit::utils::datadir;
use serde_json::Value;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

const BASE_URL: &str = "https://storage.googleapis.com/astrokit-astro-data";
const FILES_REFRESH_URL: &str =
    "https://storage.googleapis.com/astrokit-astro-data/files_refresh.json";
const HTTP_POLL_INTERVAL: Duration = Duration::from_millis(200);
const COPY_BUFFER_SIZE: usize = 8192;

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
    let mut dest = std::fs::File::create(&fullpath)
        .map_err(|err| UpdateDataError::Other(err.to_string()))?;
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
                download_from_json(child, basedir.clone(), baseurl.clone(), overwrite, log, abort)?;
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
                download_file(&url, basedir, true, log, abort)?;
            } else {
                return Err(UpdateDataError::Other(
                    "invalid refresh json entry".into(),
                ));
            }
        }
    }
    Ok(())
}

/// Downloads satkit data files with progress logging and cooperative abort checks.
pub fn update_datafiles_with_log(
    log: &impl Fn(&str),
    abort: &AtomicBool,
) -> Result<(), UpdateDataError> {
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

    log(&format!(
        "Downloading data files to {}",
        downloaddir.to_str().unwrap_or("<unknown>")
    ));
    download_datadir(
        downloaddir.clone(),
        BASE_URL.to_string(),
        false,
        log,
        abort,
    )?;

    log("Now downloading files that are regularly updated:");
    log("  Space Weather & Earth Orientation Parameters");
    download_from_url_json(FILES_REFRESH_URL, &downloaddir, log, abort)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn abort_before_download_returns_aborted() {
        let abort = AtomicBool::new(true);
        let result = update_datafiles_with_log(&|_| {}, &abort);
        assert_eq!(result, Err(UpdateDataError::Aborted));
    }
}
