//! Binary self-attestation.
//!
//! At process start the binary computes a SHA-256 of its own on-disk
//! executable and emits a structured `tracing` event + (optionally) a
//! first line of the JSON event log. The hash is the operator's
//! ground-truth answer to "is the binary I'm running the one I built
//! and audited?"
//!
//! This is provenance recording, NOT a security boundary against an
//! attacker who already has code execution on the device:
//!   - An attacker who can replace `/proc/self/exe` can also bypass this.
//!   - A TPM-backed measured boot is the right primitive for that threat.
//!   - We do this anyway because it's free post-incident forensic data
//!     and catches the dominant operational failure mode: deploying the
//!     wrong build by accident.

use serde::Serialize;
use sha2::{Digest, Sha256};
use std::fs::File;
use std::io::{self, Read};
use std::path::PathBuf;

/// Snapshot of build/runtime identity. Emitted once at process start.
#[derive(Debug, Clone, Serialize)]
pub struct Attestation {
    pub crate_version: String,
    pub build_profile: &'static str,
    pub host_os: &'static str,
    pub host_arch: &'static str,
    pub binary_path: Option<String>,
    /// Hex-encoded SHA-256 of the binary at `binary_path`. `None` if we
    /// couldn't read the file (rare — e.g., container with /proc disabled).
    pub binary_sha256: Option<String>,
    /// Optional `git rev-parse HEAD` value, populated only if the build
    /// captured it via `OUT_DIR` / `build.rs`. We DON'T do that today so
    /// this is `None`, but the field exists so operators have a stable
    /// schema to consume.
    pub git_commit: Option<String>,
}

impl Attestation {
    /// Capture the current process's identity. Cheap-ish (one SHA-256 over
    /// the binary, ~30 MB → ~50 ms on a Pi 4). Run once at startup.
    pub fn capture() -> Self {
        // Resolve current_exe ONCE and derive both fields from it (audit D5):
        // two separate lookups could disagree if the executable is swapped
        // mid-startup, producing a path/hash pair that describes two
        // different files — poisonous for a record whose whole job is
        // provenance.
        let exe = std::env::current_exe().ok();
        let binary_path = exe.as_deref().map(|p| p.display().to_string());
        let binary_sha256 = exe.as_deref().and_then(|p| sha256_of_file(p).ok());
        Attestation {
            crate_version: env!("CARGO_PKG_VERSION").to_string(),
            build_profile: if cfg!(debug_assertions) {
                "debug"
            } else {
                "release"
            },
            host_os: std::env::consts::OS,
            host_arch: std::env::consts::ARCH,
            binary_path,
            binary_sha256,
            git_commit: option_env!("FLYINGSQUIRREL_GIT_COMMIT").map(str::to_string),
        }
    }

    /// Emit the attestation as a `tracing::warn!` (visible by default at
    /// any log level above `error`) and as the FIRST line of the JSON
    /// event log if `json_log_path` is set.
    pub async fn emit(&self, json_log_path: Option<&PathBuf>) {
        tracing::warn!(
            version = %self.crate_version,
            profile = self.build_profile,
            os = self.host_os,
            arch = self.host_arch,
            binary = self.binary_path.as_deref().unwrap_or("?"),
            sha256 = self.binary_sha256.as_deref().unwrap_or("?"),
            "ATTESTATION: binary identity captured at startup"
        );
        if let Some(p) = json_log_path {
            // Best-effort: any failure here logs a warning but does not
            // abort startup. The tracing log line above is the durable
            // record; the JSON file is a convenience for downstream tools.
            if let Err(e) = self.write_to_jsonl(p).await {
                tracing::warn!(error = %e, path = %p.display(),
                    "failed to write attestation line to JSON event log");
            }
        }
    }

    async fn write_to_jsonl(&self, path: &PathBuf) -> io::Result<()> {
        use tokio::io::AsyncWriteExt;
        let line = serde_json::to_string(&AttestationLogLine {
            kind: "Attestation",
            ..self.into()
        })
        .map_err(io::Error::other)?;
        let mut file = tokio::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .await?;
        file.write_all(line.as_bytes()).await?;
        file.write_all(b"\n").await?;
        file.flush().await?;
        Ok(())
    }
}

#[derive(Serialize)]
struct AttestationLogLine<'a> {
    kind: &'a str,
    version: &'a str,
    profile: &'a str,
    os: &'a str,
    arch: &'a str,
    binary: Option<&'a str>,
    sha256: Option<&'a str>,
    git: Option<&'a str>,
}

impl<'a> From<&'a Attestation> for AttestationLogLine<'a> {
    fn from(a: &'a Attestation) -> Self {
        AttestationLogLine {
            kind: "Attestation",
            version: &a.crate_version,
            profile: a.build_profile,
            os: a.host_os,
            arch: a.host_arch,
            binary: a.binary_path.as_deref(),
            sha256: a.binary_sha256.as_deref(),
            git: a.git_commit.as_deref(),
        }
    }
}

fn sha256_of_file(path: &std::path::Path) -> io::Result<String> {
    let mut file = File::open(path)?;
    let mut hasher = Sha256::new();
    let mut buf = [0u8; 64 * 1024];
    loop {
        let n = file.read(&mut buf)?;
        if n == 0 {
            break;
        }
        hasher.update(&buf[..n]);
    }
    Ok(hex_encode(&hasher.finalize()))
}

fn hex_encode(bytes: &[u8]) -> String {
    let mut s = String::with_capacity(bytes.len() * 2);
    for b in bytes {
        use std::fmt::Write;
        let _ = write!(s, "{:02x}", b);
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capture_returns_finite_values() {
        let a = Attestation::capture();
        assert!(!a.crate_version.is_empty());
        // Hash is 64 hex chars (32 bytes × 2) when present.
        if let Some(h) = &a.binary_sha256 {
            assert_eq!(h.len(), 64);
            assert!(h.chars().all(|c| c.is_ascii_hexdigit()));
        }
    }

    #[test]
    fn hex_encode_known_value() {
        // sha256 of empty input = e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855
        let h = hex_encode(&Sha256::new().finalize());
        assert_eq!(
            h,
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        );
    }
}
