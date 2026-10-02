//! CLI-owned usage errors: `retopo: error:` convention, exit 2.
//!
//! The errors that flow (flag parsing, constraint files, batch input
//! collection) are all usage errors; runtime failures (load/remesh/
//! write) report inline at their call sites with exit 1. See
//! `docs/cli-ux-redesign.md` §5. All text and codes are pinned by the
//! CLI contract goldens.

/// A usage error the CLI reports on stderr (exit 2).
pub(crate) enum CliError {
    /// One UTF-8 line, printed with a trailing newline.
    Usage(String),
    /// Raw bytes (a constraint-file line that may not be UTF-8),
    /// written verbatim followed by a newline.
    UsageRaw(Vec<u8>),
}

impl CliError {
    pub(crate) fn usage(text: impl Into<String>) -> Self {
        Self::Usage(text.into())
    }

    /// Print the error (exactly one line plus newline).
    pub(crate) fn emit(&self) {
        match self {
            Self::Usage(text) => eprintln!("{text}"),
            Self::UsageRaw(bytes) => {
                use std::io::Write;
                let mut err = std::io::stderr().lock();
                let _ = err.write_all(bytes);
                let _ = err.write_all(b"\n");
                let _ = err.flush();
            }
        }
    }
}

/// Portable OS-failure reason: common kinds map to static strings (so
/// goldens hold cross-platform); exotic kinds keep the full OS text.
pub(crate) fn os_reason(err: &std::io::Error) -> String {
    use std::io::ErrorKind::*;
    match err.kind() {
        NotFound => "no such file or directory".to_string(),
        PermissionDenied => "permission denied".to_string(),
        _ => err.to_string(),
    }
}
