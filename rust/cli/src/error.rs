//! CLI-owned errors: historical bytes, centralized emission.
//!
//! Every error line the CLI prints keeps its exact historical text (the
//! CLI contract goldens pin it byte-for-byte). What changed is the
//! plumbing: fallible helpers return [`CliError`] and `main` emits it in
//! one place, instead of every parser printing to stderr as it fails.

/// An error the CLI reports on stderr.
pub(crate) enum CliError {
    /// One UTF-8 line, printed with a trailing newline.
    Message(String),
    /// Raw bytes (a constraint-file line that may not be UTF-8),
    /// written verbatim followed by a newline.
    Raw(Vec<u8>),
}

impl CliError {
    pub(crate) fn message(text: impl Into<String>) -> Self {
        Self::Message(text.into())
    }

    /// Print the error exactly as the CLI always has.
    pub(crate) fn emit(&self) {
        match self {
            Self::Message(text) => eprintln!("{text}"),
            Self::Raw(bytes) => {
                use std::io::Write;
                let mut err = std::io::stderr().lock();
                let _ = err.write_all(bytes);
                let _ = err.write_all(b"\n");
                let _ = err.flush();
            }
        }
    }
}
