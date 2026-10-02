//! Report output: the `--report` file and the per-rung stdout lines.
//!
//! Writes accumulate into a sticky `ok` flag (a failed write poisons the
//! report but the pipeline keeps going; the error surfaces once, at the
//! end). All text is pinned by the CLI contract goldens.

use crate::format::g_format;
use std::fs::File;
use std::io::Write;
use std::path::Path;

pub(crate) struct Report {
    file: File,
    ok: bool,
}

impl Report {
    pub(crate) fn create(path: &Path) -> Option<Self> {
        File::create(path).map(|file| Self { file, ok: true }).ok()
    }

    pub(crate) fn line(&mut self, text: &str) {
        self.ok &= writeln!(self.file, "{text}").is_ok();
    }

    pub(crate) fn blank(&mut self) {
        self.ok &= writeln!(self.file).is_ok();
    }

    /// Flush and report whether every write succeeded.
    pub(crate) fn finish(mut self) -> bool {
        self.ok &= self.file.flush().is_ok();
        self.ok
    }
}

/// One `target-quads=... quads=... ...` stdout line per remeshed rung.
pub(crate) fn print_rung_line(
    label: &str,
    output_path: &Path,
    target_quads: i64,
    quad_count: usize,
    non_quad_count: usize,
    vertex_count: usize,
    elapsed_seconds: f64,
) {
    println!(
        "{label}target-quads={target_quads} output={} quads={quad_count} non-quads={non_quad_count} vertices={vertex_count} time={} seconds",
        output_path.display(),
        g_format(elapsed_seconds),
    );
}
