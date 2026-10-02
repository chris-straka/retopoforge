//! The `retopo` headless CLI: remesh `.obj`/`.glb` files or whole
//! directories, with single-file, `--lods`, and batch modes.
//!
//! Layout: [`args`] parses the command line into a [`args::Config`];
//! [`run`] executes it; [`mesh_io`], [`constraint_files`], [`report`],
//! [`progress`], and [`format`] are the pieces both use; [`error`]
//! carries CLI-owned failures to the single emission point in [`run_bin`].
//! Every user-visible byte (help, errors, reports, meshes) and exit code
//! is pinned by the CLI contract test (`tests/cli_contract.rs`).

mod args;
mod constraint_files;
mod error;
mod format;
mod mesh_io;
mod progress;
mod report;
mod run;

use args::{Action, parse_args, print_usage};
use std::io::Write as _;

/// CLI + mesh-stamp version, single-sourced from the workspace
/// `Cargo.toml` (synced to the 0.3.0 packaging in the CLI UX pass).
pub(crate) const RETOPO_VERSION: &str = env!("CARGO_PKG_VERSION");

fn flush_stdout() {
    let _ = std::io::stdout().flush();
}

/// Parse argv and run the pipeline, returning the process exit code:
/// 0 success (warned-about island drops included), 1 runtime failure,
/// 2 usage error. Usage-shape failures point at `--help` (never a usage
/// dump); clamp warnings print before anything else runs.
fn run_bin(argv: &[String]) -> i32 {
    let action = match parse_args(argv) {
        Ok(action) => action,
        Err(failure) => {
            failure.error.emit();
            if failure.show_pointer {
                eprintln!("Run 'retopo --help' for usage.");
            }
            return 2;
        }
    };
    match action {
        Action::Help => {
            print_usage(&argv[0]);
            0
        }
        Action::Version => {
            println!("retopoforge {RETOPO_VERSION}");
            0
        }
        Action::Run(config) => {
            for warning in &config.warnings {
                eprintln!("{warning}");
            }
            let batch = config.input.is_dir();
            if !config.lod_targets.is_empty() || batch {
                run::run_multi_mode(&config, batch)
            } else {
                run::run_single_mode(&config)
            }
        }
    }
}

fn main() {
    // `args_os` + lossy conversion: `env::args` panics on non-UTF8 argv
    // while raw bytes must keep working; lossy keeps every UTF-8 case
    // byte-exact and degrades gracefully otherwise.
    let argv: Vec<String> = std::env::args_os()
        .map(|a| a.to_string_lossy().into_owned())
        .collect();
    let code = run_bin(&argv);
    // `process::exit` does not flush `stdout`.
    flush_stdout();
    std::process::exit(code);
}
