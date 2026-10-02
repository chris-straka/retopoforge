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

pub(crate) const RETOPO_VERSION: &str = "0.1.0";

fn flush_stdout() {
    let _ = std::io::stdout().flush();
}

/// Parse argv and run the pipeline, returning the process exit code.
/// Parse failures print their error (plus usage where historical) and
/// yield 1; `--help`/`--version` print and yield 0.
fn run_bin(argv: &[String]) -> i32 {
    let action = match parse_args(argv) {
        Ok(action) => action,
        Err(failure) => {
            failure.error.emit();
            if failure.show_usage {
                print_usage(&argv[0]);
            }
            return 1;
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
