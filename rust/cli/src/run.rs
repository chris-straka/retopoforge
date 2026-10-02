//! Run modes: single-file, `--lods`, and batch-directory.
//!
//! A single input without `--lods` takes the single path (stdout report
//! block plus an optional `--report` file). `--lods` and batch mode
//! share the multi path (one stdout line per rung, optional `--report`
//! file, `Failed files:` summary in batch). Exit 0 when every file
//! produced output, 1 otherwise. All text and exit codes are pinned by
//! the CLI contract goldens.

use crate::args::Config;
use crate::constraint_files::{Constraints, parse_density_file, parse_guides_file};
use crate::error::CliError;
use crate::error::os_reason;
use crate::format::g_format;
use crate::mesh_io::{
    LoadFailure, LoadedMesh, cpp_stem_ext, file_name_of, load_mesh, lod_output_path, report_loaded,
    save_mesh, warn_dropped_non_finite,
};
use crate::progress::{ProgressState, attach_progress};
use crate::report::{Report, print_rung_line};
use retopo_core::auto_remesher::{AutoRemesher, CoverageReport, ModelType};
use retopo_core::glb as glb_io;
use retopo_core::mesh_separator::MeshSeparator;
use retopo_core::vector3::Vector3;
use std::path::Path;
use std::sync::Mutex;
use std::time::Instant;

#[derive(Default)]
struct RungResult {
    ok: bool,
    quad_count: usize,
    non_quad_count: usize,
    vertex_count: usize,
    island_count: usize,
    failed_islands: usize,
    coverage_reports: Vec<CoverageReport>,
    elapsed_seconds: f64,
    error: String,
}

/// Coverage retry outcomes, reported like failed islands (same
/// unconditional `Warning:` channel; batch runs prefix the file name).
fn print_coverage_warnings(reports: &[CoverageReport], batch_name: Option<&str>) {
    let prefix = batch_name
        .map(|n| format!("FILE {n}: "))
        .unwrap_or_default();
    for r in reports {
        let side = if r.input_side {
            "input verts"
        } else {
            "working verts"
        };
        if r.recovered {
            eprintln!(
                "retopo: warning: {prefix}island {} failed the coverage check ({} {side} uncovered) and recovered on retry {} ({} verts uncovered).",
                r.island_index, r.initial_uncovered, r.retries_made, r.final_uncovered
            );
        } else if r.kept_attempt == 0 {
            eprintln!(
                "retopo: warning: {prefix}island {} failed the coverage check ({} {side} uncovered); retries did not recover, kept the original.",
                r.island_index, r.initial_uncovered
            );
        } else {
            eprintln!(
                "retopo: warning: {prefix}island {} failed the coverage check ({} {side} uncovered); retries did not recover, kept attempt {} (working-side cover; input side still short).",
                r.island_index, r.initial_uncovered, r.kept_attempt
            );
        }
    }
}

/// Plural-correct island-drop verb ("1 ... was", "2 ... were").
fn island_verb(count: usize) -> &'static str {
    if count == 1 { "was" } else { "were" }
}

fn count_islands_without_output(
    islands: &[Vec<Vec<usize>>],
    input_vertices: &[Vector3],
    output_vertices: &[Vector3],
) -> usize {
    let mut failed = 0;
    for island in islands {
        let mut first = true;
        let (mut min_x, mut min_y, mut min_z) = (0.0, 0.0, 0.0);
        let (mut max_x, mut max_y, mut max_z) = (0.0, 0.0, 0.0);
        for face in island {
            for &index in face {
                let v = &input_vertices[index];
                if first {
                    min_x = v.x();
                    max_x = v.x();
                    min_y = v.y();
                    max_y = v.y();
                    min_z = v.z();
                    max_z = v.z();
                    first = false;
                } else {
                    if v.x() < min_x {
                        min_x = v.x();
                    }
                    if v.x() > max_x {
                        max_x = v.x();
                    }
                    if v.y() < min_y {
                        min_y = v.y();
                    }
                    if v.y() > max_y {
                        max_y = v.y();
                    }
                    if v.z() < min_z {
                        min_z = v.z();
                    }
                    if v.z() > max_z {
                        max_z = v.z();
                    }
                }
            }
        }
        if first {
            failed += 1;
            continue;
        }
        let dx = max_x - min_x;
        let dy = max_y - min_y;
        let dz = max_z - min_z;
        let pad = (dx * dx + dy * dy + dz * dz).sqrt() * 0.01 + 1e-6;
        let mut found = false;
        for v in output_vertices {
            if v.x() >= min_x - pad
                && v.x() <= max_x + pad
                && v.y() >= min_y - pad
                && v.y() <= max_y + pad
                && v.z() >= min_z - pad
                && v.z() <= max_z + pad
            {
                found = true;
                break;
            }
        }
        if !found {
            failed += 1;
        }
    }
    failed
}

/// Islands that produced no output: exact when the engine's per-island
/// quad counts align 1:1 with the input islands (a zero-output island
/// failed, whatever stage ate it), else the bbox heuristic.
fn dropped_island_count(
    engine_quad_counts: &[usize],
    islands: &[Vec<Vec<usize>>],
    input_vertices: &[Vector3],
    output_vertices: &[Vector3],
) -> usize {
    if engine_quad_counts.len() == islands.len() {
        return engine_quad_counts.iter().filter(|&&q| q == 0).count();
    }
    count_islands_without_output(islands, input_vertices, output_vertices)
}

/// A `--density` mask must cover exactly the loaded vertices (an absent
/// mask is fine). Returns the shared mismatch message (no prefix or
/// period: the callers report it in convention shape).
fn check_density_len(density: &[f64], vertices: usize) -> Result<(), String> {
    if !density.is_empty() && density.len() != vertices {
        return Err(format!(
            "--density file holds {} multipliers but the input has {} vertices (need exactly one per input vertex)",
            density.len(),
            vertices
        ));
    }
    Ok(())
}

/// Apply the CLI configuration to a fresh remesher. `target_triangles`
/// is the precomputed `2 * target_quads` (kept at the call sites: the
/// single and multi paths cast from different integer types).
fn configure_remesher(
    remesher: &mut AutoRemesher,
    config: &Config,
    guides: Vec<Vec<Vector3>>,
    features: Vec<Vec<Vector3>>,
    density: &[f64],
    target_triangles: usize,
) {
    remesher.set_target_triangle_count(target_triangles);
    remesher.set_symmetry_enabled(config.symmetry.enabled());
    remesher.set_symmetry_plane(config.symmetry.plane());
    remesher.set_guide_polylines(guides);
    remesher.set_sharp_polylines(features);
    remesher.set_density_multipliers(density);
    remesher.set_dipoles(config.dipoles);
    if config.edge_scaling > 0.0 {
        remesher.set_scaling(config.edge_scaling);
    }
    remesher.set_model_type(config.model_type);
    remesher.set_gradient_adaptivity(config.adaptivity);
    remesher.set_anisotropy(config.anisotropy);
    remesher.set_sharp_edge_degrees(config.sharp_edge_degrees);
    remesher.set_smooth_normal_degrees(config.smooth_normal_degrees);
    remesher.set_compute_remeshed_uvs(config.emit_uvs);
    remesher.set_quiet(config.quiet);
}

/// Remesh one loaded mesh at one target count and save it: the unit of
/// work in `--lods`/batch mode.
fn remesh_loaded_mesh(
    config: &Config,
    vertices: &[Vector3],
    triangles: &[Vec<usize>],
    constraints: &Constraints,
    target_quads: i64,
    output_path: &Path,
) -> RungResult {
    let mut result = RungResult::default();
    let start_time = Instant::now();

    if let Err(error) = check_density_len(&constraints.density, vertices.len()) {
        result.error = error;
        return result;
    }

    let mut remesher = AutoRemesher::new(vertices, triangles);
    configure_remesher(
        &mut remesher,
        config,
        constraints.guides.clone(),
        constraints.features.clone(),
        &constraints.density,
        (target_quads as usize).wrapping_mul(2),
    );
    let progress_state = Mutex::new(ProgressState::default());
    if !config.quiet {
        attach_progress(&mut remesher, &progress_state);
    }

    if !remesher.remesh() {
        result.error = "remeshing produced no result".to_string();
        return result;
    }

    if !config.quiet {
        for line in remesher.phase_report() {
            eprintln!("  {line}");
        }
    }

    let remeshed_vertices = remesher.remeshed_vertices();
    let remeshed_quads = remesher.remeshed_quads();

    for face in remeshed_quads {
        if face.len() == 4 {
            result.quad_count += 1;
        } else {
            result.non_quad_count += 1;
        }
    }
    result.vertex_count = remeshed_vertices.len();

    let mut input_islands: Vec<Vec<Vec<usize>>> = Vec::new();
    MeshSeparator::split_to_islands(triangles, &mut input_islands);
    result.island_count = input_islands.len();
    result.failed_islands = dropped_island_count(
        remesher.island_output_quad_counts(),
        &input_islands,
        vertices,
        remeshed_vertices,
    );
    result.coverage_reports = remesher.coverage_reports().to_vec();

    // The save below fails on empty verts: name the real cause (a
    // collapsed remesh), never the disk.
    if remeshed_vertices.is_empty() {
        let noun = if result.island_count == 1 {
            "island"
        } else {
            "islands"
        };
        result.error = format!(
            "remeshing produced an empty mesh (all {} {noun} dropped); nothing written to '{}'. Try a larger --target-quads",
            result.island_count,
            output_path.display()
        );
        return result;
    }

    let uvs = if config.emit_uvs {
        Some(remesher.remeshed_vertex_uvs())
    } else {
        None
    };
    if !save_mesh(output_path, remeshed_vertices, remeshed_quads, uvs) {
        result.error = format!("failed to write {}", output_path.display());
        return result;
    }

    result.elapsed_seconds = start_time.elapsed().as_secs_f64();
    result.ok = true;
    result
}

fn model_type_name(model_type: ModelType) -> &'static str {
    if model_type == ModelType::Organic {
        "organic"
    } else {
        "hardsurface"
    }
}

fn write_multi_header(report: &mut Report, config: &Config) {
    report.line("retopoforge Report");
    report.line("==================");
    report.blank();
    report.line(&format!("Edge scaling: {}", g_format(config.edge_scaling)));
    report.line(&format!(
        "Sharp edge degrees: {}",
        g_format(config.sharp_edge_degrees)
    ));
    report.line(&format!(
        "Smooth normal degrees: {}",
        g_format(config.smooth_normal_degrees)
    ));
    report.line(&format!("Adaptivity: {}", g_format(config.adaptivity)));
    report.line(&format!("Anisotropy: {}", g_format(config.anisotropy)));
    report.line(&format!(
        "Model type: {}",
        model_type_name(config.model_type)
    ));
    report.blank();
}

/// Collect the batch inputs: regular files with a supported extension,
/// following symlinks, sorted by name. Also counts skipped non-mesh
/// files (reported as one `note:` by the caller, never silently).
fn collect_batch_inputs(config: &Config) -> Result<(Vec<String>, usize), CliError> {
    let entries = match std::fs::read_dir(&config.input) {
        Ok(entries) => entries,
        Err(err) => {
            return Err(CliError::usage(format!(
                "retopo: error: cannot read directory {}: {}.",
                config.input.display(),
                os_reason(&err)
            )));
        }
    };
    let mut inputs: Vec<String> = Vec::new();
    let mut skipped = 0usize;
    for entry in entries {
        let entry = match entry {
            Ok(entry) => entry,
            Err(err) => {
                return Err(CliError::usage(format!(
                    "retopo: error: cannot read directory {}: {}.",
                    config.input.display(),
                    os_reason(&err)
                )));
            }
        };
        // `metadata` follows symlinks: a symlink to a mesh counts.
        let path = entry.path();
        match std::fs::metadata(&path) {
            Ok(meta) if meta.is_file() => {}
            _ => continue,
        }
        let name = path.to_string_lossy().to_string();
        if glb_io::is_supported_input_extension(&name) {
            inputs.push(name);
        } else {
            skipped += 1;
        }
    }
    inputs.sort();
    if inputs.is_empty() {
        return Err(CliError::usage(format!(
            "retopo: error: no .obj/.glb files in {}.",
            config.input.display()
        )));
    }
    Ok((inputs, skipped))
}

/// Load `--guides`/`--features`/`--density`, rejecting them in batch
/// mode (they address one specific mesh). All failures are usage
/// errors (exit 2): unreadable/malformed files fail before any mesh
/// loads.
fn load_constraints(config: &Config, batch: bool) -> Result<Constraints, CliError> {
    if batch && config.guides.is_some() {
        return Err(CliError::usage(
            "retopo: error: --guides needs a single input mesh, not a batch directory.",
        ));
    }
    if batch && config.features.is_some() {
        return Err(CliError::usage(
            "retopo: error: --features needs a single input mesh, not a batch directory.",
        ));
    }
    if batch && config.density.is_some() {
        return Err(CliError::usage(
            "retopo: error: --density needs a single input mesh, not a batch directory.",
        ));
    }
    let mut constraints = Constraints::default();
    if let Some(path) = &config.guides {
        parse_guides_file(path, &mut constraints.guides, "--guides")?;
    }
    if let Some(path) = &config.features {
        parse_guides_file(path, &mut constraints.features, "--features")?;
    }
    if let Some(path) = &config.density {
        parse_density_file(path, &mut constraints.density)?;
    }
    Ok(constraints)
}

/// Batch guard: a nonexistent `--output` whose final component has a
/// mesh extension is a mistyped file, not a directory to create.
fn output_looks_like_file(output: &std::path::Path) -> bool {
    glb_io::is_supported_input_extension(&file_name_of(output))
}

/// Fail-fast output check (single-file and `--lods` runs never create
/// directories): the parent must exist and be a directory. Bare file
/// names resolve against the working directory (proven at save time).
fn probe_output_parent(path: &std::path::Path) -> Result<(), String> {
    let parent = match path.parent() {
        Some(parent) if !parent.as_os_str().is_empty() => parent,
        _ => return Ok(()),
    };
    match std::fs::metadata(parent) {
        Ok(meta) if meta.is_dir() => Ok(()),
        Ok(_) => Err("not a directory".to_string()),
        Err(err) => Err(os_reason(&err)),
    }
}

/// Multi-mode load failure: one stderr line, the byte-stable stdout
/// `FAILED to load` line, and a `Failed:` report entry (failed inputs
/// are no longer silent in `--report`).
fn report_load_failure(
    report: Option<&mut Report>,
    input_path: &str,
    failure: &LoadFailure,
    file_label: &str,
    failed_files: &mut Vec<String>,
    fail_name: &str,
) {
    eprintln!(
        "retopo: error: cannot {} input '{input_path}': {}.",
        failure.verb(),
        failure.reason()
    );
    println!("{file_label}FAILED to load {input_path}");
    if let Some(report) = report {
        report.line(&format!("Input file: {input_path}"));
        report.line(&format!(
            "Failed: cannot {} input: {}",
            failure.verb(),
            failure.reason()
        ));
        report.blank();
    }
    push_failed(failed_files, fail_name);
}

fn push_failed(failed_files: &mut Vec<String>, name: &str) {
    if !failed_files.iter().any(|n| n == name) {
        failed_files.push(name.to_string());
    }
}

pub(crate) fn run_multi_mode(config: &Config, batch: bool) -> i32 {
    let constraints = match load_constraints(config, batch) {
        Ok(constraints) => constraints,
        Err(error) => {
            error.emit();
            return 2;
        }
    };
    if batch {
        let (inputs, skipped) = match collect_batch_inputs(config) {
            Ok(collected) => collected,
            Err(error) => {
                error.emit();
                return 2;
            }
        };
        if skipped > 0 {
            let noun = if skipped == 1 { "file" } else { "files" };
            eprintln!(
                "note: skipped {skipped} non-mesh {noun} in {}",
                config.input.display()
            );
        }
        return run_multi_inputs(config, true, &inputs, &constraints);
    }
    run_multi_inputs(
        config,
        false,
        &[config.input.to_string_lossy().into_owned()],
        &constraints,
    )
}

/// The multi-mode input loop, shared by batch and `--lods` runs.
fn run_multi_inputs(
    config: &Config,
    batch: bool,
    inputs: &[String],
    constraints: &Constraints,
) -> i32 {
    if batch {
        match std::fs::metadata(&config.output) {
            Ok(meta) if !meta.is_dir() => {
                eprintln!(
                    "retopo: error: --output must be a directory when --input is a directory."
                );
                return 2;
            }
            Ok(_) => {}
            Err(err) if err.kind() == std::io::ErrorKind::NotFound => {
                if output_looks_like_file(&config.output) {
                    eprintln!(
                        "retopo: error: --input is a directory, so --output must be a directory: '{}' looks like a file (hint: drop the extension or point --output at an existing directory).",
                        config.output.display()
                    );
                    return 2;
                }
            }
            Err(_) => {}
        }
        if let Err(err) = std::fs::create_dir_all(&config.output) {
            eprintln!(
                "retopo: error: cannot create output directory {}: {}.",
                config.output.display(),
                os_reason(&err)
            );
            return 1;
        }
    }

    let targets: Vec<i64> = if config.lod_targets.is_empty() {
        vec![config.target_quads as i64]
    } else {
        config.lod_targets.clone()
    };
    let lod_mode = !config.lod_targets.is_empty();

    let mut report: Option<Report> = None;
    if let Some(path) = &config.report {
        let mut opened = match Report::create(path) {
            Ok(opened) => opened,
            Err(err) => {
                eprintln!(
                    "retopo: error: cannot write --report file '{}': {}.",
                    path.display(),
                    os_reason(&err)
                );
                return 1;
            }
        };
        write_multi_header(&mut opened, config);
        report = Some(opened);
    }
    if !batch && lod_mode {
        // Fail fast: rung outputs land next to `--output`, whose parent
        // is never created for us.
        if let Err(reason) = probe_output_parent(&config.output) {
            eprintln!(
                "retopo: error: cannot write output '{}': {reason}.",
                config.output.display()
            );
            return 1;
        }
    }

    let mut failed_files: Vec<String> = Vec::new();

    for input_path in inputs {
        let file_label = if batch {
            let name = file_name_of(Path::new(input_path));
            if lod_mode {
                format!("FILE {name} ")
            } else {
                format!("FILE {name}: ")
            }
        } else {
            String::new()
        };
        let fail_name = if batch {
            file_name_of(Path::new(input_path))
        } else {
            input_path.clone()
        };

        let loaded = match load_mesh(Path::new(input_path)) {
            Ok(loaded) => loaded,
            Err(failure) => {
                report_load_failure(
                    report.as_mut(),
                    input_path,
                    &failure,
                    &file_label,
                    &mut failed_files,
                    &fail_name,
                );
                continue;
            }
        };
        report_loaded(&loaded, config.quiet);
        warn_dropped_non_finite(loaded.weld_stats.non_finite_dropped);

        for (rung, target) in targets.iter().enumerate() {
            let output_path = if batch {
                let in_file = Path::new(input_path);
                let raw_name = in_file
                    .file_name()
                    .and_then(|n| n.to_str())
                    .unwrap_or(input_path);
                let (stem, lod_ext) = cpp_stem_ext(raw_name);
                if lod_mode {
                    config.output.join(format!("{stem}_lod{rung}{lod_ext}"))
                } else {
                    config.output.join(raw_name)
                }
            } else {
                lod_output_path(&config.output, rung)
            };
            let mut label = file_label.clone();
            if lod_mode {
                label.push_str(&format!("LOD {rung}: "));
            }

            let result = remesh_loaded_mesh(
                config,
                &loaded.vertices,
                &loaded.triangles,
                constraints,
                *target,
                &output_path,
            );
            if !result.ok {
                eprintln!("retopo: error: {fail_name}: {}.", result.error);
                println!("{label}FAILED {}", result.error);
                if let Some(report) = report.as_mut() {
                    report.line(&format!("Input file: {input_path}"));
                    report.line(&format!("Output file: {}", output_path.display()));
                    report.line(&format!("Target quads: {target}"));
                    report.line(&format!("Failed: {}", result.error));
                    report.blank();
                }
                push_failed(&mut failed_files, &fail_name);
                continue;
            }
            if result.failed_islands > 0 {
                if batch {
                    let name = file_name_of(Path::new(input_path));
                    eprintln!(
                        "retopo: warning: FILE {name}: {} of {} islands produced no output and {} dropped from the mesh.",
                        result.failed_islands,
                        result.island_count,
                        island_verb(result.failed_islands)
                    );
                } else {
                    eprintln!(
                        "retopo: warning: {} of {} islands produced no output and {} dropped from the mesh.",
                        result.failed_islands,
                        result.island_count,
                        island_verb(result.failed_islands)
                    );
                }
            }
            if !result.coverage_reports.is_empty() {
                let name = batch.then(|| file_name_of(Path::new(input_path)));
                print_coverage_warnings(&result.coverage_reports, name.as_deref());
            }
            print_rung_line(
                &label,
                &output_path,
                *target,
                result.quad_count,
                result.non_quad_count,
                result.vertex_count,
                result.elapsed_seconds,
            );
            if let Some(report) = report.as_mut() {
                report.line(&format!("Input file: {input_path}"));
                report.line(&format!("Output file: {}", output_path.display()));
                report.line(&format!("Target quads: {target}"));
                report.line("Results:");
                report.line(&format!("  Islands: {}", result.island_count));
                report.line(&format!("  Failed islands: {}", result.failed_islands));
                report.line(&format!("  Quads: {}", result.quad_count));
                report.line(&format!("  Non-quads: {}", result.non_quad_count));
                report.line(&format!("  Vertices: {}", result.vertex_count));
                report.line(&format!(
                    "  Total time: {} seconds",
                    g_format(result.elapsed_seconds)
                ));
                report.blank();
            }
        }
    }

    if let (Some(path), Some(report)) = (&config.report, report.take()) {
        if !report.finish() {
            eprintln!("retopo: error: failed to write {}.", path.display());
            return 1;
        }
    }

    if batch {
        if failed_files.is_empty() {
            println!("Failed files: none");
        } else {
            print!("Failed files ({}):", failed_files.len());
            for name in &failed_files {
                print!(" {name}");
            }
            println!();
        }
    }
    if failed_files.is_empty() { 0 } else { 1 }
}

pub(crate) fn run_single_mode(config: &Config) -> i32 {
    let start_time = Instant::now();

    // Fail fast (multi-mode parity): constraint files before the mesh
    // loads, input before output/report writability, all before the
    // first remesh millisecond.
    let mut constraints = Constraints::default();
    if let Some(path) = &config.guides {
        if let Err(error) = parse_guides_file(path, &mut constraints.guides, "--guides") {
            error.emit();
            return 2;
        }
        if !config.quiet {
            eprintln!("Guide polylines: {}", constraints.guides.len());
        }
    }
    if let Some(path) = &config.features {
        if let Err(error) = parse_guides_file(path, &mut constraints.features, "--features") {
            error.emit();
            return 2;
        }
        if !config.quiet {
            eprintln!("Feature polylines: {}", constraints.features.len());
        }
    }
    if let Some(path) = &config.density {
        if let Err(error) = parse_density_file(path, &mut constraints.density) {
            error.emit();
            return 2;
        }
    }

    let loaded: LoadedMesh = match load_mesh(&config.input) {
        Ok(loaded) => loaded,
        Err(failure) => {
            eprintln!(
                "retopo: error: cannot {} input '{}': {}.",
                failure.verb(),
                config.input.display(),
                failure.reason()
            );
            return 1;
        }
    };
    report_loaded(&loaded, config.quiet);
    warn_dropped_non_finite(loaded.weld_stats.non_finite_dropped);

    // The density count needs the mesh to judge (exit 1, still before
    // remeshing); unreadable/malformed masks already failed above.
    if !constraints.density.is_empty() {
        if let Err(message) = check_density_len(&constraints.density, loaded.vertices.len()) {
            eprintln!("retopo: error: {message}.");
            return 1;
        }
        if !config.quiet {
            eprintln!("Density multipliers: {}", constraints.density.len());
        }
    }

    let mut report: Option<Report> = None;
    if let Some(path) = &config.report {
        match Report::create(path) {
            Ok(opened) => report = Some(opened),
            Err(err) => {
                eprintln!(
                    "retopo: error: cannot write --report file '{}': {}.",
                    path.display(),
                    os_reason(&err)
                );
                return 1;
            }
        }
    }
    if let Err(reason) = probe_output_parent(&config.output) {
        eprintln!(
            "retopo: error: cannot write output '{}': {reason}.",
            config.output.display()
        );
        return 1;
    }

    let mut remesher = AutoRemesher::new(&loaded.vertices, &loaded.triangles);
    configure_remesher(
        &mut remesher,
        config,
        constraints.guides,
        constraints.features,
        &constraints.density,
        (config.target_quads as usize).wrapping_mul(2),
    );
    let progress_state = Mutex::new(ProgressState::default());
    if !config.quiet {
        attach_progress(&mut remesher, &progress_state);
    }

    if !remesher.remesh() {
        eprintln!("retopo: error: remeshing produced no result.");
        return 1;
    }

    if !config.quiet {
        for line in remesher.phase_report() {
            eprintln!("  {line}");
        }
    }

    let remeshed_vertices = remesher.remeshed_vertices();
    let remeshed_quads = remesher.remeshed_quads();

    let mut input_islands: Vec<Vec<Vec<usize>>> = Vec::new();
    MeshSeparator::split_to_islands(&loaded.triangles, &mut input_islands);
    let failed_islands = dropped_island_count(
        remesher.island_output_quad_counts(),
        &input_islands,
        &loaded.vertices,
        remeshed_vertices,
    );
    if failed_islands > 0 {
        eprintln!(
            "retopo: warning: {failed_islands} of {} islands produced no output and {} dropped from the mesh.",
            input_islands.len(),
            island_verb(failed_islands)
        );
    }
    print_coverage_warnings(remesher.coverage_reports(), None);

    let mut quad_count = 0usize;
    let mut non_quad_count = 0usize;
    for face in remeshed_quads {
        if face.len() == 4 {
            quad_count += 1;
        } else {
            non_quad_count += 1;
        }
    }

    // The save below fails on empty verts: name the real cause (a
    // collapsed remesh), never the disk.
    if remeshed_vertices.is_empty() {
        let noun = if input_islands.len() == 1 {
            "island"
        } else {
            "islands"
        };
        eprintln!(
            "retopo: error: remeshing produced an empty mesh (all {} {noun} dropped); nothing written to '{}'. Try a larger --target-quads.",
            input_islands.len(),
            config.output.display()
        );
        return 1;
    }

    let uvs = if config.emit_uvs {
        Some(remesher.remeshed_vertex_uvs())
    } else {
        None
    };
    if !save_mesh(&config.output, remeshed_vertices, remeshed_quads, uvs) {
        eprintln!(
            "retopo: error: failed to write {}.",
            config.output.display()
        );
        return 1;
    }

    let elapsed_seconds = start_time.elapsed().as_secs_f64();

    println!("=== retopoforge Report ===");
    println!("Input: {}", config.input.display());
    println!("Output: {}", config.output.display());
    println!("Islands: {}", input_islands.len());
    println!("Failed islands: {failed_islands}");
    println!("Quads: {quad_count}");
    println!("Non-quads: {non_quad_count}");
    println!("Vertices: {}", remeshed_vertices.len());
    println!("Time: {} seconds", g_format(elapsed_seconds));
    println!("==========================");

    if let Some(path) = &config.report {
        let mut report = report.take().expect("created before remeshing");
        report.line("retopoforge Report");
        report.line("==================");
        report.blank();
        report.line(&format!("Input file: {}", config.input.display()));
        report.line(&format!("Output file: {}", config.output.display()));
        report.line(&format!("Target quads: {}", config.target_quads));
        report.line(&format!("Edge scaling: {}", g_format(config.edge_scaling)));
        report.line(&format!(
            "Sharp edge degrees: {}",
            g_format(config.sharp_edge_degrees)
        ));
        report.line(&format!(
            "Smooth normal degrees: {}",
            g_format(config.smooth_normal_degrees)
        ));
        report.line(&format!("Adaptivity: {}", g_format(config.adaptivity)));
        report.line(&format!("Anisotropy: {}", g_format(config.anisotropy)));
        report.line(&format!(
            "Model type: {}",
            model_type_name(config.model_type)
        ));
        report.blank();
        report.line("Results:");
        report.line(&format!("  Islands: {}", input_islands.len()));
        report.line(&format!("  Failed islands: {failed_islands}"));
        report.line(&format!("  Quads: {quad_count}"));
        report.line(&format!("  Non-quads: {non_quad_count}"));
        report.line(&format!("  Vertices: {}", remeshed_vertices.len()));
        report.line(&format!(
            "  Total time: {} seconds",
            g_format(elapsed_seconds)
        ));
        if !report.finish() {
            eprintln!("retopo: error: failed to write {}.", path.display());
            return 1;
        }
    }

    0
}

#[cfg(test)]
mod island_accounting_tests {
    use super::*;

    fn tri(a: usize, b: usize, c: usize) -> Vec<usize> {
        vec![a, b, c]
    }

    #[test]
    fn engine_counts_zero_means_failed() {
        // Every 0-output island counts as failed, whatever stage ate it
        // (empty resample, cover failure, or starved extraction).
        let islands = vec![vec![tri(0, 1, 2)], vec![tri(3, 4, 5)], vec![tri(6, 7, 8)]];
        let verts = vec![Vector3::new(0.0, 0.0, 0.0); 9];
        assert_eq!(
            dropped_island_count(&[880, 112, 0], &islands, &verts, &verts),
            1
        );
        assert_eq!(
            dropped_island_count(&[5, 7, 9], &islands, &verts, &verts),
            0
        );
        assert_eq!(
            dropped_island_count(&[0, 0, 0], &islands, &verts, &verts),
            3
        );
    }

    #[test]
    fn bbox_fallback_catches_dropped_islands() {
        // Length mismatch (defensive only: engine counts align 1:1 with
        // input islands) falls back to the bbox heuristic.
        let islands = vec![vec![tri(0, 1, 2)], vec![tri(3, 4, 5)]];
        let input = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(50.0, 50.0, 50.0),
            Vector3::new(51.0, 50.0, 50.0),
            Vector3::new(50.0, 51.0, 50.0),
        ];
        let near_first = vec![Vector3::new(0.5, 0.5, 0.0)];
        assert_eq!(
            dropped_island_count(&[1], &islands, &input, &near_first),
            1,
            "far island with no nearby output counts as dropped"
        );
        let near_both = vec![Vector3::new(0.5, 0.5, 0.0), Vector3::new(50.5, 50.5, 50.0)];
        assert_eq!(dropped_island_count(&[1], &islands, &input, &near_both), 0);
    }
}

#[cfg(test)]
mod batch_guard_tests {
    use super::*;

    #[test]
    fn file_looking_outputs_spotted() {
        assert!(output_looks_like_file(Path::new("somefile.obj")));
        assert!(output_looks_like_file(Path::new("UP.GLB")));
        assert!(output_looks_like_file(Path::new("a/b/c.glb")));
        assert!(!output_looks_like_file(Path::new("remeshed")));
        assert!(!output_looks_like_file(Path::new("out.d")));
        assert!(!output_looks_like_file(Path::new("dir.objects/x")));
    }

    #[test]
    fn output_probe_catches_missing_parents() {
        assert!(probe_output_parent(Path::new("bare.obj")).is_ok());
        assert!(probe_output_parent(Path::new("/tmp")).is_ok());
        let missing = probe_output_parent(Path::new("/tmp/ux-probe-does-not-exist-ux/x.obj"));
        assert_eq!(missing, Err("no such file or directory".to_string()));
    }
}
