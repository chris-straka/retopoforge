//! Run modes: single-file, `--lods`, and batch-directory.
//!
//! A single input without `--lods` takes the single path (stdout report
//! block plus an optional `--report` file). `--lods` and batch mode
//! share the multi path (one stdout line per rung, optional `--report`
//! file, `Failed files:` summary in batch). Exit 0 when every file
//! produced output, 1 otherwise. All text and exit codes are pinned by
//! the CLI contract goldens.

use crate::args::{Backend, Config};
use crate::constraint_files::{Constraints, parse_density_file, parse_guides_file};
use crate::format::g_format;
use crate::mesh_io::{
    LoadedMesh, cpp_stem_ext, file_name_of, load_mesh, lod_output_path, report_loaded, save_mesh,
    warn_dropped_non_finite,
};
use crate::progress::{ProgressState, attach_progress};
use crate::report::{Report, print_rung_line};
use retopo_core::auto_remesher::{AutoRemesher, CoverageReport, ModelType};
use retopo_core::glb as glb_io;
use retopo_core::mesh_separator::MeshSeparator;
use retopo_core::patch_backend::PatchRemesher;
use retopo_core::vector2::Vector2;
use retopo_core::vector3::Vector3;
use std::ffi::c_void;
use std::io::Write;
use std::path::Path;
use std::sync::Mutex;
use std::time::Instant;

/// The output surface both back ends share, so the single and multi
/// paths run one reporting body however the mesh was produced.
trait Engine {
    fn remesh(&mut self) -> bool;
    fn phase_report(&self) -> &[String];
    fn remeshed_vertices(&self) -> &[Vector3];
    fn remeshed_quads(&self) -> &[Vec<usize>];
    fn remeshed_vertex_uvs(&self) -> &[Vector2];
    fn island_output_quad_counts(&self) -> &[usize];
    fn coverage_reports(&self) -> &[CoverageReport];
}

impl Engine for AutoRemesher {
    fn remesh(&mut self) -> bool {
        self.remesh()
    }
    fn phase_report(&self) -> &[String] {
        self.phase_report()
    }
    fn remeshed_vertices(&self) -> &[Vector3] {
        self.remeshed_vertices()
    }
    fn remeshed_quads(&self) -> &[Vec<usize>] {
        self.remeshed_quads()
    }
    fn remeshed_vertex_uvs(&self) -> &[Vector2] {
        self.remeshed_vertex_uvs()
    }
    fn island_output_quad_counts(&self) -> &[usize] {
        self.island_output_quad_counts()
    }
    fn coverage_reports(&self) -> &[CoverageReport] {
        self.coverage_reports()
    }
}

impl Engine for PatchRemesher {
    fn remesh(&mut self) -> bool {
        self.remesh()
    }
    fn phase_report(&self) -> &[String] {
        self.phase_report()
    }
    fn remeshed_vertices(&self) -> &[Vector3] {
        self.remeshed_vertices()
    }
    fn remeshed_quads(&self) -> &[Vec<usize>] {
        self.remeshed_quads()
    }
    fn remeshed_vertex_uvs(&self) -> &[Vector2] {
        self.remeshed_vertex_uvs()
    }
    fn island_output_quad_counts(&self) -> &[usize] {
        self.island_output_quad_counts()
    }
    fn coverage_reports(&self) -> &[CoverageReport] {
        self.coverage_reports()
    }
}

/// Progress state for patch runs (same `N% done. <status>` shape as
/// the default path's reporter).
#[derive(Default)]
struct PatchProgressState {
    last_percent: i32,
    last_status: String,
}

fn report_patch_progress(tag: *mut c_void, progress: f32, status: &str) {
    let state = unsafe { &*(tag as *const Mutex<PatchProgressState>) };
    let mut state = state.lock().unwrap();
    let percent = (progress * 100.0) as i32;
    if percent == state.last_percent && status == state.last_status {
        return;
    }
    state.last_percent = percent;
    state.last_status = status.to_string();
    let mut out = std::io::stdout().lock();
    if status.is_empty() {
        let _ = writeln!(out, "{percent}% done.");
    } else {
        let _ = writeln!(out, "{percent}% done. {status}");
    }
    let _ = out.flush();
}

/// Attach progress reporting to a patch remesher. The caller must hold
/// `state` in a local that outlives the `remesh()` call without moving.
fn attach_patch_progress(remesher: &mut PatchRemesher, state: &Mutex<PatchProgressState>) {
    // SAFETY: same contract as the default path's attach — the engine
    // only dereferences the tag while `remesh()` runs, synchronously,
    // and the caller's local outlives that call.
    remesher.set_tag(state as *const Mutex<PatchProgressState> as *mut c_void);
    remesher.set_progress_handler(Some(report_patch_progress));
}

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
                "Warning: {prefix}island {} failed the coverage check ({} {side} uncovered) and recovered on retry {} ({} verts uncovered)",
                r.island_index, r.initial_uncovered, r.retries_made, r.final_uncovered
            );
        } else if r.kept_attempt == 0 {
            eprintln!(
                "Warning: {prefix}island {} failed the coverage check ({} {side} uncovered); retries did not recover, kept the original",
                r.island_index, r.initial_uncovered
            );
        } else {
            eprintln!(
                "Warning: {prefix}island {} failed the coverage check ({} {side} uncovered); retries did not recover, kept attempt {} (working-side cover; input side still short)",
                r.island_index, r.initial_uncovered, r.kept_attempt
            );
        }
    }
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
/// mask is fine). Returns the shared mismatch message.
fn check_density_len(density: &[f64], vertices: usize) -> Result<(), String> {
    if !density.is_empty() && density.len() != vertices {
        return Err(format!(
            "--density file holds {} multipliers, input has {} vertices",
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

/// Patch twin of [`configure_remesher`]: same settings, no dipoles
/// (the patch back end has no dipole pass yet).
fn configure_patch_remesher(
    remesher: &mut PatchRemesher,
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

    // The back ends construct (and attach progress) differently but
    // report identically through `Engine`.
    match config.backend {
        Backend::Default => {
            let mut engine = AutoRemesher::new(vertices, triangles);
            configure_remesher(
                &mut engine,
                config,
                constraints.guides.clone(),
                constraints.features.clone(),
                &constraints.density,
                (target_quads as usize).wrapping_mul(2),
            );
            let progress_state = Mutex::new(ProgressState::default());
            if !config.quiet {
                attach_progress(&mut engine, &progress_state);
            }
            finish_rung(
                config,
                vertices,
                triangles,
                &mut engine,
                output_path,
                start_time,
                &mut result,
            );
        }
        Backend::Patch => {
            let mut engine = PatchRemesher::new(vertices, triangles);
            configure_patch_remesher(
                &mut engine,
                config,
                constraints.guides.clone(),
                constraints.features.clone(),
                &constraints.density,
                (target_quads as usize).wrapping_mul(2),
            );
            let progress_state = Mutex::new(PatchProgressState::default());
            if !config.quiet {
                attach_patch_progress(&mut engine, &progress_state);
            }
            finish_rung(
                config,
                vertices,
                triangles,
                &mut engine,
                output_path,
                start_time,
                &mut result,
            );
        }
    }
    result
}

/// Run the engine, count and save its output: the shared tail of
/// [`remesh_loaded_mesh`] for both back ends.
fn finish_rung(
    config: &Config,
    vertices: &[Vector3],
    triangles: &[Vec<usize>],
    engine: &mut impl Engine,
    output_path: &Path,
    start_time: Instant,
    result: &mut RungResult,
) {
    if !engine.remesh() {
        result.error = "remeshing produced no result".to_string();
        return;
    }

    if !config.quiet {
        for line in engine.phase_report() {
            eprintln!("  {line}");
        }
    }

    let remeshed_vertices = engine.remeshed_vertices();
    let remeshed_quads = engine.remeshed_quads();

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
        engine.island_output_quad_counts(),
        &input_islands,
        vertices,
        remeshed_vertices,
    );
    result.coverage_reports = engine.coverage_reports().to_vec();

    let uvs = if config.emit_uvs {
        Some(engine.remeshed_vertex_uvs())
    } else {
        None
    };
    if !save_mesh(output_path, remeshed_vertices, remeshed_quads, uvs) {
        result.error = format!("failed to write {}", output_path.display());
        return;
    }

    result.elapsed_seconds = start_time.elapsed().as_secs_f64();
    result.ok = true;
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
/// following symlinks, sorted by name.
fn collect_batch_inputs(config: &Config) -> Result<Vec<String>, ()> {
    let entries = match std::fs::read_dir(&config.input) {
        Ok(entries) => entries,
        Err(_) => {
            eprintln!("Error: cannot read directory {}", config.input.display());
            return Err(());
        }
    };
    let mut inputs: Vec<String> = Vec::new();
    for entry in entries {
        let entry = match entry {
            Ok(entry) => entry,
            Err(_) => {
                eprintln!("Error: cannot read directory {}", config.input.display());
                return Err(());
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
        }
    }
    inputs.sort();
    if inputs.is_empty() {
        eprintln!("Error: no .obj/.glb files in {}", config.input.display());
        return Err(());
    }
    Ok(inputs)
}

/// Load `--guides`/`--features`/`--density`, rejecting them in batch
/// mode (they address one specific mesh).
fn load_constraints(config: &Config, batch: bool) -> Result<Constraints, ()> {
    if batch && config.guides.is_some() {
        eprintln!("Error: --guides needs a single input mesh, not a batch directory");
        return Err(());
    }
    if batch && config.features.is_some() {
        eprintln!("Error: --features needs a single input mesh, not a batch directory");
        return Err(());
    }
    if batch && config.density.is_some() {
        eprintln!("Error: --density needs a single input mesh, not a batch directory");
        return Err(());
    }
    let mut constraints = Constraints::default();
    if let Some(path) = &config.guides
        && let Err(error) = parse_guides_file(path, &mut constraints.guides, "--guides")
    {
        error.emit();
        return Err(());
    }
    if let Some(path) = &config.features
        && let Err(error) = parse_guides_file(path, &mut constraints.features, "--features")
    {
        error.emit();
        return Err(());
    }
    if let Some(path) = &config.density
        && let Err(error) = parse_density_file(path, &mut constraints.density)
    {
        error.emit();
        return Err(());
    }
    Ok(constraints)
}

fn push_failed(failed_files: &mut Vec<String>, name: &str) {
    if !failed_files.iter().any(|n| n == name) {
        failed_files.push(name.to_string());
    }
}

pub(crate) fn run_multi_mode(config: &Config, batch: bool) -> i32 {
    let constraints = match load_constraints(config, batch) {
        Ok(constraints) => constraints,
        Err(()) => return 1,
    };
    let inputs: Vec<String> = if batch {
        match collect_batch_inputs(config) {
            Ok(inputs) => inputs,
            Err(()) => return 1,
        }
    } else {
        vec![config.input.to_string_lossy().into_owned()]
    };
    if batch {
        if let Ok(meta) = std::fs::metadata(&config.output) {
            if !meta.is_dir() {
                eprintln!("Error: --output must be a directory when --input is a directory");
                return 1;
            }
        }
        if std::fs::create_dir_all(&config.output).is_err() {
            eprintln!(
                "Error: cannot create output directory {}",
                config.output.display()
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
            Some(opened) => opened,
            None => {
                eprintln!("Error: failed to write {}", path.display());
                return 1;
            }
        };
        write_multi_header(&mut opened, config);
        report = Some(opened);
    }

    let mut failed_files: Vec<String> = Vec::new();

    for input_path in &inputs {
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
            Some(loaded) => loaded,
            None => {
                eprintln!("Error: failed to load {input_path}");
                println!("{file_label}FAILED to load {input_path}");
                push_failed(&mut failed_files, &fail_name);
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
                &constraints,
                *target,
                &output_path,
            );
            if !result.ok {
                eprintln!("Error: {} ({})", result.error, output_path.display());
                println!("{label}FAILED {}", result.error);
                push_failed(&mut failed_files, &fail_name);
                continue;
            }
            if result.failed_islands > 0 {
                if batch {
                    let name = file_name_of(Path::new(input_path));
                    eprintln!(
                        "Warning: FILE {name}: {} of {} islands produced no output and were dropped from the mesh",
                        result.failed_islands, result.island_count
                    );
                } else {
                    eprintln!(
                        "Warning: {} of {} islands produced no output and were dropped from the mesh",
                        result.failed_islands, result.island_count
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
            eprintln!("Error: failed to write {}", path.display());
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

    let loaded: LoadedMesh = match load_mesh(&config.input) {
        Some(loaded) => loaded,
        None => {
            eprintln!("Error: failed to load {}", config.input.display());
            return 1;
        }
    };
    report_loaded(&loaded, config.quiet);
    warn_dropped_non_finite(loaded.weld_stats.non_finite_dropped);

    // Parsed after the mesh loads (so the counts read naturally), one
    // file at a time: each count line precedes the next file's errors.
    let mut constraints = Constraints::default();
    if let Some(path) = &config.guides {
        if let Err(error) = parse_guides_file(path, &mut constraints.guides, "--guides") {
            error.emit();
            return 1;
        }
        if !config.quiet {
            eprintln!("Guide polylines: {}", constraints.guides.len());
        }
    }
    if let Some(path) = &config.features {
        if let Err(error) = parse_guides_file(path, &mut constraints.features, "--features") {
            error.emit();
            return 1;
        }
        if !config.quiet {
            eprintln!("Feature polylines: {}", constraints.features.len());
        }
    }
    if let Some(path) = &config.density {
        if let Err(error) = parse_density_file(path, &mut constraints.density) {
            error.emit();
            return 1;
        }
        if let Err(message) = check_density_len(&constraints.density, loaded.vertices.len()) {
            eprintln!("Error: {message}");
            return 1;
        }
        if !config.quiet {
            eprintln!("Density multipliers: {}", constraints.density.len());
        }
    }

    // The back ends construct (and attach progress) differently but
    // report identically through `Engine`.
    match config.backend {
        Backend::Default => {
            let mut engine = AutoRemesher::new(&loaded.vertices, &loaded.triangles);
            configure_remesher(
                &mut engine,
                config,
                constraints.guides,
                constraints.features,
                &constraints.density,
                (config.target_quads as usize).wrapping_mul(2),
            );
            let progress_state = Mutex::new(ProgressState::default());
            if !config.quiet {
                attach_progress(&mut engine, &progress_state);
            }
            finish_single_mode(config, &loaded, &mut engine, start_time)
        }
        Backend::Patch => {
            let mut engine = PatchRemesher::new(&loaded.vertices, &loaded.triangles);
            configure_patch_remesher(
                &mut engine,
                config,
                constraints.guides,
                constraints.features,
                &constraints.density,
                (config.target_quads as usize).wrapping_mul(2),
            );
            let progress_state = Mutex::new(PatchProgressState::default());
            if !config.quiet {
                attach_patch_progress(&mut engine, &progress_state);
            }
            finish_single_mode(config, &loaded, &mut engine, start_time)
        }
    }
}

/// Run the engine, count and save its output, print the report: the
/// shared tail of [`run_single_mode`] for both back ends.
fn finish_single_mode(
    config: &Config,
    loaded: &LoadedMesh,
    engine: &mut impl Engine,
    start_time: Instant,
) -> i32 {
    if !engine.remesh() {
        eprintln!("Error: remeshing produced no result");
        return 1;
    }

    if !config.quiet {
        for line in engine.phase_report() {
            eprintln!("  {line}");
        }
    }

    let remeshed_vertices = engine.remeshed_vertices();
    let remeshed_quads = engine.remeshed_quads();

    let mut input_islands: Vec<Vec<Vec<usize>>> = Vec::new();
    MeshSeparator::split_to_islands(&loaded.triangles, &mut input_islands);
    let failed_islands = dropped_island_count(
        engine.island_output_quad_counts(),
        &input_islands,
        &loaded.vertices,
        remeshed_vertices,
    );
    if failed_islands > 0 {
        eprintln!(
            "Warning: {failed_islands} of {} islands produced no output and were dropped from the mesh",
            input_islands.len()
        );
    }
    print_coverage_warnings(engine.coverage_reports(), None);

    let mut quad_count = 0usize;
    let mut non_quad_count = 0usize;
    for face in remeshed_quads {
        if face.len() == 4 {
            quad_count += 1;
        } else {
            non_quad_count += 1;
        }
    }

    let uvs = if config.emit_uvs {
        Some(engine.remeshed_vertex_uvs())
    } else {
        None
    };
    if !save_mesh(&config.output, remeshed_vertices, remeshed_quads, uvs) {
        eprintln!("Error: failed to write {}", config.output.display());
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
        let mut report = match Report::create(path) {
            Some(report) => report,
            None => {
                eprintln!("Error: failed to write {}", path.display());
                return 1;
            }
        };
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
            eprintln!("Error: failed to write {}", path.display());
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
