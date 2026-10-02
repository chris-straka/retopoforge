// Permanent regression: beast@1000-native dropped the whole left
// appendage (x < -62, 416 working tris -> 0 quads) when a region-scale
// uv fold traced the island onto the body (docs/beast-knife-edge-
// bisection.md). The coverage retry must fire and recover it.
//
// Needs the owner's gitignored corpus (`bench/fetch_models.sh`); skips
// loudly when beast.obj is absent. Single test per binary: setting
// `RETOPO_DECIMATOR` here cannot race sibling tests (separate process
// per integration binary).
use retopo_core::auto_remesher::{AutoRemesher, ModelType};
use retopo_core::obj_reader;
use retopo_core::quad_parameterizer::DipoleConfig;
use retopo_core::vector3::Vector3;
use std::path::Path;

const BEAST: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../bench/models/beast.obj");

/// Mirrors the CLI load path (`mesh_io::load_with` for .obj): parse,
/// weld, f32 -> f64.
fn load_cli_like(filename: &Path) -> Option<(Vec<Vector3>, Vec<Vec<usize>>)> {
    let mut positions: Vec<f32> = Vec::new();
    let mut triangles: Vec<Vec<usize>> = Vec::new();
    let ok = obj_reader::load_obj_positions_and_triangles(
        filename,
        &mut positions,
        &mut triangles,
        None,
        None,
    );
    if !ok {
        return None;
    }
    obj_reader::weld_positions_and_triangles(&mut positions, &mut triangles, None);
    let mut vertices = Vec::with_capacity(positions.len() / 3);
    for i in 0..positions.len() / 3 {
        vertices.push(Vector3::new(
            positions[3 * i] as f64,
            positions[3 * i + 1] as f64,
            positions[3 * i + 2] as f64,
        ));
    }
    Some((vertices, triangles))
}

/// Mirrors the CLI bench invocation (defaults + `--target-quads 1000`).
fn remesh_like_cli(vertices: &[Vector3], triangles: &[Vec<usize>]) -> AutoRemesher {
    let mut remesher = AutoRemesher::new(vertices, triangles);
    remesher.set_target_triangle_count(2 * 1000);
    remesher.set_symmetry_enabled(false);
    remesher.set_scaling(1.0);
    remesher.set_model_type(ModelType::Organic);
    remesher.set_gradient_adaptivity(1.0);
    remesher.set_anisotropy(1.0);
    remesher.set_sharp_edge_degrees(90.0);
    remesher.set_smooth_normal_degrees(0.0);
    remesher.set_dipoles(DipoleConfig::automatic());
    remesher.set_quiet(true);
    remesher
}

#[test]
fn beast_native_1000_recovers_appendage() {
    if !Path::new(BEAST).exists() {
        println!("SKIP: {BEAST} absent (bench/fetch_models.sh); regression needs the corpus");
        return;
    }
    // SAFETY: this binary holds a single test, so no sibling thread
    // can observe the environment mid-mutation (and the engine reads
    // this variable only during `remesh` below).
    unsafe {
        std::env::set_var("RETOPO_DECIMATOR", "native");
    }
    let (vertices, triangles) = load_cli_like(Path::new(BEAST)).expect("beast.obj loads");
    let mut remesher = remesh_like_cli(&vertices, &triangles);
    assert!(remesher.remesh(), "remesh succeeds");

    // The retry must have fired exactly once and recovered.
    let reports = remesher.coverage_reports();
    assert_eq!(
        reports.len(),
        1,
        "one island fires the coverage retry, got {} reports",
        reports.len()
    );
    let r = reports[0];
    assert_eq!(r.island_index, 0);
    assert!(
        r.recovered,
        "retry recovers the appendage (initially {} verts uncovered)",
        r.initial_uncovered
    );
    assert!(
        r.initial_uncovered >= 25,
        "attempt 0 genuinely failed ({} verts uncovered)",
        r.initial_uncovered
    );

    // Ground truth: output reaches into the appendage (x < -62 held 416
    // working tris emitting 0 quads pre-retry; recovered mesh spans to
    // about -126).
    let min_x = remesher
        .remeshed_vertices()
        .iter()
        .map(|v| v.x())
        .fold(f64::INFINITY, f64::min);
    assert!(
        min_x < -100.0,
        "output covers the appendage (min_x = {min_x:.1})"
    );
}
