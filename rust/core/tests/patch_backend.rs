// Patch-backend first-attempt coverage: beast@1000 must cover the left
// appendage (x < -62) WITHOUT any retry — the patch extractor covers by
// construction (every working face belongs to exactly one patch, every
// patch emits), while the default back end drops it through a global uv
// fold on the native path (docs/beast-knife-edge-bisection.md) and needs
// the coverage retry (docs/coverage-retry.md).
//
// Needs the gitignored bench corpus (`bench/fetch_models.sh`); skips
// loudly when beast.obj is absent. Single test per binary: the second
// half sets `RETOPO_DECIMATOR`, which cannot race sibling tests
// (separate process per integration binary, one test here).
use retopo_core::obj_reader;
use retopo_core::patch_backend::PatchRemesher;
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

/// Mirrors the CLI bench invocation (defaults + `--target-quads 1000`
/// + `--backend patch`).
fn remesh_like_cli(vertices: &[Vector3], triangles: &[Vec<usize>]) -> PatchRemesher {
    let mut remesher = PatchRemesher::new(vertices, triangles);
    remesher.set_target_triangle_count(2 * 1000);
    remesher.set_symmetry_enabled(false);
    remesher.set_scaling(1.0);
    remesher.set_gradient_adaptivity(1.0);
    remesher.set_anisotropy(1.0);
    remesher.set_sharp_edge_degrees(90.0);
    remesher.set_smooth_normal_degrees(0.0);
    remesher.set_quiet(true);
    remesher
}

fn assert_first_attempt_coverage(
    label: &str,
    vertices: &[Vector3],
    triangles: &[Vec<usize>],
) {
    let mut remesher = remesh_like_cli(vertices, triangles);
    assert!(remesher.remesh(), "{label}: remesh succeeds");
    // No retry exists in this back end: coverage must hold on the
    // single attempt, and no coverage report may fire.
    assert!(
        remesher.coverage_reports().is_empty(),
        "{label}: no coverage retry fires"
    );
    // Ground truth: output reaches into the appendage (x < -62 held 416
    // working tris emitting 0 quads in the default backend pre-retry;
    // recovered meshes span to about -126).
    let min_x = remesher
        .remeshed_vertices()
        .iter()
        .map(|v| v.x())
        .fold(f64::INFINITY, f64::min);
    assert!(
        min_x < -100.0,
        "{label}: output covers the appendage on the first attempt (min_x = {min_x:.1})"
    );
    // Sanity: a real quad-dominant mesh came out.
    let mut quads = 0usize;
    let mut non_quads = 0usize;
    for face in remesher.remeshed_quads() {
        if face.len() == 4 {
            quads += 1;
        } else {
            non_quads += 1;
        }
    }
    // Loose sanity bounds only (quality numbers live in the
    // bench/matched.py comparison, not here): the output must be a
    // real quad-dominant mesh, not a collapsed or all-triangle one.
    assert!(
        quads >= 250,
        "{label}: yielded {quads} quads (target 1000)"
    );
    assert!(
        non_quads < quads,
        "{label}: majority-quad output ({non_quads} non-quads vs {quads} quads)"
    );
}

#[test]
fn beast_1000_covers_appendage_first_attempt() {
    if !Path::new(BEAST).exists() {
        println!("SKIP: {BEAST} absent (bench/fetch_models.sh); regression needs the corpus");
        return;
    }
    let (vertices, triangles) = load_cli_like(Path::new(BEAST)).expect("beast.obj loads");
    // Default (FFI meshopt) decimator, ambient environment.
    assert_first_attempt_coverage("default", &vertices, &triangles);
    // Native decimator: the knife-edge path that drops the appendage in
    // the default back end (attempt 0 has 67-156 verts beyond 3 widths).
    // SAFETY: this binary holds a single test, so no sibling thread can
    // observe the environment mid-mutation (and the engine reads this
    // variable only during `remesh` below).
    unsafe {
        std::env::set_var("RETOPO_DECIMATOR", "native");
    }
    assert_first_attempt_coverage("native", &vertices, &triangles);
}
