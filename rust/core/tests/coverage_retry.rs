// Permanent regression: beast@1000 dropped the whole left appendage
// (x < -62, 416 working tris -> 0 quads) when a region-scale uv fold
// traced the island onto the body (docs/beast-knife-edge-bisection.md).
// The coverage retry must fire and recover it.
//
// Needs the owner's gitignored corpus (`bench/fetch_models.sh`); skips
// loudly when beast.obj is absent.
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

/// Mirrors the CLI bench invocation (defaults + `--target-quads`).
fn remesh_like_cli(
    vertices: &[Vector3],
    triangles: &[Vec<usize>],
    target_quads: usize,
) -> AutoRemesher {
    let mut remesher = AutoRemesher::new(vertices, triangles);
    remesher.set_target_triangle_count(2 * target_quads);
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
    // Native is the only decimator since the item-6 flip, so this
    // exercises the default path with no env switch.
    let (vertices, triangles) = load_cli_like(Path::new(BEAST)).expect("beast.obj loads");
    let mut remesher = remesh_like_cli(&vertices, &triangles, 1000);
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

    // Retry jitter steers only the parameterization: the recovered
    // output embeds in the UNJITTERED working mesh (single island, so
    // the merged post-isotropic-remesh stage mesh is the island's;
    // `decimated_*` is a stale pre-isotropic snapshot), not the
    // jittered retry input. Median output-to-working distance is
    // floating-point noise (crossings lerp working edges); jittered
    // embedding would sit near +-0.1 (1e-3 of the 387-unit diagonal,
    // halved by the lerp). Median, not max: a few repair midpoints
    // legitimately leave the surface (0.1 on the unretried path too).
    assert_eq!(remesher.island_output_quad_counts().len(), 1);
    let working_v = remesher.isotropic_vertices();
    let working_t = remesher.isotropic_triangles();
    assert!(!working_v.is_empty() && !working_t.is_empty());
    let mut distances: Vec<f64> = remesher
        .remeshed_vertices()
        .iter()
        .map(|p| {
            working_t
                .iter()
                .map(|t| {
                    point_triangle_dist(p, &working_v[t[0]], &working_v[t[1]], &working_v[t[2]])
                })
                .fold(f64::INFINITY, f64::min)
        })
        .collect();
    distances.sort_by(|a, b| a.total_cmp(b));
    let median = distances[distances.len() / 2];
    assert!(
        median < 1e-6,
        "recovered output embeds in the unjittered working mesh (median output-to-working distance = {median:.2e})"
    );
}

/// Thin-claw input-side fixture: 3x3x3 body with a dense 0.25-wide,
/// 6-tall claw (16 around x 24 rings + tip cap). At 50 target quads
/// decimation thins the claw past what the working-side check can see
/// (few working verts, all inside the bar), while 49 connected input
/// verts strand beyond it — the input side fires (and only it) and a
/// jitter re-tiling re-passes the bar. Recovery here is bar-relative,
/// not geometric: the 0.25-wide claw sits below what 50 quads can
/// resolve, so the retry wins by re-tiling inside a coarser bar rather
/// than by covering the tip (no max-z ground truth is asserted — the
/// bar verdict is the contract). No corpus needed (procedural).
fn thin_claw_obj() -> String {
    let mut out = String::new();
    let mut v = |x: f64, y: f64, z: f64| {
        out.push_str(&format!("v {x} {y} {z}\n"));
    };
    for (z, h) in [(-1.5, 1.5), (1.5, 1.5)] {
        for (sx, sy) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
            v(sx * h, sy * h, z);
        }
    }
    let sides = 16usize;
    let rings = 24usize;
    let base = 8usize;
    for r in 0..=rings {
        let z = 1.5 + 6.0 * r as f64 / rings as f64;
        for s in 0..sides {
            let a = 2.0 * std::f64::consts::PI * s as f64 / sides as f64;
            v(0.125 * a.cos(), 0.125 * a.sin(), z);
        }
    }
    let tip = base + (rings + 1) * sides;
    v(0.0, 0.0, 7.5);
    let mut f = |a: usize, b: usize, c: usize| {
        out.push_str(&format!("f {} {} {}\n", a + 1, b + 1, c + 1));
    };
    f(0, 2, 1);
    f(0, 3, 2);
    for (a0, b0) in [(0, 1), (1, 2), (2, 3), (3, 0)] {
        f(a0, 4 + b0, 4 + a0);
        f(a0, b0, 4 + b0);
        f(4 + a0, 4 + b0, base + b0);
        f(4 + a0, base + b0, base + a0);
    }
    for r in 0..rings {
        for s in 0..sides {
            let a0 = base + r * sides + s;
            let a1 = base + r * sides + (s + 1) % sides;
            let b0 = base + (r + 1) * sides + s;
            let b1 = base + (r + 1) * sides + (s + 1) % sides;
            f(a0, b1, b0);
            f(a0, a1, b1);
        }
    }
    for s in 0..sides {
        f(
            base + rings * sides + s,
            base + rings * sides + (s + 1) % sides,
            tip,
        );
    }
    out
}

#[test]
fn thin_claw_fires_input_side_and_recovers() {
    let path = std::env::temp_dir().join(format!(
        "retopo_input_claw_{}_{}.obj",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0),
    ));
    std::fs::write(&path, thin_claw_obj()).expect("claw obj writes");
    let loaded = load_cli_like(&path);
    let _ = std::fs::remove_file(&path);
    let (vertices, triangles) = loaded.expect("claw obj loads");
    let mut remesher = remesh_like_cli(&vertices, &triangles, 50);
    assert!(remesher.remesh(), "remesh succeeds");
    let reports = remesher.coverage_reports();
    assert_eq!(
        reports.len(),
        1,
        "one island fires the coverage retry, got {} reports",
        reports.len()
    );
    let r = reports[0];
    assert!(
        r.input_side,
        "the input side (not working-side) must fire: {r:?}"
    );
    assert!(
        r.recovered,
        "retry recovers the claw (initially {} input verts uncovered)",
        r.initial_uncovered
    );
    assert_eq!(r.retries_made, 2, "recovery lands on retry 2: {r:?}");
    assert_eq!(r.initial_uncovered, 49, "attempt 0 strands 49: {r:?}");
    assert_eq!(r.final_uncovered, 0, "exact-zero residuals: {r:?}");
    // The attempt-0 drop is genuine (working-side quiet is implied by
    // `input_side`, which is set only when working-side passed): the
    // bar verdict is the contract, so no geometric ground truth beyond
    // the report pins (see the fixture doc on bar-relative recovery).
    assert!(!remesher.remeshed_quads().is_empty());
}

/// Distance from point `p` to triangle `(a, b, c)` (Ericson 5.1.5,
/// f64). Test-local copy (the engine's is private); standard
/// algorithm, pinned by known values below.
fn point_triangle_dist(p: &Vector3, a: &Vector3, b: &Vector3, c: &Vector3) -> f64 {
    let ab = (b.x() - a.x(), b.y() - a.y(), b.z() - a.z());
    let ac = (c.x() - a.x(), c.y() - a.y(), c.z() - a.z());
    let ap = (p.x() - a.x(), p.y() - a.y(), p.z() - a.z());
    let d1 = ab.0 * ap.0 + ab.1 * ap.1 + ab.2 * ap.2;
    let d2 = ac.0 * ap.0 + ac.1 * ap.1 + ac.2 * ap.2;
    if d1 <= 0.0 && d2 <= 0.0 {
        return (ap.0 * ap.0 + ap.1 * ap.1 + ap.2 * ap.2).sqrt();
    }
    let bp = (p.x() - b.x(), p.y() - b.y(), p.z() - b.z());
    let d3 = ab.0 * bp.0 + ab.1 * bp.1 + ab.2 * bp.2;
    let d4 = ac.0 * bp.0 + ac.1 * bp.1 + ac.2 * bp.2;
    if d3 >= 0.0 && d4 <= d3 {
        return (bp.0 * bp.0 + bp.1 * bp.1 + bp.2 * bp.2).sqrt();
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        let q = (
            a.x() + v * ab.0 - p.x(),
            a.y() + v * ab.1 - p.y(),
            a.z() + v * ab.2 - p.z(),
        );
        return (q.0 * q.0 + q.1 * q.1 + q.2 * q.2).sqrt();
    }
    let cp = (p.x() - c.x(), p.y() - c.y(), p.z() - c.z());
    let d5 = ab.0 * cp.0 + ab.1 * cp.1 + ab.2 * cp.2;
    let d6 = ac.0 * cp.0 + ac.1 * cp.1 + ac.2 * cp.2;
    if d6 >= 0.0 && d5 <= d6 {
        return (cp.0 * cp.0 + cp.1 * cp.1 + cp.2 * cp.2).sqrt();
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        let q = (
            a.x() + w * ac.0 - p.x(),
            a.y() + w * ac.1 - p.y(),
            a.z() + w * ac.2 - p.z(),
        );
        return (q.0 * q.0 + q.1 * q.1 + q.2 * q.2).sqrt();
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let q = (
            b.x() + w * (c.x() - b.x()) - p.x(),
            b.y() + w * (c.y() - b.y()) - p.y(),
            b.z() + w * (c.z() - b.z()) - p.z(),
        );
        return (q.0 * q.0 + q.1 * q.1 + q.2 * q.2).sqrt();
    }
    let n = (
        ab.1 * ac.2 - ab.2 * ac.1,
        ab.2 * ac.0 - ab.0 * ac.2,
        ab.0 * ac.1 - ab.1 * ac.0,
    );
    let n2 = n.0 * n.0 + n.1 * n.1 + n.2 * n.2;
    if n2 == 0.0 {
        return (ap.0 * ap.0 + ap.1 * ap.1 + ap.2 * ap.2).sqrt();
    }
    ((ap.0 * n.0 + ap.1 * n.1 + ap.2 * n.2).abs() / n2.sqrt()).min(
        (ap.0 * ap.0 + ap.1 * ap.1 + ap.2 * ap.2)
            .sqrt()
            .min((bp.0 * bp.0 + bp.1 * bp.1 + bp.2 * bp.2).sqrt())
            .min((cp.0 * cp.0 + cp.1 * cp.1 + cp.2 * cp.2).sqrt()),
    )
}

#[test]
fn point_triangle_dist_agrees_with_known_values() {
    let a = Vector3::new(0.0, 0.0, 0.0);
    let b = Vector3::new(1.0, 0.0, 0.0);
    let c = Vector3::new(0.0, 1.0, 0.0);
    // On-face interior, edge, vertex: zero.
    assert!(point_triangle_dist(&Vector3::new(0.2, 0.2, 0.0), &a, &b, &c) < 1e-12);
    assert!(point_triangle_dist(&Vector3::new(0.5, 0.0, 0.0), &a, &b, &c) < 1e-12);
    assert!(point_triangle_dist(&a, &a, &b, &c) < 1e-12);
    // Off-face: exact perpendicular / vertex distances.
    assert!((point_triangle_dist(&Vector3::new(0.2, 0.2, 1.0), &a, &b, &c) - 1.0).abs() < 1e-12);
    assert!((point_triangle_dist(&Vector3::new(2.0, 0.0, 0.0), &a, &b, &c) - 1.0).abs() < 1e-12);
}
