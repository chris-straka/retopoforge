// Replicated goldens: mirrors the C++ engine-level test groups 1:1 at the
// same tolerance.
// - `tests/test_input_validation.cpp` (engine backstop): out-of-range
//   corners and non-triangle faces fail `remesh()` loudly.
// - `tests/test_island_stats.cpp` (engine contract): per-island output
//   accounting on one- and two-box meshes (the CLI-surfacing half needs
//   the built binary and stays C++-side).
// Plus API-contract pins the C++ suite only covers through the CLI:
// defaults, the quiet phase-report shape, the --uvs size contract, and
// the symmetry getters.
use retopo_core::auto_remesher::{AutoRemesher, ModelType};
use retopo_core::mesh_separator::MeshSeparator;
use retopo_core::vector3::Vector3;

fn build_box(offset: f64) -> (Vec<Vector3>, Vec<Vec<usize>>) {
    let vertices = vec![
        Vector3::new(-1.0 + offset, -1.0, -1.0),
        Vector3::new(1.0 + offset, -1.0, -1.0),
        Vector3::new(1.0 + offset, 1.0, -1.0),
        Vector3::new(-1.0 + offset, 1.0, -1.0),
        Vector3::new(-1.0 + offset, -1.0, 1.0),
        Vector3::new(1.0 + offset, -1.0, 1.0),
        Vector3::new(1.0 + offset, 1.0, 1.0),
        Vector3::new(-1.0 + offset, 1.0, 1.0),
    ];
    // Same faces, same order as test_island_stats.cpp.
    let triangles = vec![
        vec![0, 3, 2],
        vec![0, 2, 1],
        vec![4, 5, 6],
        vec![4, 6, 7],
        vec![0, 1, 5],
        vec![0, 5, 4],
        vec![3, 7, 6],
        vec![3, 6, 2],
        vec![0, 4, 7],
        vec![0, 7, 3],
        vec![1, 2, 6],
        vec![1, 6, 5],
    ];
    (vertices, triangles)
}

fn two_boxes() -> (Vec<Vector3>, Vec<Vec<usize>>) {
    let (mut vertices, mut triangles) = build_box(0.0);
    let (bv, bt) = build_box(5.0);
    let base = vertices.len();
    vertices.extend(bv);
    for t in bt {
        triangles.push(t.iter().map(|i| i + base).collect());
    }
    (vertices, triangles)
}

// --- test_input_validation.cpp: engine backstop ---

#[test]
fn rejects_out_of_range_corner() {
    let vertices = vec![Vector3::default(); 2];
    let triangles = vec![vec![0, 1, 5_000_000]];
    let mut remesher = AutoRemesher::new(&vertices, &triangles);
    remesher.set_target_triangle_count(20);
    assert!(!remesher.remesh());
}

#[test]
fn rejects_non_triangle_face() {
    let vertices = vec![Vector3::default(); 4];
    let triangles = vec![vec![0, 1]];
    let mut remesher = AutoRemesher::new(&vertices, &triangles);
    remesher.set_target_triangle_count(20);
    assert!(!remesher.remesh());
}

#[test]
fn rejects_empty_inputs_and_zero_target() {
    // Empty vertices.
    let mut r = AutoRemesher::new(&[], &[vec![0, 1, 2]]);
    r.set_target_triangle_count(20);
    assert!(!r.remesh());
    // Empty triangles.
    let v = vec![Vector3::default(); 3];
    let mut r = AutoRemesher::new(&v, &[]);
    r.set_target_triangle_count(20);
    assert!(!r.remesh());
    // Zero target (would divide by zero in the voxel sizing).
    let (v, t) = build_box(0.0);
    let mut r = AutoRemesher::new(&v, &t);
    assert!(!r.remesh());
}

// --- test_island_stats.cpp: engine contract ---

#[test]
fn two_boxes_two_productive_counters() {
    let (vertices, triangles) = two_boxes();
    let mut islands = Vec::new();
    MeshSeparator::split_to_islands(&triangles, &mut islands);
    assert_eq!(islands.len(), 2);

    let mut remesher = AutoRemesher::new(&vertices, &triangles);
    remesher.set_target_triangle_count(800);
    assert!(remesher.remesh());
    let counts = remesher.island_output_quad_counts();
    assert_eq!(counts.len(), 2);
    assert!(counts[0] > 0 && counts[1] > 0);
    assert_eq!(
        counts.iter().sum::<usize>(),
        remesher.remeshed_quads().len()
    );
}

#[test]
fn one_box_single_counter() {
    let (vertices, triangles) = build_box(0.0);
    let mut remesher = AutoRemesher::new(&vertices, &triangles);
    remesher.set_target_triangle_count(800);
    assert!(remesher.remesh());
    let counts = remesher.island_output_quad_counts();
    assert_eq!(counts.len(), 1);
    assert_eq!(counts[0], remesher.remeshed_quads().len());
}

// --- API contracts ---

#[test]
fn defaults_match_cpp() {
    let (v, t) = build_box(0.0);
    let r = AutoRemesher::new(&v, &t);
    assert!(!r.quiet());
    assert!(!r.decimated());
    assert_eq!(r.symmetry_plane_axis(), -1);
    assert_eq!(r.symmetry_plane_offset(), 0.0);
    assert_eq!(r.symmetry_plane_score(), 0.0);
    assert!(r.remeshed_vertices().is_empty());
    assert!(r.remeshed_quads().is_empty());
    assert!(r.remeshed_vertex_uvs().is_empty());
    assert!(r.phase_report().is_empty());
    assert!(r.island_output_quad_counts().is_empty());
    assert_eq!(
        AutoRemesher::DEFAULT_SHARP_EDGE_DEGREES.to_bits(),
        90.0f64.to_bits()
    );
}

#[test]
fn setters_cover_the_full_cli_surface() {
    // Every setter the CLI touches, in CLI order, then a successful run.
    let (v, t) = build_box(0.0);
    let mut r = AutoRemesher::new(&v, &t);
    r.set_target_triangle_count(800);
    r.set_symmetry_enabled(false);
    r.set_symmetry_plane(-1);
    r.set_guide_polylines(vec![]);
    r.set_sharp_polylines(vec![]);
    r.set_feature_polylines(vec![]);
    r.set_density_multipliers(&[]);
    r.set_scaling(1.5);
    r.set_model_type(ModelType::HardSurface);
    r.set_gradient_adaptivity(0.7);
    r.set_anisotropy(1.2);
    r.set_sharp_edge_degrees(60.0);
    r.set_smooth_normal_degrees(10.0);
    r.set_compute_remeshed_uvs(true);
    r.set_quiet(false);
    assert!(r.remesh());
    assert!(!r.remeshed_quads().is_empty());
    // The --uvs contract: one UV per vertex.
    assert_eq!(r.remeshed_vertex_uvs().len(), r.remeshed_vertices().len());
}

#[test]
fn quiet_run_omits_leaf_stage_lines() {
    let (v, t) = build_box(0.0);
    let mut noisy = AutoRemesher::new(&v, &t);
    noisy.set_target_triangle_count(800);
    assert!(noisy.remesh());
    assert!(
        noisy.phase_report().iter().any(|l| l.starts_with("    ")),
        "noisy run must collect leaf-stage lines"
    );

    let mut quiet = AutoRemesher::new(&v, &t);
    quiet.set_target_triangle_count(800);
    quiet.set_quiet(true);
    assert!(quiet.remesh());
    assert!(
        !quiet.phase_report().iter().any(|l| l.starts_with("    ")),
        "quiet run must omit leaf-stage lines"
    );
    // Quiet never alters geometry: bitwise-identical outputs.
    assert_eq!(
        quiet.remeshed_vertices().len(),
        noisy.remeshed_vertices().len()
    );
    for (q, n) in quiet
        .remeshed_vertices()
        .iter()
        .zip(noisy.remeshed_vertices().iter())
    {
        assert_eq!(q.x().to_bits(), n.x().to_bits());
        assert_eq!(q.y().to_bits(), n.y().to_bits());
        assert_eq!(q.z().to_bits(), n.z().to_bits());
    }
    assert_eq!(quiet.remeshed_quads(), noisy.remeshed_quads());
}

#[test]
fn symmetry_fallback_reports_minus_one() {
    // A single box is mirror-symmetric: fixed-axis detection activates.
    let (v, t) = build_box(0.0);
    let mut r = AutoRemesher::new(&v, &t);
    r.set_target_triangle_count(200);
    r.set_symmetry_enabled(true);
    r.set_symmetry_plane(0);
    assert!(r.remesh());
    assert_eq!(r.symmetry_plane_axis(), 0);
    assert!(r.symmetry_plane_score() >= 0.75);

    // An asymmetric input falls back to unconstrained output (axis -1).
    let asym_t = vec![vec![0, 1, 2], vec![0, 2, 3], vec![4, 5, 6]];
    let asym_v = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.3, 0.2, 0.7),
        Vector3::new(5.0, 5.0, 5.0),
        Vector3::new(5.2, 5.0, 5.0),
        Vector3::new(5.0, 5.4, 5.0),
    ];
    let mut r = AutoRemesher::new(&asym_v, &asym_t);
    r.set_target_triangle_count(50);
    r.set_symmetry_enabled(true);
    assert!(r.remesh());
    assert_eq!(r.symmetry_plane_axis(), -1);
}

#[test]
fn density_wrong_size_disables() {
    let (v, t) = build_box(0.0);
    let mut plain = AutoRemesher::new(&v, &t);
    plain.set_target_triangle_count(800);
    assert!(plain.remesh());

    // Wrong-sized field: bit-identical to no field.
    let mut masked = AutoRemesher::new(&v, &t);
    masked.set_target_triangle_count(800);
    masked.set_density_multipliers(&vec![2.0; v.len() - 1]);
    assert!(masked.remesh());
    assert_eq!(
        masked.remeshed_vertices().len(),
        plain.remeshed_vertices().len()
    );
    for (m, p) in masked
        .remeshed_vertices()
        .iter()
        .zip(plain.remeshed_vertices().iter())
    {
        assert_eq!(m.x().to_bits(), p.x().to_bits());
        assert_eq!(m.y().to_bits(), p.y().to_bits());
        assert_eq!(m.z().to_bits(), p.z().to_bits());
    }
    assert_eq!(masked.remeshed_quads(), plain.remeshed_quads());
}
