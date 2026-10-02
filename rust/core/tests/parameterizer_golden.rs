// Replicated goldens for the parameterizer port. No C++ unit test
// covers Parameterizer directly (it is exercised end to end through the
// engine), so these groups freeze small C++-solved cases as literals:
// inputs transcribed bit-for-bit from tests/fixtures/
// parameterizer_diff.txt (noted per group), outputs asserted at
// scale-aware 1e-6 with exact structural facts (ok, singular indices,
// original/taken agreement, progress name sequence, which pins the taken
// branches and the MILS rounding iteration count).
use retopo_core::parameterizer::Parameterizer;
use retopo_core::progress::ProgressHandler;
use retopo_core::symmetry::SymmetryPlane;
use retopo_core::vector2::Vector2;
use retopo_core::vector3::Vector3;
use std::sync::{Arc, Mutex};

const TOL: f64 = 1e-6;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

fn check_uvs(case: &str, taken: &[Vec<Vector2>], expected: &[[[f64; 2]; 3]]) {
    assert_eq!(taken.len(), expected.len(), "{case}: face count");
    for (i, (got_tri, exp_tri)) in taken.iter().zip(expected.iter()).enumerate() {
        assert_eq!(got_tri.len(), 3, "{case}: face[{i}] corner count");
        for (l, (got, exp)) in got_tri.iter().zip(exp_tri.iter()).enumerate() {
            for (coord, (g, e)) in [(got.x(), exp[0]), (got.y(), exp[1])]
                .into_iter()
                .enumerate()
            {
                let d = (g - e).abs();
                let tol = TOL * e.abs().max(1.0);
                assert!(
                    d <= tol,
                    "{case}: uv[{i}][{l}][{coord}] rust={g} cpp={e} diff={d}"
                );
            }
        }
    }
}

fn check_structural(case: &str, p: &Parameterizer, taken: &[Vec<Vector2>], singulars: &[usize]) {
    assert_eq!(p.singular_vertex_indices(), singulars, "{case}: singulars");
    assert_eq!(
        p.original_triangle_uvs().len(),
        taken.len(),
        "{case}: original/taken length"
    );
    for (f, (o_tri, t_tri)) in p
        .original_triangle_uvs()
        .iter()
        .zip(taken.iter())
        .enumerate()
    {
        for (l, (o, t)) in o_tri.iter().zip(t_tri.iter()).enumerate() {
            assert!(
                o.x().to_bits() == t.x().to_bits() && o.y().to_bits() == t.y().to_bits(),
                "{case}: original[{f}][{l}] != taken (bitwise)"
            );
        }
    }
}

// Fixture case 321 (KIND single): 3 vertices, 1 faces, scaling 1, hard 90, adapt 0.5, aniso 1, simp 1, maxpd 6, symaxis -1, guides 0, sharps 0, density-mode 0, fieldvec-mode 0.
#[test]
fn golden_single_triangle() {
    let vertices = vec![v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0)];
    let triangles = vec![vec![0, 1, 2]];
    let guides: Vec<Vec<Vector3>> = vec![];
    let sharps: Vec<Vec<Vector3>> = vec![];
    let events: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let handler: ProgressHandler = Box::new({
        let events = events.clone();
        move |_fraction: f32, name: &str| {
            events.lock().unwrap().push(name.to_string());
        }
    });
    let mut p = Parameterizer::new(&vertices, &triangles, None);
    p.set_scaling(1.0);
    p.set_gradient_adaptivity(0.5);
    p.set_sharp_edge_degrees(90.0);
    p.set_anisotropy(1.0);
    p.set_singularity_simplification(true);
    p.set_maximum_singularity_pair_distance(6);
    p.set_guide_polylines(Some(&guides));
    p.set_sharp_polylines(Some(&sharps));
    p.set_progress_handler(handler);
    assert!(p.parameterize(), "golden must solve");
    let taken = p.take_triangle_uvs().expect("took UVs");
    check_uvs(
        "single_triangle",
        &taken,
        &[[
            [0.0, 0.0],
            [0.878_679_656_352_489_5, 0.0],
            [0.0, 0.878_679_656_352_489_5],
        ]],
    );
    check_structural("single_triangle", &p, &taken, &[]);
    assert_eq!(
        *events.lock().unwrap(),
        [
            "Computing vertex normals",
            "Computing scaling field",
            "Building surface topology",
            "Solving frame field",
            "Simplifying singularities",
            "Computing anisotropy field",
            "Initializing cover field",
            "Smoothing cross field",
            "Computing corner rotations",
            "Correcting field curl",
            "Building cover system",
            "Eliminating cover constraints",
            "Rounding cover to integers",
            "Rounding cover to integers",
            "Rounding cover to integers",
            "Building cover uvs",
            "Collecting singularities",
            "",
        ],
        "single_triangle: progress names"
    );
}
// Fixture case 48 (KIND fan): 4 vertices, 3 faces, scaling 1, hard 60, adapt 0.5, aniso 1, simp 0, maxpd 0, symaxis -1, guides 0, sharps 2, density-mode 0, fieldvec-mode 0.
#[test]
fn golden_fan_singularity() {
    let vertices = vec![
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(-0.49999999999999983, 0.866_025_403_784_438_7, 0.0),
        v(-0.500_000_000_000_000_4, -0.866_025_403_784_438_4, 0.0),
    ];
    let triangles = vec![vec![0, 1, 2], vec![0, 2, 3], vec![0, 3, 1]];
    let guides: Vec<Vec<Vector3>> = vec![];
    let sharps: Vec<Vec<Vector3>> = vec![
        vec![v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0)],
        vec![
            v(0.0, 0.0, 0.0),
            v(-0.49999999999999983, 0.866_025_403_784_438_7, 0.0),
        ],
    ];
    let events: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let handler: ProgressHandler = Box::new({
        let events = events.clone();
        move |_fraction: f32, name: &str| {
            events.lock().unwrap().push(name.to_string());
        }
    });
    let mut p = Parameterizer::new(&vertices, &triangles, None);
    p.set_scaling(1.0);
    p.set_gradient_adaptivity(0.5);
    p.set_sharp_edge_degrees(60.0);
    p.set_anisotropy(1.0);
    p.set_singularity_simplification(false);
    p.set_maximum_singularity_pair_distance(0);
    p.set_guide_polylines(Some(&guides));
    p.set_feature_polylines(Some(&sharps));
    p.set_progress_handler(handler);
    assert!(p.parameterize(), "golden must solve");
    let taken = p.take_triangle_uvs().expect("took UVs");
    check_uvs(
        "fan_singularity",
        &taken,
        &[
            [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]],
            [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]],
            [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]],
        ],
    );
    check_structural("fan_singularity", &p, &taken, &[0]);
    assert_eq!(
        *events.lock().unwrap(),
        [
            "Computing vertex normals",
            "Computing scaling field",
            "Building surface topology",
            "Solving frame field",
            "Computing anisotropy field",
            "Initializing cover field",
            "Smoothing cross field",
            "Computing corner rotations",
            "Correcting field curl",
            "Building cover system",
            "Eliminating cover constraints",
            "Rounding cover to integers",
            "Rounding cover to integers",
            "Rounding cover to integers",
            "Building cover uvs",
            "Collecting singularities",
            "",
        ],
        "fan_singularity: progress names"
    );
}
// Fixture case 57 (KIND tetra): 4 vertices, 4 faces, scaling 1, hard 10, adapt 0.5, aniso 1, simp 1, maxpd 6, symaxis -1, guides 1, sharps 1, density-mode 1, fieldvec-mode 0.
#[test]
fn golden_tetra_guided_dense() {
    let vertices = vec![
        v(0.197_4, -0.189, -0.033),
        v(1.180_6, -0.2802, -0.136_2),
        v(0.032_4, 1.192_9, -0.1308),
        v(0.185_1, 0.26339999999999997, 1.158_4),
    ];
    let triangles = vec![vec![0, 2, 1], vec![0, 1, 3], vec![0, 3, 2], vec![1, 2, 3]];
    let guides: Vec<Vec<Vector3>> =
        vec![vec![v(10.1974, 9.811, 9.967), v(11.1806, 9.719_8, 9.863_8)]];
    let sharps: Vec<Vec<Vector3>> = vec![vec![
        v(0.032_4, 0.642_942_666_666_666_8, 0.675_082_666_666_666_7),
        v(0.606_5, 0.269_757_333_333_333_3, 0.347_117_333_333_333_4),
        v(1.180_6, 0.269_757_333_333_333_3, 0.347_117_333_333_333_3),
    ]];
    let rho: Vec<f64> = vec![2.04, 0.86, 2.8200000000000003, -0.036_000_000_000_000_03];
    let events: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let handler: ProgressHandler = Box::new({
        let events = events.clone();
        move |_fraction: f32, name: &str| {
            events.lock().unwrap().push(name.to_string());
        }
    });
    let mut p = Parameterizer::new(&vertices, &triangles, None);
    p.set_scaling(1.0);
    p.set_gradient_adaptivity(0.5);
    p.set_sharp_edge_degrees(10.0);
    p.set_anisotropy(1.0);
    p.set_singularity_simplification(true);
    p.set_maximum_singularity_pair_distance(6);
    p.set_guide_polylines(Some(&guides));
    p.set_sharp_polylines(Some(&sharps));
    p.set_density_field(rho);
    p.set_progress_handler(handler);
    assert!(p.parameterize(), "golden must solve");
    let taken = p.take_triangle_uvs().expect("took UVs");
    check_uvs(
        "tetra_guided_dense",
        &taken,
        &[
            [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]],
            [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]],
            [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]],
            [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]],
        ],
    );
    check_structural("tetra_guided_dense", &p, &taken, &[0, 1, 2, 3]);
    assert_eq!(
        *events.lock().unwrap(),
        [
            "Computing vertex normals",
            "Computing scaling field",
            "Building surface topology",
            "Solving frame field",
            "Simplifying singularities",
            "Computing anisotropy field",
            "Initializing cover field",
            "Smoothing cross field",
            "Computing corner rotations",
            "Correcting field curl",
            "Building cover system",
            "Eliminating cover constraints",
            "Rounding cover to integers",
            "Rounding cover to integers",
            "Rounding cover to integers",
            "Building cover uvs",
            "Collecting singularities",
            "",
        ],
        "tetra_guided_dense: progress names"
    );
}
// Fixture case 245 (KIND fvec): 4 vertices, 2 faces, scaling 1, hard 90, adapt 0.5, aniso 1, simp 1, maxpd 6, symaxis -1, guides 0, sharps 0, density-mode 0, fieldvec-mode 1.
#[test]
#[allow(clippy::approx_constant)] // Golden expected value, not the named constant.
fn golden_provided_field() {
    let vertices = vec![
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(1.0, 1.0, 0.0),
        v(0.0, 1.0, 0.0),
    ];
    let triangles = vec![vec![0, 1, 2], vec![0, 2, 3]];
    let _guides: Vec<Vec<Vector3>> = vec![];
    let _sharps: Vec<Vec<Vector3>> = vec![];
    let fvecs: Vec<Vector3> = vec![v(-0.91, -0.563, -0.123), v(0.392, 0.318, -0.155)];
    let events: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let handler: ProgressHandler = Box::new({
        let events = events.clone();
        move |_fraction: f32, name: &str| {
            events.lock().unwrap().push(name.to_string());
        }
    });
    let mut p = Parameterizer::new(&vertices, &triangles, Some(&fvecs));
    p.set_scaling(1.0);
    p.set_gradient_adaptivity(0.5);
    p.set_sharp_edge_degrees(90.0);
    p.set_anisotropy(1.0);
    p.set_singularity_simplification(true);
    p.set_maximum_singularity_pair_distance(6);
    p.set_guide_polylines(None);
    p.set_sharp_polylines(None);
    p.set_progress_handler(handler);
    assert!(p.parameterize(), "golden must solve");
    let taken = p.take_triangle_uvs().expect("took UVs");
    check_uvs(
        "provided_field",
        &taken,
        &[
            [
                [0.0, 0.0],
                [-0.7530406320634434, 0.526_732_537_016_551_4],
                [-1.2858995489186271, -0.21798957846239736],
            ],
            [
                [0.0, 0.0],
                [-1.2858995489186271, -0.21798957846239736],
                [-0.534_905_937_092_539_9, -0.758_451_674_279_465_2],
            ],
        ],
    );
    check_structural("provided_field", &p, &taken, &[]);
    assert_eq!(
        *events.lock().unwrap(),
        [
            "Computing vertex normals",
            "Computing scaling field",
            "Building surface topology",
            "Solving frame field",
            "Simplifying singularities",
            "Computing anisotropy field",
            "Initializing cover field",
            "Smoothing cross field",
            "Computing corner rotations",
            "Correcting field curl",
            "Building cover system",
            "Eliminating cover constraints",
            "Rounding cover to integers",
            "Rounding cover to integers",
            "Rounding cover to integers",
            "Building cover uvs",
            "Collecting singularities",
            "",
        ],
        "provided_field: progress names"
    );
}
// Fixture case 32 (KIND quad): 4 vertices, 2 faces, scaling 0.5, hard 135, adapt 0.25, aniso 2, simp 1, maxpd 6, symaxis 1, guides 1, sharps 1, density-mode 0, fieldvec-mode 0.
#[test]
fn golden_symmetric_quad() {
    let vertices = vec![
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(1.0, 1.0, 0.0),
        v(0.0, 1.0, 0.0),
    ];
    let triangles = vec![vec![0, 1, 2], vec![0, 2, 3]];
    let guides: Vec<Vec<Vector3>> = vec![vec![v(10.0, 10.0, 10.0), v(11.0, 11.0, 10.0)]];
    let sharps: Vec<Vec<Vector3>> = vec![vec![
        v(0.0, 0.72, 0.0),
        v(0.5, 0.28, 0.0),
        v(1.0, 0.28, 0.0),
    ]];
    let events: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let handler: ProgressHandler = Box::new({
        let events = events.clone();
        move |_fraction: f32, name: &str| {
            events.lock().unwrap().push(name.to_string());
        }
    });
    let mut p = Parameterizer::new(&vertices, &triangles, None);
    p.set_scaling(0.5);
    p.set_gradient_adaptivity(0.25);
    p.set_sharp_edge_degrees(135.0);
    p.set_anisotropy(2.0);
    p.set_singularity_simplification(true);
    p.set_maximum_singularity_pair_distance(6);
    p.set_symmetry_plane(SymmetryPlane {
        axis: 1,
        offset: 0.5,
        score: 0.0,
    });
    p.set_guide_polylines(Some(&guides));
    p.set_feature_polylines(Some(&sharps));
    p.set_progress_handler(handler);
    assert!(p.parameterize(), "golden must solve");
    let taken = p.take_triangle_uvs().expect("took UVs");
    check_uvs(
        "symmetric_quad",
        &taken,
        &[
            [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]],
            [[0.0, 0.0], [2.0, 2.0], [0.0, 2.0]],
        ],
    );
    check_structural("symmetric_quad", &p, &taken, &[]);
    assert_eq!(
        *events.lock().unwrap(),
        [
            "Computing vertex normals",
            "Computing scaling field",
            "Building surface topology",
            "Solving frame field",
            "Symmetrizing frame field",
            "Simplifying singularities",
            "Computing anisotropy field",
            "Initializing cover field",
            "Smoothing cross field",
            "Computing corner rotations",
            "Correcting field curl",
            "Building cover system",
            "Eliminating cover constraints",
            "Rounding cover to integers",
            "Rounding cover to integers",
            "Rounding cover to integers",
            "Building cover uvs",
            "Collecting singularities",
            "",
        ],
        "symmetric_quad: progress names"
    );
}
// Early-false paths mirror the C++ `false` returns.
#[test]
fn golden_invalid_inputs_return_false() {
    // Empty mesh: no faces to solve.
    let empty_v: Vec<Vector3> = Vec::new();
    let empty_t: Vec<Vec<usize>> = Vec::new();
    assert!(!Parameterizer::new(&empty_v, &empty_t, None).parameterize());
    // Vertices but no triangles.
    let one = vec![v(0.0, 0.0, 0.0)];
    assert!(!Parameterizer::new(&one, &empty_t, None).parameterize());
    // Non-positive scaling fails the cover solve.
    let quad_v = vec![
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(1.0, 1.0, 0.0),
        v(0.0, 1.0, 0.0),
    ];
    let quad_t = vec![vec![0, 1, 2], vec![0, 2, 3]];
    let mut p = Parameterizer::new(&quad_v, &quad_t, None);
    p.set_scaling(0.0);
    assert!(!p.parameterize());
    let mut p = Parameterizer::new(&quad_v, &quad_t, None);
    p.set_scaling(-1.0);
    assert!(!p.parameterize());
    // Wrong-size provided field vectors.
    let bad_field = vec![v(1.0, 0.0, 0.0)];
    assert!(!Parameterizer::new(&quad_v, &quad_t, Some(&bad_field)).parameterize());
    // Take-before-solve and double-take semantics.
    let mut p = Parameterizer::new(&quad_v, &quad_t, None);
    assert!(p.take_triangle_uvs().is_none());
    assert!(p.original_triangle_uvs().is_empty());
    assert!(p.singular_vertex_indices().is_empty());
    assert!(p.singular_vertex_positions().is_empty());
    assert!(p.parameterize());
    assert!(p.take_triangle_uvs().is_some());
    assert!(p.take_triangle_uvs().is_none());
}
