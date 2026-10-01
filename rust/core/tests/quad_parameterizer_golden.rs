// Replicated goldens for the quad parameterizer port. No C++ unit test
// covers QuadParameterizer directly (it is exercised end to end through
// Parameterizer), so these groups freeze small C++-solved cases as
// literals: inputs transcribed bit-for-bit from tests/fixtures/
// quadparam_diff.txt (noted per group), outputs asserted at scale-aware
// 1e-6 with exact structural facts (ok, rotations, singulars, progress
// alias sequence, which pins the MILS rounding iteration count).
use retopo_core::progress::ProgressHandler;
use retopo_core::quad_parameterizer::{DipoleConfig, ParameterizeResult, QuadParameterizer};
use retopo_core::vector3::Vector3;
use std::sync::{Arc, Mutex};

const TOL: f64 = 1e-6;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

fn check_values(case: &str, result: &ParameterizeResult, uvs: &[[f64; 2]], field: &[[f64; 3]]) {
    assert_eq!(
        result.triangle_uvs.len(),
        uvs.len() / 3,
        "{case}: face count"
    );
    assert_eq!(result.field.len(), field.len(), "{case}: field count");
    for (i, (got_tri, exp_tri)) in result.triangle_uvs.iter().zip(uvs.chunks(3)).enumerate() {
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
    for (i, (got, exp)) in result.field.iter().zip(field.iter()).enumerate() {
        for (coord, (g, e)) in [(got.x(), exp[0]), (got.y(), exp[1]), (got.z(), exp[2])]
            .into_iter()
            .enumerate()
        {
            let d = (g - e).abs();
            let tol = TOL * e.abs().max(1.0);
            assert!(
                d <= tol,
                "{case}: field[{i}][{coord}] rust={g} cpp={e} diff={d}"
            );
        }
    }
}

fn check_structural(
    case: &str,
    result: &ParameterizeResult,
    rotations: &[i32],
    singulars: &[usize],
) {
    assert_eq!(result.corner_rotations, rotations, "{case}: rotations");
    assert_eq!(result.singular_vertices, singulars, "{case}: singulars");
}

fn run_with_prog(
    vertices: &[Vector3],
    triangles: &[Vec<usize>],
    guidance: &[Vector3],
    scaling: f64,
    hard: f64,
    face_scaling: &[f64],
    fsu: &[f64],
    fsv: &[f64],
    sharps: Option<&[Vec<Vector3>]>,
) -> (Option<ParameterizeResult>, Vec<String>) {
    let events: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let handler: ProgressHandler = Box::new({
        let events = events.clone();
        move |_fraction: f32, name: &str| {
            events.lock().unwrap().push(name.to_string());
        }
    });
    let got = QuadParameterizer::parameterize(
        vertices,
        triangles,
        guidance,
        scaling,
        hard,
        face_scaling,
        fsu,
        fsv,
        Some(&handler),
        sharps,
        &[],
        DipoleConfig::off(),
    );
    let aliases = events.lock().unwrap().clone();
    (got, aliases)
}

// Fixture case 248 (KIND single): one triangle, scaling 1, hard 90.
#[test]
fn golden_single_triangle() {
    let vertices = vec![v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0)];
    let triangles = vec![vec![0, 1, 2]];
    let (got, aliases) = run_with_prog(&vertices, &triangles, &[], 1.0, 90.0, &[], &[], &[], None);
    let result = got.expect("single triangle must solve");
    check_values(
        "single",
        &result,
        &[
            [0.0, 0.0],
            [0.87867965635248946, 0.0],
            [0.0, 0.87867965635248946],
        ],
        &[[1.0, 6.123233995736766e-17, 0.0]],
    );
    check_structural("single", &result, &[0, 0, 0], &[]);
    assert_eq!(
        aliases,
        [
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
        ],
        "single: progress aliases (3 ROUNDs = 2 iterations + final)"
    );
}

// Fixture case 199 (KIND quad): flat unit quad, scaling 0.5, hard 30.
// UVs snap to exact integers through the 0.01 round.
#[test]
fn golden_flat_quad() {
    let vertices = vec![
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(1.0, 1.0, 0.0),
        v(0.0, 1.0, 0.0),
    ];
    let triangles = vec![vec![0, 1, 2], vec![0, 2, 3]];
    let (got, aliases) = run_with_prog(&vertices, &triangles, &[], 0.5, 30.0, &[], &[], &[], None);
    let result = got.expect("flat quad must solve");
    check_values(
        "quad",
        &result,
        &[
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, 2.0],
            [0.0, 0.0],
            [2.0, 2.0],
            [0.0, 2.0],
        ],
        &[
            [1.0, -6.123233995736766e-17, 0.0],
            [1.0, 6.123233995736766e-17, 0.0],
        ],
    );
    check_structural("quad", &result, &[0, 0, 0, 0, 0, 0], &[]);
    assert_eq!(aliases.len(), 10, "quad: progress event count");
    assert_eq!(aliases[0], "Initializing cover field");
    assert_eq!(aliases[9], "Building cover uvs");
}

// Fixture case 180 (KIND box): jittered cube, scaling 1, hard 100.
// Exercises dihedral marks, nonzero rotations, and singularities.
#[test]
fn golden_box_hard_edges() {
    let vertices = vec![
        v(-1.0378000000000001, -1.0762, -0.9607),
        v(
            1.1238999999999999,
            -0.81489999999999996,
            -0.94479999999999997,
        ),
        v(0.85750000000000004, 1.0209999999999999, -1.2481),
        v(-0.77290000000000003, 1.1314, -0.84970000000000001),
        v(
            -0.85719999999999996,
            -1.0407999999999999,
            1.1400999999999999,
        ),
        v(1.153, -0.97629999999999995, 1.2513999999999998),
        v(1.2583, 1.1383000000000001, 0.7762),
        v(-1.1587000000000001, 1.2324999999999999, 1.2463),
    ];
    let triangles = vec![
        vec![0, 2, 1],
        vec![0, 3, 2],
        vec![4, 5, 6],
        vec![4, 6, 7],
        vec![0, 1, 5],
        vec![0, 5, 4],
        vec![2, 3, 7],
        vec![2, 7, 6],
        vec![0, 4, 7],
        vec![0, 7, 3],
        vec![1, 2, 6],
        vec![1, 6, 5],
    ];
    let (got, _) = run_with_prog(&vertices, &triangles, &[], 1.0, 100.0, &[], &[], &[], None);
    let result = got.expect("box must solve");
    let a = [0.47458074027070446, -0.75785761384284611];
    check_values(
        "box",
        &result,
        &[
            [0.0, 0.0],
            [1.0, -1.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [-1.0, -1.0],
            [1.0, -1.0],
            [1.0, -1.0],
            [1.0, 0.0],
            a,
            [1.0, -1.0],
            a,
            [0.0, -2.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 1.0],
            [0.0, 0.0],
            [0.0, 1.0],
            [-1.0, 1.0],
            [1.0, -1.0],
            [-1.0, -1.0],
            [0.0, 0.0],
            [1.0, -1.0],
            [0.0, -2.0],
            a,
            [0.0, 0.0],
            [-1.0, -1.0],
            [0.0, -2.0],
            [0.0, 0.0],
            [0.0, -2.0],
            [-1.0, -1.0],
            [0.0, 0.0],
            [1.0, -1.0],
            a,
            [0.0, 0.0],
            a,
            [1.0, 0.0],
        ],
        &[
            [
                0.89838711109408353,
                -0.42916861175552073,
                0.093353635729231624,
            ],
            [
                0.82239516910412658,
                -0.51702436418866271,
                -0.23738574655927464,
            ],
            [
                0.018910576097195545,
                -0.9745804924014827,
                0.22323811041611097,
            ],
            [
                0.30953701338830336,
                -0.94752657271837049,
                -0.079876350287353681,
            ],
            [
                0.95654651233746024,
                0.095462266830737733,
                0.27550993692892095,
            ],
            [
                0.99795871314267104,
                0.032020862102130281,
                0.055254603906466582,
            ],
            [
                0.73396099515713431,
                -0.068292086442716796,
                -0.67574954570258461,
            ],
            [
                0.46902813470990534,
                0.043262679932182822,
                0.88212297860084821,
            ],
            [
                0.027624115228447179,
                -0.67683843610488481,
                -0.73561310460658558,
            ],
            [
                0.11990081940844635,
                -0.40560917493478532,
                -0.90614843746149332,
            ],
            [
                0.21156090269488401,
                -0.75045838162339418,
                0.62614231761007966,
            ],
            [
                -0.022974957356120412,
                -0.66682079495063995,
                0.74486386592174092,
            ],
        ],
    );
    check_structural(
        "box",
        &result,
        &[
            0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 1, 0, 3, 0, 0, 1, 1, 0, 3, 1, 3, 0, 0, 3, 3, 0, 0, 1,
            0, 0, 0, 0, 0, 0, 1,
        ],
        &[0, 1, 2, 3, 4, 5, 7],
    );
}

// Fixture case 8 (KIND tetra): tetrahedron with raw guidance (skips the
// smoothing pass), scaling 0.5, hard 10.
#[test]
fn golden_guided_tetra() {
    let vertices = vec![
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(0.0, 1.0, 0.0),
        v(0.0, 0.0, 1.0),
    ];
    let triangles = vec![vec![0, 2, 1], vec![0, 1, 3], vec![0, 3, 2], vec![1, 2, 3]];
    let guidance = vec![
        v(
            0.93799999999999994,
            0.72799999999999998,
            0.28999999999999998,
        ),
        v(
            -0.65400000000000003,
            -0.69499999999999995,
            0.92800000000000005,
        ),
        v(-0.373, 0.56899999999999995, 0.85599999999999998),
        v(-0.437, -0.095000000000000001, 0.222),
    ];
    let (got, _) = run_with_prog(
        &vertices,
        &triangles,
        &guidance,
        0.5,
        10.0,
        &[],
        &[],
        &[],
        None,
    );
    let result = got.expect("guided tetra must solve");
    check_values(
        "tetra",
        &result,
        &[
            [0.0, 0.0],
            [1.0, -1.0],
            [1.0, 1.0],
            [0.0, 0.0],
            [1.0, 1.0],
            [0.0, 1.0],
            [0.0, 0.0],
            [-1.0, 0.0],
            [1.0, -1.0],
            [1.0, 1.0],
            [1.0, -1.0],
            [2.0, 1.0],
        ],
        &[
            [
                0.76742864358934126,
                0.59561626069620521,
                0.23726471923337844,
            ],
            [0.57606055232965336, 0.0, -0.81740702226593009],
            [0.0, 0.83262583157703551, -0.55383591847283642],
            [-0.8752380718242736, -0.190330507414009, 0.44467135682080272],
        ],
    );
    check_structural(
        "tetra",
        &result,
        &[0, 0, 0, 0, 2, 1, 3, 3, 0, 0, 1, 2],
        &[0, 1, 2, 3],
    );
}

// Fixture case 250 (KIND fan12): valence-12 flat fan, scaling 1, hard 90.
#[test]
fn golden_fan12() {
    let vertices = vec![
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(0.86602540378443871, 0.49999999999999994, 0.0),
        v(0.50000000000000011, 0.8660254037844386, 0.0),
        v(6.123233995736766e-17, 1.0, 0.0),
        v(-0.49999999999999983, 0.86602540378443871, 0.0),
        v(-0.86602540378443871, 0.49999999999999994, 0.0),
        v(-1.0, 1.2246467991473532e-16, 0.0),
        v(-0.86602540378443882, -0.49999999999999978, 0.0),
        v(-0.50000000000000044, -0.86602540378443837, 0.0),
        v(-1.8369701987210297e-16, -1.0, 0.0),
        v(0.50000000000000011, -0.8660254037844386, 0.0),
        v(0.86602540378443837, -0.50000000000000044, 0.0),
    ];
    let triangles = vec![
        vec![0, 1, 2],
        vec![0, 2, 3],
        vec![0, 3, 4],
        vec![0, 4, 5],
        vec![0, 5, 6],
        vec![0, 6, 7],
        vec![0, 7, 8],
        vec![0, 8, 9],
        vec![0, 9, 10],
        vec![0, 10, 11],
        vec![0, 11, 12],
        vec![0, 12, 1],
    ];
    let (got, _) = run_with_prog(&vertices, &triangles, &[], 1.0, 90.0, &[], &[], &[], None);
    let result = got.expect("fan12 must solve");
    let neg0 = -0.0;
    check_values(
        "fan12",
        &result,
        &[
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, 0.0],
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, 0.0],
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, 0.0],
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, neg0],
            [0.0, 0.0],
            [2.0, neg0],
            [2.0, 0.0],
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, 0.0],
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, 0.0],
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, neg0],
            [0.0, 0.0],
            [2.0, neg0],
            [2.0, neg0],
            [0.0, 0.0],
            [2.0, neg0],
            [2.0, 0.0],
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, neg0],
            [0.0, 0.0],
            [2.0, neg0],
            [2.0, 0.0],
        ],
        &[
            [0.96592582628906831, 0.25881904510252063, 0.0],
            [0.70710678118654768, 0.70710678118654746, 0.0],
            [0.25881904510252102, 0.9659258262890682, 0.0],
            [-0.25881904510252057, 0.96592582628906831, 0.0],
            [-0.70710678118654735, 0.70710678118654791, 0.0],
            [-0.96592582628906831, 0.25881904510252085, 0.0],
            [-0.96592582628906842, -0.25881904510252057, 0.0],
            [-0.70710678118654768, -0.70710678118654735, 0.0],
            [-0.25881904510252113, -0.9659258262890682, 0.0],
            [0.25881904510252068, -0.96592582628906831, 0.0],
            [0.70710678118654746, -0.70710678118654768, 0.0],
            [0.96592582628906831, -0.25881904510252102, 0.0],
        ],
    );
    check_structural("fan12", &result, &[0; 36], &[]);
}

// Early-false paths mirror the C++ `false` returns as `None`.
#[test]
fn golden_invalid_inputs_return_none() {
    let one = vec![v(0.0, 0.0, 0.0)];
    let tri = vec![vec![0, 1, 2]];
    let quad_v = vec![
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(1.0, 1.0, 0.0),
        v(0.0, 1.0, 0.0),
    ];
    let quad_t = vec![vec![0, 1, 2], vec![0, 2, 3]];
    // Empty vertices / empty triangles.
    assert!(
        QuadParameterizer::parameterize(
            &[],
            &tri,
            &[],
            1.0,
            90.0,
            &[],
            &[],
            &[],
            None,
            None,
            &[],
            DipoleConfig::off()
        )
            .is_none()
    );
    assert!(
        QuadParameterizer::parameterize(
            &one,
            &[],
            &[],
            1.0,
            90.0,
            &[],
            &[],
            &[],
            None,
            None,
            &[],
            DipoleConfig::off()
        )
            .is_none()
    );
    // Non-positive scaling.
    assert!(
        QuadParameterizer::parameterize(
            &quad_v,
            &quad_t,
            &[],
            0.0,
            90.0,
            &[],
            &[],
            &[],
            None,
            None,
            &[],
            DipoleConfig::off()
        )
        .is_none()
    );
    assert!(
        QuadParameterizer::parameterize(
            &quad_v,
            &quad_t,
            &[],
            -2.0,
            90.0,
            &[],
            &[],
            &[],
            None,
            None,
            &[],
            DipoleConfig::off()
        )
        .is_none()
    );
    // Dropped triangles (wrong corner counts).
    let bad2 = vec![vec![0, 1, 2], vec![0, 1]];
    let bad4 = vec![vec![0, 1, 2], vec![0, 1, 2, 3]];
    assert!(
        QuadParameterizer::parameterize(
            &quad_v,
            &bad2,
            &[],
            1.0,
            90.0,
            &[],
            &[],
            &[],
            None,
            None,
            &[],
            DipoleConfig::off()
        )
        .is_none()
    );
    assert!(
        QuadParameterizer::parameterize(
            &quad_v,
            &bad4,
            &[],
            1.0,
            90.0,
            &[],
            &[],
            &[],
            None,
            None,
            &[],
            DipoleConfig::off()
        )
        .is_none()
    );
}

// Progress reporting is observation-only: None and Some handlers solve
// identical values.
#[test]
fn golden_none_progress_matches_recording() {
    let vertices = vec![
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(1.0, 1.0, 0.0),
        v(0.0, 1.0, 0.0),
    ];
    let triangles = vec![vec![0, 1, 2], vec![0, 2, 3]];
    let bare = QuadParameterizer::parameterize(
        &vertices,
        &triangles,
        &[],
        0.5,
        30.0,
        &[],
        &[],
        &[],
        None,
        None,
        &[],
        DipoleConfig::off(),
    )
    .expect("bare solve must succeed");
    let (recorded, aliases) =
        run_with_prog(&vertices, &triangles, &[], 0.5, 30.0, &[], &[], &[], None);
    let recorded = recorded.expect("recorded solve must succeed");
    assert_eq!(aliases.len(), 10);
    assert_eq!(bare.triangle_uvs.len(), recorded.triangle_uvs.len());
    for (bt, rt) in bare.triangle_uvs.iter().zip(recorded.triangle_uvs.iter()) {
        for (b, r) in bt.iter().zip(rt.iter()) {
            assert_eq!(b.x().to_bits(), r.x().to_bits());
            assert_eq!(b.y().to_bits(), r.y().to_bits());
        }
    }
    for (b, r) in bare.field.iter().zip(recorded.field.iter()) {
        assert_eq!(b.x().to_bits(), r.x().to_bits());
        assert_eq!(b.y().to_bits(), r.y().to_bits());
        assert_eq!(b.z().to_bits(), r.z().to_bits());
    }
    assert_eq!(bare.corner_rotations, recorded.corner_rotations);
    assert_eq!(bare.singular_vertices, recorded.singular_vertices);
}
