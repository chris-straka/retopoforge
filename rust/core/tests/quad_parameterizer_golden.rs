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

#[allow(clippy::too_many_arguments)] // Port keeps the C++ parameter list 1:1.
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
            [0.878_679_656_352_489_5, 0.0],
            [0.0, 0.878_679_656_352_489_5],
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
        v(-1.037_8, -1.0762, -0.9607),
        v(1.123_9, -0.814_9, -0.944_8),
        v(0.857_5, 1.021, -1.2481),
        v(-0.772_9, 1.1314, -0.849_7),
        v(-0.857_2, -1.040_8, 1.140_1),
        v(1.153, -0.976_3, 1.2513999999999998),
        v(1.2583, 1.138_3, 0.7762),
        v(-1.158_7, 1.232_5, 1.2463),
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
    let a = [0.47458074027070446, -0.757_857_613_842_846_1];
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
                0.898_387_111_094_083_5,
                -0.42916861175552073,
                0.093_353_635_729_231_62,
            ],
            [
                0.822_395_169_104_126_6,
                -0.517_024_364_188_662_7,
                -0.23738574655927464,
            ],
            [
                0.018910576097195545,
                -0.9745804924014827,
                0.22323811041611097,
            ],
            [
                0.30953701338830336,
                -0.947_526_572_718_370_5,
                -0.079_876_350_287_353_68,
            ],
            [
                0.956_546_512_337_460_2,
                0.095_462_266_830_737_73,
                0.27550993692892095,
            ],
            [
                0.997_958_713_142_671,
                0.032_020_862_102_130_28,
                0.055_254_603_906_466_58,
            ],
            [
                0.733_960_995_157_134_3,
                -0.068_292_086_442_716_8,
                -0.675_749_545_702_584_6,
            ],
            [
                0.46902813470990534,
                0.043_262_679_932_182_82,
                0.882_122_978_600_848_2,
            ],
            [
                0.027_624_115_228_447_18,
                -0.676_838_436_104_884_8,
                -0.735_613_104_606_585_6,
            ],
            [
                0.11990081940844635,
                -0.405_609_174_934_785_3,
                -0.906_148_437_461_493_3,
            ],
            [
                0.211_560_902_694_884,
                -0.750_458_381_623_394_2,
                0.626_142_317_610_079_7,
            ],
            [
                -0.022_974_957_356_120_41,
                -0.666_820_794_950_64,
                0.744_863_865_921_740_9,
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
        v(0.938, 0.728, 0.29),
        v(-0.654, -0.695, 0.928),
        v(-0.373, 0.569, 0.856),
        v(-0.437, -0.095, 0.222),
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
                0.767_428_643_589_341_3,
                0.595_616_260_696_205_2,
                0.23726471923337844,
            ],
            [0.576_060_552_329_653_4, 0.0, -0.817_407_022_265_930_1],
            [0.0, 0.832_625_831_577_035_5, -0.553_835_918_472_836_4],
            [
                -0.8752380718242736,
                -0.190330507414009,
                0.444_671_356_820_802_7,
            ],
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
        v(0.866_025_403_784_438_7, 0.49999999999999994, 0.0),
        v(0.500_000_000_000_000_1, 0.8660254037844386, 0.0),
        v(6.123233995736766e-17, 1.0, 0.0),
        v(-0.49999999999999983, 0.866_025_403_784_438_7, 0.0),
        v(-0.866_025_403_784_438_7, 0.49999999999999994, 0.0),
        v(-1.0, 1.2246467991473532e-16, 0.0),
        v(-0.866_025_403_784_438_8, -0.499_999_999_999_999_8, 0.0),
        v(-0.500_000_000_000_000_4, -0.866_025_403_784_438_4, 0.0),
        v(-1.8369701987210297e-16, -1.0, 0.0),
        v(0.500_000_000_000_000_1, -0.8660254037844386, 0.0),
        v(0.866_025_403_784_438_4, -0.500_000_000_000_000_4, 0.0),
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
            [0.965_925_826_289_068_3, 0.25881904510252063, 0.0],
            [0.707_106_781_186_547_7, 0.707_106_781_186_547_5, 0.0],
            [0.258_819_045_102_521, 0.9659258262890682, 0.0],
            [-0.258_819_045_102_520_6, 0.965_925_826_289_068_3, 0.0],
            [-0.707_106_781_186_547_4, 0.707_106_781_186_547_9, 0.0],
            [-0.965_925_826_289_068_3, 0.25881904510252085, 0.0],
            [-0.965_925_826_289_068_4, -0.258_819_045_102_520_6, 0.0],
            [-0.707_106_781_186_547_7, -0.707_106_781_186_547_4, 0.0],
            [-0.25881904510252113, -0.9659258262890682, 0.0],
            [0.258_819_045_102_520_7, -0.965_925_826_289_068_3, 0.0],
            [0.707_106_781_186_547_5, -0.707_106_781_186_547_7, 0.0],
            [0.965_925_826_289_068_3, -0.258_819_045_102_521, 0.0],
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
