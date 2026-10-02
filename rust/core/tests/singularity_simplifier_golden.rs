// Hand-derived golden tests for the singularitysimplifier port. No C++
// unit test covers SingularitySimplifier, so (per the guides-lane
// precedent) these pin small cases directly: edge behavior derived from
// the documented contract (empty/mismatched fields), plus small meshes
// whose C++-solved outputs are transcribed as literals from
// tests/fixtures/singularitysimplifier_diff.txt (case IDs noted per
// group). Structural facts (charges, counts) assert exactly; output
// fields assert at scale-aware 1e-6, except identity cases (no cancel),
// which assert the field bitwise unchanged — the port only writes the
// field on the success path.
use retopo_core::singularity_simplifier::SingularitySimplifier;
use retopo_core::surface_mesh::SurfaceMesh;
use retopo_core::vector3::Vector3;

const TOL: f64 = 1e-6;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

// Triangulated w x h grid over (w+1) x (h+1) vertices, CCW from +z
// (mirrors the dump tool's makeGrid; relief 0 = planar).
fn grid(w: usize, h: usize) -> (Vec<Vector3>, Vec<Vec<usize>>) {
    let mut positions = Vec::new();
    for y in 0..=h {
        for x in 0..=w {
            positions.push(v(x as f64, y as f64, 0.0));
        }
    }
    let id = |x: usize, y: usize| y * (w + 1) + x;
    let mut triangles = Vec::new();
    for y in 0..h {
        for x in 0..w {
            let a = id(x, y);
            let b = id(x + 1, y);
            let c = id(x + 1, y + 1);
            let d = id(x, y + 1);
            triangles.push(vec![a, b, c]);
            triangles.push(vec![a, c, d]);
        }
    }
    (positions, triangles)
}

fn octa() -> (Vec<Vector3>, Vec<Vec<usize>>) {
    let positions = vec![
        v(1.0, 0.0, 0.0),
        v(-1.0, 0.0, 0.0),
        v(0.0, 1.0, 0.0),
        v(0.0, -1.0, 0.0),
        v(0.0, 0.0, 1.0),
        v(0.0, 0.0, -1.0),
    ];
    let triangles = vec![
        vec![4, 0, 2],
        vec![4, 2, 1],
        vec![4, 1, 3],
        vec![4, 3, 0],
        vec![5, 2, 0],
        vec![5, 1, 2],
        vec![5, 3, 1],
        vec![5, 0, 3],
    ];
    (positions, triangles)
}

fn tetra() -> (Vec<Vector3>, Vec<Vec<usize>>) {
    let positions = vec![
        v(0.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
        v(0.0, 1.0, 0.0),
        v(0.0, 0.0, 1.0),
    ];
    let triangles = vec![vec![0, 2, 1], vec![0, 1, 3], vec![0, 3, 2], vec![1, 2, 3]];
    (positions, triangles)
}

// Double-strength winding dipole on the 4x4 planar grid (poles (1,2)+2.0
// and (3,2)-2.0): the fixture case 11/12 input, transcribed as literals
// (the dump tool builds it with a backend-fused sincos, so rebuilding it
// here with separate cos/sin could drift by 1 ulp and flip a charge).
fn dipole_4x4_input() -> Vec<Vector3> {
    vec![
        v(0.9299293532683236, 0.36773821929459194, 0.0),
        v(0.948_683_298_050_513_8, 0.316_227_766_016_837_9, 0.0),
        v(0.864_005_952_520_259, 0.503_481_592_523_063_2, 0.0),
        v(0.841_621_860_009_949_3, 0.540_067_259_471_811_8, 0.0),
        v(0.881_674_598_767_943_7, 0.471_857_925_532_024_3, 0.0),
        v(0.811_242_185_175_560_8, 0.584_710_284_663_765, 0.0),
        v(0.950_534_146_061_684_1, 0.310_620_084_944_269_1, 0.0),
        v(0.919_756_860_054_768_8, 0.39248862197800394, 0.0),
        v(0.9153481929073074, 0.402_663_241_110_145, 0.0),
        v(0.985_644_543_834_867_9, 0.16883374428281572, 0.0),
        v(0.584_710_284_663_765, 0.811_242_185_175_560_8, 0.0),
        v(0.47185792553202427, 0.881_674_598_767_943_7, 0.0),
        v(0.677_109_488_984_706_2, 0.735_882_286_732_647_2, 0.0),
        v(0.34694624773493626, 0.937_885_014_904_624_7, 0.0),
        v(0.963_714_928_210_760_9, 0.26693358189581157, 0.0),
        v(0.948_683_298_050_513_9, 0.316_227_766_016_837_8, 0.0),
        v(0.948_683_298_050_513_8, -0.316_227_766_016_837_9, 0.0),
        v(0.963_714_928_210_761, -0.26693358189581146, 0.0),
        v(0.34694624773493626, -0.937_885_014_904_624_7, 0.0),
        v(0.677_109_488_984_706_1, -0.735_882_286_732_647_3, 0.0),
        v(0.47185792553202444, -0.881_674_598_767_943_6, 0.0),
        v(0.584_710_284_663_764_8, -0.8112421851755609, 0.0),
        v(0.985_644_543_834_867_9, -0.16883374428281595, 0.0),
        v(0.9153481929073074, -0.402_663_241_110_145, 0.0),
        v(0.919_756_860_054_768_8, -0.39248862197800394, 0.0),
        v(0.950_534_146_061_684_2, -0.310_620_084_944_269, 0.0),
        v(0.811_242_185_175_560_8, -0.584_710_284_663_765, 0.0),
        v(0.881_674_598_767_943_7, -0.471_857_925_532_024_3, 0.0),
        v(0.841_621_860_009_949_4, -0.540_067_259_471_811_7, 0.0),
        v(0.864_005_952_520_259, -0.503_481_592_523_063_2, 0.0),
        v(0.948_683_298_050_513_8, -0.316227766016838, 0.0),
        v(0.9299293532683236, -0.36773821929459194, 0.0),
    ]
}

fn check_field(case: &str, got: &[Vector3], expected: &[[f64; 3]]) {
    assert_eq!(got.len(), expected.len(), "{case}: field arity");
    for (i, (g, e)) in got.iter().zip(expected.iter()).enumerate() {
        for (coord, (gv, ev)) in [(g.x(), e[0]), (g.y(), e[1]), (g.z(), e[2])]
            .into_iter()
            .enumerate()
        {
            let d = (gv - ev).abs();
            let tol = TOL * ev.abs().max(1.0);
            assert!(
                d <= tol,
                "{case}: field[{i}][{coord}] rust={gv} cpp={ev} diff={d}"
            );
        }
    }
}

fn check_bits_unchanged(case: &str, got: &[Vector3], before: &[Vector3]) {
    assert_eq!(got.len(), before.len(), "{case}: field arity");
    for (i, (g, b)) in got.iter().zip(before.iter()).enumerate() {
        assert_eq!(
            g.x().to_bits(),
            b.x().to_bits(),
            "{case}: field[{i}].x bits"
        );
        assert_eq!(
            g.y().to_bits(),
            b.y().to_bits(),
            "{case}: field[{i}].y bits"
        );
        assert_eq!(
            g.z().to_bits(),
            b.z().to_bits(),
            "{case}: field[{i}].z bits"
        );
    }
}

#[test]
fn empty_mesh_is_noop() {
    let mesh = SurfaceMesh::new(&[], &[]);
    let mut field: Vec<Vector3> = Vec::new();
    let mut simp = SingularitySimplifier::new(&mesh, &mut field);
    // simplify() early-returns on an empty field; the counts stay zero.
    simp.simplify();
    assert_eq!(simp.singularity_count_before(), 0);
    assert_eq!(simp.singularity_count_after(), 0);
    assert_eq!(simp.cancelled_pair_count(), 0);
    assert!(simp.vertex_charges().is_empty());
}

#[test]
fn field_length_mismatch_yields_zero_charges() {
    let (positions, triangles) = grid(4, 4);
    let mesh = SurfaceMesh::new(&positions, &triangles);
    assert_eq!(mesh.face_count(), 32);
    // 3 vectors for 32 faces: out of contract, defined branch returns zeros.
    let mut field = vec![v(1.0, 0.0, 0.0); 3];
    let simp = SingularitySimplifier::new(&mesh, &mut field);
    assert_eq!(simp.vertex_charges(), vec![0; 25]);
    assert_eq!(simp.singularity_count(), 0);
}

#[test]
fn uniform_planar_grid_is_untouched() {
    // Fixture case 0 (4x4 planar grid, uniform field): a constant field on
    // a flat mesh has no rotation, hence no singularities and no writes.
    let (positions, triangles) = grid(4, 4);
    let mesh = SurfaceMesh::new(&positions, &triangles);
    let mut field = vec![v(1.0, 0.0, 0.0); mesh.face_count()];
    let input = field.clone();
    let mut simp = SingularitySimplifier::new(&mesh, &mut field);
    simp.set_maximum_pair_distance(6);
    simp.set_sharp_edge_degrees(90.0);
    assert_eq!(simp.vertex_charges(), vec![0; 25]);
    simp.simplify();
    assert_eq!(simp.singularity_count_before(), 0);
    assert_eq!(simp.singularity_count_after(), 0);
    assert_eq!(simp.cancelled_pair_count(), 0);
    assert_eq!(simp.vertex_charges(), vec![0; 25]);
    drop(simp);
    check_bits_unchanged("uniform-planar", &field, &input);
}

#[test]
fn octa_uniform_forces_total_charge_eight() {
    // Fixture case 5 (octahedron, uniform field): a constant field on a
    // closed curved mesh must carry singularities totaling 4*chi = 8
    // (Poincare-Hopf for cross fields); the C++ leaves the four
    // charge-2 vertices uncancelled.
    let (positions, triangles) = octa();
    let mesh = SurfaceMesh::new(&positions, &triangles);
    let mut field = vec![v(1.0, 0.0, 0.0); mesh.face_count()];
    let mut simp = SingularitySimplifier::new(&mesh, &mut field);
    simp.set_maximum_pair_distance(6);
    simp.set_sharp_edge_degrees(90.0);
    let charges = simp.vertex_charges();
    assert_eq!(charges, vec![0, 0, 2, 2, 2, 2]);
    assert_eq!(charges.iter().sum::<i32>(), 8);
    simp.simplify();
    assert_eq!(simp.singularity_count_before(), 4);
    assert_eq!(simp.singularity_count_after(), 4);
    assert_eq!(simp.cancelled_pair_count(), 0);
    assert_eq!(simp.vertex_charges(), vec![0, 0, 2, 2, 2, 2]);
}

#[test]
fn dipole_pair_cancels() {
    // Fixture case 11: R 2 0 1, output field transcribed below. The poles
    // form a {1, 3} charge pair, which sums to 0 mod 4 and cancels.
    let (positions, triangles) = grid(4, 4);
    let mut field = dipole_4x4_input();
    let mesh = SurfaceMesh::new(&positions, &triangles);
    let mut simp = SingularitySimplifier::new(&mesh, &mut field);
    simp.set_maximum_pair_distance(6);
    simp.set_sharp_edge_degrees(90.0);
    let mut expected_chi = vec![0; 25];
    expected_chi[11] = 1;
    expected_chi[13] = 3;
    assert_eq!(simp.vertex_charges(), expected_chi);
    simp.simplify();
    assert_eq!(simp.singularity_count_before(), 2);
    assert_eq!(simp.singularity_count_after(), 0);
    assert_eq!(simp.cancelled_pair_count(), 1);
    assert_eq!(simp.vertex_charges(), vec![0; 25]);
    drop(simp);
    check_field("dipole-cancel", &field, &EXPECTED_DIPOLE_FO);
}

// Transcribed FO line of fixture case 11 (32 faces).
const EXPECTED_DIPOLE_FO: [[f64; 3]; 32] = [
    [0.9299293532683236, 0.36773821929459194, 0.0],
    [0.948_683_298_050_513_8, 0.316_227_766_016_837_9, 0.0],
    [0.864_005_952_520_259, 0.503_481_592_523_063_2, 0.0],
    [0.927_362_246_507_142_7, 0.37416475482496453, 0.0],
    [0.881_674_598_767_943_7, 0.471_857_925_532_024_3, 0.0],
    [0.914_093_882_414_285, 0.40550261914416735, 0.0],
    [0.950_534_146_061_684_1, 0.310_620_084_944_269_1, 0.0],
    [0.938_029_370_850_600_6, 0.346_555_766_683_554_9, 0.0],
    [0.981_143_685_900_016_7, 0.193_279_765_155_407_5, 0.0],
    [0.985_644_543_834_867_9, 0.16883374428281572, 0.0],
    [0.982_642_742_162_642, 0.185_508_062_560_858_7, 0.0],
    [0.994_760_572_889_064, 0.10223210173629921, 0.0],
    [0.979_472_589_916_838_3, 0.20157739357775506, 0.0],
    [0.994_095_011_652_429_2, 0.10851316882183903, 0.0],
    [0.963_714_928_210_760_9, 0.26693358189581157, 0.0],
    [0.992_605_599_809_550_5, 0.12138419677504364, 0.0],
    [0.992_605_599_801_575_4, -0.12138419684025958, 0.0],
    [0.963_714_928_210_761, -0.26693358189581146, 0.0],
    [0.994_095_011_622_181_2, -0.10851316909894139, 0.0],
    [0.979_472_589_878_047_6, -0.20157739376624062, 0.0],
    [0.994_760_572_862_196_8, -0.102_232_101_997_728_9, 0.0],
    [0.982_642_742_112_157, -0.18550806282828025, 0.0],
    [0.985_644_543_834_867_9, -0.16883374428281595, 0.0],
    [0.981_143_685_881_419_8, -0.193_279_765_249_810_9, 0.0],
    [0.938_029_370_850_600_5, -0.34655576668355503, 0.0],
    [0.950_534_146_061_684_2, -0.310_620_084_944_269, 0.0],
    [0.914_093_882_384_768_9, -0.405_502_619_210_702_9, 0.0],
    [0.881_674_598_767_943_7, -0.471_857_925_532_024_3, 0.0],
    [0.927_362_246_468_626_6, -0.374_164_754_920_425_7, 0.0],
    [0.864_005_952_520_259, -0.503_481_592_523_063_2, 0.0],
    [0.948_683_298_050_513_8, -0.316227766016838, 0.0],
    [0.9299293532683236, -0.36773821929459194, 0.0],
];

#[test]
fn zero_pair_distance_blocks_cancel() {
    // Fixture case 12: same dipole with maxPair = 0 admits no candidates,
    // so the pair survives and the field is bitwise untouched.
    let (positions, triangles) = grid(4, 4);
    let mut field = dipole_4x4_input();
    let input = field.clone();
    let mesh = SurfaceMesh::new(&positions, &triangles);
    let mut simp = SingularitySimplifier::new(&mesh, &mut field);
    simp.set_maximum_pair_distance(0);
    simp.set_sharp_edge_degrees(90.0);
    let mut expected_chi = vec![0; 25];
    expected_chi[11] = 1;
    expected_chi[13] = 3;
    assert_eq!(simp.vertex_charges(), expected_chi);
    simp.simplify();
    assert_eq!(simp.singularity_count_before(), 2);
    assert_eq!(simp.singularity_count_after(), 2);
    assert_eq!(simp.cancelled_pair_count(), 0);
    assert_eq!(simp.vertex_charges(), expected_chi);
    drop(simp);
    check_bits_unchanged("dipole-blocked", &field, &input);
}

#[test]
#[allow(clippy::approx_constant)] // Golden expected value, not the named constant.
fn tetra_defect_input_charges() {
    // Fixture case 23 (tetrahedron, face 1 rotated 45 degrees about its
    // normal): the tiny mesh admits no dipole path, so nothing cancels.
    // Input field transcribed from the fixture F line.
    let (positions, triangles) = tetra();
    let mesh = SurfaceMesh::new(&positions, &triangles);
    let mut field = vec![
        v(1.0, 0.0, 0.0),
        v(0.707_106_781_186_547_6, 0.0, 0.707_106_781_186_547_5),
        v(1.0, 0.0, 0.0),
        v(1.0, 0.0, 0.0),
    ];
    let mut simp = SingularitySimplifier::new(&mesh, &mut field);
    simp.set_maximum_pair_distance(6);
    simp.set_sharp_edge_degrees(90.0);
    assert_eq!(simp.vertex_charges(), vec![1, 2, 3, 2]);
    simp.simplify();
    assert_eq!(simp.singularity_count_before(), 3);
    assert_eq!(simp.singularity_count_after(), 3);
    assert_eq!(simp.cancelled_pair_count(), 0);
}
