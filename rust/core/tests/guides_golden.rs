// Direct unit goldens for Guides::influence_radius / tangent_near.
//
// There are no direct C++ unit tests of the Guides queries to replicate
// (`tests/test_guides.cpp` exercises them only through FrameField and the
// end-to-end remesher), so these goldens assert independently hand-derived
// expectations: axis-aligned geometries whose midpoint projections,
// clamps, and normalizations are exact in binary FP, plus the exact
// radius-tie and first-wins selection rules. The randomized differential
// oracle (`guides_diff.rs`) proves bitwise agreement with C++ on top.
use retopo_core::guides::Guides;
use retopo_core::surface_mesh::SurfaceMesh;
use retopo_core::vector3::Vector3;

fn single_triangle() -> SurfaceMesh {
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    let triangles = vec![vec![0, 1, 2]];
    SurfaceMesh::new(&positions, &triangles)
}

#[test]
fn influence_radius_empty_mesh_is_zero() {
    let mesh = SurfaceMesh::new(&[], &[]);
    assert_eq!(Guides::influence_radius(&mesh), 0.0);
}

#[test]
fn influence_radius_is_six_edge_lengths() {
    let mesh = single_triangle();
    // Exactly 6x the average edge length (single rounding-free scaling
    // of the proven average_edge_length).
    assert_eq!(
        Guides::influence_radius(&mesh),
        6.0 * mesh.average_edge_length()
    );
    // Closed form: corner order visits edges 1, sqrt(2), 1, accumulated
    // left to right, then /3, then x6 — same op order, so bitwise equal.
    assert_eq!(
        Guides::influence_radius(&mesh),
        6.0 * ((1.0 + 2.0f64.sqrt() + 1.0) / 3.0)
    );
}

#[test]
fn tangent_near_empty_guides_is_zero() {
    let t = Guides::tangent_near(
        &[],
        &Vector3::new(0.25, 0.25, 0.0),
        &Vector3::new(0.0, 0.0, 1.0),
        6.0,
    );
    assert_eq!(t.as_array(), &[0.0, 0.0, 0.0]);
}

#[test]
fn tangent_near_nonpositive_radius_is_zero() {
    let guides = vec![vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
    ]];
    for radius in [0.0, -1.0, f64::NAN] {
        let t = Guides::tangent_near(
            &guides,
            &Vector3::new(1.0, 0.0, 0.0),
            &Vector3::new(0.0, 0.0, 1.0),
            radius,
        );
        assert_eq!(t.as_array(), &[0.0, 0.0, 0.0], "radius {radius}");
    }
}

#[test]
fn tangent_near_degenerate_only_is_zero() {
    let p = Vector3::new(1.0, 2.0, 3.0);
    let guides = vec![vec![], vec![p], vec![p, p, p]];
    let t = Guides::tangent_near(&guides, &p, &Vector3::new(0.0, 0.0, 1.0), 6.0);
    assert_eq!(t.as_array(), &[0.0, 0.0, 0.0]);
}

#[test]
fn tangent_near_on_segment_is_direction() {
    // Exact: along = 1, projection exact, dot with z-normal exact 0,
    // unit direction normalizes exactly.
    let guides = vec![vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
    ]];
    let t = Guides::tangent_near(
        &guides,
        &Vector3::new(1.0, 0.0, 0.0),
        &Vector3::new(0.0, 0.0, 1.0),
        5.0,
    );
    assert_eq!(t.as_array(), &[1.0, 0.0, 0.0]);
}

#[test]
fn tangent_near_clamps_past_endpoints() {
    // along = 5 clamps to length 1; distance 16 < 25 selects it.
    let guides = vec![vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
    ]];
    let t = Guides::tangent_near(
        &guides,
        &Vector3::new(5.0, 0.0, 0.0),
        &Vector3::new(0.0, 0.0, 1.0),
        5.0,
    );
    assert_eq!(t.as_array(), &[1.0, 0.0, 0.0]);
}

#[test]
fn tangent_near_exact_radius_tie_misses() {
    // Distance exactly 1, radius exactly 1: strict < misses -> zero.
    let guides = vec![vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
    ]];
    let t = Guides::tangent_near(
        &guides,
        &Vector3::new(1.0, 1.0, 0.0),
        &Vector3::new(0.0, 0.0, 1.0),
        1.0,
    );
    assert_eq!(t.as_array(), &[0.0, 0.0, 0.0]);
}

#[test]
fn tangent_near_equidistant_keeps_first() {
    // Both segments sit exactly 1.0 from the origin; strict < keeps A.
    let guides = vec![
        vec![Vector3::new(1.0, 1.0, 0.0), Vector3::new(-1.0, 1.0, 0.0)],
        vec![Vector3::new(1.0, -1.0, 0.0), Vector3::new(-1.0, -1.0, 0.0)],
    ];
    let t = Guides::tangent_near(
        &guides,
        &Vector3::default(),
        &Vector3::new(0.0, 0.0, 1.0),
        6.0,
    );
    assert_eq!(t.as_array(), &[-1.0, 0.0, 0.0]);
}

#[test]
fn tangent_near_parallel_normal_is_zero() {
    // Projection of (1,0,0) out of the x-axis vanishes exactly.
    let guides = vec![vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
    ]];
    let t = Guides::tangent_near(
        &guides,
        &Vector3::new(1.0, 0.0, 0.0),
        &Vector3::new(1.0, 0.0, 0.0),
        5.0,
    );
    assert_eq!(t.as_array(), &[0.0, 0.0, 0.0]);
}

#[test]
fn tangent_near_selects_nearest_not_first() {
    // Far segment first, near segment second: the Y segment wins.
    let guides = vec![
        vec![Vector3::new(5.0, 0.0, 0.0), Vector3::new(6.0, 0.0, 0.0)],
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 2.0, 0.0)],
    ];
    let t = Guides::tangent_near(
        &guides,
        &Vector3::new(0.1, 1.0, 0.0),
        &Vector3::new(0.0, 0.0, 1.0),
        6.0,
    );
    assert_eq!(t.as_array(), &[0.0, 1.0, 0.0]);
}
