// Golden tests for the quad_extractor port: tiny hand-built inputs with
// outputs committed below, verified against the C++ dump tool
// (tests/quadextractor_diff_dump.cpp cases 0-4 produce these exact
// values; see tests/fixtures/quadextractor_diff.txt).
use retopo_core::quad_extractor::QuadExtractor;
use retopo_core::vector2::Vector2;
use retopo_core::vector3::Vector3;

fn single_triangle() -> (Vec<Vector3>, Vec<Vec<usize>>, Vec<Vec<Vector2>>) {
    (
        vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(0.0, 2.0, 0.0),
        ],
        vec![vec![0, 1, 2]],
        vec![vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(2.0, 0.0),
            Vector2::new(0.0, 2.0),
        ]],
    )
}

fn conn_points(conns: &[(Vector3, Vector3)]) -> Vec<[f64; 6]> {
    conns
        .iter()
        .map(|(a, b)| [a.x(), a.y(), a.z(), b.x(), b.y(), b.z()])
        .collect()
}

#[test]
fn golden_empty_mesh() {
    let mut extractor = QuadExtractor::new(&[], &[], &[]);
    assert!(extractor.extract());
    assert!(extractor.extracted_connections().is_empty());
    assert!(extractor.extracted_connection_moved().is_empty());
    assert!(extractor.remeshed_vertices().is_empty());
    assert!(extractor.remeshed_quads().is_empty());
    assert!(extractor.remeshed_vertex_uvs().is_empty());
}

#[test]
fn golden_single_triangle() {
    let (vertices, triangles, uvs) = single_triangle();
    let mut extractor = QuadExtractor::new(&vertices, &triangles, &uvs);
    extractor.set_compute_vertex_uvs(true);
    assert!(extractor.extract());
    // The u=1 and v=1 isolines cut the triangle into a 6-segment grid;
    // one triangle cannot close a quad, so the remesh stays empty.
    assert_eq!(
        conn_points(extractor.extracted_connections()),
        vec![
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 2.0, 0.0],
            [0.0, 1.0, 0.0, 1.0, 1.0, 0.0],
            [1.0, 0.0, 0.0, 1.0, 1.0, 0.0],
            [1.0, 0.0, 0.0, 2.0, 0.0, 0.0],
        ]
    );
    assert_eq!(extractor.extracted_connection_moved(), &[0, 0, 0, 0, 0, 0]);
    assert!(extractor.remeshed_vertices().is_empty());
    assert!(extractor.remeshed_quads().is_empty());
    assert!(extractor.remeshed_vertex_uvs().is_empty());
}

#[test]
fn golden_zero_area_triangle() {
    let vertices = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 2.0, 0.0),
    ];
    let triangles = vec![vec![0, 1, 2]];
    let uvs = vec![vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(2.0, 0.0),
        Vector2::new(0.0, 2.0),
    ]];
    let mut extractor = QuadExtractor::new(&vertices, &triangles, &uvs);
    assert!(extractor.extract());
    // The collapsed corner contributes no segment; only the surviving
    // edge's isoline crossing remains.
    assert_eq!(
        conn_points(extractor.extracted_connections()),
        vec![
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 2.0, 0.0],
        ]
    );
    assert_eq!(extractor.extracted_connection_moved(), &[0, 0]);
    assert!(extractor.remeshed_vertices().is_empty());
    assert!(extractor.remeshed_quads().is_empty());
}

#[test]
fn golden_collapsed_uvs() {
    let (vertices, triangles, _) = single_triangle();
    let uvs = vec![vec![
        Vector2::new(0.5, 0.5),
        Vector2::new(0.5, 0.5),
        Vector2::new(0.5, 0.5),
    ]];
    let mut extractor = QuadExtractor::new(&vertices, &triangles, &uvs);
    extractor.set_compute_vertex_uvs(true);
    assert!(extractor.extract());
    // No isoline crosses a collapsed parameterization.
    assert!(extractor.extracted_connections().is_empty());
    assert!(extractor.remeshed_vertices().is_empty());
    assert!(extractor.remeshed_quads().is_empty());
    assert!(extractor.remeshed_vertex_uvs().is_empty());
}

#[test]
fn golden_two_disjoint_triangles() {
    let (mut vertices, mut triangles, mut uvs) = single_triangle();
    vertices.extend([
        Vector3::new(5.0, 5.0, 0.0),
        Vector3::new(7.0, 5.0, 0.0),
        Vector3::new(5.0, 7.0, 0.0),
    ]);
    triangles.push(vec![3, 4, 5]);
    uvs.push(vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(2.0, 0.0),
        Vector2::new(0.0, 2.0),
    ]);
    let mut extractor = QuadExtractor::new(&vertices, &triangles, &uvs);
    assert!(extractor.extract());
    // Each island extracts its own 6 connections in position order.
    assert_eq!(
        conn_points(extractor.extracted_connections()),
        vec![
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 2.0, 0.0],
            [0.0, 1.0, 0.0, 1.0, 1.0, 0.0],
            [1.0, 0.0, 0.0, 1.0, 1.0, 0.0],
            [1.0, 0.0, 0.0, 2.0, 0.0, 0.0],
            [5.0, 5.0, 0.0, 5.0, 6.0, 0.0],
            [5.0, 5.0, 0.0, 6.0, 5.0, 0.0],
            [5.0, 6.0, 0.0, 5.0, 7.0, 0.0],
            [5.0, 6.0, 0.0, 6.0, 6.0, 0.0],
            [6.0, 5.0, 0.0, 6.0, 6.0, 0.0],
            [6.0, 5.0, 0.0, 7.0, 5.0, 0.0],
        ]
    );
    assert!(extractor.remeshed_vertices().is_empty());
    assert!(extractor.remeshed_quads().is_empty());
}
