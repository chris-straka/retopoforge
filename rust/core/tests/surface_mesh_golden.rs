// Replica goldens mirroring tests/test_surfacemesh.cpp 1:1 — same groups,
// same values, exact `==` where C++ uses it and `is_equal` where C++ uses
// it.
use retopo_core::double_utils::is_equal;
use retopo_core::surface_mesh::SurfaceMesh;
use retopo_core::vector3::Vector3;
use std::f64::consts::PI;

const NPOS: usize = SurfaceMesh::NPOS;

#[test]
fn single_triangle_all_boundary() {
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    let triangles = vec![vec![0, 1, 2]];
    let mesh = SurfaceMesh::new(&positions, &triangles);
    assert_eq!(mesh.vertex_count(), 3);
    assert_eq!(mesh.face_count(), 1);
    assert_eq!(mesh.corner_count(), 3);
    assert_eq!(mesh.corner_face(2), 0);
    assert_eq!(mesh.corner_local(2), 2);
    assert_eq!(mesh.corner_vertex(1), 1);
    assert_eq!(mesh.next_corner(2), 0);
    assert_eq!(mesh.previous_corner(0), 2);
    assert_eq!(mesh.triangle(0), &[0, 1, 2]);
    for c in 0..3 {
        assert_eq!(mesh.opposite_corner(c), NPOS);
        assert_eq!(mesh.adjacent_face(c), NPOS);
        assert!(mesh.is_boundary_corner(c));
        assert_eq!(mesh.normal_angle(c), PI);
    }
    assert_eq!(mesh.corners_around_vertex(0), &vec![0]);
    assert_eq!(mesh.corners_around_vertex(1), &vec![1]);
    assert_eq!(mesh.corners_around_vertex(2), &vec![2]);
    // Corner 0 runs v0 -> v1, i.e. the +X unit edge.
    let edge = mesh.edge_vector(0);
    assert_eq!(edge.x(), 1.0);
    assert_eq!(edge.y(), 0.0);
    assert_eq!(edge.z(), 0.0);
    // Counter-clockwise in XY: +Z normal.
    let normal = mesh.face_normal(0);
    assert_eq!(normal.x(), 0.0);
    assert_eq!(normal.y(), 0.0);
    assert_eq!(normal.z(), 1.0);
    // Edges 1, sqrt(2), 1.
    assert!(is_equal(
        mesh.average_edge_length(),
        (2.0 + 2.0f64.sqrt()) / 3.0
    ));
}

#[test]
fn unit_square_shared_diagonal() {
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    let triangles = vec![vec![0, 1, 2], vec![0, 2, 3]];
    let mesh = SurfaceMesh::new(&positions, &triangles);
    assert_eq!(mesh.vertex_count(), 4);
    assert_eq!(mesh.face_count(), 2);
    assert_eq!(mesh.corner_count(), 6);
    // Corner 2 (face 0 edge v2 -> v0) pairs with corner 3
    // (face 1 edge v0 -> v2); nothing else pairs.
    assert_eq!(mesh.opposite_corner(2), 3);
    assert_eq!(mesh.opposite_corner(3), 2);
    assert_eq!(mesh.adjacent_face(2), 1);
    assert_eq!(mesh.adjacent_face(3), 0);
    assert!(!mesh.is_boundary_corner(2));
    assert!(!mesh.is_boundary_corner(3));
    for c in [0, 1, 4, 5] {
        assert_eq!(mesh.opposite_corner(c), NPOS);
        assert_eq!(mesh.adjacent_face(c), NPOS);
        assert!(mesh.is_boundary_corner(c));
    }
    assert_eq!(mesh.corners_around_vertex(0), &vec![0, 3]);
    assert_eq!(mesh.corners_around_vertex(1), &vec![1]);
    assert_eq!(mesh.corners_around_vertex(2), &vec![2, 4]);
    assert_eq!(mesh.corners_around_vertex(3), &vec![5]);
    // Coplanar neighbours: zero dihedral angle across the diagonal.
    assert_eq!(mesh.normal_angle(2), 0.0);
    assert_eq!(mesh.normal_angle(3), 0.0);
    // Four unit boundary edges + one sqrt(2) diagonal.
    assert!(is_equal(
        mesh.average_edge_length(),
        (4.0 + 2.0f64.sqrt()) / 5.0
    ));
}

#[test]
fn closed_tetrahedron_no_boundary() {
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    ];
    let triangles = vec![vec![0, 2, 1], vec![0, 1, 3], vec![0, 3, 2], vec![1, 2, 3]];
    let mesh = SurfaceMesh::new(&positions, &triangles);
    assert_eq!(mesh.vertex_count(), 4);
    assert_eq!(mesh.face_count(), 4);
    assert_eq!(mesh.corner_count(), 12);
    for c in 0..12 {
        assert_ne!(mesh.opposite_corner(c), NPOS);
        assert_ne!(mesh.adjacent_face(c), NPOS);
        assert!(!mesh.is_boundary_corner(c));
        // Pairing is symmetric.
        assert_eq!(mesh.opposite_corner(mesh.opposite_corner(c)), c);
    }
    for v in 0..4 {
        assert_eq!(mesh.corners_around_vertex(v).len(), 3);
    }
    // Corner 0 is face 0's edge v0 -> v2; its reverse is corner 8
    // (face 2's edge v2 -> v0).
    assert_eq!(mesh.opposite_corner(0), 8);
    // Bottom face (z = 0, outward): -Z normal.
    let bottom = mesh.face_normal(0);
    assert_eq!(bottom.x(), 0.0);
    assert_eq!(bottom.y(), 0.0);
    assert_eq!(bottom.z(), -1.0);
    // Slanted face BCD: (1,1,1) normalized.
    let slanted = mesh.face_normal(3);
    let unit = 1.0 / 3.0f64.sqrt();
    assert!(is_equal(slanted.x(), unit));
    assert!(is_equal(slanted.y(), unit));
    assert!(is_equal(slanted.z(), unit));
    // Three unit edges + three sqrt(2) edges.
    assert!(is_equal(
        mesh.average_edge_length(),
        (3.0 + 3.0 * 2.0f64.sqrt()) / 6.0
    ));
}

#[test]
fn non_triangle_faces_skipped() {
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    let triangles = vec![vec![0, 1], vec![0, 1, 2, 0], vec![0, 1, 2]];
    let mesh = SurfaceMesh::new(&positions, &triangles);
    assert_eq!(mesh.face_count(), 1);
    assert_eq!(mesh.corner_count(), 3);
    assert_eq!(mesh.triangle(0), &[0, 1, 2]);
}

#[test]
fn empty_input_is_empty_mesh() {
    let mesh = SurfaceMesh::new(&[], &[]);
    assert_eq!(mesh.vertex_count(), 0);
    assert_eq!(mesh.face_count(), 0);
    assert_eq!(mesh.corner_count(), 0);
    assert_eq!(mesh.average_edge_length(), 0.0);
}
