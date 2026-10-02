// Harness indexes parallel arrays/cursors in lockstep; index loops stay.
#![allow(clippy::needless_range_loop)]

// Replicated goldens: mirrors the FrameField-level test groups of
// tests/test_guides.cpp and tests/test_sharp.cpp 1:1 at the same tolerance
// (engine end-to-end groups are out of scope for this lane). All builders
// mirror the C++ helpers exactly (same winding, same coordinates).
use retopo_core::frame_field::FrameField;
use retopo_core::surface_mesh::SurfaceMesh;
use retopo_core::vector3::Vector3;
use std::f64::consts::PI;

fn build_sphere(lat_bands: i32, lon_bands: i32, radius: f64) -> (Vec<Vector3>, Vec<Vec<usize>>) {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    vertices.push(Vector3::new(0.0, radius, 0.0));
    for j in 1..lat_bands {
        let theta = PI * f64::from(j) / f64::from(lat_bands);
        let y = radius * theta.cos();
        let ring_radius = radius * theta.sin();
        for i in 0..lon_bands {
            let phi = 2.0 * PI * f64::from(i) / f64::from(lon_bands);
            vertices.push(Vector3::new(
                ring_radius * phi.cos(),
                y,
                ring_radius * phi.sin(),
            ));
        }
    }
    let south_pole = vertices.len();
    vertices.push(Vector3::new(0.0, -radius, 0.0));
    let ring_index =
        |j: i32, i: i32| 1 + ((j - 1) * lon_bands + (i + lon_bands) % lon_bands) as usize;
    for i in 0..lon_bands {
        triangles.push(vec![0, ring_index(1, i + 1), ring_index(1, i)]);
    }
    for j in 1..lat_bands - 1 {
        for i in 0..lon_bands {
            let a = ring_index(j, i);
            let b = ring_index(j, i + 1);
            let cc = ring_index(j + 1, i + 1);
            let d = ring_index(j + 1, i);
            triangles.push(vec![a, b, d]);
            triangles.push(vec![b, cc, d]);
        }
    }
    for i in 0..lon_bands {
        triangles.push(vec![
            south_pole,
            ring_index(lat_bands - 1, i),
            ring_index(lat_bands - 1, i + 1),
        ]);
    }
    (vertices, triangles)
}

fn equatorial_guide(radius: f64, points: i32) -> Vec<Vector3> {
    let mut guide = Vec::new();
    for i in 0..=points {
        let phi = 2.0 * PI * f64::from(i) / f64::from(points);
        guide.push(Vector3::new(radius * phi.cos(), 0.0, radius * phi.sin()));
    }
    guide
}

fn equatorial_tangent(point: &Vector3) -> Vector3 {
    Vector3::new(-point.z(), 0.0, point.x()).normalized()
}

fn build_box() -> (Vec<Vector3>, Vec<Vec<usize>>) {
    let vertices = vec![
        Vector3::new(-1.0, -1.0, -1.0),
        Vector3::new(1.0, -1.0, -1.0),
        Vector3::new(1.0, 1.0, -1.0),
        Vector3::new(-1.0, 1.0, -1.0),
        Vector3::new(-1.0, -1.0, 1.0),
        Vector3::new(1.0, -1.0, 1.0),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(-1.0, 1.0, 1.0),
    ];
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

fn vertical_sharps() -> Vec<Vec<Vector3>> {
    vec![
        vec![
            Vector3::new(-1.0, -1.0, -1.0),
            Vector3::new(-1.0, 1.0, -1.0),
        ],
        vec![Vector3::new(1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, -1.0)],
        vec![Vector3::new(-1.0, -1.0, 1.0), Vector3::new(-1.0, 1.0, 1.0)],
        vec![Vector3::new(1.0, -1.0, 1.0), Vector3::new(1.0, 1.0, 1.0)],
    ]
}

fn diagonal_guide() -> Vec<Vector3> {
    vec![Vector3::new(2.0, -1.0, -1.0), Vector3::new(2.0, 1.0, 1.0)]
}

fn cross_misalignment_degrees(direction: &Vector3, tangent: &Vector3) -> f64 {
    let cosine = (0.0f64).max(
        Vector3::dot_product(&direction.normalized(), &tangent.normalized())
            .abs()
            .min(1.0),
    );
    let angle = cosine.acos() * 180.0 / PI;
    angle.min(90.0 - angle)
}

fn max_position_difference(a: &[Vector3], b: &[Vector3]) -> f64 {
    if a.len() != b.len() {
        return -1.0;
    }
    let mut worst = 0.0f64;
    for i in 0..a.len() {
        worst = worst.max((a[i] - b[i]).length());
    }
    worst
}

fn junk_lines() -> Vec<Vec<Vector3>> {
    vec![
        vec![],
        vec![Vector3::new(1.0, 0.0, 0.0)],
        vec![Vector3::new(0.0, 0.0, 1.0), Vector3::new(0.0, 0.0, 1.0)],
        vec![Vector3::new(100.0, 0.0, 0.0), Vector3::new(101.0, 0.0, 0.0)],
    ]
}

// test_guides.cpp group 1: faces near the guide lock to the guide tangent.
#[test]
fn guide_band_locks_to_guide_tangent() {
    let (vertices, triangles) = build_sphere(16, 32, 1.0);
    assert!(!vertices.is_empty() && !triangles.is_empty());
    for t in &triangles {
        let normal = Vector3::normal(&vertices[t[0]], &vertices[t[1]], &vertices[t[2]]);
        assert!(normal.length() > 0.5);
        let centroid = (vertices[t[0]] + vertices[t[1]] + vertices[t[2]]) / 3.0;
        assert!(Vector3::dot_product(&normal, &centroid) > 0.0);
    }
    let guides = vec![equatorial_guide(1.0, 64)];
    let mesh = SurfaceMesh::new(&vertices, &triangles);
    let field = FrameField::create(&mesh, 90.0, &guides, &[]).expect("must solve");
    assert_eq!(field.len(), mesh.face_count());
    let radius = 2.0 * mesh.average_edge_length();
    let mut locked = 0usize;
    let mut worst = 0.0f64;
    for f in 0..mesh.face_count() {
        let t = mesh.triangle(f);
        let centroid = (*mesh.position(t[0]) + *mesh.position(t[1]) + *mesh.position(t[2])) / 3.0;
        if centroid.y().abs() > radius {
            continue;
        }
        locked += 1;
        worst = worst.max(cross_misalignment_degrees(
            &field[f],
            &equatorial_tangent(&centroid),
        ));
    }
    eprintln!("guide band: {locked} faces within {radius:.4} of equator, worst {worst:.3} deg");
    assert!(locked > 0);
    assert!(worst < 2.0, "worst misalignment {worst:.3} deg");
}

// test_guides.cpp group 2: degenerate-only guides are a no-op.
#[test]
fn degenerate_guides_are_noop() {
    let (vertices, triangles) = build_sphere(16, 32, 1.0);
    let mesh = SurfaceMesh::new(&vertices, &triangles);
    let plain = FrameField::create(&mesh, 90.0, &[], &[]).expect("must solve");
    let junk = junk_lines();
    let degenerate = FrameField::create(&mesh, 90.0, &junk, &[]).expect("must solve");
    assert_eq!(plain.len(), degenerate.len());
    let junk_diff = max_position_difference(&plain, &degenerate);
    let plain_again = FrameField::create(&mesh, 90.0, &[], &[]).expect("must solve");
    let rerun_diff = max_position_difference(&plain, &plain_again);
    eprintln!("degenerate-vs-plain {junk_diff:.3e}, rerun-vs-plain {rerun_diff:.3e}");
    assert!((0.0..1e-9).contains(&junk_diff));
    // Sequential port: reruns are bitwise identical (the C++ only asserts
    // the no-op bound, but determinism is a structural fact worth pinning).
    assert_eq!(rerun_diff, 0.0);
}

// test_sharp.cpp group 1: side faces near a vertical sharp lock to vertical.
#[test]
fn sharp_box_side_faces_lock_to_vertical() {
    let (vertices, triangles) = build_box();
    for t in &triangles {
        let normal = Vector3::normal(&vertices[t[0]], &vertices[t[1]], &vertices[t[2]]);
        assert!(normal.length() > 0.5);
        let centroid = (vertices[t[0]] + vertices[t[1]] + vertices[t[2]]) / 3.0;
        assert!(Vector3::dot_product(&normal, &centroid) > 0.0);
    }
    let sharps = vertical_sharps();
    let vertical = Vector3::new(0.0, 1.0, 0.0);
    let mesh = SurfaceMesh::new(&vertices, &triangles);
    let field = FrameField::create(&mesh, 90.0, &[], &sharps).expect("must solve");
    assert_eq!(field.len(), mesh.face_count());
    let mut side_faces = 0usize;
    let mut worst = 0.0f64;
    for f in 0..mesh.face_count() {
        if mesh.face_normal(f).y().abs() > 0.5 {
            continue;
        }
        side_faces += 1;
        worst = worst.max(cross_misalignment_degrees(&field[f], &vertical));
    }
    eprintln!("sharp box: {side_faces} side faces, worst {worst:.3} deg");
    assert_eq!(side_faces, 8);
    assert!(worst < 2.0, "worst misalignment {worst:.3} deg");
}

// test_sharp.cpp group 2: sharps win ties over guides.
#[test]
fn sharps_win_ties_over_guides() {
    let (vertices, triangles) = build_box();
    let guides = vec![diagonal_guide()];
    let sharps = vertical_sharps();
    let vertical = Vector3::new(0.0, 1.0, 0.0);
    let mesh = SurfaceMesh::new(&vertices, &triangles);
    let sharp_wins = FrameField::create(&mesh, 90.0, &guides, &sharps).expect("must solve");
    let guide_only = FrameField::create(&mesh, 90.0, &guides, &[]).expect("must solve");
    let mut side_faces = 0usize;
    let mut worst_sharp = 0.0f64;
    let mut guide_differs = false;
    for f in 0..mesh.face_count() {
        if mesh.face_normal(f).y().abs() > 0.5 {
            continue;
        }
        side_faces += 1;
        worst_sharp = worst_sharp.max(cross_misalignment_degrees(&sharp_wins[f], &vertical));
        if cross_misalignment_degrees(&guide_only[f], &vertical) > 20.0 {
            guide_differs = true;
        }
    }
    eprintln!(
        "tie: {side_faces} side faces, sharp+guide worst {worst_sharp:.3} deg, guide-only claims one: {guide_differs}"
    );
    assert_eq!(side_faces, 8);
    assert!(worst_sharp < 2.0);
    assert!(guide_differs);
}

// test_sharp.cpp group 3: degenerate-only sharps are a no-op.
#[test]
fn degenerate_sharps_are_noop() {
    let (vertices, triangles) = build_box();
    let mesh = SurfaceMesh::new(&vertices, &triangles);
    let plain = FrameField::create(&mesh, 90.0, &[], &[]).expect("must solve");
    let junk = junk_lines();
    let degenerate = FrameField::create(&mesh, 90.0, &[], &junk).expect("must solve");
    assert_eq!(plain.len(), degenerate.len());
    let junk_diff = max_position_difference(&plain, &degenerate);
    eprintln!("degenerate-vs-plain field diff {junk_diff:.3e}");
    assert!((0.0..1e-9).contains(&junk_diff));
}

// Early-false path: an empty mesh has no field.
#[test]
fn empty_mesh_returns_none() {
    let mesh = SurfaceMesh::new(&[], &[]);
    assert!(FrameField::create(&mesh, 90.0, &[], &[]).is_none());
    let mesh = SurfaceMesh::new(&[Vector3::new(0.0, 0.0, 0.0)], &[]);
    assert!(FrameField::create(&mesh, 90.0, &[], &[]).is_none());
}
