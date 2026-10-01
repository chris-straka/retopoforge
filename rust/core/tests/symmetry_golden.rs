//! Replica goldens mirroring the unit groups of tests/test_symmetry.cpp 1:1
//! — same groups, same values, same thresholds.
//!
//! The three end-to-end groups (auto plane, explicit-plane fallback,
//! default off) need `AutoRemesher` and are not portable: no engine port
//! exists on this branch. Everything else mirrors exactly, including the
//! welded-cube builder.
use retopo_core::symmetry::{Symmetry, SymmetryPlane};
use retopo_core::vector3::Vector3;
use std::collections::BTreeMap;

/// Subdivided cube centered at the origin, welded across face edges. With
/// `bumpy`, vertices are displaced radially by f(x, z) = 0.1*sin(2x+0.5) *
/// sin(2z+0.3), which keeps exact Y symmetry while breaking X and Z
/// symmetry (mirrors `buildCube`).
fn build_cube(subdivisions: i32, bumpy: bool) -> (Vec<Vector3>, Vec<Vec<usize>>) {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    let mut welded: BTreeMap<(i64, i64, i64), usize> = BTreeMap::new();
    let mut vertex_index = |point: Vector3| -> usize {
        // llround rounds halfway away from zero, like f64::round.
        let key = (
            (point.x() * 1e9).round() as i64,
            (point.y() * 1e9).round() as i64,
            (point.z() * 1e9).round() as i64,
        );
        *welded.entry(key).or_insert_with(|| {
            vertices.push(point);
            vertices.len() - 1
        })
    };
    // (fixedAxis, fixedSign, uAxis, vAxis) per face; (u, v) ordered so that
    // every face winds outward (single connected island).
    let faces: [[i32; 4]; 6] = [
        [0, 1, 1, 2],
        [0, -1, 2, 1],
        [1, 1, 2, 0],
        [1, -1, 0, 2],
        [2, 1, 0, 1],
        [2, -1, 1, 0],
    ];
    for face in &faces {
        let mut grid = vec![vec![0usize; subdivisions as usize + 1]; subdivisions as usize + 1];
        for i in 0..=subdivisions {
            for j in 0..=subdivisions {
                let mut point = Vector3::default();
                point[face[0] as usize] = face[1] as f64;
                point[face[2] as usize] = -1.0 + 2.0 * i as f64 / subdivisions as f64;
                point[face[3] as usize] = -1.0 + 2.0 * j as f64 / subdivisions as f64;
                if bumpy {
                    let bump = 0.1 * (2.0 * point.x() + 0.5).sin() * (2.0 * point.z() + 0.3).sin();
                    point = point + point.normalized() * bump;
                }
                grid[i as usize][j as usize] = vertex_index(point);
            }
        }
        for i in 0..subdivisions {
            for j in 0..subdivisions {
                let a = grid[i as usize][j as usize];
                let b = grid[(i + 1) as usize][j as usize];
                let c = grid[(i + 1) as usize][(j + 1) as usize];
                let d = grid[i as usize][(j + 1) as usize];
                triangles.push(vec![a, b, c]);
                triangles.push(vec![a, c, d]);
            }
        }
    }
    (vertices, triangles)
}

fn bounding_diagonal(points: &[Vector3]) -> f64 {
    let mut lower = points[0];
    let mut upper = points[0];
    for point in points {
        for i in 0..3 {
            lower[i] = lower[i].min(point[i]);
            upper[i] = upper[i].max(point[i]);
        }
    }
    (upper - lower).length()
}

/// Largest distance from any vertex's mirror to its nearest vertex.
fn max_mirror_deviation(points: &[Vector3], plane: &SymmetryPlane) -> f64 {
    let mut worst = 0.0f64;
    for point in points {
        let mirrored = Symmetry::mirror_point(point, plane);
        let mut best = -1.0f64;
        for other in points {
            let distance = (*other - mirrored).length();
            if best < 0.0 || distance < best {
                best = distance;
            }
        }
        worst = worst.max(best);
    }
    worst
}

#[test]
fn mirror_point_direction_basics() {
    let plane = SymmetryPlane {
        axis: 1,
        offset: 2.0,
        score: 1.0,
    };
    assert!(plane.valid());
    let mirrored = Symmetry::mirror_point(&Vector3::new(1.0, 5.0, 3.0), &plane);
    assert_eq!((mirrored.x(), mirrored.y(), mirrored.z()), (1.0, -1.0, 3.0));
    let direction = Symmetry::mirror_direction(&Vector3::new(1.0, 1.0, 0.0), &plane);
    assert_eq!(
        (direction.x(), direction.y(), direction.z()),
        (1.0, -1.0, 0.0)
    );
    assert!(!SymmetryPlane::default().valid());
    let identity = Symmetry::mirror_point(&Vector3::new(1.0, 2.0, 3.0), &SymmetryPlane::default());
    assert!((identity - Vector3::new(1.0, 2.0, 3.0)).length() == 0.0);
}

#[test]
fn detect_plain_cube_all_axes_tie_to_y() {
    let (cube_vertices, cube_triangles) = build_cube(4, false);
    assert!(!cube_vertices.is_empty() && !cube_triangles.is_empty());
    let detected = Symmetry::detect_plane(&cube_vertices);
    assert!(detected.valid());
    assert!(detected.score > 0.999);
    assert!(detected.offset.abs() < 1e-12);
    // Ties prefer Y, then Z, then X.
    assert_eq!(detected.axis, 1);
    let fixed = Symmetry::fixed_plane(&cube_vertices, 2);
    assert!(fixed.axis == 2 && fixed.score > 0.999);
    assert!(!Symmetry::fixed_plane(&cube_vertices, 7).valid());
    assert!(!Symmetry::detect_plane(&[]).valid());
}

#[test]
fn detect_bumpy_cube_y_only() {
    let (bumpy_vertices, _bumpy_triangles) = build_cube(4, true);
    let detected = Symmetry::detect_plane(&bumpy_vertices);
    assert_eq!(detected.axis, 1);
    assert!(detected.score > 0.999);
    let diagonal = bounding_diagonal(&bumpy_vertices);
    let tolerance = (0.01 * diagonal).max(1e-9);
    assert!(Symmetry::score_plane(&bumpy_vertices, 0, 0.0, tolerance) < 0.75);
    assert!(Symmetry::score_plane(&bumpy_vertices, 2, 0.0, tolerance) < 0.75);
}

#[test]
fn detect_sheared_half_breaks_every_plane() {
    let (bumpy_vertices, _bumpy_triangles) = build_cube(4, true);
    let mut sheared = bumpy_vertices;
    for vertex in &mut sheared {
        if vertex.x() > 0.0 {
            vertex[1] += 0.3;
        }
    }
    let detected = Symmetry::detect_plane(&sheared);
    assert!(detected.score < 0.75);
}

#[test]
fn symmetrize_vertices_exact_pairing() {
    let (bumpy_vertices, _bumpy_triangles) = build_cube(4, true);
    let mut perturbed = bumpy_vertices.clone();
    for vertex in &mut perturbed {
        if vertex.y() > 0.0 {
            vertex[1] += 0.01;
        }
    }
    let plane = SymmetryPlane {
        axis: 1,
        offset: 0.0,
        score: 1.0,
    };
    assert!(max_mirror_deviation(&perturbed, &plane) > 1e-6);
    Symmetry::symmetrize_vertices(&mut perturbed, &plane);
    assert_eq!(perturbed.len(), bumpy_vertices.len());
    assert!(max_mirror_deviation(&perturbed, &plane) < 1e-9);
    // An invalid plane is a no-op.
    let before = perturbed.clone();
    Symmetry::symmetrize_vertices(&mut perturbed, &SymmetryPlane::default());
    assert_eq!(perturbed.len(), before.len());
    assert!(max_mirror_deviation(&perturbed, &plane) < 1e-9);
}

#[test]
fn symmetrize_frame_field_cross_average() {
    let vertices = vec![
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(0.0, 1.0, 1.0),
        Vector3::new(0.0, -1.0, 0.0),
        Vector3::new(1.0, -1.0, 0.0),
        Vector3::new(0.0, -1.0, 1.0),
    ];
    let triangles = vec![vec![0, 1, 2], vec![3, 4, 5]];
    let mut field = vec![Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)];
    let plane = SymmetryPlane {
        axis: 1,
        offset: 0.0,
        score: 1.0,
    };
    Symmetry::symmetrize_frame_field(&vertices, &triangles, &mut field, &plane);
    assert!((field[0] - Vector3::new(1.0, 0.0, 0.0)).length() < 1e-9);
    assert!((field[1] - Symmetry::mirror_direction(&field[0], &plane)).length() < 1e-9);
    // No-op on size mismatch or an invalid plane.
    let mut untouched = vec![Vector3::new(1.0, 0.0, 0.0)];
    Symmetry::symmetrize_frame_field(&vertices, &triangles, &mut untouched, &plane);
    assert!(untouched.len() == 1 && untouched[0].x() == 1.0);
}
