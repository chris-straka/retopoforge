//! Timing mirror: builds the bit-identical 40x40 relief grid + two
//! fractional dipoles the C++ `singularitysimplifier_diff_dump` tool times
//! (pure formula, no RNG on either side) and prints Rust-side simplify ms
//! for the calibration runtime ratio. Run release: `cargo test --release
//! --offline --test singularity_simplifier_timing -- --nocapture`
//!
//! Input construction mirrors the dump tool exactly, including its
//! backend-fused `sincos` in the winding field (separate `cos`/`sin` here
//! would drift by 1 ulp on some of the 3200 faces and break the bitwise
//! xsum pin).
use retopo_core::double_utils::joint_sin_cos;
use retopo_core::singularity_simplifier::SingularitySimplifier;
use retopo_core::surface_mesh::SurfaceMesh;
use retopo_core::vector3::Vector3;
use std::time::Instant;

// Triangulated 40x40 grid with integer relief in 0.1 steps (mirrors the
// dump tool's makeGrid(40, 40, 1)).
fn relief_grid() -> (Vec<Vector3>, Vec<Vec<usize>>) {
    let (w, h) = (40usize, 40usize);
    let mut positions = Vec::new();
    for y in 0..=h {
        for x in 0..=w {
            let z = 0.1 * ((x * 7 + y * 13) % 5) as f64;
            positions.push(Vector3::new(x as f64, y as f64, z));
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

// Fractional-winding field over (center, strength) poles (mirrors the
// dump tool's windingField, including the fused sincos evaluation).
fn winding_field(
    positions: &[Vector3],
    triangles: &[Vec<usize>],
    poles: &[(Vector3, f64)],
) -> Vec<Vector3> {
    let mut field = Vec::new();
    for tri in triangles {
        let (a, b, c) = (&positions[tri[0]], &positions[tri[1]], &positions[tri[2]]);
        let (mx, my) = ((a.x() + b.x() + c.x()) / 3.0, (a.y() + b.y() + c.y()) / 3.0);
        let mut theta = 0.0;
        for (center, s) in poles {
            let (dx, dy) = (mx - center.x(), my - center.y());
            if dx == 0.0 && dy == 0.0 {
                continue;
            }
            theta += s * dy.atan2(dx) / 4.0;
        }
        let (s, c) = joint_sin_cos(theta);
        field.push(Vector3::new(c, s, 0.0));
    }
    field
}

#[test]
fn timing_dipoles() {
    let (positions, triangles) = relief_grid();
    let mesh = SurfaceMesh::new(&positions, &triangles);
    assert_eq!(mesh.face_count(), 3200);
    let at = |x: usize, y: usize| positions[y * 41 + x];
    let poles = [
        (at(10, 20), 1.0),
        (at(14, 20), -1.0),
        (at(26, 22), 1.0),
        (at(30, 22), -1.0),
    ];
    for _ in 0..3 {
        let mut field = winding_field(&positions, &triangles, &poles);
        let mut simp = SingularitySimplifier::new(&mesh, &mut field);
        let t0 = Instant::now();
        simp.simplify();
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        let (before, after, cancelled) = (
            simp.singularity_count_before(),
            simp.singularity_count_after(),
            simp.cancelled_pair_count(),
        );
        drop(simp);
        let mut xsum = 0.0;
        for f in &field {
            xsum += f.x();
        }
        // Pinned to the C++ T lines in
        // tests/fixtures/singularitysimplifier_diff.txt.
        assert_eq!(before, BEFORE);
        assert_eq!(after, AFTER);
        assert_eq!(cancelled, CANCELLED);
        assert_eq!(
            xsum.to_bits(),
            XSUM.to_bits(),
            "xsum drift: rust={xsum} cpp={XSUM}"
        );
        eprintln!(
            "T rust singsimp faces={} before={before} after={after} cancelled={cancelled} xsum={xsum:?} ms={ms:.3}",
            mesh.face_count()
        );
    }
}

// Pinned to the C++ T lines in
// tests/fixtures/singularitysimplifier_diff.txt.
const BEFORE: usize = 4;
const AFTER: usize = 0;
const CANCELLED: usize = 2;
const XSUM: f64 = 3_176.467_085_510_049;
