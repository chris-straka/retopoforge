// Grid timing mirror: builds the bit-identical 200x200 triangulated grid
// with integer relief that the C++ surfacemesh_diff_dump tool times (pure
// formula, no RNG on either side), runs the same query sweep (construct +
// all normal angles + average edge length), and prints Rust-side ms for
// the calibration runtime ratio. Also asserts the sweep reproduces the
// C++ anglesum/avg bitwise (values from the SMESH1 fixture's T lines).
// Run release: cargo test --release --offline --test surface_mesh_timing
// -- --nocapture
use retopo_core::surface_mesh::SurfaceMesh;
use retopo_core::vector3::Vector3;
use std::time::Instant;

fn make_grid(w: usize, h: usize) -> (Vec<Vector3>, Vec<Vec<usize>>) {
    let mut positions = Vec::with_capacity((w + 1) * (h + 1));
    for y in 0..=h {
        for x in 0..=w {
            let z = 0.1 * ((x * 7 + y * 13) % 5) as f64;
            positions.push(Vector3::new(x as f64, y as f64, z));
        }
    }
    let mut triangles = Vec::with_capacity(w * h * 2);
    let id = |x: usize, y: usize| y * (w + 1) + x;
    for y in 0..h {
        for x in 0..w {
            let a = id(x, y);
            let b = id(x + 1, y);
            let cc = id(x + 1, y + 1);
            let d = id(x, y + 1);
            triangles.push(vec![a, b, cc]);
            triangles.push(vec![a, cc, d]);
        }
    }
    (positions, triangles)
}

#[test]
fn timing_grid() {
    let (positions, triangles) = make_grid(200, 200);
    assert_eq!(positions.len(), 40401);
    assert_eq!(triangles.len(), 80_000);
    for _ in 0..3 {
        let t0 = Instant::now();
        let mesh = SurfaceMesh::new(&positions, &triangles);
        let mut angle_sum = 0.0;
        for c in 0..mesh.corner_count() {
            angle_sum += mesh.normal_angle(c);
        }
        let avg = mesh.average_edge_length();
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        assert_eq!(mesh.vertex_count(), 40401);
        assert_eq!(mesh.face_count(), 80_000);
        // Bitwise cross-check against the C++ T lines in the fixture.
        assert_eq!(angle_sum.to_bits(), 2513.2744600469987f64.to_bits());
        assert_eq!(avg.to_bits(), 1.1573082100886585f64.to_bits());
        eprintln!(
            "T rust surfacemesh verts={} faces={} anglesum={angle_sum:.17e} avg={avg:.17e} ms={ms:.3}",
            mesh.vertex_count(),
            mesh.face_count(),
        );
    }
}
