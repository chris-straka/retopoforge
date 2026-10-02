// Query-batch timing mirror: rebuilds the bit-identical 120x120 grid,
// 4x500-point zigzag guides, and 2000 query points/normals that the C++
// guides_diff_dump tool times (pure integer formulas, no libm on either
// side), runs the same sweep (one influence radius + 2000 tangent
// queries with alternating downstream/plausible radii), and prints
// Rust-side ms for the calibration runtime ratio. Also asserts the sweep
// reproduces the C++ influence/checksum bitwise (values from the GUIDES1
// fixture's T lines).
// Run release: cargo test --release --offline --test guides_timing
// -- --nocapture
use retopo_core::guides::Guides;
use retopo_core::surface_mesh::SurfaceMesh;
use retopo_core::vector3::Vector3;
use std::time::Instant;

/// Grid positions, triangles, zigzag guides, query points, query normals.
type GuideInputs = (
    Vec<Vector3>,
    Vec<Vec<usize>>,
    Vec<Vec<Vector3>>,
    Vec<Vector3>,
    Vec<Vector3>,
);

fn make_inputs() -> GuideInputs {
    let (w, h) = (120usize, 120usize);
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
            triangles.push(vec![id(x, y), id(x + 1, y), id(x + 1, y + 1)]);
            triangles.push(vec![id(x, y), id(x + 1, y + 1), id(x, y + 1)]);
        }
    }
    let mut guides = Vec::with_capacity(4);
    for line in 0..4 {
        let mut polyline = Vec::with_capacity(500);
        for j in 0..500 {
            polyline.push(Vector3::new(
                0.5 * j as f64,
                2.0 * (j % 2) as f64 + line as f64,
                1.0 * (j % 7) as f64,
            ));
        }
        guides.push(polyline);
    }
    let mut queries = Vec::with_capacity(2000);
    let mut normals = Vec::with_capacity(2000);
    for i in 0..2000 {
        queries.push(Vector3::new(
            0.25 * (i % 400) as f64,
            0.5 * (i % 11) as f64,
            0.25 * (i % 13) as f64,
        ));
        normals.push(Vector3::new((i % 5) as f64 - 2.0, (i % 3) as f64 - 1.0, 1.0).normalized());
    }
    (positions, triangles, guides, queries, normals)
}

#[test]
fn timing_batch() {
    let (positions, triangles, guides, queries, normals) = make_inputs();
    assert_eq!(positions.len(), 14641);
    assert_eq!(triangles.len(), 28_800);
    assert_eq!(guides.len(), 4);
    assert_eq!(queries.len(), 2000);
    let mesh = SurfaceMesh::new(&positions, &triangles);
    let influence = Guides::influence_radius(&mesh);
    // Bitwise cross-check against the C++ T lines in the fixture.
    assert_eq!(influence.to_bits(), 6.9421544944560445f64.to_bits());
    for _ in 0..3 {
        let t0 = Instant::now();
        let mut checksum = influence;
        for (i, query) in queries.iter().enumerate() {
            let radius = if i % 2 == 0 { influence } else { 2.0 };
            let t = Guides::tangent_near(&guides, query, &normals[i], radius);
            checksum += t.x() + t.y() + t.z();
        }
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        assert_eq!(checksum.to_bits(), 53.649163440233195f64.to_bits());
        eprintln!(
            "T rust guides queries={} checksum={checksum:.17e} ms={ms:.3}",
            queries.len()
        );
    }
}
