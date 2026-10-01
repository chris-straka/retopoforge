//! Timing mirror: builds the bit-identical 80x80 bumpy grid the C++
//! `isoremesh_diff_dump` tool times (pure formula, no RNG on either side)
//! and prints Rust-side remesh ms for the calibration runtime ratio. Run
//! release: `cargo test --release --offline --test isotropic_remesher_timing
//! -- --nocapture`
use retopo_core::isotropic_remesher::IsotropicRemesher;
use retopo_core::vector3::Vector3;
use std::time::Instant;

fn timing_verts(w: usize, h: usize) -> Vec<Vector3> {
    let mut verts = Vec::new();
    for y in 0..=h {
        for x in 0..=w {
            let z = ((x * y) % 7) as f64 * 0.05;
            verts.push(Vector3::new(x as f64, y as f64, z));
        }
    }
    verts
}

fn grid_tris(w: usize, h: usize) -> Vec<Vec<usize>> {
    let mut tris = Vec::new();
    let id = |x: usize, y: usize| y * (w + 1) + x;
    for y in 0..h {
        for x in 0..w {
            let a = id(x, y);
            let b = id(x + 1, y);
            let c = id(x + 1, y + 1);
            let d = id(x, y + 1);
            tris.push(vec![a, b, c]);
            tris.push(vec![a, c, d]);
        }
    }
    tris
}

#[test]
fn timing_grid() {
    let vertices = timing_verts(80, 80);
    let triangles = grid_tris(80, 80);
    assert_eq!(vertices.len(), 6561);
    assert_eq!(triangles.len(), 12800);
    for _ in 0..3 {
        let mut remesher = IsotropicRemesher::new(&vertices, &triangles);
        let t0 = Instant::now();
        let ok = remesher.remesh();
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        assert!(ok);
        let outv = remesher.remeshed_vertices().len();
        let outt = remesher.remeshed_triangles().len();
        // Pinned to the C++ T lines in tests/fixtures/isoremesh_diff.txt.
        assert_eq!(outv, OUTV);
        assert_eq!(outt, OUTT);
        eprintln!(
            "T rust isoremesh nv={} nt={} outv={outv} outt={outt} ok={ok} ms={ms:.3}",
            vertices.len(),
            triangles.len()
        );
    }
}

// Pinned to the C++ T lines in tests/fixtures/isoremesh_diff.txt: the
// near-uniform grid remeshes to itself (all passes reject / project back).
const OUTV: usize = 6561;
const OUTT: usize = 12800;
