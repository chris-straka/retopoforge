// Timing mirror for the autoremesher engine: regenerates the 64x64
// analytic grid (z = 0.1 * sin(i) * cos(j)) and runs the same
// target-2000 default-settings remesh() call 3x. Structural facts (ok,
// quad/vert counts, checksum within 1e-6) are pinned (determinism
// tripwire); the elapsed ms are the measurement. Run release:
// cargo test --release -p retopo_core --test auto_remesher_timing
use retopo_core::auto_remesher::AutoRemesher;
use retopo_core::vector3::Vector3;
use std::time::Instant;

fn timing_mesh() -> (Vec<Vector3>, Vec<Vec<usize>>) {
    const W: usize = 64;
    const H: usize = 64;
    let mut vertices = Vec::with_capacity((W + 1) * (H + 1));
    for j in 0..=H {
        for i in 0..=W {
            vertices.push(Vector3::new(
                i as f64,
                j as f64,
                0.1 * (i as f64).sin() * (j as f64).cos(),
            ));
        }
    }
    let mut triangles = Vec::with_capacity(2 * W * H);
    for j in 0..H {
        for i in 0..W {
            let a = j * (W + 1) + i;
            let b = a + 1;
            let c = a + W + 1;
            let d = c + 1;
            triangles.push(vec![a, b, d]);
            triangles.push(vec![a, d, c]);
        }
    }
    (vertices, triangles)
}

#[test]
fn timing_remesh() {
    let (vertices, triangles) = timing_mesh();
    assert_eq!(vertices.len(), 65 * 65);
    assert_eq!(triangles.len(), 2 * 64 * 64);
    // Bitwise mesh pins: same formula + same libm as the C++ side must
    // give these exact vertices (spot-checks at indices 0, 1000, last).
    assert_eq!(vertices[0].z().to_bits(), 0.0f64.to_bits());
    assert_eq!(
        vertices[1000].z().to_bits(),
        (0.1 * (25.0f64).sin() * (15.0f64).cos()).to_bits()
    );
    assert_eq!(
        vertices[65 * 65 - 1].z().to_bits(),
        (0.1 * (64.0f64).sin() * (64.0f64).cos()).to_bits()
    );
    // Warmup (unprinted), like the C++ side.
    {
        let mut warmup = AutoRemesher::new(&vertices, &triangles);
        warmup.set_target_triangle_count(2000);
        assert!(warmup.remesh(), "warmup remesh must succeed");
    }
    for sample in 0..3 {
        let mut remesher = AutoRemesher::new(&vertices, &triangles);
        remesher.set_target_triangle_count(2000);
        let t0 = Instant::now();
        let ok = remesher.remesh();
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        assert!(ok, "timing remesh must succeed");
        let quads = remesher.remeshed_quads().len();
        let verts = remesher.remeshed_vertices().len();
        assert_eq!(quads, 694, "timing quad count");
        assert_eq!(verts, 745, "timing vert count");
        let mut checksum = 0.0;
        for v in remesher.remeshed_vertices() {
            checksum += v.x() + v.y() + v.z();
        }
        // Pinned checksum for the same mesh: proves repeated runs
        // remesh the identical system (determinism tripwire).
        // Scale-aware 1e-6, like the oracle values.
        const PINNED_CHECKSUM: f64 = 47965.0223020472913;
        let tol = 1e-6 * PINNED_CHECKSUM.abs().max(1.0);
        assert!(
            (checksum - PINNED_CHECKSUM).abs() <= tol,
            "checksum diverged: rust={checksum:.17e} pinned={PINNED_CHECKSUM:.17e}"
        );
        eprintln!(
            "T rust sample={sample} ms={ms:.3} quads={quads} verts={verts} checksum={checksum:.17e}"
        );
    }
}
