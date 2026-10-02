// Timing mirror for the parameterizer: regenerates the bit-identical 64x64
// analytic grid the C++ parameterizer_diff_dump tool times (z = 0.1 * sin(i) *
// cos(j); same libm calls, same order) and runs the same default-settings
// parameterize() call. Structural facts (ok, face count, singular count,
// checksum within 1e-6) must match the C++ T lines; only the elapsed ms
// are compared across sides. Run release:
// cargo test --release -p retopo_core --test parameterizer_timing
use retopo_core::parameterizer::Parameterizer;
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
fn timing_parameterize() {
    let (vertices, triangles) = timing_mesh();
    assert_eq!(vertices.len(), 65 * 65);
    assert_eq!(triangles.len(), 2 * 64 * 64);
    // Bitwise mesh pins: same formula + same libm as the C++ side must give
    // these exact vertices (spot-checks at indices 0, 1000, last).
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
        let mut warmup = Parameterizer::new(&vertices, &triangles, None);
        assert!(warmup.parameterize(), "warmup solve must succeed");
    }
    for sample in 0..3 {
        let mut parameterizer = Parameterizer::new(&vertices, &triangles, None);
        let t0 = Instant::now();
        let ok = parameterizer.parameterize();
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        assert!(ok, "timing solve must succeed");
        let uvs = parameterizer.take_triangle_uvs().expect("took UVs");
        assert_eq!(uvs.len(), 8192);
        assert_eq!(
            parameterizer.singular_vertex_indices().len(),
            2,
            "timing singular count"
        );
        let mut checksum = 0.0;
        for tri in &uvs {
            for uv in tri {
                checksum += uv.x() + uv.y();
            }
        }
        // C++ T-line checksum for the same mesh (see the committed
        // fixture): proves both sides solved the same system. Scale-aware
        // 1e-6, like the oracle values (absorbs the ~1e-14 run-to-run
        // threading wobble on the C++ side with wide margin).
        const CPP_CHECKSUM: f64 = 883_758.564_268_023_4;
        let tol = 1e-6 * CPP_CHECKSUM.abs().max(1.0);
        assert!(
            (checksum - CPP_CHECKSUM).abs() <= tol,
            "checksum diverged: rust={checksum:.17e} cpp={CPP_CHECKSUM:.17e}"
        );
        eprintln!(
            "T rust sample={sample} ms={ms:.3} faces={} sing={} checksum={checksum:.17e}",
            uvs.len(),
            parameterizer.singular_vertex_indices().len()
        );
    }
}
