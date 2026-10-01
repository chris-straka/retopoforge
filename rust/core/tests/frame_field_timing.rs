// Timing mirror for the frame field: regenerates the bit-identical 64x64
// analytic grid the C++ framefield_diff_dump tool times (z = 0.1 * sin(i) *
// cos(j); same libm calls, same order) and runs the same create call.
// Structural facts (ok, face count, checksum within 1e-6) must match the C++
// T lines; only the elapsed ms are compared across sides. Run release:
// cargo test --release -p retopo_core --test frame_field_timing
use retopo_core::frame_field::FrameField;
use retopo_core::surface_mesh::SurfaceMesh;
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
fn timing_field_solve() {
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
    let mesh = SurfaceMesh::new(&vertices, &triangles);
    // Warmup (unprinted), like the C++ side.
    {
        let field = FrameField::create(&mesh, 90.0, &[], &[]);
        assert!(field.is_some(), "warmup solve must succeed");
    }
    for sample in 0..3 {
        let t0 = Instant::now();
        let field = FrameField::create(&mesh, 90.0, &[], &[]);
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        let field = field.expect("timing solve must succeed");
        assert_eq!(field.len(), 8192);
        let mut checksum = 0.0;
        for f in &field {
            checksum += f.x() + f.y() + f.z();
        }
        // C++ T-line checksum for the same mesh (see the committed
        // fixture): proves both sides solved the same system. Scale-aware
        // 1e-6, like the oracle values.
        const CPP_CHECKSUM: f64 = 7804.9411148705358;
        let tol = 1e-6 * CPP_CHECKSUM.abs().max(1.0);
        assert!(
            (checksum - CPP_CHECKSUM).abs() <= tol,
            "checksum diverged: rust={checksum:.17e} cpp={CPP_CHECKSUM:.17e}"
        );
        eprintln!(
            "T rust sample={sample} ms={ms:.3} faces={} checksum={checksum:.17e}",
            field.len()
        );
    }
}
