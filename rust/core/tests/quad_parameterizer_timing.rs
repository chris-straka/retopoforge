// Cover-sized timing mirror for the quad parameterizer: regenerates the
// bit-identical 64x64 analytic grid the C++ quadparam_diff_dump tool times
// (z = 0.1 * sin(i) * cos(j); same libm calls, same order) and runs the
// same parameterize call. Structural facts (ok, ROUND count, rotation sum)
// must match the C++ T lines; only the elapsed ms are compared across
// sides. Run release: cargo test --release -p retopo_core --test
// quad_parameterizer_timing
use retopo_core::progress::ProgressHandler;
use retopo_core::quad_parameterizer::{DipoleConfig, QuadParameterizer};
use retopo_core::vector3::Vector3;
use std::sync::{Arc, Mutex};
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
fn timing_cover_solve() {
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
        let r = QuadParameterizer::parameterize(
            &vertices,
            &triangles,
            &[],
            1.0,
            90.0,
            &[],
            &[],
            &[],
            None,
            None,
            &[],
            DipoleConfig::off(),
        );
        assert!(r.is_some(), "warmup solve must succeed");
    }
    for sample in 0..3 {
        // Recording handler (Arc: the Box<dyn Fn> alias is 'static). A few
        // mutex locks over a ~50ms solve add no measurable overhead.
        let rounds: Arc<Mutex<usize>> = Arc::new(Mutex::new(0));
        let handler: ProgressHandler = Box::new({
            let rounds = rounds.clone();
            move |_, name| {
                if name == "Rounding cover to integers" {
                    *rounds.lock().unwrap() += 1;
                }
            }
        });
        let t0 = Instant::now();
        let result = QuadParameterizer::parameterize(
            &vertices,
            &triangles,
            &[],
            1.0,
            90.0,
            &[],
            &[],
            &[],
            Some(&handler),
            None,
            &[],
            DipoleConfig::off(),
        );
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        let result = result.expect("timing solve must succeed");
        let rounds = *rounds.lock().unwrap();
        let rot_sum: i64 = result.corner_rotations.iter().map(|&r| r as i64).sum();
        eprintln!("T rust qp sample={sample} ms={ms:.3} rounds={rounds} rotsum={rot_sum}");
        assert_eq!(rounds, 3, "ROUND count must match C++ (3)");
        assert_eq!(rot_sum, 0, "rotation sum must match C++ (0)");
    }
}
