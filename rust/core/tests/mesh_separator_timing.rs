// Grid timing mirror: builds the bit-identical 200x200 triangulated grid
// the C++ meshsep_diff_dump tool times (pure formula, no RNG on either
// side) and prints Rust-side split ms for the calibration runtime ratio.
// Run release: cargo test --release --offline --test mesh_separator_timing
// -- --nocapture
use retopo_core::mesh_separator::MeshSeparator;
use std::time::Instant;

fn make_grid(w: usize, h: usize) -> Vec<Vec<usize>> {
    let mut faces = Vec::with_capacity(w * h * 2);
    let id = |x: usize, y: usize| y * (w + 1) + x;
    for y in 0..h {
        for x in 0..w {
            let a = id(x, y);
            let b = id(x + 1, y);
            let cc = id(x + 1, y + 1);
            let d = id(x, y + 1);
            faces.push(vec![a, b, cc]);
            faces.push(vec![a, cc, d]);
        }
    }
    faces
}

#[test]
fn timing_grid() {
    let faces = make_grid(200, 200);
    assert_eq!(faces.len(), 80_000);
    for _ in 0..3 {
        let mut islands = Vec::new();
        let t0 = Instant::now();
        MeshSeparator::split_to_islands(&faces, &mut islands);
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        assert_eq!(islands.len(), 1);
        assert_eq!(islands[0].len(), 80_000);
        eprintln!(
            "T rust meshsep faces={} islands={} ms={ms:.3}",
            faces.len(),
            islands.len()
        );
    }
}
