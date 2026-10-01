//! Timing mirror for symmetry: regenerates the bit-identical 20k-vertex
//! stream the C++ symmetry_diff_dump tool times (same splitmix64 stream,
//! same draw order) and runs the same pipeline (detect + 3 fixed planes +
//! symmetrize vertices + symmetrize frame field). The sink must match C++
//! bitwise (proves identical streams AND identical ops); only the elapsed
//! ms are compared across sides.
//! Run release: cargo test --release -p retopo_core --test symmetry_timing
use retopo_core::symmetry::Symmetry;
use retopo_core::vector3::Vector3;
use std::time::Instant;

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^ (z >> 31)
    }
    // Same draw as the C++ timingDouble, branch for branch.
    fn timing_double(&mut self) -> f64 {
        ((self.next() % 2_000_000) as f64 - 1_000_000.0) / 100_000.0
    }
    fn v3(&mut self) -> Vector3 {
        Vector3::new(
            self.timing_double(),
            self.timing_double(),
            self.timing_double(),
        )
    }
}

const TIMING_SEED: u64 = 0x51EED01E55;

#[test]
fn timing_symmetry_pipeline() {
    let mut rng = Rng(TIMING_SEED);
    const N: usize = 20000;
    let mut vertices = Vec::with_capacity(N);
    for _ in 0..N {
        vertices.push(rng.v3());
    }
    let mut triangles = Vec::with_capacity(N);
    for _ in 0..N {
        triangles.push(vec![
            (rng.next() % N as u64) as usize,
            (rng.next() % N as u64) as usize,
            (rng.next() % N as u64) as usize,
        ]);
    }
    let mut field = Vec::with_capacity(N);
    for _ in 0..N {
        field.push(rng.v3());
    }
    let t0 = Instant::now();
    let detected = Symmetry::detect_plane(&vertices);
    let fixed0 = Symmetry::fixed_plane(&vertices, 0);
    let fixed1 = Symmetry::fixed_plane(&vertices, 1);
    let fixed2 = Symmetry::fixed_plane(&vertices, 2);
    let mut snapped = vertices.clone();
    Symmetry::symmetrize_vertices(&mut snapped, &detected);
    let mut sym_field = field.clone();
    Symmetry::symmetrize_frame_field(&vertices, &triangles, &mut sym_field, &detected);
    // Same accumulation order as the C++ sink.
    let mut sink = 0.0;
    sink += detected.score + detected.offset;
    sink += fixed0.score + fixed0.offset;
    sink += fixed1.score + fixed1.offset;
    sink += fixed2.score + fixed2.offset;
    for v in &snapped {
        sink += v.x();
        sink += v.y();
        sink += v.z();
    }
    for v in &sym_field {
        sink += v.x();
        sink += v.y();
        sink += v.z();
    }
    let ms = t0.elapsed().as_secs_f64() * 1000.0;
    eprintln!("T rust sym ms={ms:.3} sink={sink:.17e}");
    // Bitwise sink proves identical streams and identical ops.
    assert_eq!(
        sink.to_bits(),
        (-65.539505422980611f64).to_bits(),
        "sink diverged: streams or ops differ from C++"
    );
}
