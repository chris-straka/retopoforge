// Timing mirror for density: regenerates the bit-identical resample +
// scaling-field inputs the C++ density_diff_dump tool times (same
// splitmix64 stream, same draw order) and runs the same two calls. The
// sink must match C++ bitwise (proves identical inputs AND identical
// ops); only the elapsed ms are compared across sides.
// Run release: cargo test --release -p retopo_core --test density_timing
use retopo_core::density::Density;
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
    // Same draws as the C++ tdouble/tdensity, in the same order.
    fn tdouble(&mut self) -> f64 {
        ((self.next() % 20001) as f64 - 10000.0) / 100.0
    }
    fn tdensity(&mut self) -> f64 {
        ((self.next() % 401) as f64 + 25.0) / 100.0
    }
    fn v3(&mut self) -> Vector3 {
        Vector3::new(self.tdouble(), self.tdouble(), self.tdouble())
    }
}

const TIMING_SEED: u64 = 0xDE4517A11C;

#[test]
fn timing_density_loop() {
    let mut rng = Rng(TIMING_SEED);
    const NSRC: usize = 2048;
    const NDST: usize = 2048;
    let mut src = Vec::with_capacity(NSRC);
    let mut field = Vec::with_capacity(NSRC);
    for _ in 0..NSRC {
        src.push(rng.v3());
        field.push(rng.tdensity());
    }
    let mut dst = Vec::with_capacity(NDST);
    for _ in 0..NDST {
        dst.push(rng.v3());
    }
    const NV: usize = 1024;
    const NT: usize = 2048;
    let mut verts = Vec::with_capacity(NV);
    for _ in 0..NV {
        verts.push(rng.v3());
    }
    let mut tris = Vec::with_capacity(NT);
    for i in 0..NT {
        tris.push(vec![i % NV, (i + 1) % NV, (i + 2) % NV]);
    }
    let mut dens = Vec::with_capacity(NV);
    for _ in 0..NV {
        dens.push(rng.tdensity());
    }
    let mut scal = vec![1.0; NT];
    let t0 = Instant::now();
    let resampled = Density::resample_nearest(&src, &field, &dst);
    Density::apply_to_scaling_field(&verts, &tris, &dens, &mut scal);
    let ms = t0.elapsed().as_secs_f64() * 1000.0;
    let mut sink = 0.0;
    for v in &resampled {
        sink += v;
    }
    for v in &scal {
        sink += v;
    }
    eprintln!("T rust dens ms={ms:.3} sink={sink:.17e}");
    // Bitwise sink proves identical inputs and identical ops.
    assert_eq!(
        sink.to_bits(),
        0x40ba4f475a835fab, // C++ sink 6735.2787248714985, stable over 3 runs
        // (proves identical inputs; bitwise ops proven by density_diff)
        "sink diverged: inputs or ops differ from C++"
    );
}
