// Cover-sized timing mirror for vector2/vector3: regenerates the
// bit-identical 1M-vector streams the C++ vector_diff_dump tool times
// (same splitmix64 stream, same draw order) and runs the same mixed-op
// loop. The sink must match C++ bitwise (proves identical streams AND
// identical ops); only the elapsed ms are compared across sides.
// Run release: cargo test --release -p retopo_core --test vector_timing
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
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
    // Same draw order as the C++ randDouble, branch for branch.
    fn double(&mut self) -> f64 {
        match self.below(6) {
            0 => self.below(21) as i64 as f64 - 10.0,
            1 => (self.below(41) as i64 - 20) as f64 / 4.0,
            2 => ((self.next() % 2_000_000) as f64 - 1_000_000.0) / 100.0,
            3 => ((self.next() % 2_000_000) as f64 - 1_000_000.0) * 1e-9,
            4 => {
                let m = ((self.next() % 2_000_000) as f64 - 1_000_000.0) / 1_000_000.0;
                // ldexp(m, e) with e in [-20, 19]: no over/underflow
                // possible, so `m * 2^e` is exactly ldexp.
                m * 2f64.powi(self.below(40) as i32 - 20)
            }
            _ => (self.below(2001) as i64 - 1000) as f64 / 1000.0,
        }
    }
    fn v3(&mut self) -> Vector3 {
        Vector3::new(self.double(), self.double(), self.double())
    }
}

const TIMING_SEED: u64 = 0x9EC70A5C10C;

#[test]
fn timing_vector_loop() {
    let mut rng = Rng(TIMING_SEED);
    const N: usize = 1 << 20;
    let mut a = Vec::with_capacity(N);
    let mut b = Vec::with_capacity(N);
    let mut c = Vec::with_capacity(N);
    for _ in 0..N {
        a.push(rng.v3());
        b.push(rng.v3());
        c.push(rng.v3());
    }
    let t0 = Instant::now();
    let mut sink = 0.0;
    for _ in 0..5 {
        for i in 0..N {
            let cr = Vector3::cross_product(&a[i], &b[i]);
            let d = Vector3::dot_product(&cr, &c[i]);
            let nn = Vector3::normal(&a[i], &b[i], &c[i]);
            sink += d + nn.x() + Vector3::area(&a[i], &b[i], &c[i]);
        }
    }
    let ms = t0.elapsed().as_secs_f64() * 1000.0;
    eprintln!("T rust vec ms={ms:.3} sink={sink:.17e}");
    // Bitwise sink proves identical streams and identical ops.
    assert_eq!(
        sink.to_bits(),
        (-93_303_047_457_626_352f64).to_bits(),
        "sink diverged: streams or ops differ from C++"
    );
}
