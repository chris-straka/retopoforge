// Timing mirror for positionkey: regenerates the bit-identical 1M-pair
// streams the C++ positionkey_diff_dump tool times (same splitmix64
// stream, same draw order) and runs the same construct+compare loop.
// The sink must match C++ bitwise (proves identical streams AND identical
// quantization/comparison); only the elapsed ms are compared across sides.
// Run release: cargo test --release -p retopo_core --test position_key_timing
use retopo_core::position_key::PositionKey;
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

const TIMING_SEED: u64 = 0x51CE1CA1ED;
const N: usize = 1 << 20;
// %.17g round-trips, so this literal is the exact C++ sink.
const EXPECT_SINK: f64 = 232492174.82974946;

fn one_sample() -> f64 {
    let mut rng = Rng(TIMING_SEED);
    let mut a = Vec::with_capacity(N);
    let mut b = Vec::with_capacity(N);
    for _ in 0..N {
        a.push(rng.v3());
        b.push(rng.v3());
    }
    let t0 = Instant::now();
    let mut sink = 0.0;
    for _ in 0..5 {
        for i in 0..N {
            let ka = PositionKey::from_vector(&a[i]);
            let kb = PositionKey::from_vector(&b[i]);
            // Same association as C++: ((x + lt) + eq).
            let lt = if ka < kb { 1.0 } else { 0.0 };
            let eq = if ka == kb { 2.0 } else { 0.0 };
            sink += (ka.position().x() + lt) + eq;
        }
    }
    let ms = t0.elapsed().as_secs_f64() * 1000.0;
    eprintln!("T rust pk ms={ms:.3} sink={sink:.17e}");
    // Bitwise sink proves identical streams and identical ops.
    assert_eq!(
        sink.to_bits(),
        EXPECT_SINK.to_bits(),
        "sink diverged: streams or ops differ from C++"
    );
    ms
}

#[test]
fn timing_position_key_loop() {
    // 3 samples, mirroring the 3 C++ samples in the fixture tail.
    let (m0, m1, m2) = (one_sample(), one_sample(), one_sample());
    eprintln!("rust pk samples: {m0:.3} {m1:.3} {m2:.3} ms");
}
