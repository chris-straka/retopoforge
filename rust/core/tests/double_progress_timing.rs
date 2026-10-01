//! Timing mirror: regenerates the bit-identical input the C++
//! `double_progress_diff_dump` tool times (same splitmix64 stream, same
//! draw order) and prints Rust-side ms for the runtime ratio. Run release:
//! `cargo test --release --offline --test double_progress_timing -- --nocapture`
use retopo_core::double_utils::is_zero;
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
    fn uniform(&mut self, lo: f64, hi: f64) -> f64 {
        const UNIT: f64 = 1.0 / 9007199254740992.0; // 2^-53
        let u = ((self.next() >> 11) as f64) * UNIT;
        lo + u * (hi - lo)
    }
    fn any_double(&mut self) -> f64 {
        match self.below(5) {
            0 => self.uniform(-4.0, 4.0) * f64::EPSILON,
            1 => self.uniform(-10.0, 10.0),
            2 => self.uniform(-1.0e6, 1.0e6),
            3 => {
                let bits = self.next();
                let exp = self.below(108) as i32 - 53;
                let mant = 1.0 + ((bits >> 12) as f64) / 4503599627370496.0;
                let v = mant * 2f64.powi(exp);
                if bits & 1 == 1 { -v } else { v }
            }
            _ => self.uniform(-1.0, 1.0),
        }
    }
}

const TIMING_SEED: u64 = 0xD0B1E77A11CE ^ 0x71E77A11CE;

#[test]
fn timing_zero_loop() {
    let mut rng = Rng(TIMING_SEED);
    let xs: Vec<f64> = (0..4096).map(|_| rng.any_double()).collect();
    for s in 0..3 {
        let mut sink = 0;
        let t0 = Instant::now();
        for i in 0..1_000_000 {
            if is_zero(xs[i & 4095]) {
                sink += 1;
            }
        }
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        eprintln!("T rust zero sample={s} calls=1000000 hits={sink} ms={ms:.3}");
    }
}
