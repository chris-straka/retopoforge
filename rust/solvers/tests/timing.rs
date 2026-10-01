// Cover-sized timing mirror: regenerates the bit-identical systems the C++
// solver_diff_dump tool times (same splitmix64 stream, same draw order) and
// prints Rust-side solve ms for the calibration runtime ratio. Run release:
// cargo test --release --offline --test timing -- --nocapture
use retopo_solvers::constrained::ConstrainedLeastSquares;
use retopo_solvers::mixed_integer::MixedIntegerLeastSquares;
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
    fn nz_coeff(&mut self) -> f64 {
        loop {
            let v = self.below(17) as i64 - 8;
            if v != 0 {
                return v as f64 / 4.0;
            }
        }
    }
    fn rhs(&mut self) -> f64 {
        (self.below(41) as i64 - 20) as f64 / 4.0
    }
    fn weight(&mut self) -> f64 {
        const W: [f64; 5] = [0.25, 0.5, 1.0, 2.0, 4.0];
        W[self.below(5) as usize]
    }
    fn small_int(&mut self) -> f64 {
        loop {
            let v = self.below(5) as i64 - 2;
            if v != 0 {
                return v as f64;
            }
        }
    }
    fn row(&mut self, nvars: usize, small_ints: bool) -> Vec<(usize, f64)> {
        // Mirror of the C++ guard: k must not exceed nvars.
        let k = 1 + self.below(nvars.min(3) as u64) as usize;
        let mut vars = Vec::new();
        while vars.len() < k {
            let v = self.below(nvars as u64) as usize;
            if !vars.contains(&v) {
                vars.push(v);
            }
        }
        vars.into_iter()
            .map(|v| {
                let c = if small_ints {
                    self.small_int()
                } else {
                    self.nz_coeff()
                };
                (v, c)
            })
            .collect()
    }
}

const TIMING_SEED: u64 = 0xC10C4C0FFEE;

#[test]
fn timing_cover() {
    // CLS cover: 2000 vars, 4000 energies, 200 constraints.
    let mut rng = Rng(TIMING_SEED);
    let mut s = ConstrainedLeastSquares::new(2000);
    for _ in 0..4000 {
        let row = rng.row(2000, false);
        let rhs = rng.rhs();
        let w = rng.weight();
        s.add_energy(&row, rhs, w);
    }
    for _ in 0..200 {
        let row = rng.row(2000, false);
        s.add_constraint(&row, rng.rhs());
    }
    let t0 = Instant::now();
    let ok = s.solve().is_some();
    let cls_ms = t0.elapsed().as_secs_f64() * 1000.0;
    eprintln!("T rust cls ok={ok} ms={cls_ms:.3}");

    // MILS cover: 500 vars, 300 equalities, 600 energies.
    let mut rng = Rng(TIMING_SEED ^ 0xF00DBABE);
    let mut m = MixedIntegerLeastSquares::new(500);
    for v in 0..500 {
        if rng.below(10) < 5 {
            m.set_variable_period(v, 1);
        }
    }
    for _ in 0..300 {
        if rng.below(10) < 7 {
            let a = rng.below(500) as usize;
            let mut b = rng.below(500) as usize;
            if a == b {
                b = (b + 1) % 500;
            }
            m.add_constraint(&[(a, 1.0), (b, -1.0)]);
        } else {
            let row = rng.row(500, true);
            m.add_constraint(&row);
        }
    }
    for _ in 0..600 {
        let row = rng.row(500, false);
        let rhs = rng.rhs();
        let w = rng.weight();
        m.add_energy(&row, rhs, w);
    }
    let t0 = Instant::now();
    m.finalize_constraints();
    let t1 = Instant::now();
    let mut ok = true;
    let mut iters = 0;
    while iters < 100 {
        if !m.solve_iteration() {
            ok = false;
            break;
        }
        if m.converged() {
            break;
        }
        iters += 1;
    }
    let t2 = Instant::now();
    let mils_ms = (t2 - t0).as_secs_f64() * 1000.0;
    let fin_ms = (t1 - t0).as_secs_f64() * 1000.0;
    let loop_ms = (t2 - t1).as_secs_f64() * 1000.0;
    eprintln!(
        "T rust mils ok={ok} ms={mils_ms:.3} conv={} iters={iters} fin={fin_ms:.3} loop={loop_ms:.3}",
        m.converged()
    );
}
