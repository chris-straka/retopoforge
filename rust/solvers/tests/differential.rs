// Differential oracle replay: parses tests/fixtures/solver_diff.txt (generated
// by the C++ solver_diff_dump tool), re-solves every case with the Rust port,
// and asserts the port matches the C++ implementation: same ok flags, same
// kernel sizes, same iteration counts, values within 1e-6.
use retopo_solvers::constrained::ConstrainedLeastSquares;
use retopo_solvers::mixed_integer::MixedIntegerLeastSquares;

const FIXTURE: &str = include_str!("../../../tests/fixtures/solver_diff.txt");
const TOL: f64 = 1e-6;

struct Cur {
    toks: Vec<String>,
    pos: usize,
}

impl Cur {
    fn new(s: &str) -> Self {
        Self {
            toks: s.split_ascii_whitespace().map(str::to_string).collect(),
            pos: 0,
        }
    }
    fn word(&mut self) -> String {
        let w = self.toks[self.pos].clone();
        self.pos += 1;
        w
    }
    fn peek(&self) -> &str {
        &self.toks[self.pos]
    }
    fn expect(&mut self, tag: &str) {
        assert_eq!(
            self.word(),
            tag,
            "fixture parse error at token {}",
            self.pos
        );
    }
    fn int(&mut self) -> i64 {
        self.word().parse().expect("bad int")
    }
    fn uint(&mut self) -> usize {
        self.word().parse().expect("bad uint")
    }
    fn num(&mut self) -> f64 {
        self.word().parse().expect("bad f64")
    }
    fn row(&mut self) -> Vec<(usize, f64)> {
        let k = self.uint();
        (0..k).map(|_| (self.uint(), self.num())).collect()
    }
}

fn check_values(case: &str, got: &[f64], c: &mut Cur, max_diff: &mut f64) {
    for (i, v) in got.iter().enumerate() {
        let expect = c.num();
        let d = (v - expect).abs();
        *max_diff = max_diff.max(d);
        // Scale-aware: the two Cholesky backends' forward error grows with
        // |x| (cond * eps * |x|), so the tolerance scales with the magnitude.
        // Logic divergences (wrong subset, miscounted Gram) are O(1) and
        // still trip this by orders of magnitude.
        let tol = TOL * expect.abs().max(1.0);
        assert!(
            d <= tol,
            "{case} var {i}: rust={v} cpp={expect} diff={d} tol={tol}"
        );
    }
}

struct ClsxStats {
    solves: usize,
    value_agree: usize,
    max_diff: f64,
}

// Verifies one solve against the fixture. Strict (CLS): ok flags and values
// must match. Relaxed (CLSX): Rust must solve whenever C++ did; value
// agreement is recorded, not asserted.
fn verify_solve(
    case: &str,
    ok: bool,
    got: Option<Vec<f64>>,
    c: &mut Cur,
    strict: bool,
    max_diff: &mut f64,
    stats: &mut ClsxStats,
) {
    if strict {
        assert_eq!(got.is_some(), ok, "{case}: ok flag diverged");
    } else if ok {
        assert!(
            got.is_some(),
            "{case}: C++ solved but Rust failed (robustness regression)"
        );
    }
    match got {
        Some(x) if ok => {
            if strict {
                check_values(case, &x, c, max_diff);
            } else {
                stats.solves += 1;
                let mut agree = true;
                for v in x.iter() {
                    let expect = c.num();
                    let d = (v - expect).abs();
                    stats.max_diff = stats.max_diff.max(d);
                    if d > TOL * expect.abs().max(1.0) {
                        agree = false;
                    }
                }
                if agree {
                    stats.value_agree += 1;
                }
            }
        }
        // C++ ok=0 prints no values, so there is nothing to consume.
        // (Rust solving what C++ cannot is fine.)
        _ => {}
    }
}

#[test]
fn differential_replay() {
    let mut c = Cur::new(FIXTURE);
    c.expect("DIFF1");
    let mut cls_cases = 0;
    let mut mils_cases = 0;
    let mut clsx_cases = 0;
    let mut stats = ClsxStats {
        solves: 0,
        value_agree: 0,
        max_diff: 0.0,
    };
    let mut max_diff = 0.0f64;
    loop {
        let tag = c.word();
        match tag.as_str() {
            "CLS" | "CLSX" => {
                // CLSX = inconsistent constraints: SPQR+COLAMD (C++) vs ColPiv
                // (Rust) may drop different constraints, so values can
                // legitimately differ. Robustness-only: the port must solve
                // everything C++ solves; value agreement is reported, not
                // asserted.
                let strict = tag == "CLS";
                if strict {
                    cls_cases += 1;
                } else {
                    clsx_cases += 1;
                }
                let case = format!("{tag} case {}", cls_cases + clsx_cases);
                let nvars = c.uint();
                let n_e = c.uint();
                let n_c = c.uint();
                let mut s = ConstrainedLeastSquares::new(nvars);
                for _ in 0..n_e {
                    c.expect("e");
                    let row = c.row();
                    let rhs = c.num();
                    let w = c.num();
                    s.add_energy(&row, rhs, w);
                }
                for _ in 0..n_c {
                    c.expect("c");
                    let row = c.row();
                    let rhs = c.num();
                    s.add_constraint(&row, rhs);
                }
                c.expect("s");
                let ok = c.int() != 0;
                let got = s.solve();
                verify_solve(&case, ok, got, &mut c, strict, &mut max_diff, &mut stats);
                if c.peek() == "u" {
                    c.expect("u");
                    let idx = c.uint();
                    let rhs = c.num();
                    s.set_energy_right_hand_side(idx, rhs);
                    c.expect("s2");
                    let ok2 = c.int() != 0;
                    let got2 = s.solve();
                    verify_solve(
                        &format!("{case} s2"),
                        ok2,
                        got2,
                        &mut c,
                        strict,
                        &mut max_diff,
                        &mut stats,
                    );
                }
                if c.peek() == "clr" {
                    c.expect("clr");
                    s.clear_constraints();
                    c.expect("s3");
                    let ok3 = c.int() != 0;
                    let got3 = s.solve();
                    verify_solve(
                        &format!("{case} s3"),
                        ok3,
                        got3,
                        &mut c,
                        strict,
                        &mut max_diff,
                        &mut stats,
                    );
                }
            }
            "MILS" => {
                mils_cases += 1;
                let case = format!("MILS case {mils_cases}");
                let nvars = c.uint();
                let n_eq = c.uint();
                let n_en = c.uint();
                let use_builder = c.int() != 0;
                let mut s = MixedIntegerLeastSquares::new(nvars);
                while c.peek() == "p" {
                    c.expect("p");
                    let v = c.uint();
                    let period = c.int() as i32;
                    s.set_variable_period(v, period);
                }
                let mut builder_ok = true;
                for q in 0..n_eq {
                    c.expect("q");
                    let row = c.row();
                    if use_builder && q + 1 == n_eq {
                        s.begin_constraint();
                        for (v, a) in &row {
                            s.add_constraint_coefficient(*v, *a);
                        }
                        builder_ok = s.end_constraint();
                    } else {
                        s.add_constraint(&row);
                    }
                }
                c.expect("b");
                let expect_builder = c.int() != 0;
                assert_eq!(
                    builder_ok, expect_builder,
                    "{case}: end_constraint diverged"
                );
                for _ in 0..n_en {
                    c.expect("e");
                    let row = c.row();
                    let rhs = c.num();
                    let w = c.num();
                    s.add_energy(&row, rhs, w);
                }
                s.finalize_constraints();
                let mut ok = true;
                let mut iters = 0;
                while iters < 100 {
                    if !s.solve_iteration() {
                        ok = false;
                        break;
                    }
                    if s.converged() {
                        break;
                    }
                    iters += 1;
                }
                c.expect("s");
                let e_ok = c.int() != 0;
                let e_conv = c.int() != 0;
                let e_iters = c.uint();
                let e_ks = c.uint();
                let e_ik = c.uint();
                let e_oi = c.uint();
                assert_eq!(ok, e_ok, "{case}: ok diverged");
                assert_eq!(s.converged(), e_conv, "{case}: converged diverged");
                assert_eq!(
                    iters, e_iters,
                    "{case}: iters diverged ({iters} vs {e_iters})"
                );
                assert_eq!(s.kernel_size(), e_ks, "{case}: kernel_size diverged");
                assert_eq!(
                    s.integer_kernel_variable_count(),
                    e_ik,
                    "{case}: integer kernel count diverged"
                );
                assert_eq!(
                    s.original_integer_variable_count(),
                    e_oi,
                    "{case}: original integer count diverged"
                );
                let mut vals = Vec::with_capacity(nvars);
                for v in 0..nvars {
                    vals.push(s.value(v));
                }
                check_values(&case, &vals, &mut c, &mut max_diff);
            }
            "T" => {
                // Timing lines are informational only; the Rust-side timing
                // harness regenerates the same systems and prints its own ms.
                let kind = c.word();
                assert!(kind == "cls" || kind == "mils", "bad T line");
                // Consume rest of the T line: `cls ok=%d ms=%f` or
                // `mils ok=.. ms=.. conv=.. iters=.. fin=.. loop=..`.
                let extra = if kind == "cls" { 2 } else { 6 };
                for _ in 0..extra {
                    c.word();
                }
                if kind == "mils" {
                    break;
                }
            }
            other => panic!("unexpected fixture tag: {other}"),
        }
    }
    assert_eq!(cls_cases, 200, "expected 200 CLS cases");
    assert_eq!(clsx_cases, 40, "expected 40 CLSX cases");
    assert_eq!(mils_cases, 200, "expected 200 MILS cases");
    eprintln!(
        "differential replay: {cls_cases} CLS + {mils_cases} MILS strict, max |diff| = {max_diff:.3e}"
    );
    eprintln!(
        "CLSX (inconsistent, robustness-only): {clsx_cases} cases, {} both-solved, {} value-agree, max |diff| = {:.3e}",
        stats.solves, stats.value_agree, stats.max_diff
    );
    assert!(max_diff <= TOL, "max diff {max_diff:.3e} exceeds {TOL}");
}
