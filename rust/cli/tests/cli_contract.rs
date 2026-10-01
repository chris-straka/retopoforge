// CLI contract test: the Rust `retopo` binary against committed goldens.
//
// Replaces the retired C++-vs-Rust differential oracle (`e2e_diff.rs`,
// removed with the frozen C++ tree). Same case corpus, Rust-only checks:
// - arg matrix + IO failure paths: exit code + normalized stdout/stderr
//   must equal `tests/fixtures/cli_golden/{arg,io}-<case>.txt` exactly.
// - remesh cases: every case runs twice and must be byte-identical
//   run-to-run (artifacts, normalized stdout, stderr, report); exit code,
//   artifact presence, and warning/error lines must equal the golden;
//   count lines (quads/verts/islands) must land within 0.5% + 8 of the
//   golden numbers (same tolerance as the golden count pins).
// - hex-float / `nan(payload)` numerics are rejected loudly (the
//   specified divergence from the old C++ `strtod` acceptance).
//
// Regenerate goldens after an intended CLI/engine change:
//   UPDATE_GOLDENS=1 cargo test --release -p retopo --test cli_contract
// and review the diff. Std-only, no dev-dependencies (offline).

use std::io::Read;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

const RUN_TIMEOUT: Duration = Duration::from_secs(300);
// Count tolerance for remesh goldens: |actual - golden| <= 0.5% + 8.
const COUNT_REL_TOL: f64 = 0.005;
const COUNT_ABS_TOL: f64 = 8.0;

struct Bins {
    rs: PathBuf,
}

fn bins() -> Bins {
    let rs = PathBuf::from(env!("CARGO_BIN_EXE_retopo"));
    assert!(rs.is_file(), "rust binary missing: {}", rs.display());
    Bins { rs }
}

fn fixtures_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/fixtures")
}

fn case_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("rs-main-e2e-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("case dir");
    dir
}

struct RunOutput {
    code: Option<i32>,
    signal: Option<i32>,
    timed_out: bool,
    stdout: Vec<u8>,
    stderr: Vec<u8>,
}

impl RunOutput {
    fn crashed(&self) -> bool {
        !self.timed_out && self.code.is_none()
    }
}

#[cfg(unix)]
fn signal_of(status: &std::process::ExitStatus) -> Option<i32> {
    use std::os::unix::process::ExitStatusExt;
    status.signal()
}

#[cfg(not(unix))]
fn signal_of(_status: &std::process::ExitStatus) -> Option<i32> {
    None
}

// Spawn with drained pipes (reader threads so a verbose run can never
// wedge on a full pipe buffer) and a kill-on-timeout watchdog.
fn run(bin: &Path, args: &[String], timeout: Duration) -> RunOutput {
    let mut child = Command::new(bin)
        .args(args)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn binary");
    let mut out_pipe = child.stdout.take().expect("stdout pipe");
    let mut err_pipe = child.stderr.take().expect("stderr pipe");
    let out_thread = std::thread::spawn(move || {
        let mut buf = Vec::new();
        let _ = out_pipe.read_to_end(&mut buf);
        buf
    });
    let err_thread = std::thread::spawn(move || {
        let mut buf = Vec::new();
        let _ = err_pipe.read_to_end(&mut buf);
        buf
    });
    let start = Instant::now();
    let mut timed_out = false;
    let status = loop {
        match child.try_wait().expect("try_wait") {
            Some(status) => break status,
            None => {
                if start.elapsed() > timeout {
                    timed_out = true;
                    let _ = child.kill();
                    break child.wait().expect("wait after kill");
                }
                std::thread::sleep(Duration::from_millis(50));
            }
        }
    };
    let stdout = out_thread.join().expect("stdout drain");
    let stderr = err_thread.join().expect("stderr drain");
    RunOutput {
        code: status.code(),
        signal: if timed_out { None } else { signal_of(&status) },
        timed_out,
        stdout,
        stderr,
    }
}

// --- normalization ------------------------------------------------------
// Times, case-dir paths, and argv0 can never match byte-for-byte across
// sides; everything else must. Each rule below names exactly what it
// erases so the audit stays reviewable.

fn normalize_line(line: &str, case_tag: &str) -> String {
    let mut out = line.to_string();
    // Case-dir prefixes (both sides' scratch paths collapse to CASE).
    // Applied by the caller via `normalize_bytes`; kept here for the
    // pure rewrites only.
    let _ = case_tag;
    // "Time: 0.512153 seconds" / "time=0.5 seconds" / "Total time: ..."
    // (also rung "time=" and report "Total time:"): erase the number.
    if let Some(pos) = out.find(" seconds") {
        let mut start = pos;
        let bytes = out.as_bytes();
        while start > 0
            && matches!(
                bytes[start - 1],
                b'0'..=b'9' | b'.' | b'e' | b'E' | b'+' | b'-'
            )
        {
            start -= 1;
        }
        // Only erase when a label precedes the number (never touch mesh
        // data, which never contains " seconds" anyway).
        if start < pos {
            out.replace_range(start..pos, "T");
        }
    }
    // Phase-report timings "  Foo bar: 12.5 ms" — and the colon-less
    // form "  ...count), 0.1 ms" (mesh-simplifier SKIPPED line). The old
    // rule required ": " and leaked the colon-less timings raw, which
    // flaked run-to-run asserts (0.0 vs 0.1 under CI load) and poisoned
    // the census phase-block match. Erase any trailing numeric run
    // before " ms" (mesh data never contains " ms").
    if out.ends_with(" ms") {
        let body = &out[..out.len() - 3];
        let run_len = body
            .bytes()
            .rev()
            .take_while(|b| matches!(b, b'0'..=b'9' | b'.'))
            .count();
        if run_len > 0 && run_len < body.len() {
            let before = body.as_bytes()[body.len() - run_len - 1];
            if matches!(before, b' ' | b',' | b':') {
                out.replace_range(body.len() - run_len..body.len(), "T");
            }
        }
    }
    // Timing-derived core ratio (varies run-to-run by construction).
    const CORES: &str = "Cores kept busy across the parallel phase: ";
    if let Some(pos) = out.find(CORES) {
        let num_start = pos + CORES.len();
        if let Some(paren) = out[num_start..].find(" (") {
            out.replace_range(num_start..num_start + paren, "R");
        }
    }
    out
}

fn normalize_bytes(bytes: &[u8], case_dir: &str) -> Vec<String> {
    let text = String::from_utf8_lossy(bytes);
    text.split('\n')
        .map(|line| {
            let mut line = line.to_string();
            if !case_dir.is_empty() {
                line = line.replace(case_dir, "CASE");
            }
            // Side subdirs (`cpp`, `cpp-3`, `rs`, `rs2`) collapse too:
            // both sides write the same relative layout.
            let mut scan = 0;
            loop {
                let Some(rel) = line[scan..].find("CASE/") else {
                    break;
                };
                let pos = scan + rel;
                let rest = &line[pos + 5..];
                let seg_end = rest.find('/').map(|p| pos + 5 + p).unwrap_or(line.len());
                let seg = line[pos + 5..seg_end].to_string();
                if seg == "cpp" || seg == "rs" || seg == "rs2" || seg.starts_with("cpp-") {
                    let cut_end = if seg_end < line.len() {
                        seg_end + 1
                    } else {
                        seg_end
                    };
                    line.replace_range(pos + 5..cut_end, "");
                    scan = pos;
                } else {
                    scan = seg_end;
                }
            }
            normalize_line(&line, "")
        })
        .collect()
}

fn normalize_argv0(line: &str) -> String {
    // "Usage: <argv0> --input ..." -> argv0-independent.
    if let Some(rest) = line.strip_prefix("Usage: ") {
        if let Some(pos) = rest.find(" --input ") {
            return format!("Usage: BIN{}", &rest[pos..]);
        }
    }
    line.to_string()
}

// --- generated inputs ----------------------------------------------------

fn write_grid_obj(path: &Path, w: usize, h: usize) {
    let mut out = String::new();
    out.push_str("# analytic grid z = 0.1 * sin(i) * cos(j)\n");
    for j in 0..=h {
        for i in 0..=w {
            out.push_str(&format!(
                "v {} {} {}\n",
                i,
                j,
                0.1 * (i as f64).sin() * (j as f64).cos()
            ));
        }
    }
    for j in 0..h {
        for i in 0..w {
            let a = j * (w + 1) + i + 1;
            let b = a + 1;
            let c = a + w + 1;
            let d = c + 1;
            out.push_str(&format!("f {a} {b} {d}\n"));
            out.push_str(&format!("f {a} {d} {c}\n"));
        }
    }
    std::fs::write(path, out).expect("write grid");
}

fn write_two_cubes_obj(path: &Path) {
    let mut out = String::from("# two disjoint cubes (2 islands)\n");
    for ox in [0.0, 5.0] {
        let v = [
            [ox, 0.0, 0.0],
            [ox + 1.0, 0.0, 0.0],
            [ox + 1.0, 1.0, 0.0],
            [ox, 1.0, 0.0],
            [ox, 0.0, 1.0],
            [ox + 1.0, 0.0, 1.0],
            [ox + 1.0, 1.0, 1.0],
            [ox, 1.0, 1.0],
        ];
        for p in v {
            out.push_str(&format!("v {} {} {}\n", p[0], p[1], p[2]));
        }
    }
    let tris = [
        [1, 2, 3],
        [1, 3, 4],
        [5, 8, 7],
        [5, 7, 6],
        [1, 5, 6],
        [1, 6, 2],
        [2, 6, 7],
        [2, 7, 3],
        [3, 7, 8],
        [3, 8, 4],
        [4, 8, 5],
        [4, 5, 1],
    ];
    for t in tris {
        out.push_str(&format!("f {} {} {}\n", t[0], t[1], t[2]));
        out.push_str(&format!("f {} {} {}\n", t[0] + 8, t[1] + 8, t[2] + 8));
    }
    std::fs::write(path, out).expect("write two cubes");
}

fn write_tetra_obj(path: &Path) {
    std::fs::write(
        path,
        "# single tet\nv 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\nf 1 2 4\nf 1 3 4\nf 2 3 4\n",
    )
    .expect("write tetra");
}

fn write_grid_guides(path: &Path, w: usize, h: usize) {
    // Two polylines riding the grid surface (mid-rows, x = 2..w-2).
    let mut out = String::from("# grid guides\n");
    for (n, j) in [(0, h / 3), (1, 2 * h / 3)] {
        if n > 0 {
            out.push('\n');
        }
        for i in 2..=(w - 2) {
            out.push_str(&format!(
                "{} {} {}\n",
                i as f64,
                j as f64,
                0.1 * (i as f64).sin() * (j as f64).cos()
            ));
        }
    }
    std::fs::write(path, out).expect("write guides");
}

fn write_grid_features(path: &Path, w: usize, h: usize) {
    // One vertical polyline riding the grid surface.
    let mut out = String::from("# grid features\n");
    for j in 2..=(h - 2) {
        out.push_str(&format!(
            "{} {} {}\n",
            (w / 2) as f64,
            j as f64,
            0.1 * ((w / 2) as f64).sin() * (j as f64).cos()
        ));
    }
    std::fs::write(path, out).expect("write features");
}

fn write_grid_density(path: &Path, w: usize, h: usize) {
    // Mild mask: 2x bump in the middle, 1.0 elsewhere.
    let mut out = String::from("# grid density\n");
    for j in 0..=h {
        for i in 0..=w {
            let center = (i as f64 - w as f64 / 2.0).hypot(j as f64 - h as f64 / 2.0);
            let m = if center < w as f64 / 4.0 { 2.0 } else { 1.0 };
            out.push_str(&format!("{m}\n"));
        }
    }
    std::fs::write(path, out).expect("write density");
}

// --- goldens ----------------------------------------------------------------

fn updating_goldens() -> bool {
    std::env::var_os("UPDATE_GOLDENS").is_some()
}

fn golden_path(kind: &str, name: &str) -> PathBuf {
    fixtures_dir()
        .join("cli_golden")
        .join(format!("{kind}-{name}.txt"))
}

fn write_golden(path: &Path, text: &str) {
    std::fs::create_dir_all(path.parent().expect("golden dir")).expect("golden dir");
    std::fs::write(path, text).expect("write golden");
}

// Exact golden check (arg / io cases). With UPDATE_GOLDENS set, writes
// the actual text instead and passes.
fn check_golden_exact(kind: &str, name: &str, actual: &str, failures: &mut Vec<String>) {
    let path = golden_path(kind, name);
    if updating_goldens() {
        write_golden(&path, actual);
        return;
    }
    match std::fs::read_to_string(&path) {
        Ok(expected) if expected == actual => {}
        Ok(expected) => failures.push(format!(
            "{kind}-{name}: golden mismatch ({})\n--- expected ---\n{expected}\n--- actual ---\n{actual}",
            path.display()
        )),
        Err(_) => failures.push(format!(
            "{kind}-{name}: missing golden {} (run with UPDATE_GOLDENS=1)",
            path.display()
        )),
    }
}

// Splits a line into alternating non-digit / digit runs.
fn digit_runs(line: &str) -> Vec<(bool, String)> {
    let mut runs: Vec<(bool, String)> = Vec::new();
    for ch in line.chars() {
        let is_digit = ch.is_ascii_digit();
        match runs.last_mut() {
            Some((d, s)) if *d == is_digit => s.push(ch),
            _ => runs.push((is_digit, ch.to_string())),
        }
    }
    runs
}

// Same text with every number within the count tolerance.
fn counts_agree(expected: &str, actual: &str) -> bool {
    let e = digit_runs(expected);
    let a = digit_runs(actual);
    e.len() == a.len()
        && e.iter().zip(&a).all(|((ed, es), (ad, at))| {
            if ed != ad {
                return false;
            }
            if !ed {
                return es == at;
            }
            match (es.parse::<f64>(), at.parse::<f64>()) {
                (Ok(x), Ok(y)) => (x - y).abs() <= COUNT_REL_TOL * x.abs() + COUNT_ABS_TOL,
                _ => es == at,
            }
        })
}

// Remesh golden check: lines inside the `counts:` section compare under
// the count tolerance, every other line exactly.
fn check_golden_counts(name: &str, actual: &str, failures: &mut Vec<String>) {
    let path = golden_path("remesh", name);
    if updating_goldens() {
        write_golden(&path, actual);
        return;
    }
    let Ok(expected) = std::fs::read_to_string(&path) else {
        failures.push(format!(
            "remesh-{name}: missing golden {} (run with UPDATE_GOLDENS=1)",
            path.display()
        ));
        return;
    };
    let e: Vec<&str> = expected.split('\n').collect();
    let a: Vec<&str> = actual.split('\n').collect();
    let mut in_counts = false;
    let mut ok = e.len() == a.len();
    if ok {
        for (el, al) in e.iter().zip(&a) {
            if *el == "counts:" || *el == "diagnostics:" {
                in_counts = *el == "counts:";
            }
            let same = if in_counts {
                counts_agree(el, al)
            } else {
                el == al
            };
            if !same {
                ok = false;
                break;
            }
        }
    }
    if !ok {
        failures.push(format!(
            "remesh-{name}: golden mismatch ({})\n--- expected ---\n{expected}\n--- actual ---\n{actual}",
            path.display()
        ));
    }
}

// normalize_bytes plus the checkout-specific fixtures path.
fn normalize_output(bytes: &[u8], case_dir: &str) -> Vec<String> {
    let fixtures = fixtures_dir().to_string_lossy().to_string();
    normalize_bytes(bytes, case_dir)
        .into_iter()
        .map(|l| l.replace(&fixtures, "FIXTURES"))
        .collect()
}

fn render_run(code: Option<i32>, stdout: &[String], stderr: &[String]) -> String {
    format!(
        "exit: {code:?}\n--- stdout ---\n{}\n--- stderr ---\n{}\n",
        stdout.join("\n"),
        stderr.join("\n")
    )
}

// --- arg matrix (pure parser paths) ------------------------------------------

fn arg_case(bins: &Bins, name: &str, args: &[&str], failures: &mut Vec<String>) {
    let argv: Vec<String> = args.iter().map(|s| s.to_string()).collect();
    let rs = run(&bins.rs, &argv, RUN_TIMEOUT);
    assert!(!rs.timed_out, "{name}: timeout");
    assert!(!rs.crashed(), "{name}: crash (signal {:?})", rs.signal);
    let out: Vec<String> = normalize_output(&rs.stdout, "")
        .iter()
        .map(|l| normalize_argv0(l))
        .collect();
    let err: Vec<String> = normalize_output(&rs.stderr, "")
        .iter()
        .map(|l| normalize_argv0(l))
        .collect();
    check_golden_exact("arg", name, &render_run(rs.code, &out, &err), failures);
}

// --- io failure paths (deterministic loader/FS errors) -----------------------

fn io_case(
    bins: &Bins,
    name: &str,
    setup: &dyn Fn(&Path) -> Vec<String>,
    failures: &mut Vec<String>,
) {
    let dir = case_dir(&format!("io-{name}"));
    let case_tag = dir.to_string_lossy().to_string();
    let rs_dir = dir.join("rs");
    std::fs::create_dir_all(&rs_dir).expect("rs dir");
    let shared = dir.join("shared");
    std::fs::create_dir_all(&shared).expect("shared dir");
    let argv = setup(&shared)
        .iter()
        .map(|a| a.replace("SHARED", &shared.to_string_lossy()))
        .map(|a| a.replace("SIDE", &rs_dir.to_string_lossy()))
        .collect::<Vec<_>>();
    let rs = run(&bins.rs, &argv, RUN_TIMEOUT);
    assert!(!rs.timed_out, "{name}: timeout");
    assert!(!rs.crashed(), "{name}: crash (signal {:?})", rs.signal);
    let out = normalize_output(&rs.stdout, &case_tag);
    let err = normalize_output(&rs.stderr, &case_tag);
    check_golden_exact("io", name, &render_run(rs.code, &out, &err), failures);
}

// --- remesh case runner -------------------------------------------------------

struct CaseRow {
    name: String,
    tier: String,
    detail: String,
}

// Quads / non-quads / vertices of an OBJ artifact.
fn obj_counts(bytes: &[u8]) -> String {
    let text = String::from_utf8_lossy(bytes);
    let (mut verts, mut quads, mut other) = (0usize, 0usize, 0usize);
    for line in text.split('\n') {
        if line.starts_with("v ") {
            verts += 1;
        } else if let Some(rest) = line.strip_prefix("f ") {
            if rest.split_whitespace().count() == 4 {
                quads += 1;
            } else {
                other += 1;
            }
        }
    }
    format!("quads={quads} nonquads={other} verts={verts}")
}

// "37% done. Step": emitted from whichever island thread advances the
// shared bar, so which percentages print (and their order) depends on
// scheduling in multi-island runs. Never compared.
fn is_progress_line(line: &str) -> bool {
    let digits = line.bytes().take_while(|b| b.is_ascii_digit()).count();
    digits > 0 && line[digits..].starts_with("% done.")
}

fn sorted(lines: &[String]) -> Vec<String> {
    let mut v = lines.to_vec();
    v.sort();
    v
}

fn is_count_line(line: &str) -> bool {
    let l = line.to_ascii_lowercase();
    l.contains("quads") || l.contains("vertices") || l.contains("islands")
}

fn is_diagnostic_line(line: &str) -> bool {
    let l = line.to_ascii_lowercase();
    l.contains("error") || l.contains("warning") || l.contains("failed")
}

struct Side {
    code: Option<i32>,
    stdout: Vec<String>,
    stderr: Vec<String>,
    artifacts: Vec<Option<Vec<u8>>>,
    report: Option<Vec<String>>,
    ms: f64,
}

// One remesh case: `make_args(side_dir)` builds full argv with
// side-specific outputs, `outputs(side_dir)` lists expected artifacts,
// `report(side_dir)` the optional report file. Runs twice (`rs`, `rs2`)
// for the determinism check, then compares the first run to the golden.
#[allow(clippy::too_many_arguments)]
fn remesh_case(
    bins: &Bins,
    name: &str,
    make_args: &dyn Fn(&Path) -> Vec<String>,
    outputs: &dyn Fn(&Path) -> Vec<PathBuf>,
    report: &dyn Fn(&Path) -> Option<PathBuf>,
    failures: &mut Vec<String>,
) -> CaseRow {
    let dir = case_dir(&format!("remesh-{name}"));
    let case_tag = dir.to_string_lossy().to_string();
    let run_side = |side_name: &str| -> Side {
        let side = dir.join(side_name);
        std::fs::create_dir_all(&side).expect("side dir");
        let args = make_args(&side);
        let start = Instant::now();
        let out = run(&bins.rs, &args, RUN_TIMEOUT);
        let ms = start.elapsed().as_secs_f64() * 1000.0;
        assert!(!out.timed_out, "{name}: timeout");
        assert!(!out.crashed(), "{name}: crash (signal {:?})", out.signal);
        Side {
            code: out.code,
            stdout: normalize_output(&out.stdout, &case_tag)
                .into_iter()
                .filter(|l| !is_progress_line(l))
                .collect(),
            stderr: normalize_output(&out.stderr, &case_tag),
            artifacts: outputs(&side)
                .iter()
                .map(|p| std::fs::read(p).ok())
                .collect(),
            report: report(&side)
                .and_then(|p| std::fs::read(p).ok())
                .map(|b| normalize_output(&b, &case_tag)),
            ms,
        }
    };
    let a = run_side("rs");
    let b = run_side("rs2");
    // Multi-island runs remesh islands on parallel threads, so engine
    // diagnostics interleave run-to-run: line multisets must match, while
    // artifacts and the report file must be byte-identical.
    let stdout_same = sorted(&a.stdout) == sorted(&b.stdout);
    let stderr_same = sorted(&a.stderr) == sorted(&b.stderr);
    let deterministic = a.code == b.code
        && a.artifacts == b.artifacts
        && stdout_same
        && stderr_same
        && a.report == b.report;
    if !deterministic {
        failures.push(format!(
            "remesh-{name}: NOT deterministic run-to-run (exit {:?}/{:?}, artifacts equal {}, stdout equal {}, stderr equal {}, report equal {})",
            a.code,
            b.code,
            a.artifacts == b.artifacts,
            stdout_same,
            stderr_same,
            a.report == b.report,
        ));
    }
    let labels: Vec<String> = outputs(Path::new("SIDE"))
        .iter()
        .map(|p| p.to_string_lossy().replace("SIDE/", ""))
        .collect();
    let mut text = format!("exit: {:?}\nartifacts:\n", a.code);
    for (label, art) in labels.iter().zip(&a.artifacts) {
        let state = if art.is_some() { "present" } else { "missing" };
        text.push_str(&format!("{label} {state}\n"));
    }
    text.push_str(&format!(
        "report: {}\ncounts:\n",
        if a.report.is_some() {
            "present"
        } else {
            "absent"
        }
    ));
    for (label, art) in labels.iter().zip(&a.artifacts) {
        if let Some(bytes) = art {
            if label.ends_with(".obj") {
                text.push_str(&format!("{label} {}\n", obj_counts(bytes)));
            }
        }
    }
    for line in a.stdout.iter().chain(a.report.iter().flatten()) {
        if is_count_line(line) {
            text.push_str(line);
            text.push('\n');
        }
    }
    text.push_str("diagnostics:\n");
    let diagnostics: Vec<String> = a
        .stdout
        .iter()
        .chain(&a.stderr)
        .filter(|l| is_diagnostic_line(l) && !is_count_line(l))
        .cloned()
        .collect();
    for line in sorted(&diagnostics) {
        text.push_str(&line);
        text.push('\n');
    }
    check_golden_counts(name, &text, failures);
    CaseRow {
        name: name.to_string(),
        tier: if deterministic { "golden" } else { "NONDET" }.to_string(),
        detail: format!("exit={:?} ms={:.0}/{:.0}", a.code, a.ms, b.ms),
    }
}

#[test]
fn arg_matrix() {
    let bins = bins();
    let mut failures: Vec<String> = Vec::new();
    // Help / version (short, long, `=` suffixed, mid-parse).
    arg_case(&bins, "help-long", &["--help"], &mut failures);
    arg_case(&bins, "help-short", &["-h"], &mut failures);
    arg_case(&bins, "help-eq", &["--help=x"], &mut failures);
    arg_case(
        &bins,
        "help-midparse",
        &["-i", "a", "--help", "--bogus"],
        &mut failures,
    );
    arg_case(&bins, "version-long", &["--version"], &mut failures);
    arg_case(&bins, "version-short", &["-v"], &mut failures);
    // Unknown / missing / required.
    arg_case(&bins, "unknown", &["--bogus"], &mut failures);
    arg_case(
        &bins,
        "unknown-after-valid",
        &["-i", "a", "-o", "b", "--bogus"],
        &mut failures,
    );
    arg_case(
        &bins,
        "positional",
        &["guidespos", "-i", "a", "-o", "b"],
        &mut failures,
    );
    arg_case(&bins, "missing-value", &["--input"], &mut failures);
    arg_case(
        &bins,
        "missing-value-2",
        &["-i", "a", "--output"],
        &mut failures,
    );
    arg_case(&bins, "missing-required", &["-i", "a"], &mut failures);
    arg_case(&bins, "missing-all", &[], &mut failures);
    arg_case(
        &bins,
        "input-prefix-noeq",
        &["--inputx", "a"],
        &mut failures,
    );
    // Enum flags.
    arg_case(
        &bins,
        "uvs-bad",
        &["--uvs", "maybe", "-i", "a", "-o", "b"],
        &mut failures,
    );
    arg_case(&bins, "uvs-missing", &["--uvs"], &mut failures);
    arg_case(
        &bins,
        "symmetry-bad",
        &["--symmetry", "diag", "-i", "a", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "modeltype-bad",
        &["--model-type", "nurbs", "-i", "a", "-o", "b"],
        &mut failures,
    );
    // Integer parsing incl. strtol quirks.
    arg_case(
        &bins,
        "tq-bad",
        &["--target-quads", "xyz", "-i", "a", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "tq-neg",
        &["--target-quads", "-5", "-i", "a", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "tq-negzero",
        &["--target-quads", "-0", "-i", "nofile", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "tq-plus",
        &["--target-quads", "+5", "-i", "nofile", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "tq-leadspace",
        &["--target-quads", " 5", "-i", "nofile", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "tq-trailspace",
        &["--target-quads", "5 ", "-i", "a", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "tq-empty",
        &["--target-quads", "", "-i", "nofile", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "tq-huge",
        &[
            "--target-quads",
            "99999999999999999999999",
            "-i",
            "nofile",
            "-o",
            "b",
        ],
        &mut failures,
    );
    arg_case(
        &bins,
        "tq-2p32",
        &["--target-quads", "4294967296", "-i", "nofile", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "tq-zero",
        &["--target-quads", "0", "-i", "nofile", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "tq-underscore",
        &["--target-quads", "1_0", "-i", "a", "-o", "b"],
        &mut failures,
    );
    // Double parsing incl. strtod quirks.
    arg_case(
        &bins,
        "es-bad",
        &["--edge-scaling", "1.5x", "-i", "a", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "es-empty",
        &["--edge-scaling", "", "-i", "nofile", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "es-inf",
        &["--edge-scaling", "inf", "-i", "nofile", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "es-nan",
        &["--edge-scaling", "nan", "-i", "nofile", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "es-nan-upper",
        &["--edge-scaling", "NAN", "-i", "nofile", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "es-infinity",
        &["--edge-scaling", "infinity", "-i", "nofile", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "es-huge",
        &["--edge-scaling", "1e999", "-i", "nofile", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "es-neg",
        &["--edge-scaling", "-2.5", "-i", "nofile", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "es-leadspace",
        &["--edge-scaling", " 1.5", "-i", "nofile", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "es-trailspace",
        &["--edge-scaling", "1.5 ", "-i", "a", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "es-underscore",
        &["--edge-scaling", "1_0", "-i", "a", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "adapt-dot5",
        &["--adaptivity", ".5", "-i", "nofile", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "adapt-5dot",
        &["--adaptivity", "5.", "-i", "nofile", "-o", "b"],
        &mut failures,
    );
    // LOD lists.
    arg_case(
        &bins,
        "lods-zero",
        &["--lods", "10,0,5", "-i", "a", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "lods-empty",
        &["--lods", "", "-i", "a", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "lods-spaces",
        &["--lods", "100, 200", "-i", "nofile", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "lods-trailcomma",
        &["--lods", "100,", "-i", "a", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "lods-bad",
        &["--lods", "100,x", "-i", "a", "-o", "b"],
        &mut failures,
    );
    // `--flag=value` forms.
    arg_case(
        &bins,
        "eq-forms",
        &["--input=x", "--output=y", "--target-quads=abc"],
        &mut failures,
    );
    arg_case(&bins, "eq-empty", &["--input=", "--output="], &mut failures);
    arg_case(
        &bins,
        "quiet-twice",
        &["--quiet", "--quiet", "-i", "a", "-o", "b"],
        &mut failures,
    );
    arg_case(
        &bins,
        "quiet-eq",
        &["--quiet=x", "-i", "a", "-o", "b"],
        &mut failures,
    );
    arg_case(&bins, "short-eq", &["-i=x", "-o=y"], &mut failures);
    if !failures.is_empty() {
        panic!("arg matrix failures:\n{}", failures.join("\n====\n"));
    }
    println!("arg matrix: all green");
}

// Specified divergence from the retired C++ oracle: C++ `strtod` accepted
// hex floats and `nan(payload)`; the Rust parser rejects both loudly.
#[test]
fn arg_hex_rejected() {
    let bins = bins();
    for value in ["0x10", "0x1p3", "0X1P3", "nan(x)"] {
        let argv = vec![
            "--edge-scaling".to_string(),
            value.to_string(),
            "-i".to_string(),
            "nofile".to_string(),
            "-o".to_string(),
            "b".to_string(),
        ];
        let rs = run(&bins.rs, &argv, RUN_TIMEOUT);
        assert!(!rs.timed_out);
        let err = String::from_utf8_lossy(&rs.stderr);
        assert!(
            err.contains("expects a number"),
            "{value}: rust unexpectedly accepts? stderr={err}"
        );
        assert_ne!(rs.code, Some(0));
    }
}

#[test]
fn io_failure_paths() {
    let bins = bins();
    let fx = fixtures_dir();
    let mut failures: Vec<String> = Vec::new();
    io_case(
        &bins,
        "missing-input",
        &|_| {
            vec![
                "-i".into(),
                "SHARED/does-not-exist.obj".into(),
                "-o".into(),
                "SIDE/out.obj".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "missing-input-glb",
        &|_| {
            vec![
                "-i".into(),
                "SHARED/does-not-exist.glb".into(),
                "-o".into(),
                "SIDE/out.obj".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "guides-missing",
        &|shared| {
            let fx = fixtures_dir();
            vec![
                "-i".into(),
                fx.join("nasty-single-tetra.obj")
                    .to_string_lossy()
                    .to_string(),
                "-o".into(),
                "SIDE/out.obj".into(),
                "--target-quads".into(),
                "50".into(),
                "--guides".into(),
                shared.join("no-guides.txt").to_string_lossy().to_string(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "guides-malformed",
        &|shared| {
            let fx = fixtures_dir();
            std::fs::write(shared.join("bad-guides.txt"), "1 2\n\n3 4 5 6\n").unwrap();
            vec![
                "-i".into(),
                fx.join("nasty-single-tetra.obj")
                    .to_string_lossy()
                    .to_string(),
                "-o".into(),
                "SIDE/out.obj".into(),
                "--target-quads".into(),
                "50".into(),
                "--guides".into(),
                "SHARED/bad-guides.txt".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "guides-empty",
        &|shared| {
            let fx = fixtures_dir();
            std::fs::write(shared.join("empty-guides.txt"), "# nothing\n\n").unwrap();
            vec![
                "-i".into(),
                fx.join("nasty-single-tetra.obj")
                    .to_string_lossy()
                    .to_string(),
                "-o".into(),
                "SIDE/out.obj".into(),
                "--target-quads".into(),
                "50".into(),
                "--guides".into(),
                "SHARED/empty-guides.txt".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "guides-single-point",
        &|shared| {
            let fx = fixtures_dir();
            std::fs::write(shared.join("single-guides.txt"), "1 2 3\n\n4 5 6\n").unwrap();
            vec![
                "-i".into(),
                fx.join("nasty-single-tetra.obj")
                    .to_string_lossy()
                    .to_string(),
                "-o".into(),
                "SIDE/out.obj".into(),
                "--target-quads".into(),
                "50".into(),
                "--guides".into(),
                "SHARED/single-guides.txt".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "features-malformed",
        &|shared| {
            let fx = fixtures_dir();
            std::fs::write(shared.join("bad-features.txt"), "1 2 three\n").unwrap();
            vec![
                "-i".into(),
                fx.join("nasty-single-tetra.obj")
                    .to_string_lossy()
                    .to_string(),
                "-o".into(),
                "SIDE/out.obj".into(),
                "--target-quads".into(),
                "50".into(),
                "--features".into(),
                "SHARED/bad-features.txt".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "density-missing",
        &|_| {
            let fx = fixtures_dir();
            vec![
                "-i".into(),
                fx.join("nasty-single-tetra.obj")
                    .to_string_lossy()
                    .to_string(),
                "-o".into(),
                "SIDE/out.obj".into(),
                "--target-quads".into(),
                "50".into(),
                "--density".into(),
                "SHARED/no-density.txt".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "density-malformed",
        &|shared| {
            let fx = fixtures_dir();
            std::fs::write(shared.join("bad-density.txt"), "1.0\nabc\n").unwrap();
            vec![
                "-i".into(),
                fx.join("nasty-single-tetra.obj")
                    .to_string_lossy()
                    .to_string(),
                "-o".into(),
                "SIDE/out.obj".into(),
                "--target-quads".into(),
                "50".into(),
                "--density".into(),
                "SHARED/bad-density.txt".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "density-empty",
        &|shared| {
            let fx = fixtures_dir();
            std::fs::write(shared.join("empty-density.txt"), "# nothing\n").unwrap();
            vec![
                "-i".into(),
                fx.join("nasty-single-tetra.obj")
                    .to_string_lossy()
                    .to_string(),
                "-o".into(),
                "SIDE/out.obj".into(),
                "--target-quads".into(),
                "50".into(),
                "--density".into(),
                "SHARED/empty-density.txt".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "density-mismatch",
        &|shared| {
            let fx = fixtures_dir();
            std::fs::write(shared.join("short-density.txt"), "1.0\n1.0\n").unwrap();
            vec![
                "-i".into(),
                fx.join("nasty-single-tetra.obj")
                    .to_string_lossy()
                    .to_string(),
                "-o".into(),
                "SIDE/out.obj".into(),
                "--target-quads".into(),
                "50".into(),
                "--quiet".into(),
                "--density".into(),
                "SHARED/short-density.txt".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "output-unwritable",
        &|_| {
            let fx = fixtures_dir();
            vec![
                "-i".into(),
                fx.join("nasty-single-tetra.obj")
                    .to_string_lossy()
                    .to_string(),
                "-o".into(),
                "SIDE/no-such-dir/out.obj".into(),
                "--target-quads".into(),
                "50".into(),
                "--quiet".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "report-unwritable",
        &|_| {
            let fx = fixtures_dir();
            vec![
                "-i".into(),
                fx.join("nasty-single-tetra.obj")
                    .to_string_lossy()
                    .to_string(),
                "-o".into(),
                "SIDE/out.obj".into(),
                "--target-quads".into(),
                "50".into(),
                "--quiet".into(),
                "--report".into(),
                "SIDE/no-such-dir/report.txt".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "oor-loader-error",
        &|_| {
            let fx = fixtures_dir();
            vec![
                "-i".into(),
                fx.join("nasty-oor.obj").to_string_lossy().to_string(),
                "-o".into(),
                "SIDE/out.obj".into(),
                "--target-quads".into(),
                "50".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "batch-empty-dir",
        &|shared| {
            std::fs::create_dir_all(shared.join("empty")).unwrap();
            vec![
                "-i".into(),
                "SHARED/empty".into(),
                "-o".into(),
                "SIDE/out".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "batch-output-is-file",
        &|shared| {
            let fx = fixtures_dir();
            std::fs::create_dir_all(shared.join("indir")).unwrap();
            std::fs::copy(
                fx.join("nasty-single-tetra.obj"),
                shared.join("indir/a.obj"),
            )
            .unwrap();
            std::fs::write(shared.join("afile"), "x").unwrap();
            vec![
                "-i".into(),
                "SHARED/indir".into(),
                "-o".into(),
                "SHARED/afile".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "batch-guides-rejected",
        &|shared| {
            let fx = fixtures_dir();
            std::fs::create_dir_all(shared.join("indir")).unwrap();
            std::fs::copy(
                fx.join("nasty-single-tetra.obj"),
                shared.join("indir/a.obj"),
            )
            .unwrap();
            std::fs::write(shared.join("g.txt"), "0 0 0\n1 0 0\n").unwrap();
            vec![
                "-i".into(),
                "SHARED/indir".into(),
                "-o".into(),
                "SIDE/out".into(),
                "--guides".into(),
                "SHARED/g.txt".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "batch-features-rejected",
        &|shared| {
            let fx = fixtures_dir();
            std::fs::create_dir_all(shared.join("indir")).unwrap();
            std::fs::copy(
                fx.join("nasty-single-tetra.obj"),
                shared.join("indir/a.obj"),
            )
            .unwrap();
            std::fs::write(shared.join("f.txt"), "0 0 0\n1 0 0\n").unwrap();
            vec![
                "-i".into(),
                "SHARED/indir".into(),
                "-o".into(),
                "SIDE/out".into(),
                "--features".into(),
                "SHARED/f.txt".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "batch-density-rejected",
        &|shared| {
            let fx = fixtures_dir();
            std::fs::create_dir_all(shared.join("indir")).unwrap();
            std::fs::copy(
                fx.join("nasty-single-tetra.obj"),
                shared.join("indir/a.obj"),
            )
            .unwrap();
            std::fs::write(shared.join("d.txt"), "1.0\n").unwrap();
            vec![
                "-i".into(),
                "SHARED/indir".into(),
                "-o".into(),
                "SIDE/out".into(),
                "--density".into(),
                "SHARED/d.txt".into(),
            ]
        },
        &mut failures,
    );
    io_case(
        &bins,
        "batch-unreadable-dir",
        &|_| {
            vec![
                "-i".into(),
                "SHARED/does-not-exist".into(),
                "-o".into(),
                "SIDE/out".into(),
            ]
        },
        &mut failures,
    );
    let _ = &fx;
    if !failures.is_empty() {
        panic!("io failure-path failures:\n{}", failures.join("\n====\n"));
    }
    println!("io failure paths: all green");
}

// --- remesh oracle (censused flag matrix x mesh corpus) ----------------------

// Materialize shared (side-independent) inputs under the case dir.
// Idempotent: `make_args` runs per census run, rewriting the same bytes.
fn shared_dir(side: &Path) -> PathBuf {
    let shared = side.parent().expect("side has parent").join("shared");
    std::fs::create_dir_all(&shared).expect("shared dir");
    shared
}

fn s(path: &Path) -> String {
    path.to_string_lossy().to_string()
}

#[test]
fn remesh_contract() {
    let bins = bins();
    let fx = fixtures_dir();
    let mut failures: Vec<String> = Vec::new();
    let mut rows: Vec<CaseRow> = Vec::new();

    // Single-file quiet base: sphere, grid, two-cube, tetra variants.
    let sphere = s(&fx.join("sphere-pole.obj"));
    let tetra_glb = s(&fx.join("tetra.glb"));
    for nasty in [
        "nasty-multicomp.obj",
        "nasty-floaters.obj",
        "nasty-soup.obj",
        "nasty-nan.obj",
        "nasty-degenerate.obj",
        "nasty-nonmanifold.obj",
        "nasty-single-tetra.obj",
    ] {
        let input = s(&fx.join(nasty));
        let case = nasty.strip_suffix(".obj").unwrap().to_string();
        rows.push(remesh_case(
            &bins,
            &case,
            &|side| {
                vec![
                    "-i".into(),
                    input.clone(),
                    "-o".into(),
                    s(&side.join("out.obj")),
                    "--target-quads".into(),
                    "200".into(),
                    "--quiet".into(),
                ]
            },
            &|side| vec![side.join("out.obj")],
            &|_| None,
            &mut failures,
        ));
    }
    // Engine-failure exits (audit the ungated input diagnostics).
    for nasty in [
        "nasty-empty.obj",
        "nasty-all-degenerate.obj",
        "nasty-all-zeroarea.obj",
    ] {
        let input = s(&fx.join(nasty));
        let case = nasty.strip_suffix(".obj").unwrap().to_string();
        rows.push(remesh_case(
            &bins,
            &case,
            &|side| {
                vec![
                    "-i".into(),
                    input.clone(),
                    "-o".into(),
                    s(&side.join("out.obj")),
                    "--target-quads".into(),
                    "200".into(),
                    "--quiet".into(),
                ]
            },
            &|side| vec![side.join("out.obj")],
            &|_| None,
            &mut failures,
        ));
    }

    rows.push(remesh_case(
        &bins,
        "sphere-quiet",
        &|side| {
            vec![
                "-i".into(),
                sphere.clone(),
                "-o".into(),
                s(&side.join("out.obj")),
                "--target-quads".into(),
                "200".into(),
                "--quiet".into(),
                "--report".into(),
                s(&side.join("report.txt")),
            ]
        },
        &|side| vec![side.join("out.obj")],
        &|side| Some(side.join("report.txt")),
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "sphere-loud",
        &|side| {
            vec![
                "-i".into(),
                sphere.clone(),
                "-o".into(),
                s(&side.join("out.obj")),
                "--target-quads".into(),
                "200".into(),
                "--report".into(),
                s(&side.join("report.txt")),
            ]
        },
        &|side| vec![side.join("out.obj")],
        &|side| Some(side.join("report.txt")),
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "sphere-uvs",
        &|side| {
            vec![
                "-i".into(),
                sphere.clone(),
                "-o".into(),
                s(&side.join("out.obj")),
                "--target-quads".into(),
                "200".into(),
                "--quiet".into(),
                "--uvs".into(),
                "on".into(),
            ]
        },
        &|side| vec![side.join("out.obj")],
        &|_| None,
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "sphere-symmetry",
        &|side| {
            vec![
                "-i".into(),
                sphere.clone(),
                "-o".into(),
                s(&side.join("out.obj")),
                "--target-quads".into(),
                "200".into(),
                "--quiet".into(),
                "--symmetry".into(),
                "auto".into(),
            ]
        },
        &|side| vec![side.join("out.obj")],
        &|_| None,
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "sphere-hard",
        &|side| {
            vec![
                "-i".into(),
                sphere.clone(),
                "-o".into(),
                s(&side.join("out.obj")),
                "--target-quads".into(),
                "200".into(),
                "--quiet".into(),
                "--model-type".into(),
                "hardsurface".into(),
                "--sharp-edge".into(),
                "45".into(),
                "--smooth-normal".into(),
                "30".into(),
            ]
        },
        &|side| vec![side.join("out.obj")],
        &|_| None,
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "sphere-scaled",
        &|side| {
            vec![
                "-i".into(),
                sphere.clone(),
                "-o".into(),
                s(&side.join("out.obj")),
                "--target-quads".into(),
                "200".into(),
                "--quiet".into(),
                "--edge-scaling".into(),
                "2".into(),
                "--adaptivity".into(),
                "0.3".into(),
                "--anisotropy".into(),
                "0.5".into(),
            ]
        },
        &|side| vec![side.join("out.obj")],
        &|_| None,
        &mut failures,
    ));
    // Grid cases (generated 16x16 analytic grid + constraint files).
    let grid_args = |side: &Path, extra: &[&str]| {
        let shared = shared_dir(side);
        write_grid_obj(&shared.join("grid.obj"), 16, 16);
        let mut argv = vec![
            "-i".into(),
            s(&shared.join("grid.obj")),
            "-o".into(),
            s(&side.join("out.obj")),
            "--target-quads".into(),
            "300".into(),
        ];
        argv.extend(
            extra
                .iter()
                .map(|e| e.replace("SHARED", &s(&shared)).replace("SIDE", &s(side))),
        );
        argv
    };
    rows.push(remesh_case(
        &bins,
        "grid-quiet",
        &|side| grid_args(side, &["--quiet", "--report", "SIDE/report.txt"]),
        &|side| vec![side.join("out.obj")],
        &|side| Some(side.join("report.txt")),
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "grid-loud",
        &|side| grid_args(side, &[]),
        &|side| vec![side.join("out.obj")],
        &|_| None,
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "grid-guides",
        &|side| {
            let shared = shared_dir(side);
            write_grid_guides(&shared.join("guides.txt"), 16, 16);
            grid_args(side, &["--quiet", "--guides", "SHARED/guides.txt"])
        },
        &|side| vec![side.join("out.obj")],
        &|_| None,
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "grid-features",
        &|side| {
            let shared = shared_dir(side);
            write_grid_features(&shared.join("features.txt"), 16, 16);
            grid_args(side, &["--quiet", "--features", "SHARED/features.txt"])
        },
        &|side| vec![side.join("out.obj")],
        &|_| None,
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "grid-density",
        &|side| {
            let shared = shared_dir(side);
            write_grid_density(&shared.join("density.txt"), 16, 16);
            grid_args(side, &["--quiet", "--density", "SHARED/density.txt"])
        },
        &|side| vec![side.join("out.obj")],
        &|_| None,
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "grid-all-constrained",
        &|side| {
            let shared = shared_dir(side);
            write_grid_guides(&shared.join("guides.txt"), 16, 16);
            write_grid_features(&shared.join("features.txt"), 16, 16);
            write_grid_density(&shared.join("density.txt"), 16, 16);
            grid_args(
                side,
                &[
                    "--quiet",
                    "--guides",
                    "SHARED/guides.txt",
                    "--features",
                    "SHARED/features.txt",
                    "--density",
                    "SHARED/density.txt",
                ],
            )
        },
        &|side| vec![side.join("out.obj")],
        &|_| None,
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "grid-symmetry-off",
        &|side| grid_args(side, &["--quiet", "--symmetry", "off"]),
        &|side| vec![side.join("out.obj")],
        &|_| None,
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "grid-hard-alias",
        &|side| grid_args(side, &["--quiet", "--model-type", "hard-surface"]),
        &|side| vec![side.join("out.obj")],
        &|_| None,
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "grid-eq-forms",
        &|side| {
            let shared = shared_dir(side);
            write_grid_obj(&shared.join("grid.obj"), 16, 16);
            vec![
                format!("--input={}", s(&shared.join("grid.obj"))),
                format!("--output={}", s(&side.join("out.obj"))),
                "--target-quads=300".into(),
                "--quiet".into(),
            ]
        },
        &|side| vec![side.join("out.obj")],
        &|_| None,
        &mut failures,
    ));
    // Multi-island (generated two disjoint cubes).
    let cubes_args = |side: &Path, extra: &[&str]| {
        let shared = shared_dir(side);
        write_two_cubes_obj(&shared.join("cubes.obj"));
        let mut argv = vec![
            "-i".into(),
            s(&shared.join("cubes.obj")),
            "-o".into(),
            s(&side.join("out.obj")),
            "--target-quads".into(),
            "200".into(),
        ];
        argv.extend(
            extra
                .iter()
                .map(|e| e.replace("SHARED", &s(&shared)).replace("SIDE", &s(side))),
        );
        argv
    };
    rows.push(remesh_case(
        &bins,
        "twocubes-quiet",
        &|side| cubes_args(side, &["--quiet", "--report", "SIDE/report.txt"]),
        &|side| vec![side.join("out.obj")],
        &|side| Some(side.join("report.txt")),
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "twocubes-loud",
        &|side| cubes_args(side, &[]),
        &|side| vec![side.join("out.obj")],
        &|_| None,
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "twocubes-symmetry",
        &|side| cubes_args(side, &["--quiet", "--symmetry", "auto"]),
        &|side| vec![side.join("out.obj")],
        &|_| None,
        &mut failures,
    ));
    // GLB input/output.
    rows.push(remesh_case(
        &bins,
        "tetra-glb-in",
        &|side| {
            vec![
                "-i".into(),
                tetra_glb.clone(),
                "-o".into(),
                s(&side.join("out.obj")),
                "--target-quads".into(),
                "50".into(),
                "--quiet".into(),
            ]
        },
        &|side| vec![side.join("out.obj")],
        &|_| None,
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "tetra-glb-out",
        &|side| {
            let shared = shared_dir(side);
            write_tetra_obj(&shared.join("tet.obj"));
            vec![
                "-i".into(),
                s(&shared.join("tet.obj")),
                "-o".into(),
                s(&side.join("out.glb")),
                "--target-quads".into(),
                "50".into(),
                "--quiet".into(),
            ]
        },
        &|side| vec![side.join("out.glb")],
        &|_| None,
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "tetra-glb-uvs",
        &|side| {
            let shared = shared_dir(side);
            write_tetra_obj(&shared.join("tet.obj"));
            vec![
                "-i".into(),
                s(&shared.join("tet.obj")),
                "-o".into(),
                s(&side.join("out.glb")),
                "--target-quads".into(),
                "50".into(),
                "--quiet".into(),
                "--uvs".into(),
                "on".into(),
            ]
        },
        &|side| vec![side.join("out.glb")],
        &|_| None,
        &mut failures,
    ));
    // LOD chains.
    rows.push(remesh_case(
        &bins,
        "lods-quiet",
        &|side| {
            grid_args(
                side,
                &[
                    "--quiet",
                    "--lods",
                    "300,150",
                    "--report",
                    "SIDE/report.txt",
                ],
            )
        },
        &|side| vec![side.join("out_lod0.obj"), side.join("out_lod1.obj")],
        &|side| Some(side.join("report.txt")),
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "lods-loud-guides",
        &|side| {
            let shared = shared_dir(side);
            write_grid_guides(&shared.join("guides.txt"), 16, 16);
            grid_args(
                side,
                &["--lods", "300,150", "--guides", "SHARED/guides.txt"],
            )
        },
        &|side| vec![side.join("out_lod0.obj"), side.join("out_lod1.obj")],
        &|_| None,
        &mut failures,
    ));
    // Batch runs (mixed obj/glb + ignored txt + ignored subdir).
    let batch_setup = |side: &Path, lods: bool, quiet: bool, with_bad: bool| {
        let shared = shared_dir(side);
        let indir = shared.join("indir");
        std::fs::create_dir_all(indir.join("subdir")).unwrap();
        write_tetra_obj(&indir.join("a.obj"));
        std::fs::copy(fx.join("nasty-multicomp.obj"), indir.join("b.obj")).unwrap();
        std::fs::copy(fx.join("tetra.glb"), indir.join("c.glb")).unwrap();
        std::fs::write(indir.join("skip.txt"), "not a mesh\n").unwrap();
        write_tetra_obj(&indir.join("subdir").join("d.obj"));
        if with_bad {
            std::fs::copy(fx.join("nasty-oor.obj"), indir.join("bad.obj")).unwrap();
        }
        let mut argv = vec![
            "-i".into(),
            s(&indir),
            "-o".into(),
            s(&side.join("out")),
            "--target-quads".into(),
            "100".into(),
        ];
        if lods {
            argv.push("--lods".into());
            argv.push("150,80".into());
        }
        if quiet {
            argv.push("--quiet".into());
        }
        argv.push("--report".into());
        argv.push(s(&side.join("report.txt")));
        argv
    };
    rows.push(remesh_case(
        &bins,
        "batch-quiet",
        &|side| batch_setup(side, false, true, false),
        &|side| {
            vec![
                side.join("out/a.obj"),
                side.join("out/b.obj"),
                side.join("out/c.glb"),
            ]
        },
        &|side| Some(side.join("report.txt")),
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "batch-loud",
        &|side| batch_setup(side, false, false, false),
        &|side| {
            vec![
                side.join("out/a.obj"),
                side.join("out/b.obj"),
                side.join("out/c.glb"),
            ]
        },
        &|side| Some(side.join("report.txt")),
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "batch-lods",
        &|side| batch_setup(side, true, true, false),
        &|side| {
            vec![
                side.join("out/a_lod0.obj"),
                side.join("out/a_lod1.obj"),
                side.join("out/b_lod0.obj"),
                side.join("out/b_lod1.obj"),
                side.join("out/c_lod0.glb"),
                side.join("out/c_lod1.glb"),
            ]
        },
        &|side| Some(side.join("report.txt")),
        &mut failures,
    ));
    rows.push(remesh_case(
        &bins,
        "batch-with-failure",
        &|side| batch_setup(side, false, true, true),
        &|side| {
            vec![
                side.join("out/a.obj"),
                side.join("out/b.obj"),
                side.join("out/c.glb"),
            ]
        },
        &|side| Some(side.join("report.txt")),
        &mut failures,
    ));

    println!("--- remesh contract table ---");
    for row in &rows {
        println!("{:<24} {:<9} {}", row.name, row.tier, row.detail);
    }
    if !failures.is_empty() {
        panic!("remesh contract failures:\n{}", failures.join("\n====\n"));
    }
}
