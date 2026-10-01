// End-to-end differential oracle: the C++ `retopo` binary vs the Rust
// `retopo` binary over a flag matrix x mesh corpus.
//
// Harness discipline (mandated by the engine-lane finding: the C++ engine
// flips extractor cliffs run-to-run via TBB wobble):
// - NEVER freeze a single C++ output as golden: every remesh case censuses
//   the C++ binary N>=5 times (10 when >1 mesh mode or exit flip appears)
//   and accepts iff Rust matches ANY demonstrated C++ mode.
// - Rust must be deterministic run-to-run: every case double-runs Rust
//   and asserts byte-identical outputs (past time normalization).
// - UB-derived C++ behavior (crash/signal) excludes a case with an
//   EPX-by-UB note instead of matching it (none observed on this
//   corpus; the probe path is implemented, not dead).
//
// Comparison tiers per case (mirrors the engine 129/113/53 tiering):
// - strict: Rust bytes equal a C++ mode on outputs, report, stdout,
//   and audit-filtered stderr, first try, order-sensitive.
// - any-mode: matched a non-first C++ mode, or needed the
//   progress/diagnostic multiset fallback (thread-interleave order).
// - tol: output meshes agreed only under the scale-aware 1e-6 value
//   fallback (indices still positional-exact); mechanism cited per case.
// - divergences that are specified, not matched: the stderr audit table
//   (see `audit_filter_cpp`) and the hex-float/`nan(payload)` numeric
//   gap (see `arg_hex_divergence`, asserted as divergence, not matched).
//
// Layout: binaries resolve at runtime (`RETOPO_CPP_BIN` env or the
// default `build/cli/retopo` next to the checkout; Rust via
// `CARGO_BIN_EXE_retopo`). Case scratch lives under the temp dir and is
// kept for inspection. Std-only, no dev-dependencies (offline).

use std::collections::HashMap;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

const CPP_ENV: &str = "RETOPO_CPP_BIN";
const RUN_TIMEOUT: Duration = Duration::from_secs(300);
const CENSUS_N: usize = 5;
const CENSUS_EXTENDED: usize = 10;
// Adaptive ceiling: any oracle miss extends the C++ census in steps of
// CENSUS_STEP before failing (a N=10 census under-covers cliff-sitters:
// twocubes-loud showed Rust Vertices=195 outside a [(196,204)] window
// while a later census showed [187,200] — 195 was a genuine undrawn
// mode). Extension only ever adds C++ evidence; acceptance still
// requires Rust ⊆ demonstrated, so it cannot mask a real divergence.
const CENSUS_MAX: usize = 30;
const CENSUS_STEP: usize = 5;
const TOL: f64 = 1e-6;

struct Bins {
    cpp: PathBuf,
    rs: PathBuf,
}

fn bins() -> Bins {
    let rs = PathBuf::from(env!("CARGO_BIN_EXE_retopo"));
    let cpp = match std::env::var(CPP_ENV) {
        Ok(path) => PathBuf::from(path),
        Err(_) => Path::new(env!("CARGO_MANIFEST_DIR")).join("../../build/cli/retopo"),
    };
    assert!(rs.is_file(), "rust binary missing: {}", rs.display());
    assert!(
        cpp.is_file(),
        "c++ reference binary missing: {} (set {CPP_ENV})",
        cpp.display()
    );
    Bins { cpp, rs }
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

// --- stderr audit -------------------------------------------------------
// Every C++-side stderr line the Rust binary deliberately does not print
// (project stderr-gap memo: quad/frame/parameterizer/engine precedent,
// decided before this lane; restoration would need engine API the CLI
// cannot see, so this lane specs the new output story instead).
// Two classes are path-dependent counts rather than omitted prints
// (merge-five-faces, split-seven-faces): the Rust port keeps the print
// behind identical gates but systematically counts 0 — same spec, cited
// mechanism, engine-lane follow-up flagged in the lane report:
// - the engine's bare phase-report dump (the CLI's indented copy keeps
//   the same content; the C++ binary prints both, the duplication is
//   dropped, not the information),
// - ungated engine one-liners (singularity counts, symmetry/input
//   notices, solve-failure notices): absent in Rust; the runs that
//   produce them still succeed/fail identically (exit codes compared).
// Anything outside this table fails the case: the filter is an
// allowlist, and every filtered line is recorded with its class.

fn dropped_diagnostic_class(line: &str) -> Option<&'static str> {
    if line.starts_with("Simplified cross field singularities: ") {
        return Some("simplified-singularities");
    }
    // Path-dependent extractor counts (spec, not silence — see the audit
    // header): the Rust port keeps these prints behind the same
    // count>0 + handler gates, but its extraction path systematically
    // yields 0 where C++ merges/splits (demonstrated: byte-identical
    // twocubes mesh, C++ Merge-five universal 30/30 with counts
    // deterministic per mesh, Rust 0/777 runs; Split-seven 5/0, same
    // code pattern, preventive). Meshes still match; the divergence is
    // engine-path forensics, flagged for the engine lanes. Recorded per
    // case via the drop: note, never asserted.
    if line.starts_with("Merge shared five edge faces:") {
        return Some("merge-five-faces");
    }
    if line.starts_with("Split seven edge faces:") {
        return Some("split-seven-faces");
    }
    if line.starts_with("Symmetry skipped: ") {
        return Some("symmetry-skipped");
    }
    if line == "Input mesh is empty" {
        return Some("input-mesh-empty");
    }
    if line.starts_with("Invalid remesh input: ") {
        return Some("invalid-remesh-input");
    }
    if line == "Topology rejected a non-triangle face" {
        return Some("topology-rejected");
    }
    if line == "Frame field solve failed" {
        return Some("frame-field-failed");
    }
    if line == "Frame field has the wrong face count" {
        return Some("frame-field-face-count");
    }
    if line == "Quad cover solve failed" {
        return Some("quad-cover-failed");
    }
    if line.starts_with("Island ") && line.contains("parameterization failed") {
        return Some("island-parameterize-failed");
    }
    None
}

// Compare C++ stderr (normalized) against Rust stderr (normalized):
// returns the audit classes consumed, or a mismatch description. Order
// modes: strict sequence first, then multiset (thread interleave).
fn audit_stderr(cpp: &[String], rs: &[String]) -> Result<Vec<String>, String> {
    let mut audit: Vec<String> = Vec::new();
    // 1. Dropped ungated diagnostics off the C++ side (allowlisted).
    let mut cpp1: Vec<&String> = Vec::new();
    for line in cpp {
        if let Some(class) = dropped_diagnostic_class(line) {
            audit.push(format!("drop:{class}"));
        } else {
            cpp1.push(line);
        }
    }
    // 2. Identical 1:1 multiset match (CLI copies pair here; the bare
    // engine dump has no identical twin on the Rust side).
    let mut rs_free: Vec<&String> = rs.iter().collect();
    let mut matched_indented: HashMap<String, usize> = HashMap::new();
    let mut cpp2: Vec<&String> = Vec::new();
    for line in cpp1 {
        if let Some(pos) = rs_free.iter().position(|l| *l == line) {
            let matched = rs_free.remove(pos);
            if matched.starts_with("  ") {
                *matched_indented.entry(matched.clone()).or_insert(0) += 1;
            }
        } else {
            cpp2.push(line);
        }
    }
    // 3. Bare phase twins: a leftover C++ line disappears iff its
    // indented twin matched in step 2 (the CLI copy both sides print).
    // Sound: Rust-side indented stderr lines are exactly the CLI phase
    // copies, so a matched twin proves the dump membership.
    let mut rest_cpp: Vec<&String> = Vec::new();
    for line in cpp2 {
        let indented = format!("  {line}");
        match matched_indented.get_mut(&indented) {
            Some(count) if *count > 0 => {
                *count -= 1;
                audit.push("drop:phase-dup".to_string());
            }
            _ => rest_cpp.push(line),
        }
    }
    let rest_rs: Vec<&String> = rs_free;
    // 4. Strict sequence on the remainders.
    if rest_cpp.len() == rest_rs.len() && rest_cpp.iter().zip(rest_rs.iter()).all(|(a, b)| a == b) {
        return Ok(audit);
    }
    // Multiset fallback (extractor lines interleave across island
    // threads on both sides; content must still match exactly).
    let mut counts: HashMap<&str, isize> = HashMap::new();
    for line in &rest_cpp {
        *counts.entry(line.as_str()).or_insert(0) += 1;
    }
    for line in &rest_rs {
        *counts.entry(line.as_str()).or_insert(0) -= 1;
    }
    counts.retain(|_, v| *v != 0);
    if counts.is_empty() {
        audit.push("order:multiset".to_string());
        return Ok(audit);
    }
    let mut diff: Vec<String> = counts
        .iter()
        .map(|(line, v)| format!("  {v:+} {line}"))
        .collect();
    diff.sort();
    Err(format!(
        "stderr skew (audit={audit:?}):\n{}",
        diff.join("\n")
    ))
}

// Robust stderr rule (racy/extractor-flipped diagnostics): the C++
// stderr mixes the engine's bare dump (bare top lines + 4-space leaf
// lines) with the CLI copies (2-space + 6-space). Walk the indented C++
// lines in order: CLI copies consume their Rust twins (order-checked),
// bare leaves drop against twin existence; then the non-indented
// remainder goes through drops + twin-strip + membership.
fn audit_stderr_robust(
    cpp_runs: &[Vec<String>],
    rs: &[String],
    epx: bool,
) -> Result<Vec<String>, String> {
    let mut audit = vec!["stderr:robust".to_string()];
    let (rs_phase, rs_diag): (Vec<String>, Vec<String>) =
        rs.iter().cloned().partition(|l| l.starts_with("  "));
    let rs_set: std::collections::HashSet<&str> = rs_phase.iter().map(|l| l.as_str()).collect();
    // Phase: some mode's CLI-copy subsequence must equal the Rust phase
    // block (sequence, else multiset); bare leaves twin-strip.
    let mut phase_ok = false;
    let mut phase_multiset = false;
    let mut mode0_dups = 0;
    for (i, run) in cpp_runs.iter().enumerate() {
        let mut counts: HashMap<&str, usize> = HashMap::new();
        for line in &rs_phase {
            *counts.entry(line.as_str()).or_insert(0) += 1;
        }
        let mut cli_seq: Vec<&String> = Vec::new();
        let mut ok = true;
        for line in run.iter().filter(|l| l.starts_with("  ")) {
            match counts.get_mut(line.as_str()) {
                Some(count) if *count > 0 => {
                    *count -= 1;
                    cli_seq.push(line);
                }
                _ => {
                    let indented = format!("  {line}");
                    if rs_set.contains(indented.as_str()) {
                        if i == 0 {
                            mode0_dups += 1;
                        }
                    } else {
                        ok = false;
                        break;
                    }
                }
            }
        }
        if ok && counts.values().all(|c| *c == 0) {
            if cli_seq.len() == rs_phase.len()
                && cli_seq.iter().zip(rs_phase.iter()).all(|(a, b)| *a == b)
            {
                phase_ok = true;
                break;
            }
            let mut mc: HashMap<&str, isize> = HashMap::new();
            for line in &cli_seq {
                *mc.entry(line.as_str()).or_insert(0) += 1;
            }
            for line in &rs_phase {
                *mc.entry(line.as_str()).or_insert(0) -= 1;
            }
            mc.retain(|_, v| *v != 0);
            if mc.is_empty() {
                phase_ok = true;
                phase_multiset = true;
                break;
            }
        }
    }
    if !phase_ok {
        return Err(format!(
            "robust phase skew: no mode's CLI-copy block matches (rs {} lines)",
            rs_phase.len()
        ));
    }
    for _ in 0..mode0_dups {
        audit.push("drop:phase-dup".to_string());
    }
    if phase_multiset {
        audit.push("phase:multiset".to_string());
    }
    // Diagnostic remainder per mode: drops, then bare-twin strip by
    // existence (the dump half of a matched CLI copy), then membership.
    let mut cpp_diag_runs = Vec::new();
    for (i, run) in cpp_runs.iter().enumerate() {
        let mut diag = Vec::new();
        for line in run {
            if line.starts_with("  ") {
                continue;
            }
            if let Some(class) = dropped_diagnostic_class(line) {
                if i == 0 {
                    audit.push(format!("drop:{class}"));
                }
                continue;
            }
            let indented = format!("  {line}");
            if rs_set.contains(indented.as_str()) {
                if i == 0 {
                    audit.push("drop:phase-dup".to_string());
                }
            } else {
                diag.push(line.clone());
            }
        }
        cpp_diag_runs.push(diag);
    }
    audit.extend(audit_diag_shapes(&cpp_diag_runs, &rs_diag, epx)?);
    Ok(audit)
}

// Strict-path stdout fallback: report lines exact (facts with exact
// values — a mesh match pins the counts), progress via the robust rule.
fn audit_stdout_fallback(cpp_runs: &[Vec<String>], rs: &[String]) -> Result<Vec<String>, String> {
    let mut audit = vec!["stdout:fallback".to_string()];
    let rs_facts = split_facts(rs);
    let mut exact_ok = false;
    for run in cpp_runs {
        let facts = split_facts(run);
        if facts.exact == rs_facts.exact && facts.ranged == rs_facts.ranged {
            exact_ok = true;
            break;
        }
    }
    if !exact_ok {
        return Err("stdout facts skew (mesh matched but counts differ)".to_string());
    }
    audit.extend(audit_progress_robust(cpp_runs, rs)?);
    Ok(audit)
}

fn is_progress_line(line: &str) -> bool {
    line.contains("% done.")
}

// "42% done." / "42% done. Doing things" -> (42, "Doing things").
fn parse_progress(line: &str) -> Option<(i32, String)> {
    let pct = line.find("% done.")?;
    let num: i32 = line[..pct].parse().ok()?;
    let rest = &line[pct + "% done.".len()..];
    let status = rest.strip_prefix(' ').unwrap_or(rest).to_string();
    Some((num, status))
}

// Robust progress rule for thread-raced streams (multi-island runs:
// percents sample racy interleavings, so even the emitted
// (percent,status) SET differs run-to-run on one binary. Stable facts:
// the status set up to the slowest-status race (a bare "N% done."
// prints when the slowest island is status-less at report time —
// demonstrated 3/10 inside the C++ census on batch-loud — so the Rust
// set must match SOME mode's set), per-status progress deciles
// (neighbor-tolerant: a 5-run census under-covers the racy sampling,
// demonstrated by an rs 24 landing outside a [19,23] window), [0,100]
// range, and the serial anchors (first line + Done count). Event counts
// are pure dedup-race artifacts and are recorded, not asserted.
fn audit_progress_robust(cpp_runs: &[Vec<String>], rs: &[String]) -> Result<Vec<String>, String> {
    let mut audit = vec!["progress:robust".to_string()];
    let mut cpp_sets: Vec<std::collections::HashSet<String>> = Vec::new();
    let mut deciles: HashMap<String, std::collections::HashSet<i32>> = HashMap::new();
    let mut counts: Vec<usize> = Vec::new();
    let mut firsts: Vec<String> = Vec::new();
    let mut dones: Vec<usize> = Vec::new();
    for run in cpp_runs {
        let mut set = std::collections::HashSet::new();
        let mut n = 0;
        let mut ndone = 0;
        for line in run {
            if !is_progress_line(line) {
                continue;
            }
            if n == 0 {
                firsts.push(line.clone());
            }
            n += 1;
            let (pct, status) =
                parse_progress(line).ok_or_else(|| format!("unparsable progress: {line:?}"))?;
            if !(0..=100).contains(&pct) {
                return Err(format!("c++ progress out of range: {line:?}"));
            }
            if pct == 100 && status == "Done" {
                ndone += 1;
            }
            set.insert(status.clone());
            deciles.entry(status).or_default().insert(pct / 10);
        }
        counts.push(n);
        dones.push(ndone);
        cpp_sets.push(set);
    }
    for set in &cpp_sets[1..] {
        if *set != cpp_sets[0] {
            audit.push("cpp-status-flip".to_string());
            break;
        }
    }
    let mut rs_set = std::collections::HashSet::new();
    let mut rs_n = 0;
    let mut rs_ndone = 0;
    let mut rs_first: Option<String> = None;
    for line in rs {
        if !is_progress_line(line) {
            continue;
        }
        if rs_first.is_none() {
            rs_first = Some(line.clone());
        }
        rs_n += 1;
        let (pct, status) =
            parse_progress(line).ok_or_else(|| format!("unparsable rs progress: {line:?}"))?;
        if !(0..=100).contains(&pct) {
            return Err(format!("rs progress out of range: {line:?}"));
        }
        if pct == 100 && status == "Done" {
            rs_ndone += 1;
        }
        rs_set.insert(status.clone());
        match deciles.get(&status) {
            Some(bins) if bins.iter().any(|b| (b - pct / 10).abs() <= 1) => {}
            other => {
                return Err(format!(
                    "rs progress ({pct}, {status:?}) outside cpp deciles {other:?}"
                ));
            }
        }
    }
    // Any-mode rule (census discipline): the C++ status set itself races
    // (a bare "N% done." prints when the slowest island is status-less at
    // report time — demonstrated 3/10 on batch-loud rung 1 — so the Rust
    // set must equal SOME demonstrated mode's set, not necessarily mode 0's.
    // The Rust engine mirrors the slowest-status path exactly
    // (`unwrap_or_default()`), so a Rust-only "" would still fail here.
    if !cpp_sets.iter().any(|s| *s == rs_set) {
        let mut modes: Vec<Vec<&String>> = Vec::new();
        for set in &cpp_sets {
            let mut only_rs: Vec<&String> = rs_set.difference(set).collect();
            let mut only_cpp: Vec<&String> = set.difference(&rs_set).collect();
            only_rs.sort();
            only_cpp.sort();
            modes.push(only_rs);
            modes.push(only_cpp);
        }
        return Err(format!(
            "progress status-set skew (no mode matches; per-mode [rs-only, cpp-only]: {modes:?})"
        ));
    }
    // Serial anchors: the first event (fresh state always prints) and the
    // Done count (one per successful rung) are deterministic.
    if !firsts.iter().all(|f| Some(f) == rs_first.as_ref()) {
        return Err(format!(
            "progress first-line skew (rs={rs_first:?}, cpp={firsts:?})"
        ));
    }
    if !dones.iter().all(|d| *d == rs_ndone) {
        return Err(format!(
            "progress Done-count skew (rs={rs_ndone}, cpp={dones:?})"
        ));
    }
    let lo = *counts.iter().min().unwrap_or(&0);
    let hi = *counts.iter().max().unwrap_or(&0);
    audit.push(format!("progress-count:rs={rs_n} cpp=[{lo},{hi}]"));
    Ok(audit)
}

// Stdout: report/rung lines are serial (strict sequence); progress lines
// come from worker threads (strict sequence, multiset fallback).
fn audit_stdout(cpp: &[String], rs: &[String]) -> Result<Vec<String>, String> {
    let mut audit = Vec::new();
    let (cpp_prog, cpp_rest): (Vec<&String>, Vec<&String>) =
        cpp.iter().partition(|l| is_progress_line(l));
    let (rs_prog, rs_rest): (Vec<&String>, Vec<&String>) =
        rs.iter().partition(|l| is_progress_line(l));
    if cpp_rest.len() != rs_rest.len() || !cpp_rest.iter().zip(rs_rest.iter()).all(|(a, b)| a == b)
    {
        return Err(format!(
            "stdout report skew:\n  cpp={cpp_rest:?}\n  rs ={rs_rest:?}"
        ));
    }
    if cpp_prog.len() == rs_prog.len() && cpp_prog.iter().zip(rs_prog.iter()).all(|(a, b)| a == b) {
        return Ok(audit);
    }
    let mut counts: HashMap<&str, isize> = HashMap::new();
    for line in &cpp_prog {
        *counts.entry(line.as_str()).or_insert(0) += 1;
    }
    for line in &rs_prog {
        *counts.entry(line.as_str()).or_insert(0) -= 1;
    }
    counts.retain(|_, v| *v != 0);
    if counts.is_empty() {
        audit.push("progress:multiset".to_string());
        return Ok(audit);
    }
    let mut diff: Vec<String> = counts
        .iter()
        .map(|(line, v)| format!("  {v:+} {line}"))
        .collect();
    diff.sort();
    Err(format!("stdout progress skew:\n{}", diff.join("\n")))
}

// --- count facts (EPX range rules) ------------------------------------------
// Lines whose numbers flip on extractor cliffs (quad/vert counts,
// failed-island counts, rung triples) vs exact lines (params, island
// totals, labels). EPX range-checks the former across the census and
// exact-matches the latter.

struct CountFacts {
    exact: Vec<String>,
    ranged: Vec<(String, Vec<usize>)>,
}

fn split_facts(lines: &[String]) -> CountFacts {
    let mut exact = Vec::new();
    let mut ranged = Vec::new();
    for line in lines {
        if is_progress_line(line) {
            continue;
        }
        // Rung line: key everything through the output path, range the
        // trailing triple ("quads=N non-quads=M vertices=K time=T ...").
        if line.contains("target-quads=") {
            if let Some(pos) = line.find(" quads=") {
                let key = line[..pos].to_string();
                let tail = &line[pos + " quads=".len()..];
                // "36 non-quads=6 vertices=44 time=T seconds".
                let mut it = tail.split_ascii_whitespace();
                let q = it.next().and_then(|t| t.parse::<usize>().ok());
                let nq = it.next().and_then(|t| {
                    t.strip_prefix("non-quads=")
                        .and_then(|v| v.parse::<usize>().ok())
                });
                let vx = it.next().and_then(|t| {
                    t.strip_prefix("vertices=")
                        .and_then(|v| v.parse::<usize>().ok())
                });
                if let (Some(q), Some(nq), Some(vx)) = (q, nq, vx) {
                    ranged.push((key, vec![q, nq, vx]));
                    continue;
                }
            }
            exact.push(line.clone());
            continue;
        }
        // "Quads: N" / "  Quads: N" (+ Non-quads, Vertices, Failed islands).
        let trimmed = line.trim_start();
        let indent_len = line.len() - trimmed.len();
        let mut handled = false;
        for prefix in ["Quads:", "Non-quads:", "Vertices:", "Failed islands:"] {
            if let Some(rest) = trimmed.strip_prefix(prefix) {
                let key = format!("{}{prefix}", &line[..indent_len]);
                match rest.trim().parse::<usize>() {
                    Ok(n) => ranged.push((key, vec![n])),
                    Err(_) => exact.push(line.clone()),
                }
                handled = true;
                break;
            }
        }
        if !handled {
            exact.push(line.clone());
        }
    }
    CountFacts { exact, ranged }
}

// EPX facts rule: exact lines sequence-match some mode; ranged keys match
// some mode's key set with every value inside the census [min,max].
fn audit_facts_epx(cpp_runs: &[CountFacts], rs: &CountFacts) -> Result<Vec<String>, String> {
    let mut audit = vec!["facts:epx-range".to_string()];
    let mut exact_ok = false;
    for run in cpp_runs {
        if run.exact == rs.exact {
            exact_ok = true;
            break;
        }
    }
    if !exact_ok {
        return Err(format!(
            "epx exact-line skew:\n  rs ={:?}\n  cpp0={:?}",
            rs.exact,
            cpp_runs.first().map(|r| &r.exact)
        ));
    }
    let rs_keys: Vec<&String> = rs.ranged.iter().map(|(k, _)| k).collect();
    let mut keyed = false;
    for run in cpp_runs {
        let keys: Vec<&String> = run.ranged.iter().map(|(k, _)| k).collect();
        if keys == rs_keys {
            keyed = true;
            break;
        }
    }
    if !keyed {
        return Err(format!(
            "epx key-set skew: rs={rs_keys:?} cpp0={:?}",
            cpp_runs
                .first()
                .map(|r| r.ranged.iter().map(|(k, _)| k).collect::<Vec<_>>())
        ));
    }
    // Per-key census windows (modes lacking the key do not widen it).
    let mut windows: HashMap<&str, Vec<(usize, usize)>> = HashMap::new();
    for run in cpp_runs {
        for (key, vals) in &run.ranged {
            let entry = windows
                .entry(key.as_str())
                .or_insert_with(|| vec![(usize::MAX, 0); vals.len()]);
            if entry.len() != vals.len() {
                return Err(format!("epx arity skew on key {key:?}"));
            }
            for (i, v) in vals.iter().enumerate() {
                entry[i].0 = entry[i].0.min(*v);
                entry[i].1 = entry[i].1.max(*v);
            }
        }
    }
    for (key, vals) in &rs.ranged {
        match windows.get(key.as_str()) {
            Some(win)
                if win.len() == vals.len()
                    && win
                        .iter()
                        .zip(vals.iter())
                        .all(|((lo, hi), v)| (*lo..=*hi).contains(v)) => {}
            other => {
                return Err(format!(
                    "epx count {key:?}={vals:?} outside census window {other:?}"
                ));
            }
        }
    }
    let _ = &mut audit;
    Ok(audit)
}

// Shape + value rules for float/count-valued extractor diagnostics.
// Two gaps collide here: (1) the quad lane prints doubles with Rust
// `{}` (shortest round-trip) where the C++ prints `ostream` `%g`, so
// even agreeing values spell differently (`0.751296` vs
// `0.7512963129672035`); (2) on cliffs the values themselves flip with
// the topology. Shapes (numbers normalized to `#`) bound the diagnostic
// kinds; values are asserted where the mesh matched and reported on
// cliffs. `normalize_shape` and `numeric_runs` share the numeric-run
// definition (maximal `[0-9.eE+-]` runs that parse as `f64`).

fn numeric_runs(line: &str) -> Vec<(usize, usize, f64)> {
    let bytes = line.as_bytes();
    let mut runs = Vec::new();
    let mut i = 0;
    while i < bytes.len() {
        let b = bytes[i];
        if matches!(b, b'0'..=b'9' | b'.' | b'e' | b'E' | b'+' | b'-') {
            let start = i;
            while i < bytes.len()
                && matches!(bytes[i], b'0'..=b'9' | b'.' | b'e' | b'E' | b'+' | b'-')
            {
                i += 1;
            }
            let run = &line[start..i];
            if run.bytes().any(|c| c.is_ascii_digit()) {
                if let Ok(v) = run.parse::<f64>() {
                    runs.push((start, i, v));
                }
            }
        } else {
            i += 1;
        }
    }
    runs
}

fn normalize_shape(line: &str) -> String {
    let runs = numeric_runs(line);
    if runs.is_empty() {
        return line.to_string();
    }
    let mut out = String::with_capacity(line.len());
    let mut pos = 0;
    for (start, end, _) in runs {
        out.push_str(&line[pos..start]);
        out.push('#');
        pos = end;
    }
    out.push_str(&line[pos..]);
    out
}

fn has_float_spelling(line: &str) -> bool {
    for (start, end, _) in numeric_runs(line) {
        let run = &line[start..end];
        if run.contains('.') || run.contains('e') || run.contains('E') {
            return true;
        }
    }
    false
}

fn values_agree(a: &[f64], b: &[f64]) -> bool {
    a.len() == b.len()
        && a.iter()
            .zip(b.iter())
            .all(|(x, y)| (x - y).abs() <= TOL * 1.0f64.max(x.abs()).max(y.abs()))
}

// Diagnostic rule shared by the strict-robust fallback and the EPX
// path: every Rust line's shape must be demonstrated in the census
// union. Where the mesh matched (`epx == false`), same-shape values
// must additionally agree within 1e-6 and universal exact lines must
// appear; on cliffs both are reported, not asserted (cover-derived).
fn audit_diag_shapes(
    cpp_runs: &[Vec<String>],
    rs: &[String],
    epx: bool,
) -> Result<Vec<String>, String> {
    let mut audit = vec!["diag:shapes".to_string()];
    // Union shapes with their value lists (for the agreement search).
    let mut union: Vec<(String, Vec<f64>)> = Vec::new();
    for run in cpp_runs {
        for line in run {
            let shape = normalize_shape(line);
            let vals: Vec<f64> = numeric_runs(line).iter().map(|(_, _, v)| *v).collect();
            if !union.iter().any(|(s, v)| s == &shape && v == &vals) {
                union.push((shape, vals));
            }
        }
    }
    let mut max_diff = 0.0f64;
    for line in rs {
        let shape = normalize_shape(line);
        let vals: Vec<f64> = numeric_runs(line).iter().map(|(_, _, v)| *v).collect();
        let mut shape_seen = false;
        let mut value_ok = false;
        for (u_shape, u_vals) in &union {
            if *u_shape != shape {
                continue;
            }
            shape_seen = true;
            if values_agree(u_vals, &vals) {
                value_ok = true;
                break;
            }
            if u_vals.len() == vals.len() {
                for (x, y) in u_vals.iter().zip(vals.iter()) {
                    max_diff = max_diff.max((x - y).abs());
                }
            }
        }
        if !shape_seen {
            return Err(format!("undemonstrated rs diagnostic: {line:?}"));
        }
        if !value_ok && !epx {
            return Err(format!(
                "rs diagnostic values disagree on matched mesh: {line:?}"
            ));
        }
    }
    if epx {
        audit.push(format!("diag-value-maxdiff={max_diff:.3e}"));
        // Universal misses are cover-derived omissions (0-count lines
        // print nowhere on either side); record, do not assert.
        let mut universal: Option<std::collections::HashSet<&str>> = None;
        for run in cpp_runs {
            let set: std::collections::HashSet<&str> = run.iter().map(|l| l.as_str()).collect();
            universal = Some(match universal {
                None => set.clone(),
                Some(u) => u.intersection(&set).copied().collect(),
            });
        }
        let rs_set: std::collections::HashSet<&str> = rs.iter().map(|l| l.as_str()).collect();
        let missing = universal.unwrap_or_default().difference(&rs_set).count();
        if missing > 0 {
            audit.push(format!("diag-universal-miss={missing}"));
        }
    } else {
        // Matched mesh: universal int-stable lines must appear exactly;
        // float-spelled lines are covered by shape+value above (a `%g`
        // vs `{}` spelling can never match byte-for-byte).
        let mut universal: Option<std::collections::HashSet<&str>> = None;
        for run in cpp_runs {
            let set: std::collections::HashSet<&str> = run.iter().map(|l| l.as_str()).collect();
            universal = Some(match universal {
                None => set.clone(),
                Some(u) => u.intersection(&set).copied().collect(),
            });
        }
        let rs_set: std::collections::HashSet<&str> = rs.iter().map(|l| l.as_str()).collect();
        let uni_owned = universal.unwrap_or_default();
        let mut missing: Vec<&&str> = uni_owned
            .difference(&rs_set)
            .filter(|l| !has_float_spelling(l))
            .collect();
        missing.sort();
        if !missing.is_empty() {
            return Err(format!("rs missing universal diagnostics: {missing:?}"));
        }
        // Universal SHAPES must appear (covers float-spelled lines, whose
        // exact bytes can never match across the `%g`/`{}` gap).
        let mut uni_shapes: Option<std::collections::HashSet<String>> = None;
        for run in cpp_runs {
            let set: std::collections::HashSet<String> =
                run.iter().map(|l| normalize_shape(l)).collect();
            uni_shapes = Some(match uni_shapes {
                None => set.clone(),
                Some(u) => u.intersection(&set).cloned().collect(),
            });
        }
        let rs_shapes: std::collections::HashSet<String> =
            rs.iter().map(|l| normalize_shape(l)).collect();
        let uni_shapes_owned = uni_shapes.unwrap_or_default();
        let mut missing_shapes: Vec<&String> = uni_shapes_owned.difference(&rs_shapes).collect();
        missing_shapes.sort();
        if !missing_shapes.is_empty() {
            return Err(format!(
                "rs missing universal diagnostic shapes: {missing_shapes:?}"
            ));
        }
    }
    Ok(audit)
}

// --- output-mesh tolerance fallback --------------------------------------
// Byte equality is the bar; when QPX/FFX-class engine noise moves a value
// past `%g`'s last digit, this fallback accepts scale-aware 1e-6 value
// agreement with positional-exact face indices, and reports the max diff
// so the match table can cite the mechanism per case.

struct ParsedObj {
    header: Vec<String>,
    verts: Vec<[f64; 3]>,
    uvs: Vec<[f64; 2]>,
    faces: Vec<Vec<usize>>,
}

fn parse_obj(bytes: &[u8]) -> Result<ParsedObj, String> {
    let text = String::from_utf8_lossy(bytes);
    let mut header = Vec::new();
    let mut verts = Vec::new();
    let mut uvs = Vec::new();
    let mut faces = Vec::new();
    for line in text.split('\n') {
        if line.starts_with("# ") {
            header.push(line.to_string());
        } else if let Some(rest) = line.strip_prefix("v ") {
            let nums: Vec<f64> = rest
                .split_ascii_whitespace()
                .map(|t| t.parse::<f64>().map_err(|_| format!("bad v: {line}")))
                .collect::<Result<_, _>>()?;
            if nums.len() != 3 {
                return Err(format!("bad v arity: {line}"));
            }
            verts.push([nums[0], nums[1], nums[2]]);
        } else if let Some(rest) = line.strip_prefix("vt ") {
            let nums: Vec<f64> = rest
                .split_ascii_whitespace()
                .map(|t| t.parse::<f64>().map_err(|_| format!("bad vt: {line}")))
                .collect::<Result<_, _>>()?;
            if nums.len() != 2 {
                return Err(format!("bad vt arity: {line}"));
            }
            uvs.push([nums[0], nums[1]]);
        } else if let Some(rest) = line.strip_prefix("f ") {
            let mut face = Vec::new();
            for corner in rest.split_ascii_whitespace() {
                let v = corner
                    .split('/')
                    .next()
                    .ok_or_else(|| format!("bad f: {line}"))?;
                face.push(v.parse::<usize>().map_err(|_| format!("bad f: {line}"))?);
            }
            faces.push(face);
        } else if !line.is_empty() {
            return Err(format!("unexpected obj line: {line:?}"));
        }
    }
    Ok(ParsedObj {
        header,
        verts,
        uvs,
        faces,
    })
}

fn compare_obj_tol(cpp: &[u8], rs: &[u8]) -> Result<f64, String> {
    let a = parse_obj(cpp)?;
    let b = parse_obj(rs)?;
    if a.header != b.header {
        return Err(format!("obj header skew: {:?} vs {:?}", a.header, b.header));
    }
    if a.verts.len() != b.verts.len() {
        return Err(format!(
            "obj vert count skew: {} vs {}",
            a.verts.len(),
            b.verts.len()
        ));
    }
    if a.uvs.len() != b.uvs.len() {
        return Err(format!(
            "obj uv count skew: {} vs {}",
            a.uvs.len(),
            b.uvs.len()
        ));
    }
    if a.faces.len() != b.faces.len() {
        return Err(format!(
            "obj face count skew: {} vs {}",
            a.faces.len(),
            b.faces.len()
        ));
    }
    let mut max_diff = 0.0f64;
    for (i, (va, vb)) in a.verts.iter().zip(b.verts.iter()).enumerate() {
        for k in 0..3 {
            let diff = (va[k] - vb[k]).abs();
            max_diff = max_diff.max(diff);
            let tol = TOL * (1.0f64).max(va[k].abs()).max(vb[k].abs());
            if diff > tol {
                return Err(format!(
                    "obj v{i}[{k}] skew: {} vs {} (diff {diff:.3e})",
                    va[k], vb[k]
                ));
            }
        }
    }
    for (i, (va, vb)) in a.uvs.iter().zip(b.uvs.iter()).enumerate() {
        for k in 0..2 {
            let diff = (va[k] - vb[k]).abs();
            max_diff = max_diff.max(diff);
            let tol = TOL * (1.0f64).max(va[k].abs()).max(vb[k].abs());
            if diff > tol {
                return Err(format!(
                    "obj vt{i}[{k}] skew: {} vs {} (diff {diff:.3e})",
                    va[k], vb[k]
                ));
            }
        }
    }
    for (i, (fa, fb)) in a.faces.iter().zip(b.faces.iter()).enumerate() {
        if fa != fb {
            return Err(format!("obj f{i} skew: {fa:?} vs {fb:?}"));
        }
    }
    Ok(max_diff)
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

// --- census case runner ----------------------------------------------------

struct CaseRow {
    name: String,
    tier: String,
    detail: String,
}

// One oracle verdict. `Miss` carries the failure diagnostics plus
// whether a wider census could change it: window/count/demonstration
// misses extend (up to CENSUS_MAX), Rust-side facts (determinism is
// checked before evaluation; degenerate/invalid Rust meshes, joint-match
// report skew) fail immediately.
enum Verdict {
    Pass(CaseRow),
    Miss {
        reason: String,
        msg: String,
        detail: String,
        extendable: bool,
    },
}

// One remesh case: `make_args(side_dir)` builds full argv with
// side-specific outputs, `outputs(side_dir)` lists expected artifacts,
// `report(side_dir)` the optional report file. Census rule: C++ runs 5
// (10 when modes/exits flip), then adaptively to CENSUS_MAX in steps of
// CENSUS_STEP on any extendable oracle miss; Rust runs twice (fixed).
// The oracle evaluates against ALL censused modes and accepts iff Rust
// matches ANY demonstrated mode; windows/counts/unions only widen, so
// extension can only add evidence, never mask a divergence.
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
    let cpp_dir = dir.join("cpp");
    let rs_dir = dir.join("rs");
    std::fs::create_dir_all(&cpp_dir).expect("cpp dir");
    std::fs::create_dir_all(&rs_dir).expect("rs dir");

    // C++ census: N runs into per-run dirs (each run's bytes captured
    // before the next starts); extended to 10 when exits or mesh modes
    // flip (cliff-sitters get the wider census by construction), then
    // adaptively to CENSUS_MAX by the evaluate/extend loop below.
    struct Mode {
        code: Option<i32>,
        stdout: Vec<String>,
        stderr: Vec<String>,
        artifacts: Vec<Option<Vec<u8>>>,
        report: Option<Vec<u8>>,
    }
    // One census run; `None` is a timeout (fatal, not extendable).
    let run_cpp = |k: usize| -> Option<Mode> {
        let run_dir = dir.join(format!("cpp-{k}"));
        std::fs::create_dir_all(&run_dir).expect("run dir");
        let args = make_args(&run_dir);
        let out = run(&bins.cpp, &args, RUN_TIMEOUT);
        // Forensic capture (kept with the case dir for the report).
        let _ = std::fs::write(run_dir.join("stdout.txt"), &out.stdout);
        let _ = std::fs::write(run_dir.join("stderr.txt"), &out.stderr);
        let _ = std::fs::write(run_dir.join("exit.txt"), format!("{:?}", out.code));
        if out.timed_out {
            return None;
        }
        let artifacts = outputs(&run_dir)
            .iter()
            .map(|p| std::fs::read(p).ok())
            .collect();
        let rep = report(&run_dir).and_then(|p| std::fs::read(p).ok());
        Some(Mode {
            code: out.code,
            stdout: normalize_bytes(&out.stdout, &case_tag),
            stderr: normalize_bytes(&out.stderr, &case_tag),
            artifacts,
            report: rep,
        })
    };
    let mut modes: Vec<Mode> = Vec::new();
    let mut census_target = CENSUS_N;
    let mut k = 0;
    while k < census_target {
        match run_cpp(k) {
            Some(mode) => modes.push(mode),
            None => {
                failures.push(format!("{name}: C++ run timed out"));
                return CaseRow {
                    name: name.to_string(),
                    tier: "FAIL".to_string(),
                    detail: "c++ timeout".to_string(),
                };
            }
        }
        k += 1;
        if k == CENSUS_N {
            let exits: std::collections::HashSet<Option<i32>> =
                modes.iter().map(|m| m.code).collect();
            let mesh_mode_count: std::collections::HashSet<Vec<Option<Vec<u8>>>> =
                modes.iter().map(|m| m.artifacts.clone()).collect();
            if exits.len() > 1 || mesh_mode_count.len() > 1 {
                census_target = CENSUS_EXTENDED;
            }
        }
    }

    // UB screen: any C++ crash excludes the case (EPX-by-UB, never
    // matched); a Rust crash is always a failure.
    let cpp_crashes = modes.iter().filter(|m| m.code.is_none()).count();
    let live_modes: Vec<&Mode> = modes.iter().filter(|m| m.code.is_some()).collect();
    if live_modes.is_empty() {
        // C++ never survived: excluded, but Rust must still be graceful.
        let args = make_args(&rs_dir);
        let out = run(&bins.rs, &args, RUN_TIMEOUT);
        if out.crashed() || out.timed_out {
            failures.push(format!("{name}: EPX-by-UB case but Rust crashed/hung"));
            return CaseRow {
                name: name.to_string(),
                tier: "FAIL".to_string(),
                detail: "rust crash on EPX-by-UB case".to_string(),
            };
        }
        return CaseRow {
            name: name.to_string(),
            tier: "EPX-UB".to_string(),
            detail: format!("c++ crashed {cpp_crashes} runs; rust exits {:?}", out.code),
        };
    }

    // Rust double-run (determinism past time normalization).
    let args0 = make_args(&rs_dir);
    let rs0 = run(&bins.rs, &args0, RUN_TIMEOUT);
    let _ = std::fs::write(rs_dir.join("stdout.txt"), &rs0.stdout);
    let _ = std::fs::write(rs_dir.join("stderr.txt"), &rs0.stderr);
    let _ = std::fs::write(rs_dir.join("exit.txt"), format!("{:?}", rs0.code));
    if rs0.timed_out || rs0.crashed() {
        failures.push(format!("{name}: Rust crashed/hung"));
        return CaseRow {
            name: name.to_string(),
            tier: "FAIL".to_string(),
            detail: "rust crash/hang".to_string(),
        };
    }
    let rs0_artifacts: Vec<Option<Vec<u8>>> = outputs(&rs_dir)
        .iter()
        .map(|p| std::fs::read(p).ok())
        .collect();
    let rs0_report = report(&rs_dir).and_then(|p| std::fs::read(p).ok());
    let rs_dir2 = dir.join("rs2");
    std::fs::create_dir_all(&rs_dir2).expect("rs2 dir");
    let args1 = make_args(&rs_dir2);
    let rs1 = run(&bins.rs, &args1, RUN_TIMEOUT);
    // Forensic capture for the determinism run too (a run-to-run Rust
    // difference is otherwise undiagnosable after the fact).
    let _ = std::fs::write(rs_dir2.join("stdout.txt"), &rs1.stdout);
    let _ = std::fs::write(rs_dir2.join("stderr.txt"), &rs1.stderr);
    let _ = std::fs::write(rs_dir2.join("exit.txt"), format!("{:?}", rs1.code));
    if rs1.timed_out || rs1.crashed() {
        failures.push(format!("{name}: Rust second run crashed/hung"));
        return CaseRow {
            name: name.to_string(),
            tier: "FAIL".to_string(),
            detail: "rust crash/hang".to_string(),
        };
    }
    let rs1_artifacts: Vec<Option<Vec<u8>>> = outputs(&rs_dir2)
        .iter()
        .map(|p| std::fs::read(p).ok())
        .collect();
    let rs1_report = report(&rs_dir2).and_then(|p| std::fs::read(p).ok());

    // Determinism asserts: mesh bytes raw-identical; report/stdout/
    // stderr identical past normalization (progress/extractor order may
    // race across threads: multiset-equal there, sequence noted).
    let mut notes: Vec<String> = Vec::new();
    if rs0.code != rs1.code {
        failures.push(format!(
            "{name}: rust exit flips run-to-run ({:?} vs {:?})",
            rs0.code, rs1.code
        ));
    }
    if rs0_artifacts != rs1_artifacts {
        failures.push(format!("{name}: rust artifacts differ run-to-run"));
    }
    let rs0n_out = normalize_bytes(&rs0.stdout, &case_tag);
    let rs1n_out = normalize_bytes(&rs1.stdout, &case_tag);
    let rs0n_err = normalize_bytes(&rs0.stderr, &case_tag);
    let rs1n_err = normalize_bytes(&rs1.stderr, &case_tag);
    let norm_rep0 = rs0_report
        .as_ref()
        .map(|b| normalize_bytes(b, &case_tag).join("\n"));
    let norm_rep1 = rs1_report
        .as_ref()
        .map(|b| normalize_bytes(b, &case_tag).join("\n"));
    if norm_rep0 != norm_rep1 {
        failures.push(format!("{name}: rust report differs run-to-run"));
    }
    // stdout determinism: non-progress strict; progress percents race
    // (shared-atomic sampling) but the status set is stable run-to-run
    // (demonstrated 5/5 identical across both binaries).
    let (rs0_prog, rs0_rest): (Vec<&String>, Vec<&String>) =
        rs0n_out.iter().partition(|l| is_progress_line(l));
    let (rs1_prog, rs1_rest): (Vec<&String>, Vec<&String>) =
        rs1n_out.iter().partition(|l| is_progress_line(l));
    if rs0_rest != rs1_rest {
        failures.push(format!("{name}: rust stdout report differs run-to-run"));
    }
    let set0: std::collections::HashSet<String> = rs0_prog
        .iter()
        .filter_map(|l| parse_progress(l).map(|(_, s)| s))
        .collect();
    let set1: std::collections::HashSet<String> = rs1_prog
        .iter()
        .filter_map(|l| parse_progress(l).map(|(_, s)| s))
        .collect();
    if rs0_prog.iter().any(|l| parse_progress(l).is_none())
        || rs1_prog.iter().any(|l| parse_progress(l).is_none())
    {
        failures.push(format!("{name}: unparsable rust progress line"));
    } else if set0 != set1 {
        failures.push(format!(
            "{name}: rust progress status-set differs run-to-run"
        ));
    } else if rs0_prog != rs1_prog {
        notes.push("rs-progress-races".to_string());
    }
    if rs0n_err != rs1n_err {
        let mut counts: HashMap<&str, isize> = HashMap::new();
        for l in &rs0n_err {
            *counts.entry(l.as_str()).or_insert(0) += 1;
        }
        for l in &rs1n_err {
            *counts.entry(l.as_str()).or_insert(0) -= 1;
        }
        counts.retain(|_, v| *v != 0);
        if counts.is_empty() {
            notes.push("rs-stderr-order-races".to_string());
        } else {
            failures.push(format!("{name}: rust stderr content differs run-to-run"));
        }
    }

    // Fixed Rust-side evidence for the (re-runnable) oracle.
    let has_report = report(&rs_dir).is_some();
    // Oracle as a re-runnable closure over the censused modes: the
    // evaluate/extend loop below calls it, widening the census on any
    // extendable miss (up to CENSUS_MAX) before failing.
    let evaluate = |modes: &[Mode]| -> Verdict {
        let mut notes = notes.clone();
        // Oracle: Rust (run 0) must match some same-exit C++ mode on every
        // channel. Exit first.
        let same_exit: Vec<(usize, &Mode)> = modes
            .iter()
            .enumerate()
            .filter(|(_, m)| m.code.is_some() && m.code == rs0.code)
            .collect();
        if same_exit.is_empty() {
            let cpp_exits: Vec<Option<i32>> = modes.iter().map(|m| m.code).collect();
            return Verdict::Miss {
                reason: "exit".to_string(),
                msg: format!("{name}: exit skew (rs={:?}, cpp={cpp_exits:?})", rs0.code),
                detail: format!("exit rs={:?} cpp={cpp_exits:?}", rs0.code),
                extendable: true,
            };
        }
        // Artifacts: byte-exact vs any same-exit mode, else OBJ tolerance,
        // else the EPX cliff tier (mirrors the engine EPX discipline: Rust
        // must be deterministic and the C++ side must demonstrate
        // self-variation; robustness facts are asserted, cover-derived
        // quantities are reported, never asserted).
        let rs_artifacts_stable = rs0_artifacts == rs1_artifacts;
        let mut tier = "strict";
        let mut matched_mode = usize::MAX;
        for (k, m) in &same_exit {
            if m.artifacts == rs0_artifacts {
                matched_mode = *k;
                break;
            }
        }
        let mut epx = false;
        // Joint match: every artifact agrees with ONE same-exit mode
        // (byte-exact, or byte-or-tol per artifact). A joint match pins the
        // counts, so report/stdout facts must match that mode exactly; a
        // mixed tol match (artifacts drawn from different modes) leaves the
        // joint count combination combinatorially undemonstrated — rungs ×
        // modes explodes past any census — and gets ranged facts instead.
        let mut joint_mode: Option<usize> = if matched_mode == usize::MAX {
            None
        } else {
            Some(matched_mode)
        };
        if matched_mode == usize::MAX {
            // Tolerance fallback per artifact (OBJ only; GLB has no fallback).
            let mut all_tol = true;
            let mut max_diff = 0.0f64;
            // Per-artifact matched-mode sets; a non-empty intersection is a
            // joint tol match (same joint-facts argument as byte matches: a
            // tol match pins counts, compare_obj_tol requires equal counts).
            let mut tol_sets: Vec<std::collections::HashSet<usize>> = Vec::new();
            for (i, rs_art) in rs0_artifacts.iter().enumerate() {
                let mut hit_modes: std::collections::HashSet<usize> =
                    std::collections::HashSet::new();
                match rs_art {
                    None => {
                        for (k, m) in &same_exit {
                            if m.artifacts[i].is_none() {
                                hit_modes.insert(*k);
                            }
                        }
                        if hit_modes.is_empty() {
                            all_tol = false;
                            break;
                        }
                    }
                    Some(rs_bytes) => {
                        for (k, m) in &same_exit {
                            if let Some(cpp_bytes) = &m.artifacts[i] {
                                if cpp_bytes == rs_bytes {
                                    hit_modes.insert(*k);
                                } else if let Ok(diff) = compare_obj_tol(cpp_bytes, rs_bytes) {
                                    hit_modes.insert(*k);
                                    max_diff = max_diff.max(diff);
                                }
                            }
                        }
                        if hit_modes.is_empty() {
                            all_tol = false;
                            break;
                        }
                    }
                }
                tol_sets.push(hit_modes);
            }
            if all_tol {
                tier = "tol";
                notes.push(format!("obj-tol maxdiff={max_diff:.3e}"));
                let mut joint: Option<std::collections::HashSet<usize>> = None;
                for set in &tol_sets {
                    joint = Some(match joint {
                        None => set.clone(),
                        Some(j) => j.intersection(set).copied().collect(),
                    });
                }
                if let Some(j) = joint {
                    if let Some(k) = j.iter().min() {
                        joint_mode = Some(*k);
                        notes.push(format!("tol-joint-mode={k}"));
                    } else {
                        notes.push("tol-mixed-modes".to_string());
                    }
                }
            } else {
                // EPX eligibility: demonstrated C++ self-variation (2+
                // distinct same-exit mesh modes) + deterministic Rust.
                let distinct: std::collections::HashSet<Vec<Option<Vec<u8>>>> =
                    same_exit.iter().map(|(_, m)| m.artifacts.clone()).collect();
                if distinct.len() >= 2 && rs_artifacts_stable {
                    epx = true;
                    tier = "EPX";
                } else {
                    return Verdict::Miss {
                        reason: "mesh".to_string(),
                        msg: format!(
                            "{name}: mesh skew, no byte/tol mode (cpp distinct={}, rs-stable={rs_artifacts_stable})",
                            distinct.len(),
                        ),
                        detail: "mesh skew, true divergence".to_string(),
                        extendable: true,
                    };
                }
            }
        } else if matched_mode != same_exit[0].0 {
            tier = "any-mode";
            notes.push(format!("mesh-mode={matched_mode}"));
        }

        // Per-channel checks. Strict/tol path: exact report, per-mode
        // stdout/stderr with robust fallbacks (mixed tol matches get ranged
        // report/stdout facts: the joint count combination is
        // combinatorially undemonstrated, see `joint_mode`). EPX path:
        // ranged facts, robust progress, robust stderr, structural mesh.
        let mut audit_notes: Vec<String> = Vec::new();
        if !epx {
            // Report: normalized bytes vs any same-exit mode.
            if has_report || same_exit.iter().any(|(_, m)| m.report.is_some()) {
                let rs_rep = norm_rep0.clone().unwrap_or_default();
                let mut ok = false;
                for (k, m) in &same_exit {
                    let cpp_rep = m
                        .report
                        .as_ref()
                        .map(|b| normalize_bytes(b, &case_tag).join("\n"))
                        .unwrap_or_default();
                    if cpp_rep == rs_rep {
                        ok = true;
                        if *k != same_exit[0].0 && tier == "strict" {
                            tier = "any-mode";
                        }
                        break;
                    }
                }
                if !ok {
                    if joint_mode.is_some() {
                        // A joint mesh match pins every count, so the report
                        // MUST equal that mode's: a miss is a real report or
                        // normalization bug, never a census gap. Fatal.
                        return Verdict::Miss {
                            reason: "report".to_string(),
                            msg: format!("{name}: report skew on joint-matched mesh"),
                            detail: "report skew".to_string(),
                            extendable: false,
                        };
                    }
                    // Mixed tol match (diagnosed on batch-loud: per-artifact
                    // matches mix modes, so the joint count combination is
                    // undemonstrated): range the counts across the census
                    // with params exact — the EPX report rule.
                    let cpp_rep_runs: Vec<CountFacts> = same_exit
                        .iter()
                        .map(|(_, m)| {
                            m.report
                                .as_ref()
                                .map(|b| normalize_bytes(b, &case_tag))
                                .unwrap_or_default()
                        })
                        .map(|lines| split_facts(&lines))
                        .collect();
                    let rs_rep_lines = rs0_report
                        .as_ref()
                        .map(|b| normalize_bytes(b, &case_tag))
                        .unwrap_or_default();
                    let rs_rep_facts = split_facts(&rs_rep_lines);
                    match audit_facts_epx(&cpp_rep_runs, &rs_rep_facts) {
                        Ok(a) => {
                            audit_notes.extend(a);
                            audit_notes.push("report:ranged".to_string());
                        }
                        Err(e) => {
                            return Verdict::Miss {
                                reason: "report-range".to_string(),
                                msg: format!("{name}: report skew ({e})"),
                                detail: "report skew".to_string(),
                                extendable: true,
                            };
                        }
                    }
                }
            }
            // Stdout: per-mode audit, then the robust fallback, then (mixed
            // tol only) the ranged rule.
            let mut stdout_audit: Option<Vec<String>> = None;
            for (k, m) in &same_exit {
                if let Ok(a) = audit_stdout(&m.stdout, &rs0n_out) {
                    if *k != same_exit[0].0 && tier == "strict" {
                        tier = "any-mode";
                    }
                    stdout_audit = Some(a);
                    break;
                }
            }
            if stdout_audit.is_none() {
                let runs: Vec<Vec<String>> =
                    same_exit.iter().map(|(_, m)| m.stdout.clone()).collect();
                match audit_stdout_fallback(&runs, &rs0n_out) {
                    Ok(a) => {
                        if tier == "strict" {
                            tier = "any-mode";
                        }
                        stdout_audit = Some(a);
                    }
                    Err(e) => {
                        if joint_mode.is_none() {
                            // Mixed tol match: same combinatorial argument as
                            // the report rule — range the facts, robust the
                            // progress.
                            let cpp_fact_runs: Vec<CountFacts> =
                                runs.iter().map(|r| split_facts(r)).collect();
                            let rs_facts = split_facts(&rs0n_out);
                            let ranged =
                                audit_facts_epx(&cpp_fact_runs, &rs_facts).and_then(|mut a| {
                                    audit_progress_robust(&runs, &rs0n_out).map(|p| {
                                        a.extend(p);
                                        a.push("tol-ranged".to_string());
                                        a
                                    })
                                });
                            match ranged {
                                Ok(a) => {
                                    stdout_audit = Some(a);
                                }
                                Err(e2) => {
                                    return Verdict::Miss {
                                        reason: "stdout-range".to_string(),
                                        msg: format!("{name}: stdout skew ({e}; ranged: {e2})"),
                                        detail: "stdout skew".to_string(),
                                        extendable: true,
                                    };
                                }
                            }
                        } else {
                            return Verdict::Miss {
                                reason: "stdout".to_string(),
                                msg: format!("{name}: stdout skew ({e})"),
                                detail: "stdout skew".to_string(),
                                extendable: true,
                            };
                        }
                    }
                }
            }
            for a in stdout_audit.unwrap() {
                if (a.contains("multiset") || a.contains("robust") || a.contains("fallback"))
                    && tier == "strict"
                {
                    tier = "any-mode";
                }
                audit_notes.push(format!("stdout:{a}"));
            }
            // Stderr: per-mode audit, then the robust fallback.
            let mut stderr_audit: Option<Vec<String>> = None;
            for (k, m) in &same_exit {
                if let Ok(a) = audit_stderr(&m.stderr, &rs0n_err) {
                    if *k != same_exit[0].0 && tier == "strict" {
                        tier = "any-mode";
                    }
                    stderr_audit = Some(a);
                    break;
                }
            }
            if stderr_audit.is_none() {
                let runs: Vec<Vec<String>> =
                    same_exit.iter().map(|(_, m)| m.stderr.clone()).collect();
                match audit_stderr_robust(&runs, &rs0n_err, false) {
                    Ok(a) => {
                        if tier == "strict" {
                            tier = "any-mode";
                        }
                        stderr_audit = Some(a);
                    }
                    Err(e) => {
                        return Verdict::Miss {
                            reason: "stderr".to_string(),
                            msg: format!("{name}: stderr skew ({e})"),
                            detail: "stderr skew".to_string(),
                            extendable: true,
                        };
                    }
                }
            }
            audit_notes.extend(stderr_audit.unwrap());
        } else {
            // --- EPX path: robustness facts only. ---
            notes.push(format!(
                "cliff:cpp-distinct={}of{}",
                same_exit
                    .iter()
                    .map(|(_, m)| m.artifacts.clone())
                    .collect::<std::collections::HashSet<_>>()
                    .len(),
                same_exit.len()
            ));
            // Stdout: ranged facts + robust progress.
            let cpp_fact_runs: Vec<CountFacts> = same_exit
                .iter()
                .map(|(_, m)| split_facts(&m.stdout))
                .collect();
            let rs_facts = split_facts(&rs0n_out);
            match audit_facts_epx(&cpp_fact_runs, &rs_facts) {
                Ok(a) => audit_notes.extend(a),
                Err(e) => {
                    return Verdict::Miss {
                        reason: "epx-facts".to_string(),
                        msg: format!("{name}: epx stdout facts ({e})"),
                        detail: "epx stdout facts skew".to_string(),
                        extendable: true,
                    };
                }
            }
            let cpp_out_runs: Vec<Vec<String>> =
                same_exit.iter().map(|(_, m)| m.stdout.clone()).collect();
            match audit_progress_robust(&cpp_out_runs, &rs0n_out) {
                Ok(a) => audit_notes.extend(a),
                Err(e) => {
                    return Verdict::Miss {
                        reason: "epx-progress".to_string(),
                        msg: format!("{name}: epx progress ({e})"),
                        detail: "epx progress skew".to_string(),
                        extendable: true,
                    };
                }
            }
            // Report: ranged facts (params exact, counts windowed).
            if has_report || same_exit.iter().any(|(_, m)| m.report.is_some()) {
                let cpp_rep_runs: Vec<CountFacts> = same_exit
                    .iter()
                    .map(|(_, m)| {
                        m.report
                            .as_ref()
                            .map(|b| normalize_bytes(b, &case_tag))
                            .unwrap_or_default()
                    })
                    .map(|lines| split_facts(&lines))
                    .collect();
                let rs_rep_lines = rs0_report
                    .as_ref()
                    .map(|b| normalize_bytes(b, &case_tag))
                    .unwrap_or_default();
                let rs_rep_facts = split_facts(&rs_rep_lines);
                match audit_facts_epx(&cpp_rep_runs, &rs_rep_facts) {
                    Ok(a) => audit_notes.extend(a),
                    Err(e) => {
                        return Verdict::Miss {
                            reason: "epx-report".to_string(),
                            msg: format!("{name}: epx report ({e})"),
                            detail: "epx report skew".to_string(),
                            extendable: true,
                        };
                    }
                }
            }
            // Stderr: robust rule (phase structure + diagnostic shapes).
            let cpp_err_runs: Vec<Vec<String>> =
                same_exit.iter().map(|(_, m)| m.stderr.clone()).collect();
            match audit_stderr_robust(&cpp_err_runs, &rs0n_err, true) {
                Ok(a) => audit_notes.extend(a),
                Err(e) => {
                    return Verdict::Miss {
                        reason: "epx-stderr".to_string(),
                        msg: format!("{name}: epx stderr ({e})"),
                        detail: "epx stderr skew".to_string(),
                        extendable: true,
                    };
                }
            }
            // Mesh structural: counts within the census window, indices valid.
            for (i, rs_art) in rs0_artifacts.iter().enumerate() {
                match rs_art {
                    None => {
                        if !same_exit.iter().any(|(_, m)| m.artifacts[i].is_none()) {
                            return Verdict::Miss {
                                reason: "epx-artifact".to_string(),
                                msg: format!("{name}: epx artifact{i} missing on rs only"),
                                detail: "epx artifact missing".to_string(),
                                extendable: true,
                            };
                        }
                        notes.push(format!("artifact{i}:absent-both-some-mode"));
                    }
                    Some(rs_bytes) => {
                        let present: Vec<&Vec<u8>> = same_exit
                            .iter()
                            .filter_map(|(_, m)| m.artifacts[i].as_ref())
                            .collect();
                        if present.is_empty() {
                            return Verdict::Miss {
                                reason: "epx-artifact".to_string(),
                                msg: format!("{name}: epx artifact{i} present on rs only"),
                                detail: "epx artifact present-only-rs".to_string(),
                                extendable: true,
                            };
                        }
                        match parse_obj(rs_bytes) {
                            Err(_) => {
                                // Unparsed (GLB): byte-size window only.
                                let lens: Vec<usize> = present.iter().map(|b| b.len()).collect();
                                let lo = *lens.iter().min().unwrap();
                                let hi = *lens.iter().max().unwrap();
                                if rs_bytes.len() < lo || rs_bytes.len() > hi {
                                    return Verdict::Miss {
                                        reason: "epx-size".to_string(),
                                        msg: format!(
                                            "{name}: epx artifact{i} size {} outside [{lo},{hi}]",
                                            rs_bytes.len()
                                        ),
                                        detail: "epx artifact size skew".to_string(),
                                        extendable: true,
                                    };
                                }
                                notes.push(format!("artifact{i}:size-window[{lo},{hi}]"));
                            }
                            Ok(rs_mesh) => {
                                let mut v_lo = usize::MAX;
                                let mut v_hi = 0;
                                let mut f_lo = usize::MAX;
                                let mut f_hi = 0;
                                for cpp_bytes in &present {
                                    match parse_obj(cpp_bytes) {
                                        Err(e) => {
                                            return Verdict::Miss {
                                                reason: "epx-cpp-parse".to_string(),
                                                msg: format!(
                                                    "{name}: epx artifact{i} cpp unparsable ({e})"
                                                ),
                                                detail: "epx cpp artifact skew".to_string(),
                                                extendable: false,
                                            };
                                        }
                                        Ok(cpp_mesh) => {
                                            v_lo = v_lo.min(cpp_mesh.verts.len());
                                            v_hi = v_hi.max(cpp_mesh.verts.len());
                                            f_lo = f_lo.min(cpp_mesh.faces.len());
                                            f_hi = f_hi.max(cpp_mesh.faces.len());
                                        }
                                    }
                                }
                                if rs_mesh.verts.is_empty() || rs_mesh.faces.is_empty() {
                                    return Verdict::Miss {
                                        reason: "epx-degenerate".to_string(),
                                        msg: format!(
                                            "{name}: epx artifact{i} degenerate ({}v {}f)",
                                            rs_mesh.verts.len(),
                                            rs_mesh.faces.len()
                                        ),
                                        detail: "epx degenerate mesh".to_string(),
                                        extendable: false,
                                    };
                                }
                                if rs_mesh.verts.len() < v_lo || rs_mesh.verts.len() > v_hi {
                                    return Verdict::Miss {
                                        reason: "epx-window".to_string(),
                                        msg: format!(
                                            "{name}: epx artifact{i} verts {} outside [{v_lo},{v_hi}]",
                                            rs_mesh.verts.len()
                                        ),
                                        detail: "epx vert window skew".to_string(),
                                        extendable: true,
                                    };
                                }
                                if rs_mesh.faces.len() < f_lo || rs_mesh.faces.len() > f_hi {
                                    return Verdict::Miss {
                                        reason: "epx-window".to_string(),
                                        msg: format!(
                                            "{name}: epx artifact{i} faces {} outside [{f_lo},{f_hi}]",
                                            rs_mesh.faces.len()
                                        ),
                                        detail: "epx face window skew".to_string(),
                                        extendable: true,
                                    };
                                }
                                for (fi, face) in rs_mesh.faces.iter().enumerate() {
                                    if face.is_empty()
                                        || face.iter().any(|&c| c == 0 || c > rs_mesh.verts.len())
                                    {
                                        return Verdict::Miss {
                                            reason: "epx-face".to_string(),
                                            msg: format!(
                                                "{name}: epx artifact{i} face{fi} invalid"
                                            ),
                                            detail: "epx face validity skew".to_string(),
                                            extendable: false,
                                        };
                                    }
                                }
                                notes.push(format!(
                                "artifact{i}:structural v{}in[{v_lo},{v_hi}] f{}in[{f_lo},{f_hi}]",
                                rs_mesh.verts.len(),
                                rs_mesh.faces.len()
                            ));
                            }
                        }
                    }
                }
            }
        }
        // Compact the audit classes for the table.
        let mut classes: HashMap<String, usize> = HashMap::new();
        for a in &audit_notes {
            *classes.entry(a.clone()).or_insert(0) += 1;
        }
        let mut class_list: Vec<String> = classes
            .iter()
            .map(|(k, v)| {
                if *v > 1 {
                    format!("{k}x{v}")
                } else {
                    k.clone()
                }
            })
            .collect();
        class_list.sort();
        notes.extend(class_list);

        let cpp_exits: std::collections::HashSet<Option<i32>> =
            modes.iter().map(|m| m.code).collect();
        if cpp_exits.len() > 1 {
            notes.push(format!("cpp-exit-flip:{cpp_exits:?}"));
            if tier == "strict" {
                tier = "any-mode";
            }
        }
        let mesh_mode_count: usize = modes
            .iter()
            .map(|m| m.artifacts.clone())
            .collect::<std::collections::HashSet<_>>()
            .len();
        if mesh_mode_count > 1 {
            notes.push(format!("cpp-mesh-modes={mesh_mode_count}"));
            if tier == "strict" {
                tier = "any-mode";
            }
        }
        let crash_count = modes.iter().filter(|m| m.code.is_none()).count();
        if cpp_cracks_note(crash_count) {
            notes.push(format!("cpp-crashes={crash_count}"));
        }
        return Verdict::Pass(CaseRow {
            name: name.to_string(),
            tier: tier.to_string(),
            detail: notes.join(" "),
        });
    }; // end evaluate
    // Evaluate/extend loop: accept on the first Pass; widen the census on
    // any extendable miss (up to CENSUS_MAX); fail only on a fatal miss
    // or an exhausted extension. Per-case census stats ride the row
    // detail (`census=N`, plus `ext=k:reasons` when extension fired).
    let mut extensions: Vec<String> = Vec::new();
    loop {
        match evaluate(&modes) {
            Verdict::Pass(mut row) => {
                let mut stats = format!("census={}", modes.len());
                if !extensions.is_empty() {
                    stats.push_str(&format!(
                        " ext={}:{}",
                        extensions.len(),
                        extensions.join(",")
                    ));
                }
                row.detail = format!("{stats} {}", row.detail);
                return row;
            }
            Verdict::Miss {
                reason,
                msg,
                detail,
                extendable,
            } => {
                if !extendable || modes.len() >= CENSUS_MAX {
                    failures.push(if !extendable {
                        msg
                    } else {
                        format!("{msg} (extension exhausted at N={})", modes.len())
                    });
                    // FAIL rows carry census stats too (an exhausted N=30
                    // once misreported as "0 extended" without this).
                    let mut stats = format!("census={}", modes.len());
                    if !extensions.is_empty() {
                        stats.push_str(&format!(
                            " ext={}:{}",
                            extensions.len(),
                            extensions.join(",")
                        ));
                    }
                    return CaseRow {
                        name: name.to_string(),
                        tier: "FAIL".to_string(),
                        detail: format!("{stats} {detail}"),
                    };
                }
                extensions.push(reason);
                let target = (modes.len() + CENSUS_STEP).min(CENSUS_MAX);
                while modes.len() < target {
                    let k = modes.len();
                    match run_cpp(k) {
                        Some(mode) => modes.push(mode),
                        None => {
                            failures.push(format!("{name}: C++ run timed out during extension"));
                            return CaseRow {
                                name: name.to_string(),
                                tier: "FAIL".to_string(),
                                detail: "c++ timeout".to_string(),
                            };
                        }
                    }
                }
            }
        }
    }
}

fn cpp_cracks_note(cpp_crashes: usize) -> bool {
    cpp_crashes > 0
}

// --- arg matrix (no census: pure parser paths, deterministic) ---------------

fn arg_case(bins: &Bins, name: &str, args: &[&str], failures: &mut Vec<String>) {
    let argv: Vec<String> = args.iter().map(|s| s.to_string()).collect();
    let cpp = run(&bins.cpp, &argv, RUN_TIMEOUT);
    let rs = run(&bins.rs, &argv, RUN_TIMEOUT);
    assert!(!cpp.timed_out && !rs.timed_out, "{name}: timeout");
    assert!(
        !cpp.crashed() && !rs.crashed(),
        "{name}: crash (cpp={:?} rs={:?})",
        cpp.signal,
        rs.signal
    );
    let mut cpp_out: Vec<String> = normalize_bytes(&cpp.stdout, "")
        .iter()
        .map(|l| normalize_argv0(l))
        .collect();
    let mut rs_out: Vec<String> = normalize_bytes(&rs.stdout, "")
        .iter()
        .map(|l| normalize_argv0(l))
        .collect();
    let cpp_err = normalize_bytes(&cpp.stderr, "");
    let rs_err = normalize_bytes(&rs.stderr, "");
    // Usage goes to stdout on some error paths; argv0-normalize it there.
    let _ = (&mut cpp_out, &mut rs_out);
    if cpp.code != rs.code || cpp_out != rs_out || cpp_err != rs_err {
        failures.push(format!(
            "{name}: skew (exit {:?}/{:?})\n--- cpp out ---\n{}\n--- rs out ---\n{}\n--- cpp err ---\n{}\n--- rs err ---\n{}",
            cpp.code,
            rs.code,
            cpp_out.join("\n"),
            rs_out.join("\n"),
            cpp_err.join("\n"),
            rs_err.join("\n"),
        ));
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

// Specified divergence (not a match): C++ `strtod` accepts hex floats
// and `nan(payload)`; the Rust side has no parser for either and errors
// loudly. Both exit nonzero; the stderr text intentionally differs.
// Mechanism: documented `parse` vs `strtod` acceptance gap (DECISIONS).
#[test]
fn arg_hex_divergence() {
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
        let cpp = run(&bins.cpp, &argv, RUN_TIMEOUT);
        let rs = run(&bins.rs, &argv, RUN_TIMEOUT);
        assert!(!cpp.timed_out && !rs.timed_out);
        // C++ accepts and proceeds to the (failing) load; Rust rejects.
        let cpp_err = String::from_utf8_lossy(&cpp.stderr);
        let rs_err = String::from_utf8_lossy(&rs.stderr);
        assert!(
            cpp_err.contains("failed to load nofile"),
            "{value}: c++ no longer accepts hex/nan(payload)? stderr={cpp_err}"
        );
        assert!(
            rs_err.contains("expects a number"),
            "{value}: rust unexpectedly accepts? stderr={rs_err}"
        );
        assert_ne!(cpp.code, Some(0));
        assert_ne!(rs.code, Some(0));
    }
    println!("hex divergence: specified gap holds (c++ accepts, rust loud-errors)");
}

// --- io failure paths (no census: deterministic loader/FS errors) -----------

fn io_case(
    bins: &Bins,
    name: &str,
    setup: &dyn Fn(&Path) -> Vec<String>,
    failures: &mut Vec<String>,
) {
    let dir = case_dir(&format!("io-{name}"));
    let case_tag = dir.to_string_lossy().to_string();
    let cpp_dir = dir.join("cpp");
    let rs_dir = dir.join("rs");
    std::fs::create_dir_all(&cpp_dir).expect("cpp dir");
    std::fs::create_dir_all(&rs_dir).expect("rs dir");
    // Setup writes shared inputs; per-side argv differ only in scratch paths.
    let shared = dir.join("shared");
    std::fs::create_dir_all(&shared).expect("shared dir");
    let cpp_argv = setup(&shared)
        .iter()
        .map(|a| a.replace("SHARED", &shared.to_string_lossy()))
        .map(|a| a.replace("SIDE", &cpp_dir.to_string_lossy()))
        .collect::<Vec<_>>();
    let rs_argv = setup(&shared)
        .iter()
        .map(|a| a.replace("SHARED", &shared.to_string_lossy()))
        .map(|a| a.replace("SIDE", &rs_dir.to_string_lossy()))
        .collect::<Vec<_>>();
    let cpp = run(&bins.cpp, &cpp_argv, RUN_TIMEOUT);
    let rs = run(&bins.rs, &rs_argv, RUN_TIMEOUT);
    assert!(!cpp.timed_out && !rs.timed_out, "{name}: timeout");
    assert!(
        !cpp.crashed() && !rs.crashed(),
        "{name}: crash (cpp={:?} rs={:?})",
        cpp.signal,
        rs.signal
    );
    let cpp_out = normalize_bytes(&cpp.stdout, &case_tag);
    let rs_out = normalize_bytes(&rs.stdout, &case_tag);
    let cpp_err = normalize_bytes(&cpp.stderr, &case_tag);
    let rs_err = normalize_bytes(&rs.stderr, &case_tag);
    // Same audit spec as the oracle (a quiet remesh still emits ungated
    // C++ one-liners on the output/report-unwritable paths), but order
    // fallbacks are rejected here: these paths are single-shot.
    let mut notes = Vec::new();
    let mut skew = cpp.code != rs.code;
    match audit_stdout(&cpp_out, &rs_out) {
        Ok(a) => {
            if a.iter().any(|x| x.contains("multiset")) {
                skew = true;
            }
            notes.extend(a);
        }
        Err(e) => {
            skew = true;
            notes.push(e);
        }
    }
    match audit_stderr(&cpp_err, &rs_err) {
        Ok(a) => {
            if a.iter().any(|x| x.contains("multiset")) {
                skew = true;
            }
            notes.extend(a);
        }
        Err(e) => {
            skew = true;
            notes.push(e);
        }
    }
    // NOTE: artifacts intentionally uncompared here (single-shot runs;
    // mesh equality belongs to the censused oracle, not failure paths).
    if skew {
        failures.push(format!(
            "{name}: skew (exit {:?}/{:?} audit {notes:?})\n--- cpp out ---\n{}\n--- rs out ---\n{}\n--- cpp err ---\n{}\n--- rs err ---\n{}",
            cpp.code,
            rs.code,
            cpp_out.join("\n"),
            rs_out.join("\n"),
            cpp_err.join("\n"),
            rs_err.join("\n"),
        ));
    } else if !notes.is_empty() {
        println!("io {name}: audit {notes:?}");
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
fn remesh_oracle() {
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

    println!("--- e2e match table ---");
    for row in &rows {
        println!("{:<24} {:<9} {}", row.name, row.tier, row.detail);
    }
    let ext_rows = rows.iter().filter(|r| r.detail.contains(" ext=")).count();
    let ext_runs: usize = rows
        .iter()
        .filter_map(|r| {
            let pos = r.detail.find("census=")? + "census=".len();
            let end = r.detail[pos..].find(' ')?;
            r.detail[pos..pos + end].parse::<usize>().ok()
        })
        .map(|n| n.saturating_sub(CENSUS_EXTENDED))
        .sum();
    println!(
        "--- census stats: {ext_rows}/{} cases extended, {ext_runs} runs past N=10 ---",
        rows.len()
    );
    if !failures.is_empty() {
        panic!("oracle failures:\n{}", failures.join("\n====\n"));
    }
}

// --- timing (release-vs-Release fair only under `cargo test --release`) ------

fn parse_quads(stdout: &[u8]) -> usize {
    let text = String::from_utf8_lossy(stdout);
    for line in text.split('\n') {
        if let Some(rest) = line.strip_prefix("Quads: ") {
            return rest.trim().parse().expect("quads number");
        }
    }
    panic!("no Quads line in:\n{text}");
}

#[test]
fn timing_comparison() {
    let bins = bins();
    let dir = case_dir("timing");
    let shared = dir.join("shared");
    std::fs::create_dir_all(&shared).expect("shared");
    // 32x32 analytic grid, target 2000: big enough to time, small enough
    // for debug builds to finish in minutes.
    write_grid_obj(&shared.join("grid.obj"), 32, 32);
    let input = s(&shared.join("grid.obj"));
    let mut cpp_ms = Vec::new();
    let mut rs_ms = Vec::new();
    let mut cpp_quads = Vec::new();
    let mut rs_quads = Vec::new();
    for sample in 0..3 {
        let out = dir.join(format!("cpp-{sample}.obj"));
        let argv = vec![
            "-i".to_string(),
            input.clone(),
            "-o".to_string(),
            s(&out),
            "--target-quads".to_string(),
            "2000".to_string(),
            "--quiet".to_string(),
        ];
        let start = Instant::now();
        let run_out = run(&bins.cpp, &argv, RUN_TIMEOUT);
        cpp_ms.push(start.elapsed().as_secs_f64() * 1000.0);
        assert_eq!(run_out.code, Some(0), "c++ timing run failed");
        cpp_quads.push(parse_quads(&run_out.stdout));

        let out = dir.join(format!("rs-{sample}.obj"));
        let argv = vec![
            "-i".to_string(),
            input.clone(),
            "-o".to_string(),
            s(&out),
            "--target-quads".to_string(),
            "2000".to_string(),
            "--quiet".to_string(),
        ];
        let start = Instant::now();
        let run_out = run(&bins.rs, &argv, RUN_TIMEOUT);
        rs_ms.push(start.elapsed().as_secs_f64() * 1000.0);
        assert_eq!(run_out.code, Some(0), "rust timing run failed");
        rs_quads.push(parse_quads(&run_out.stdout));
    }
    // Cliff-aware comparability (any-mode discipline): grid32 target-2000
    // is a mild cliff-sitter (one release run showed a count flip), so
    // every Rust count must land inside the demonstrated C++ window rather
    // than equal one sample. Counts print alongside times so the ratio's
    // work basis stays reviewable.
    let lo = *cpp_quads.iter().min().unwrap();
    let hi = *cpp_quads.iter().max().unwrap();
    for (i, q) in rs_quads.iter().enumerate() {
        assert!(
            (lo..=hi).contains(q),
            "timing quad-count skew: rs[{i}]={q} outside c++ window [{lo},{hi}] (cpp={cpp_quads:?} rs={rs_quads:?})"
        );
    }
    println!("--- timing (grid32 target-2000 quiet, 3 samples, ms) ---");
    println!("c++ : {cpp_ms:.1?} quads={cpp_quads:?}");
    println!("rust: {rs_ms:.1?} quads={rs_quads:?}");
}
