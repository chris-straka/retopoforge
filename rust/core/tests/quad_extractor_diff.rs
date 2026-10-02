// Harness indexes parallel arrays/cursors in lockstep; index loops stay.
#![allow(clippy::needless_range_loop)]

// Differential oracle for the quad_extractor port: replays every case in
// tests/fixtures/quadextractor_diff.txt through the Rust extractor and
// compares against the C++ dump.
//
// Comparison levels (strict first, relaxations only where iteration order
// legitimately differs — see docs/rust-port-conventions.md):
// - `OK`: exact.
// - `CONN`: connection count exact; endpoints bitwise (`%.17g`
//   round-trips); moved flags exact. The connection SET is ordered on
//   both sides (`std::set` vs `BTreeSet`), so order is exact too.
// - `REMESH`: vertex MULTISET bitwise (order canonicalized: compaction
//   and discovery order differ where the C++ iterates hash maps). Builds
//   the C++ -> Rust index map for the checks below.
// - `QUADS`: index-remapped, winding-canonicalized face SET, exact.
// - `RUV`: per remapped vertex, bitwise (empty iff the flag was off).
//
// A case failing strict comparison fails the test with diagnostics; there
// is no silent tolerance. `T ` timing lines in the fixture are skipped.

use retopo_core::quad_extractor::QuadExtractor;
use retopo_core::vector2::Vector2;
use retopo_core::vector3::Vector3;
use std::collections::{BTreeMap, VecDeque};

const FIXTURE: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../tests/fixtures/quadextractor_diff.txt"
);

struct ExpectedConn {
    a: [f64; 3],
    b: [f64; 3],
    moved: u8,
}

struct Case {
    index: usize,
    vertices: Vec<Vector3>,
    triangles: Vec<Vec<usize>>,
    uvs: Vec<Vec<Vector2>>,
    singular: Vec<usize>,
    compute_uvs: bool,
    orig: Option<Vec<Vec<Vector2>>>,
    ok: bool,
    conns: Vec<ExpectedConn>,
    remesh: Vec<[f64; 3]>,
    quads: Vec<Vec<usize>>,
    ruv: Vec<[f64; 2]>,
}

fn parse_f64(token: &str) -> f64 {
    token.parse::<f64>().unwrap()
}

fn parse_usize(token: &str) -> usize {
    token.parse::<usize>().unwrap()
}

fn parse_cases(text: &str) -> Vec<Case> {
    let mut lines = text.lines().peekable();
    let header = lines.next().unwrap();
    assert_eq!(header, "QUADEXT1", "bad fixture header");
    let mut cases = Vec::new();
    while let Some(line) = lines.next() {
        if line.is_empty() {
            continue;
        }
        if line.starts_with("T ") {
            continue;
        }
        let index: usize = parse_usize(line.strip_prefix("CASE ").unwrap());

        let n: usize = parse_usize(lines.next().unwrap().strip_prefix("V ").unwrap());
        let mut vertices = Vec::with_capacity(n);
        for _ in 0..n {
            let parts: Vec<&str> = lines.next().unwrap().split(' ').collect();
            assert_eq!(parts[0], "v");
            vertices.push(Vector3::new(
                parse_f64(parts[1]),
                parse_f64(parts[2]),
                parse_f64(parts[3]),
            ));
        }

        let n: usize = parse_usize(lines.next().unwrap().strip_prefix("TRI ").unwrap());
        let mut triangles = Vec::with_capacity(n);
        for _ in 0..n {
            let parts: Vec<&str> = lines.next().unwrap().split(' ').collect();
            assert_eq!(parts[0], "t");
            triangles.push(vec![
                parse_usize(parts[1]),
                parse_usize(parts[2]),
                parse_usize(parts[3]),
            ]);
        }

        let n: usize = parse_usize(lines.next().unwrap().strip_prefix("UV ").unwrap());
        let mut uvs = Vec::with_capacity(n);
        for _ in 0..n {
            let parts: Vec<&str> = lines.next().unwrap().split(' ').collect();
            assert_eq!(parts[0], "u");
            let mut row = Vec::new();
            let mut i = 1;
            while i + 1 < parts.len() {
                row.push(Vector2::new(parse_f64(parts[i]), parse_f64(parts[i + 1])));
                i += 2;
            }
            uvs.push(row);
        }

        let n: usize = parse_usize(lines.next().unwrap().strip_prefix("SING ").unwrap());
        let mut singular = Vec::new();
        if n > 0 {
            let parts: Vec<&str> = lines.next().unwrap().split(' ').collect();
            assert_eq!(parts[0], "s");
            for part in &parts[1..] {
                singular.push(parse_usize(part));
            }
            assert_eq!(singular.len(), n, "case {index}: SING count");
        }

        let parts: Vec<&str> = lines.next().unwrap().split(' ').collect();
        assert_eq!(parts[0], "FLAGS");
        let compute_uvs = parts[1] == "1";
        let orig_mode: i32 = parts[2].parse().unwrap();
        let mut orig = None;
        if orig_mode > 0 {
            let n: usize = parse_usize(lines.next().unwrap().strip_prefix("ORIG ").unwrap());
            let mut rows = Vec::with_capacity(n);
            for _ in 0..n {
                let parts: Vec<&str> = lines.next().unwrap().split(' ').collect();
                assert_eq!(parts[0], "u");
                let mut row = Vec::new();
                let mut i = 1;
                while i + 1 < parts.len() {
                    row.push(Vector2::new(parse_f64(parts[i]), parse_f64(parts[i + 1])));
                    i += 2;
                }
                rows.push(row);
            }
            orig = Some(rows);
        }

        let ok = lines.next().unwrap().strip_prefix("OK ").unwrap() == "1";

        let n: usize = parse_usize(lines.next().unwrap().strip_prefix("CONN ").unwrap());
        let mut conns = Vec::with_capacity(n);
        for _ in 0..n {
            let parts: Vec<&str> = lines.next().unwrap().split(' ').collect();
            assert_eq!(parts[0], "c");
            conns.push(ExpectedConn {
                a: [
                    parse_f64(parts[1]),
                    parse_f64(parts[2]),
                    parse_f64(parts[3]),
                ],
                b: [
                    parse_f64(parts[4]),
                    parse_f64(parts[5]),
                    parse_f64(parts[6]),
                ],
                moved: parts[7].parse().unwrap(),
            });
        }

        let n: usize = parse_usize(lines.next().unwrap().strip_prefix("REMESH ").unwrap());
        let mut remesh = Vec::with_capacity(n);
        for _ in 0..n {
            let parts: Vec<&str> = lines.next().unwrap().split(' ').collect();
            assert_eq!(parts[0], "v");
            remesh.push([
                parse_f64(parts[1]),
                parse_f64(parts[2]),
                parse_f64(parts[3]),
            ]);
        }

        let n: usize = parse_usize(lines.next().unwrap().strip_prefix("QUADS ").unwrap());
        let mut quads = Vec::with_capacity(n);
        for _ in 0..n {
            let parts: Vec<&str> = lines.next().unwrap().split(' ').collect();
            assert_eq!(parts[0], "q");
            let len = parse_usize(parts[1]);
            let mut face = Vec::with_capacity(len);
            for part in &parts[2..] {
                face.push(parse_usize(part));
            }
            assert_eq!(face.len(), len, "case {index}: quad length");
            quads.push(face);
        }

        let n: usize = parse_usize(lines.next().unwrap().strip_prefix("RUV ").unwrap());
        let mut ruv = Vec::with_capacity(n);
        for _ in 0..n {
            let parts: Vec<&str> = lines.next().unwrap().split(' ').collect();
            assert_eq!(parts[0], "u");
            ruv.push([parse_f64(parts[1]), parse_f64(parts[2])]);
        }

        cases.push(Case {
            index,
            vertices,
            triangles,
            uvs,
            singular,
            compute_uvs,
            orig,
            ok,
            conns,
            remesh,
            quads,
            ruv,
        });
    }
    cases
}

fn bits3(p: &[f64; 3]) -> [u64; 3] {
    [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()]
}

/// Winding-canonical face for SET comparison (both windings, all
/// rotations, lexicographic minimum). Test-local copy of the idea, not
/// the impl under test.
fn canonical_face(face: &[usize]) -> Vec<usize> {
    let mut best = Vec::new();
    let reversed: Vec<usize> = face.iter().rev().copied().collect();
    for winding in [face, &reversed] {
        for start in 0..winding.len() {
            let mut candidate = Vec::with_capacity(winding.len());
            for i in 0..winding.len() {
                candidate.push(winding[(start + i) % winding.len()]);
            }
            if best.is_empty() || candidate < best {
                best = candidate;
            }
        }
    }
    best
}

fn check_case(case: &Case, failures: &mut Vec<String>) {
    let mut extractor = QuadExtractor::new(&case.vertices, &case.triangles, &case.uvs);
    extractor.set_compute_vertex_uvs(case.compute_uvs);
    if let Some(orig) = &case.orig {
        extractor.set_original_triangle_uvs(orig);
    }
    if !case.singular.is_empty() {
        extractor.set_singular_vertices(&case.singular);
    }
    let ok = extractor.extract();
    if ok != case.ok {
        failures.push(format!("case {}: OK Rust={ok} C++={}", case.index, case.ok));
        return;
    }

    // Connections: count, bitwise endpoints, exact moved flags.
    let conns = extractor.extracted_connections();
    let moved = extractor.extracted_connection_moved();
    if conns.len() != case.conns.len() {
        failures.push(format!(
            "case {}: CONN count Rust={} C++={}",
            case.index,
            conns.len(),
            case.conns.len()
        ));
    } else {
        for (i, (got, want)) in conns.iter().zip(case.conns.iter()).enumerate() {
            let ga = [got.0.x(), got.0.y(), got.0.z()];
            let gb = [got.1.x(), got.1.y(), got.1.z()];
            if bits3(&ga) != bits3(&want.a) || bits3(&gb) != bits3(&want.b) {
                failures.push(format!(
                    "case {}: CONN[{i}] endpoints differ\n  Rust a={ga:?} b={gb:?}\n  C++  a={:?} b={:?}",
                    case.index, want.a, want.b
                ));
                break;
            }
            let got_moved = moved.get(i).copied().unwrap_or(0);
            if got_moved != want.moved {
                failures.push(format!(
                    "case {}: CONN[{i}] moved Rust={got_moved} C++={}",
                    case.index, want.moved
                ));
                break;
            }
        }
    }

    // Remeshed vertices: bitwise MULTISET (order canonicalized).
    let verts = extractor.remeshed_vertices();
    if verts.len() != case.remesh.len() {
        failures.push(format!(
            "case {}: REMESH count Rust={} C++={}",
            case.index,
            verts.len(),
            case.remesh.len()
        ));
        return;
    }
    let mut pool: BTreeMap<[u64; 3], VecDeque<usize>> = BTreeMap::new();
    for (i, v) in verts.iter().enumerate() {
        pool.entry(bits3(&[v.x(), v.y(), v.z()]))
            .or_default()
            .push_back(i);
    }
    let mut index_map = vec![usize::MAX; case.remesh.len()];
    let mut unmatched = 0;
    for (i, want) in case.remesh.iter().enumerate() {
        match pool.get_mut(&bits3(want)) {
            Some(queue) => match queue.pop_front() {
                Some(j) => index_map[i] = j,
                None => unmatched += 1,
            },
            None => unmatched += 1,
        }
    }
    if unmatched > 0 {
        // Nearest-match diagnostic: distinguishes float drift (small,
        // dense) from structural divergence (large or sparse).
        let mut worst = 0.0;
        for want in &case.remesh {
            let mut best = f64::INFINITY;
            for v in verts {
                let d = ((v.x() - want[0]).powi(2)
                    + (v.y() - want[1]).powi(2)
                    + (v.z() - want[2]).powi(2))
                .sqrt();
                if d < best {
                    best = d;
                }
            }
            if best > worst {
                worst = best;
            }
        }
        failures.push(format!(
            "case {}: REMESH {unmatched}/{} vertices lack a bitwise match (worst nearest distance {worst:.3e})",
            case.index,
            case.remesh.len()
        ));
        return;
    }

    // Quads: remapped + canonicalized SET comparison.
    let quads = extractor.remeshed_quads();
    if quads.len() != case.quads.len() {
        failures.push(format!(
            "case {}: QUADS count Rust={} C++={}",
            case.index,
            quads.len(),
            case.quads.len()
        ));
    } else {
        let mut want: Vec<Vec<usize>> = case
            .quads
            .iter()
            .map(|face| canonical_face(&face.iter().map(|v| index_map[*v]).collect::<Vec<_>>()))
            .collect();
        want.sort();
        let mut got: Vec<Vec<usize>> = quads.iter().map(|face| canonical_face(face)).collect();
        got.sort();
        if got != want {
            let mut shown = 0;
            let mut detail = String::new();
            for (g, w) in got.iter().zip(want.iter()) {
                if g != w && shown < 3 {
                    detail.push_str(&format!("\n  Rust {g:?} vs C++ {w:?}"));
                    shown += 1;
                }
            }
            failures.push(format!(
                "case {}: QUADS multiset differs{detail}",
                case.index
            ));
        }
    }

    // Vertex UVs: bitwise after the same remap.
    let ruv = extractor.remeshed_vertex_uvs();
    if ruv.len() != case.ruv.len() {
        failures.push(format!(
            "case {}: RUV count Rust={} C++={}",
            case.index,
            ruv.len(),
            case.ruv.len()
        ));
    } else {
        for (i, want) in case.ruv.iter().enumerate() {
            let got = ruv[index_map[i]];
            if got.x().to_bits() != want[0].to_bits() || got.y().to_bits() != want[1].to_bits() {
                failures.push(format!(
                    "case {}: RUV[{i}] Rust=({:?}) C++=({want:?})",
                    case.index,
                    [got.x(), got.y()]
                ));
                break;
            }
        }
    }
}

#[test]
fn diff_replay() {
    let text = std::fs::read_to_string(FIXTURE).unwrap();
    let cases = parse_cases(&text);
    assert!(!cases.is_empty(), "fixture parsed zero cases");
    let mut failures = Vec::new();
    for case in &cases {
        check_case(case, &mut failures);
    }
    let strict = cases.len() - failures.len().min(cases.len());
    println!(
        "quad_extractor diff: {strict}/{} cases strict-pass",
        cases.len()
    );
    for failure in &failures {
        println!("MISMATCH: {failure}");
    }
    assert!(
        failures.is_empty(),
        "{} mismatching cases (see MISMATCH lines above)",
        failures.len()
    );
}

// Intended-behavior regen: the C++ reference is deleted, so when the
// extractor IMPROVES (fewer non-quads, never more), the frozen C++
// expectations for the changed cases are re-pinned to the new output:
//
//   UPDATE_QUADEXT=1 cargo test --release -p retopo_core --test quad_extractor_diff regen_expect
//
// Only cases whose REMESH/QUADS/RUV counts changed are rewritten (a
// firing cleanup pass always shrinks counts; value-only drift is never
// re-pinned, it fails as usual). Review the per-case census below and
// the fixture diff, then re-run the suite green.
//
// Re-baseline mode (`UPDATE_QUADEXT_PERMISSIVE=1`, for order-only
// changes like container swaps): per-case non-quad growth is allowed,
// but the totals printed at the end must stay neutral and the bench
// noise distributions must overlap before committing.
#[test]
fn regen_expect() {
    if std::env::var_os("UPDATE_QUADEXT").is_none() {
        return;
    }
    let permissive = std::env::var_os("UPDATE_QUADEXT_PERMISSIVE").is_some();
    let text = std::fs::read_to_string(FIXTURE).unwrap();
    let cases = parse_cases(&text);
    let mut lines: Vec<String> = text.lines().map(str::to_string).collect();
    let mut rewritten = 0;
    let mut total_old_nq = 0;
    let mut total_new_nq = 0;
    for case in &cases {
        let mut extractor = QuadExtractor::new(&case.vertices, &case.triangles, &case.uvs);
        extractor.set_compute_vertex_uvs(case.compute_uvs);
        if let Some(orig) = &case.orig {
            extractor.set_original_triangle_uvs(orig);
        }
        if !case.singular.is_empty() {
            extractor.set_singular_vertices(&case.singular);
        }
        let _ = extractor.extract();
        let verts = extractor.remeshed_vertices();
        let quads = extractor.remeshed_quads();
        let ruv = extractor.remeshed_vertex_uvs();
        let counts_changed = verts.len() != case.remesh.len()
            || quads.len() != case.quads.len()
            || ruv.len() != case.ruv.len();
        if !counts_changed {
            // Permissive mode (order-only re-baselines) also snapshots
            // value drift at equal counts — but only REMESH/QUADS/RUV
            // drift. OK/CONN failures mean real breakage and stay red.
            if !permissive {
                continue;
            }
            let mut probe = Vec::new();
            check_case(case, &mut probe);
            if probe.is_empty()
                || probe.iter().any(|m| {
                    let class = m.split(": ").nth(1).unwrap_or("");
                    !(class.starts_with("REMESH")
                        || class.starts_with("QUADS")
                        || class.starts_with("RUV"))
                })
            {
                continue;
            }
        }
        let old_nq = case.quads.iter().filter(|f| f.len() != 4).count();
        let new_nq = quads.iter().filter(|f| f.len() != 4).count();
        println!(
            "case {}: verts {}->{} faces {}->{} ruv {}->{} nonquads {old_nq}->{new_nq}",
            case.index,
            case.remesh.len(),
            verts.len(),
            case.quads.len(),
            quads.len(),
            case.ruv.len(),
            ruv.len(),
        );
        total_old_nq += old_nq;
        total_new_nq += new_nq;
        assert!(
            permissive || new_nq <= old_nq,
            "case {}: regen would ADD non-quads ({old_nq}->{new_nq}), refusing",
            case.index
        );
        // Re-found per case: earlier splices shift line numbers.
        let start = find_case_start(&lines, case.index);
        replace_section(&mut lines, start, "REMESH", &format_remesh(verts));
        replace_section(&mut lines, start, "QUADS", &format_quads(quads));
        replace_section(&mut lines, start, "RUV", &format_ruv(ruv));
        rewritten += 1;
    }
    // `lines()` strips terminators; the fixture ends with exactly one.
    std::fs::write(FIXTURE, lines.join("\n") + "\n").unwrap();
    println!(
        "regen: rewrote {rewritten}/{} cases, nonquads {total_old_nq}->{total_new_nq}; re-run the suite",
        cases.len()
    );
}

/// Exact "CASE {index}" line (exact match, so "CASE 2" never matches
/// "CASE 20").
fn find_case_start(lines: &[String], index: usize) -> usize {
    let want = format!("CASE {index}");
    lines
        .iter()
        .position(|line| *line == want)
        .unwrap_or_else(|| panic!("regen: CASE {index} not found"))
}

/// Replace the `HEADER n` line plus its n payload lines, searching
/// forward from the case start (never past the next case start).
fn replace_section(lines: &mut Vec<String>, start: usize, header: &str, fresh: &[String]) {
    let mut at = None;
    for i in start..lines.len() {
        if i != start && lines[i].starts_with("CASE ") {
            break;
        }
        if let Some(rest) = lines[i].strip_prefix(header)
            && let Some(count) = rest.strip_prefix(' ')
            && let Ok(n) = count.split(' ').next().unwrap().parse::<usize>()
        {
            at = Some((i, n));
            break;
        }
    }
    let (i, n) = at.unwrap_or_else(|| panic!("regen: {header} section not found"));
    assert_eq!(
        fresh[0]
            .split(' ')
            .nth(1)
            .unwrap()
            .parse::<usize>()
            .unwrap(),
        fresh.len() - 1
    );
    lines.splice(i..i + 1 + n, fresh.iter().cloned());
}

fn format_remesh(verts: &[Vector3]) -> Vec<String> {
    let mut out = vec![format!("REMESH {}", verts.len())];
    for v in verts {
        out.push(format!("v {} {} {}", v.x(), v.y(), v.z()));
    }
    out
}

fn format_quads(quads: &[Vec<usize>]) -> Vec<String> {
    let mut out = vec![format!("QUADS {}", quads.len())];
    for q in quads {
        out.push(format!(
            "q {} {}",
            q.len(),
            q.iter().map(usize::to_string).collect::<Vec<_>>().join(" ")
        ));
    }
    out
}

fn format_ruv(ruv: &[Vector2]) -> Vec<String> {
    let mut out = vec![format!("RUV {}", ruv.len())];
    for u in ruv {
        out.push(format!("u {} {}", u.x(), u.y()));
    }
    out
}
