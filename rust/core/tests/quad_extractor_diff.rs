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
        // Container-oracle sections (replayed by the `cxx_hash_oracle` unit
        // test, not here).
        if line.starts_with("CXXHASH")
            || line.starts_with("NEXTPRIME")
            || line.starts_with("HS ")
            || line.starts_with("HM ")
            || line.starts_with("NP ")
            || line.starts_with("NP_")
        {
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
