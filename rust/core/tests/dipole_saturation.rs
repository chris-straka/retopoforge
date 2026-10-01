// Saturation-measurement harness for dipole-insertion research.
//
// Runs the finger fixtures in tests/fixtures/finger-*.obj (procedural
// curved thin tubes, see gen_finger_fixtures.py) through AutoRemesher
// with tip masks at several asks, each ask both with dipoles off and
// with dipoles auto, and reports HONEST metrics per the spike's metric
// warnings (docs/density-poles-spike.md):
//
//   faceAbs  = masked inside quads / plain inside quads (absolute gain)
//   linear   = sqrt(plain inside mean area / masked inside mean area)
//   totalFrac= masked total quads / plain total quads (budget health)
//   ratio    = inside/control conflated ratio (REPORTED WITH WARNING:
//              it mixes face gain with control-side collapse)
//
// plus a valence census (pole-identity check: dipole insertion must
// move the inside irregular-vert counts; sizing-only runs keep the
// pole set identical) and the per-island dipole flip counts.
//
// Regions (must match gen_finger_fixtures.py):
//   TIP_Y  = 2.20  masked "inside" region: output quad centroid y > TIP_Y
//   BASE_Y = 0.80  control region for finger-single: centroid y < BASE_Y
//   split/fused: masked lobe x > 0, control lobe x < 0 (both y > TIP_Y)
//
// Usage: cargo test --release -p retopo_core --test dipole_saturation
//        -- --nocapture --test-threads=1
// (--nocapture shows the tables; single-threaded avoids MT-jitter
// cross-talk between concurrent remeshes.)
//
// Assertions are weak sanity bands only (success, nonzero, no total
// collapse, no lost islands, dipole gating: off/plain/mild place
// nothing, auto 3x/4x place something); this harness MEASURES, it does
// not pin. Baseline numbers live in docs/dipole-fixtures-baseline.md.

use retopo_core::auto_remesher::AutoRemesher;
use retopo_core::obj_reader::load_obj_positions_and_triangles;
use retopo_core::quad_parameterizer::DipoleConfig;
use retopo_core::vector3::Vector3;
use std::collections::HashMap;
use std::path::PathBuf;

const TIP_Y: f64 = 2.20;
const BASE_Y: f64 = 0.80;
const TARGET_TRIS: usize = 2000; // 1000 quads
const ASKS: [f64; 4] = [1.5, 2.0, 3.0, 4.0];

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../tests/fixtures")
        .join(name)
}

fn load_fixture(name: &str) -> (Vec<Vector3>, Vec<Vec<usize>>) {
    let mut positions: Vec<f32> = Vec::new();
    let mut triangles: Vec<Vec<usize>> = Vec::new();
    let mut err = String::new();
    let ok = load_obj_positions_and_triangles(
        &fixture_path(name),
        &mut positions,
        &mut triangles,
        None,
        Some(&mut err),
    );
    assert!(ok, "load {name}: {err}");
    assert!(!triangles.is_empty(), "load {name}: no triangles");
    let verts = positions
        .chunks_exact(3)
        .map(|p| Vector3::new(p[0] as f64, p[1] as f64, p[2] as f64))
        .collect();
    (verts, triangles)
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Fixture {
    Single,
    Split,
    Fused,
}

impl Fixture {
    fn file(self) -> &'static str {
        match self {
            Fixture::Single => "finger-single.obj",
            Fixture::Split => "finger-split.obj",
            Fixture::Fused => "finger-fused.obj",
        }
    }
    fn masked(self, x: f64, y: f64) -> bool {
        match self {
            Fixture::Single => y > TIP_Y,
            Fixture::Split | Fixture::Fused => y > TIP_Y && x > 0.0,
        }
    }
    fn control(self, x: f64, y: f64) -> bool {
        match self {
            Fixture::Single => y < BASE_Y,
            Fixture::Split | Fixture::Fused => y > TIP_Y && x < 0.0,
        }
    }
}

struct RegionStat {
    quads: usize,
    area: f64,
}

struct Census {
    v3: usize,
    v4: usize,
    v5: usize,
    other: usize,
}

struct RunMetrics {
    inside: RegionStat,
    control: RegionStat,
    total_quads: usize,
    total_verts: usize,
    inside_poles: Census, // valence census over masked-region output verts
    total_poles: Census,
    island_quads: Vec<usize>, // per-island output (a 0 here is a lost island)
    dipole_flips: usize,      // total dipole flips placed over all islands
}

fn poly_area(verts: &[Vector3], poly: &[usize]) -> f64 {
    // Fan triangulation, same convention as the spike's OBJ measurements.
    let mut area = 0.0;
    let a = &verts[poly[0]];
    for w in poly[2..].iter() {
        let b = &verts[poly[1]];
        let c = &verts[*w];
        let abx = b.x() - a.x();
        let aby = b.y() - a.y();
        let abz = b.z() - a.z();
        let acx = c.x() - a.x();
        let acy = c.y() - a.y();
        let acz = c.z() - a.z();
        let cx = aby * acz - abz * acy;
        let cy = abz * acx - abx * acz;
        let cz = abx * acy - aby * acx;
        area += 0.5 * (cx * cx + cy * cy + cz * cz).sqrt();
    }
    area
}

fn measure(
    fix: Fixture,
    verts: &[Vector3],
    tris: &[Vec<usize>],
    mask: Option<&[f64]>,
    dipoles: DipoleConfig,
) -> RunMetrics {
    let mut remesher = AutoRemesher::new(verts, tris);
    remesher.set_target_triangle_count(TARGET_TRIS);
    remesher.set_quiet(true); // engine defaults otherwise (= CLI defaults)
    if let Some(m) = mask {
        remesher.set_density_multipliers(m);
    }
    remesher.set_dipoles(dipoles);
    assert!(remesher.remesh(), "remesh failed for {}", fix.file());
    let out_v = remesher.remeshed_vertices();
    let out_q = remesher.remeshed_quads();
    assert!(!out_q.is_empty(), "empty output for {}", fix.file());

    let mut inside = RegionStat {
        quads: 0,
        area: 0.0,
    };
    let mut control = RegionStat {
        quads: 0,
        area: 0.0,
    };
    for q in out_q {
        assert!(q.len() >= 3, "degenerate output poly");
        let (mut cx, mut cy) = (0.0, 0.0);
        for i in q {
            cx += out_v[*i].x();
            cy += out_v[*i].y();
        }
        cx /= q.len() as f64;
        cy /= q.len() as f64;
        let a = poly_area(out_v, q);
        if fix.masked(cx, cy) {
            inside.quads += 1;
            inside.area += a;
        } else if fix.control(cx, cy) {
            control.quads += 1;
            control.area += a;
        }
    }

    // Valence census from output polys (pole-identity check).
    let mut valence = vec![0usize; out_v.len()];
    for q in out_q {
        for i in q {
            valence[*i] += 1;
        }
    }
    let mut inside_poles = Census {
        v3: 0,
        v4: 0,
        v5: 0,
        other: 0,
    };
    let mut total_poles = Census {
        v3: 0,
        v4: 0,
        v5: 0,
        other: 0,
    };
    for (i, v) in out_v.iter().enumerate() {
        let val = valence[i];
        let bump = |c: &mut Census| match val {
            3 => c.v3 += 1,
            4 => c.v4 += 1,
            5 => c.v5 += 1,
            _ => c.other += 1,
        };
        bump(&mut total_poles);
        if fix.masked(v.x(), v.y()) {
            bump(&mut inside_poles);
        }
    }

    RunMetrics {
        inside,
        control,
        total_quads: out_q.len(),
        total_verts: out_v.len(),
        inside_poles,
        total_poles,
        island_quads: remesher.island_output_quad_counts().to_vec(),
        dipole_flips: remesher.island_dipole_counts().iter().sum(),
    }
}

fn mean_area(r: &RegionStat) -> f64 {
    if r.quads == 0 {
        f64::NAN
    } else {
        r.area / r.quads as f64
    }
}

fn saturation_curve(fix: Fixture) {
    let (verts, tris) = load_fixture(fix.file());
    println!(
        "=== {} (in: {} verts, {} tris) ===",
        fix.file(),
        verts.len(),
        tris.len()
    );
    println!(
        "target quads: {} | TIP_Y={TIP_Y} BASE_Y={BASE_Y} | engine defaults",
        TARGET_TRIS / 2
    );
    println!(
        "ask      | inQ  inMeanArea | ctlQ ctlMeanArea | totalQ/V | faceAbs linear totalFrac | ratio! | inV3/inV5 totV3/totV5 | isl/dip"
    );
    println!(
        "---------|-----------------------------------------------------------|------------------|------|---------------------------|--------"
    );

    // Plain with dipoles auto: the unmasked run must gate to a no-op.
    let plain = measure(fix, &verts, &tris, None, DipoleConfig::automatic());
    print_row("plain", &plain, None);
    assert!(plain.inside.quads > 0, "plain run: empty inside region");
    assert!(plain.control.quads > 0, "plain run: empty control region");
    assert!(
        plain.island_quads.iter().all(|&q| q > 0),
        "plain run: lost island {:?}",
        plain.island_quads
    );
    assert_eq!(
        plain.dipole_flips, 0,
        "plain run: unmasked must place no dipoles"
    );

    for ask in ASKS {
        let mask: Vec<f64> = verts
            .iter()
            .map(|v| if fix.masked(v.x(), v.y()) { ask } else { 1.0 })
            .collect();
        let n = mask.iter().filter(|m| **m != 1.0).count();
        assert!(n > 0, "mask selects no input verts");
        for (mode, dipoles) in [("off", DipoleConfig::off()), ("auto", DipoleConfig::automatic())]
        {
            let m = measure(fix, &verts, &tris, Some(&mask), dipoles);
            print_row(&format!("{ask:.1}x/{mode}"), &m, Some(&plain));
            // Weak sanity bands only: harness measures, docs pin.
            assert!(m.inside.quads > 0, "ask {ask}/{mode}: empty inside region");
            assert!(m.control.quads > 0, "ask {ask}/{mode}: empty control region");
            assert!(
                m.island_quads.iter().all(|&q| q > 0),
                "ask {ask}/{mode}: lost island {:?}",
                m.island_quads
            );
            let total_frac = m.total_quads as f64 / plain.total_quads as f64;
            assert!(
                total_frac > 0.4,
                "ask {ask}/{mode}: total collapse ({total_frac:.2}, plain {}, masked {})",
                plain.total_quads,
                m.total_quads
            );
            // Gating: off rows never place; auto places exactly on sharp
            // asks (the 1.5 gate fires at ~3x, skips <= 2x).
            if mode == "off" || ask < 2.5 {
                assert_eq!(
                    m.dipole_flips, 0,
                    "ask {ask}/{mode}: must place no dipoles"
                );
            } else {
                assert!(
                    m.dipole_flips > 0,
                    "ask {ask}/{mode}: sharp step must place dipoles"
                );
            }
        }
    }
    println!("(!) ratio = inside/control conflated ratio: mixes face gain with");
    println!("    control-side collapse. Compare faceAbs + totalFrac instead.");
}

fn print_row(label: &str, m: &RunMetrics, plain: Option<&RunMetrics>) {
    let (face_abs, linear, total_frac, ratio_note) = match plain {
        None => (
            "   -".to_string(),
            "   -".to_string(),
            "   -".to_string(),
            format!("{:>4}", "-"),
        ),
        Some(p) => {
            let fa = m.inside.quads as f64 / p.inside.quads as f64;
            let lin = (mean_area(&p.inside) / mean_area(&m.inside)).sqrt();
            let tf = m.total_quads as f64 / p.total_quads as f64;
            let r_plain = p.inside.quads as f64 / p.control.quads as f64;
            let r_mask = m.inside.quads as f64 / m.control.quads as f64;
            (
                format!("{fa:>4.2}"),
                format!("{lin:>4.2}"),
                format!("{tf:>4.2}"),
                format!("{r_plain:.2}>{r_mask:.2}"),
            )
        }
    };
    println!(
        "{label:>8} | {:>3} {:>10.6} | {:>3} {:>11.6} | {:>9} | {face_abs} {linear} {total_frac} | {ratio_note} | {:>3}/{:<3} {:>3}/{:<3} | {:?}/{}",
        m.inside.quads,
        mean_area(&m.inside),
        m.control.quads,
        mean_area(&m.control),
        format!("{}/{}", m.total_quads, m.total_verts),
        m.inside_poles.v3,
        m.inside_poles.v5,
        m.total_poles.v3,
        m.total_poles.v5,
        m.island_quads,
        m.dipole_flips,
    );
}

#[test]
fn fixture_inputs_are_clean() {
    // Fast structural check, no remesh: manifold edges, positive volume,
    // bbox matches the TIP_Y/BASE_Y region assumptions, masks nonempty.
    for fix in [Fixture::Single, Fixture::Split, Fixture::Fused] {
        let (verts, tris) = load_fixture(fix.file());
        let mut edges: HashMap<(usize, usize), usize> = HashMap::new();
        let mut directed: HashMap<(usize, usize), usize> = HashMap::new();
        let mut vol = 0.0;
        for t in &tris {
            assert_eq!(t.len(), 3, "{}: non-triangle input", fix.file());
            let (a, b, c) = (t[0], t[1], t[2]);
            for (u, v) in [(a, b), (b, c), (c, a)] {
                *edges.entry((u.min(v), u.max(v))).or_insert(0) += 1;
                *directed.entry((u, v)).or_insert(0) += 1;
            }
            let (ax, ay, az) = (verts[a].x(), verts[a].y(), verts[a].z());
            let (bx, by, bz) = (verts[b].x(), verts[b].y(), verts[b].z());
            let (cx, cy, cz) = (verts[c].x(), verts[c].y(), verts[c].z());
            vol += ax * (by * cz - bz * cy) + bx * (cy * az - cz * ay) + cx * (ay * bz - by * az);
        }
        let bad = edges.values().filter(|n| **n != 2).count();
        assert_eq!(bad, 0, "{}: {bad} non-manifold edges", fix.file());
        // Orientation consistency: every directed edge needs exactly one
        // opposite (the 2026-10-01 pole fans passed the undirected check
        // while splitting into their own orientation islands).
        let flipped = directed
            .iter()
            .filter(|(e, n)| **n != 1 || directed.get(&(e.1, e.0)) != Some(&1))
            .count();
        assert_eq!(flipped, 0, "{}: {flipped} flipped edges", fix.file());
        assert!(vol > 0.0, "{}: non-positive volume", fix.file());
        let (mut ymin, mut ymax) = (f64::INFINITY, f64::NEG_INFINITY);
        let (mut masked, mut control) = (0, 0);
        for v in &verts {
            ymin = ymin.min(v.y());
            ymax = ymax.max(v.y());
            if fix.masked(v.x(), v.y()) {
                masked += 1;
            }
            if fix.control(v.x(), v.y()) {
                control += 1;
            }
        }
        assert!(
            ymin < BASE_Y && ymax > TIP_Y,
            "{}: bbox y {ymin}..{ymax}",
            fix.file()
        );
        assert!(
            masked > 0 && control > 0,
            "{}: empty mask/control",
            fix.file()
        );
        println!(
            "{}: {} verts {} tris vol={:.3} y=[{:.2},{:.2}] masked={masked} control={control}",
            fix.file(),
            verts.len(),
            tris.len(),
            vol / 6.0,
            ymin,
            ymax
        );
    }
}

#[test]
fn unmasked_and_mild_identical_with_dipoles_enabled() {
    // Bitwise control: dipoles auto must be bit-identical to off wherever
    // the gate cannot fire (unmasked; mild 2x step below the 1.5 gate).
    let (verts, tris) = load_fixture(Fixture::Single.file());
    let mild: Vec<f64> = verts
        .iter()
        .map(|v| {
            if Fixture::Single.masked(v.x(), v.y()) {
                2.0
            } else {
                1.0
            }
        })
        .collect();
    for (name, mask) in [("plain", None), ("mild-2x", Some(mild.as_slice()))] {
        let run = |dipoles: DipoleConfig| {
            let mut r = AutoRemesher::new(&verts, &tris);
            r.set_target_triangle_count(TARGET_TRIS);
            r.set_quiet(true);
            if let Some(m) = mask {
                r.set_density_multipliers(m);
            }
            r.set_dipoles(dipoles);
            assert!(r.remesh(), "{name}: remesh failed");
            assert_eq!(
                r.island_dipole_counts().iter().sum::<usize>(),
                0,
                "{name}: gate must place nothing"
            );
            (r.remeshed_vertices().to_vec(), r.remeshed_quads().to_vec())
        };
        let (av, aq) = run(DipoleConfig::off());
        let (bv, bq) = run(DipoleConfig::automatic());
        assert_eq!(aq, bq, "{name}: quad connectivity must match bit-for-bit");
        assert_eq!(av.len(), bv.len(), "{name}: vertex count must match");
        for (a, b) in av.iter().zip(bv.iter()) {
            assert_eq!(a.x().to_bits(), b.x().to_bits(), "{name}: x bits");
            assert_eq!(a.y().to_bits(), b.y().to_bits(), "{name}: y bits");
            assert_eq!(a.z().to_bits(), b.z().to_bits(), "{name}: z bits");
        }
        println!("{name}: off == auto bitwise ({} quads)", aq.len());
    }
}

#[test]
fn saturation_curve_single() {
    saturation_curve(Fixture::Single);
}

#[test]
fn saturation_curve_split() {
    saturation_curve(Fixture::Split);
}

#[test]
fn saturation_curve_fused() {
    saturation_curve(Fixture::Fused);
}
