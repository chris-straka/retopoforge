#[cfg(test)]
mod normalization_tests {
    use super::super::*;

    fn two_points(d: f64) -> Vec<Vector3> {
        vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(d, 0.0, 0.0)]
    }

    #[test]
    fn healthy_scales_pass_through() {
        assert_eq!(AutoRemesher::normalization_scale(&two_points(1.0)), 1.0);
        assert_eq!(AutoRemesher::normalization_scale(&two_points(228.8)), 1.0);
        // Degenerate / empty / non-finite: downstream handles as today.
        assert_eq!(AutoRemesher::normalization_scale(&two_points(0.0)), 1.0);
        assert_eq!(AutoRemesher::normalization_scale(&[]), 1.0);
        assert_eq!(
            AutoRemesher::normalization_scale(&[
                Vector3::new(f64::NAN, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 0.0)
            ]),
            1.0
        );
        assert_eq!(AutoRemesher::normalization_scale(&two_points(5e-324)), 1.0);
    }

    #[test]
    fn tiny_scales_map_into_unit_band() {
        assert_eq!(AutoRemesher::normalization_scale(&two_points(0.5)), 2.0);
        assert_eq!(AutoRemesher::normalization_scale(&two_points(0.3)), 4.0);
        assert_eq!(AutoRemesher::normalization_scale(&two_points(0.999)), 2.0);
        // Sweep: every sub-unit normal diagonal lands in [1, 2) under an
        // exact power of two.
        let mut d = f64::MIN_POSITIVE;
        while d < 1.0 {
            let s = AutoRemesher::normalization_scale(&two_points(d));
            assert!(s > 1.0 && (s.log2().fract() == 0.0), "d={d:e} s={s}");
            let diag = d.hypot(0.0).hypot(0.0);
            let scaled = diag * s;
            assert!(
                (1.0..2.0).contains(&scaled),
                "d={d:e} s={s} scaled={scaled}"
            );
            d *= 1.37;
        }
    }

    #[test]
    fn overflow_risk_refuses() {
        // Huge offset, tiny extent: scaling up would overflow coords.
        let verts = vec![
            Vector3::new(8e307, 0.0, 0.0),
            Vector3::new(8e307, 1e-300, 0.0),
        ];
        assert_eq!(AutoRemesher::normalization_scale(&verts), 1.0);
    }

    #[test]
    fn round_trip_is_exact() {
        // (x * s) * (1/s) == x for the scales we emit, denormals included.
        for s in [2.0, 4.0, 2f64.powi(30), 2f64.powi(1022)] {
            for x in [0.3, -17.25, 5e-324, 2.47e-308, 1.5e-10, 123.456] {
                if !(x * s).is_finite() {
                    continue; // overflow pairs: refused by the caller guard
                }
                assert_eq!((x * s) * (1.0 / s), x, "s={s} x={x:e}");
            }
        }
    }
}

#[cfg(test)]
mod snap_tests {
    use super::super::*;

    fn bounds(diag: f64) -> (Vector3, Vector3) {
        (Vector3::new(0.0, 0.0, 0.0), Vector3::new(diag, 0.0, 0.0))
    }

    #[test]
    fn snap_kills_sub_grid_noise() {
        // Grid 1e-6 of diag 228.8; noise 1e-9 must vanish.
        let (lo, hi) = bounds(228.8);
        let clean = [10.0f32, -3.25, 0.5];
        for jitter in [0.0, 1e-7, -1e-7, 2.28e-7, -2.28e-7] {
            let mut p = [clean[0] + jitter, clean[1] + jitter, clean[2] + jitter];
            AutoRemesher::snap_decimator_input(&mut p, &lo, &hi);
            let mut q = clean;
            AutoRemesher::snap_decimator_input(&mut q, &lo, &hi);
            assert_eq!(p, q, "jitter={jitter:e}");
        }
    }

    #[test]
    fn snap_is_idempotent_and_close() {
        let (lo, hi) = bounds(100.0);
        let orig = [33.33333f32, -99.99999, 0.00001];
        let mut p = orig;
        AutoRemesher::snap_decimator_input(&mut p, &lo, &hi);
        // Displacement bounded by half a grid step (1e-4 here).
        for (c, o) in p.iter().zip(orig.iter()) {
            assert!((c - o).abs() <= 0.5e-4 * 1.01, "c={c} o={o}");
        }
        // Snapping twice is bitwise identical (fixed point).
        let mut q = p;
        AutoRemesher::snap_decimator_input(&mut q, &lo, &hi);
        assert_eq!(p, q);
    }

    #[test]
    fn snap_skips_degenerate_bounds() {
        let zero = Vector3::new(1.0, 2.0, 3.0);
        let mut p = [1.5f32, 2.5, 3.5];
        AutoRemesher::snap_decimator_input(&mut p, &zero, &zero);
        assert_eq!(p, [1.5, 2.5, 3.5]);
        let nan = Vector3::new(f64::NAN, 0.0, 0.0);
        let mut q = [1.5f32, 2.5, 3.5];
        AutoRemesher::snap_decimator_input(&mut q, &zero, &nan);
        assert_eq!(q, [1.5, 2.5, 3.5]);
    }
}

#[cfg(test)]
mod coverage_tests {
    use super::super::coverage::{COVERAGE_SCAN_TRIS, CoverageIndex};
    use super::super::*;

    fn v(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3::new(x, y, z)
    }

    fn worst_of(gaps: &[f64]) -> f64 {
        gaps.iter().cloned().fold(0.0, f64::max)
    }

    #[test]
    fn covered_grid_reports_zero_gap() {
        // 2x2 working grid, one quad covering all of it.
        let working = vec![
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(0.0, 1.0, 0.0),
            v(1.0, 1.0, 0.0),
        ];
        let quads = vec![vec![0, 1, 3, 2]];
        let gaps = AutoRemesher::coverage_gaps(&working, &working, &quads);
        assert!(worst_of(&gaps) < 1e-12, "gaps={gaps:?}");
    }

    #[test]
    fn missing_half_reports_exact_gap() {
        // Working grid 2 wide, quads cover only the left half (x in
        // [0, 1]): the x=2 column sits exactly 1.0 off the surface.
        let working = vec![
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(2.0, 0.0, 0.0),
            v(0.0, 1.0, 0.0),
            v(1.0, 1.0, 0.0),
            v(2.0, 1.0, 0.0),
        ];
        let quads = vec![vec![0, 1, 4, 3]];
        let gaps = AutoRemesher::coverage_gaps(&working, &working, &quads);
        assert!((worst_of(&gaps) - 1.0).abs() < 1e-12, "gaps={gaps:?}");
    }

    #[test]
    fn coverage_degenerates_totally() {
        let working = vec![v(0.0, 0.0, 0.0)];
        // No quads at all: infinite gap (total failure).
        assert_eq!(
            AutoRemesher::coverage_gaps(&working, &working, &[]),
            vec![f64::INFINITY]
        );
        // Degenerate quad indices (out of range): no valid triangles.
        assert_eq!(
            AutoRemesher::coverage_gaps(&working, &working, &[vec![7, 8, 9]]),
            vec![f64::INFINITY]
        );
        // No working verts: nothing to cover.
        let quads = vec![vec![0, 1, 2]];
        assert!(AutoRemesher::coverage_gaps(&[], &working, &quads).is_empty());
    }

    #[test]
    fn coverage_failed_counts_beyond_three_widths() {
        // diag 100, 100 quads: nominal width 10, bar at 30.
        let diag = 100.0;
        let nquads = 100;
        let no_tris: &[Vec<usize>] = &[];
        // 24 scattered verts at 31 (over the bar) + rest covered:
        // quiet (no adjacency, no patch).
        let mut gaps = vec![0.5; 200];
        for g in gaps.iter_mut().take(24) {
            *g = 31.0;
        }
        assert_eq!(
            AutoRemesher::coverage_failed(&gaps, diag, nquads, no_tris, 200),
            (false, 24)
        );
        // The 25th over-the-bar vert trips the count floor.
        gaps[24] = 31.0;
        assert_eq!(
            AutoRemesher::coverage_failed(&gaps, diag, nquads, no_tris, 200),
            (true, 25)
        );
        // Just under the bar never counts, however many.
        let gaps = vec![29.9; 200];
        assert_eq!(
            AutoRemesher::coverage_failed(&gaps, diag, nquads, no_tris, 200),
            (false, 0)
        );
        // Empty quads belong to the failed-island path, never retry.
        assert_eq!(
            AutoRemesher::coverage_failed(&gaps, diag, 0, no_tris, 200),
            (false, 0)
        );
        // Degenerate scale never fires.
        assert_eq!(
            AutoRemesher::coverage_failed(&gaps, 0.0, nquads, no_tris, 200),
            (false, 0)
        );
        assert_eq!(
            AutoRemesher::coverage_failed(&gaps, f64::NAN, nquads, no_tris, 200),
            (false, 0)
        );
    }

    #[test]
    fn coverage_failed_catches_connected_patch_below_floor() {
        // Same bar (30): a CONNECTED run of over-the-bar verts trips
        // the per-region patch bar at 10 even though the count floor
        // (25) stays quiet; 9 connected stays quiet.
        let diag = 100.0;
        let nquads = 100;
        // Triangle strip chaining verts 0..24 (overlapping windows).
        let strip: Vec<Vec<usize>> = (0..23).map(|i| vec![i, i + 1, i + 2]).collect();
        let mut gaps = vec![0.5; 200];
        for g in gaps.iter_mut().take(10) {
            *g = 31.0;
        }
        assert_eq!(
            AutoRemesher::coverage_failed(&gaps, diag, nquads, &strip, 200),
            (true, 10)
        );
        gaps[9] = 0.5;
        assert_eq!(
            AutoRemesher::coverage_failed(&gaps, diag, nquads, &strip, 200),
            (false, 9)
        );
        // Out-of-range corners never panic, never connect.
        let wild = vec![vec![500, 501, 502], vec![0, 1]];
        assert_eq!(
            AutoRemesher::coverage_failed(&gaps, diag, nquads, &wild, 200),
            (false, 9)
        );
    }

    #[test]
    fn jitter_is_deterministic_dupe_coherent_and_bounded() {
        let mk = || vec![v(1.0, 2.0, 3.0), v(1.0, 2.0, 3.0), v(-4.0, 0.5, 9.25)];
        let mut a = mk();
        let mut b = mk();
        AutoRemesher::jitter_working_vertices(&mut a, 100.0, 1);
        AutoRemesher::jitter_working_vertices(&mut b, 100.0, 1);
        assert_eq!(a, b);
        // Duplicates move together (weld preserved).
        assert_eq!(a[0], a[1]);
        // Bounded by eps/2 = COVERAGE_JITTER_AMPLITUDE * 100 / 2.
        let bound = AutoRemesher::COVERAGE_JITTER_AMPLITUDE * 100.0 / 2.0;
        for (orig, j) in mk().iter().zip(a.iter()) {
            assert!((j.x() - orig.x()).abs() <= bound + 1e-18);
            assert!((j.y() - orig.y()).abs() <= bound + 1e-18);
            assert!((j.z() - orig.z()).abs() <= bound + 1e-18);
        }
        // Seeds differ somewhere.
        let mut c = mk();
        AutoRemesher::jitter_working_vertices(&mut c, 100.0, 2);
        assert_ne!(a, c);
        // Degenerate diag: no-op, never NaN.
        let mut d = mk();
        AutoRemesher::jitter_working_vertices(&mut d, 0.0, 1);
        assert_eq!(d, mk());
    }

    #[test]
    fn coverage_gaps_reports_per_vertex_distances() {
        // Same slider as missing_half: the covered columns pin ~0, the
        // x=2 column sits exactly 1.0 off the covered surface.
        let working = vec![
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(2.0, 0.0, 0.0),
            v(0.0, 1.0, 0.0),
            v(1.0, 1.0, 0.0),
            v(2.0, 1.0, 0.0),
        ];
        let quads = vec![vec![0, 1, 4, 3]];
        let gaps = AutoRemesher::coverage_gaps(&working, &working, &quads);
        assert_eq!(gaps.len(), 6);
        for i in [0, 1, 3, 4] {
            assert!(gaps[i] < 1e-12, "gaps[{i}]={}", gaps[i]);
        }
        assert!((gaps[2] - 1.0).abs() < 1e-12, "gaps[2]={}", gaps[2]);
        assert!((gaps[5] - 1.0).abs() < 1e-12, "gaps[5]={}", gaps[5]);
    }

    /// Small-feature fixture: 3x3x3 body with a thin 0.5x0.5 claw
    /// rising `protrusion` above its top face, stitched into ONE
    /// edge-connected island (separate intersecting boxes split into
    /// two islands at the separator, which tests island failure, not
    /// region coverage).
    fn body_with_claw(protrusion: f64) -> (Vec<Vector3>, Vec<Vec<usize>>) {
        // Rings (CCW from +z): 0(-,-) 1(+,-) 2(+,+) 3(-,+).
        let mut verts = Vec::new();
        for (z, h) in [(-1.5, 1.5), (1.5, 1.5), (1.5, 0.25)] {
            for (sx, sy) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
                verts.push(v(sx * h, sy * h, z));
            }
        }
        for (sx, sy) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
            verts.push(v(sx * 0.25, sy * 0.25, 1.5 + protrusion));
        }
        let (b, t, f, c) = (0, 4, 8, 12);
        let mut tris: Vec<Vec<usize>> = vec![vec![b, b + 2, b + 1], vec![b, b + 3, b + 2]];
        for e in [(0, 1), (1, 2), (2, 3), (3, 0)] {
            // Body sides (low b, high t) and claw walls (low f, high c).
            for (l, h) in [(b, t), (f, c)] {
                tris.push(vec![l + e.0, h + e.1, h + e.0]);
                tris.push(vec![l + e.0, l + e.1, h + e.1]);
            }
            // Top frame (outer t, inner f).
            tris.push(vec![t + e.0, t + e.1, f + e.1]);
            tris.push(vec![t + e.0, f + e.1, f + e.0]);
        }
        tris.push(vec![c, c + 1, c + 2]);
        tris.push(vec![c, c + 2, c + 3]);
        (verts, tris)
    }

    /// Small-feature gate: losing the thin claw must trip the
    /// per-region (connected-patch) bar at protrusion 3.0 even though
    /// the 25-count floor stays quiet; the real (unclipped) outputs
    /// stay quiet; the short 2.0 nub stays quiet when clipped
    /// (fidelity-class: worst gap under the bar, healthy-comparable —
    /// coverage catches missing parts, not sub-bar deviations).
    #[test]
    fn dropped_claw_trips_patch_bar() {
        for (protrusion, expect_fire) in [(3.0, true), (2.0, false)] {
            let (vertices, triangles) = body_with_claw(protrusion);
            let mut r = AutoRemesher::new(&vertices, &triangles);
            r.set_target_triangle_count(300);
            r.set_quiet(true);
            assert!(r.remesh());
            assert!(
                r.coverage_reports().is_empty(),
                "protrusion {protrusion}: real output stays quiet"
            );
            let wv = r.isotropic_vertices();
            let wt = r.isotropic_triangles();
            let diag = AutoRemesher::bbox_diag(wv);
            // Simulate the drop: clip quads touching the claw region.
            let qv = r.remeshed_vertices();
            let clipped: Vec<Vec<usize>> = r
                .remeshed_quads()
                .iter()
                .filter(|q| !q.iter().any(|&i| qv[i].z() > 1.4))
                .cloned()
                .collect();
            assert!(
                !clipped.is_empty() && clipped.len() < r.remeshed_quads().len(),
                "protrusion {protrusion}: clip must remove some but not all quads"
            );
            let gaps = AutoRemesher::coverage_gaps(wv, qv, &clipped);
            let (failed, uncovered) =
                AutoRemesher::coverage_failed(&gaps, diag, clipped.len(), wt, wv.len());
            if expect_fire {
                assert!(
                    failed,
                    "protrusion {protrusion}: dropped claw must trip (uncovered={uncovered})"
                );
                assert!(
                    uncovered < AutoRemesher::COVERAGE_MIN_REGION_VERTS,
                    "protrusion {protrusion}: the patch bar (not the floor) must fire"
                );
                let unit = diag / (clipped.len() as f64).sqrt();
                let patch = AutoRemesher::largest_uncovered_patch(
                    &gaps,
                    wt,
                    wv.len(),
                    AutoRemesher::COVERAGE_WIDTH_MULTIPLE * unit,
                );
                assert!(
                    patch >= AutoRemesher::COVERAGE_MIN_PATCH_VERTS,
                    "protrusion {protrusion}: tip patch must clear the patch bar (patch={patch})"
                );
            } else {
                assert!(
                    !failed,
                    "protrusion {protrusion}: short nub stays quiet (uncovered={uncovered})"
                );
            }
        }
    }

    /// Dense-claw variant (6 wall rings x 8 around + tip cap, 56
    /// claw verts): stranding the claw clears the 10-patch bar by a
    /// wide margin, where the 16-vert `body_with_claw` claw cannot.
    fn body_with_dense_claw() -> (Vec<Vector3>, Vec<Vec<usize>>) {
        let mut verts = Vec::new();
        for (z, h) in [(-1.5, 1.5), (1.5, 1.5)] {
            for (sx, sy) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
                verts.push(v(sx * h, sy * h, z));
            }
        }
        let sides = 8usize;
        let rings = 6usize;
        let base = verts.len();
        for r in 0..=rings {
            let z = 1.5 + 6.0 * r as f64 / rings as f64;
            for s in 0..sides {
                let a = 2.0 * std::f64::consts::PI * s as f64 / sides as f64;
                verts.push(v(0.25 * a.cos(), 0.25 * a.sin(), z));
            }
        }
        let tip = verts.len();
        verts.push(v(0.0, 0.0, 7.5));
        let mut tris: Vec<Vec<usize>> = vec![vec![0, 2, 1], vec![0, 3, 2]];
        for e in [(0, 1), (1, 2), (2, 3), (3, 0)] {
            tris.push(vec![e.0, 4 + e.1, 4 + e.0]);
            tris.push(vec![e.0, e.1, 4 + e.1]);
            tris.push(vec![4 + e.0, 4 + e.1, base + e.1]);
            tris.push(vec![4 + e.0, base + e.1, base + e.0]);
        }
        for r in 0..rings {
            for s in 0..sides {
                let a0 = base + r * sides + s;
                let a1 = base + r * sides + (s + 1) % sides;
                let b0 = base + (r + 1) * sides + s;
                let b1 = base + (r + 1) * sides + (s + 1) % sides;
                tris.push(vec![a0, b1, b0]);
                tris.push(vec![a0, a1, b1]);
            }
        }
        for s in 0..sides {
            let a0 = base + rings * sides + s;
            let a1 = base + rings * sides + (s + 1) % sides;
            tris.push(vec![a0, a1, tip]);
        }
        (verts, tris)
    }

    /// Index/brute-force agreement: `input_coverage_failed`
    /// (`CoverageIndex`: grid or scan + early exit) must match a
    /// direct `coverage_gaps` + patch verdict on the same inputs —
    /// the index is an accelerator, not a second verdict. Checked on
    /// the real output (quiet) and the clipped output (fires,
    /// connected claw patch).
    #[test]
    fn input_side_matches_brute_force() {
        let (vertices, triangles) = body_with_dense_claw();
        let mut r = AutoRemesher::new(&vertices, &triangles);
        r.set_target_triangle_count(1200);
        r.set_quiet(true);
        assert!(r.remesh());
        let qv = r.remeshed_vertices().to_vec();
        let quads = r.remeshed_quads().to_vec();
        let clipped: Vec<Vec<usize>> = quads
            .iter()
            .filter(|q| !q.iter().any(|&i| qv[i].z() > 1.4))
            .cloned()
            .collect();
        assert!(
            !clipped.is_empty() && clipped.len() < quads.len(),
            "clip must remove some but not all quads"
        );
        for (tag, qs) in [("real", &quads), ("clipped", &clipped)] {
            assert_input_side_matches_brute_force(tag, &vertices, &triangles, &qv, qs);
        }
        // The clipped claw is a genuine connected input-side drop.
        let diag = AutoRemesher::bbox_diag(&vertices);
        let (failed, _, patch) =
            AutoRemesher::input_coverage_failed(&vertices, &triangles, &qv, &clipped, diag);
        assert!(
            failed && patch >= AutoRemesher::COVERAGE_MIN_PATCH_VERTS,
            "clipped claw must trip input-side (patch={patch})"
        );
        let (quiet, _, _) =
            AutoRemesher::input_coverage_failed(&vertices, &triangles, &qv, &quads, diag);
        assert!(!quiet, "real output stays input-quiet");
    }

    /// Asserts `input_coverage_failed` agrees with brute force
    /// (`coverage_gaps` + patch rule) exactly; returns the verdict.
    fn assert_input_side_matches_brute_force(
        tag: &str,
        vertices: &[Vector3],
        triangles: &[Vec<usize>],
        qv: &[Vector3],
        qs: &[Vec<usize>],
    ) -> (bool, usize, usize) {
        let diag = AutoRemesher::bbox_diag(vertices);
        let got = AutoRemesher::input_coverage_failed(vertices, triangles, qv, qs, diag);
        // Brute force: exact gaps, same bar, same patch rule.
        let unit = diag / (qs.len() as f64).sqrt();
        let bar = AutoRemesher::COVERAGE_WIDTH_MULTIPLE * unit;
        let gaps = AutoRemesher::coverage_gaps(vertices, qv, qs);
        let exp_uncovered = gaps.iter().filter(|g| **g > bar).count();
        let exp_patch =
            AutoRemesher::largest_uncovered_patch(&gaps, triangles, vertices.len(), bar);
        assert_eq!(
            got.1, exp_uncovered,
            "{tag}: uncovered count skew (index vs brute force)"
        );
        assert_eq!(got.2, exp_patch, "{tag}: patch size skew");
        assert_eq!(
            got.0,
            exp_patch >= AutoRemesher::COVERAGE_MIN_PATCH_VERTS,
            "{tag}: verdict skew"
        );
        got
    }

    /// n x n input grid over [-1, 1]^2: verts, input tris, and the grid
    /// cells as output quads (2 fan tris each).
    fn plane_grid(n: usize) -> (Vec<Vector3>, Vec<Vec<usize>>, Vec<Vec<usize>>) {
        let mut verts = Vec::new();
        for j in 0..n {
            for i in 0..n {
                verts.push(v(
                    -1.0 + 2.0 * i as f64 / (n - 1) as f64,
                    -1.0 + 2.0 * j as f64 / (n - 1) as f64,
                    0.0,
                ));
            }
        }
        let mut tris = Vec::new();
        let mut quads = Vec::new();
        for j in 0..n - 1 {
            for i in 0..n - 1 {
                let a = j * n + i;
                tris.push(vec![a, a + 1, a + n]);
                tris.push(vec![a + 1, a + n + 1, a + n]);
                quads.push(vec![a, a + 1, a + n + 1, a + n]);
            }
        }
        (verts, tris, quads)
    }

    /// Degenerate-output routing: 300 all-repeated-corner quads (every
    /// fan first-edge zero, so h collapses) must route to `Scan` and
    /// agree with brute force. Pre-`CoverageIndex` this input hung the
    /// suite: h pinned near 0 needs bar/h ~= 1e12+ ring iterations per
    /// uncovered query (the `differential_replay` hang); the test
    /// returning at all is the regression proof.
    #[test]
    fn degenerate_output_scans_and_matches_brute_force() {
        let (vertices, triangles, _) = plane_grid(6);
        // [v0,v0,v0,v1] fans into (v0,v0,v0) + (v0,v0,v1): both fan
        // tris zero first-edge, so h == 0 at 600 fan tris (count alone
        // would grid — this pins the h-guard, not the count floor).
        let quads = vec![vec![0, 0, 0, 1]; 300];
        let diag = AutoRemesher::bbox_diag(&vertices);
        let unit = diag / (quads.len() as f64).sqrt();
        let bar = AutoRemesher::COVERAGE_WIDTH_MULTIPLE * unit;
        assert!(
            matches!(
                CoverageIndex::build(&vertices, &quads, bar),
                Some(CoverageIndex::Scan { .. })
            ),
            "collapsed h must route to Scan even at 600 fan tris"
        );
        assert_input_side_matches_brute_force(
            "degenerate",
            &vertices,
            &triangles,
            &vertices,
            &quads,
        );
    }

    /// Spike routing: one giant fan tri (a folded-uv spike) on an
    /// otherwise healthy output must not expand its whole bounding box
    /// into the grid before the cell cap is checked. Its first edge is
    /// short, so h stays at the grid's scale and the box spans ~1e4
    /// cells per axis (~1e12 cells): the cap has to bail before
    /// inserting it, and the scan fallback still answers exactly.
    #[test]
    fn spike_tri_bails_to_scan_before_expanding() {
        let n = 20usize;
        let (mut vertices, _, mut quads) = plane_grid(n);
        let healthy = vertices.len();
        vertices.push(v(-0.99, -0.99, 0.0));
        vertices.push(v(1000.0, 1000.0, 1000.0));
        quads.push(vec![0, 1, healthy, healthy + 1]);
        let diag = 2.0 * 2.0f64.sqrt();
        let unit = diag / (quads.len() as f64).sqrt();
        let bar = AutoRemesher::COVERAGE_WIDTH_MULTIPLE * unit;
        let index =
            CoverageIndex::build(&vertices, &quads, bar).expect("valid fan tris build an index");
        assert!(
            matches!(index, CoverageIndex::Scan { .. }),
            "a spike past the cell cap must route to Scan"
        );
        let mut seen = vec![0u32; index.tri_count()];
        assert!(index.covered_within(&v(0.0, 0.0, 0.0), bar * bar, &mut seen, 1));
        assert!(!index.covered_within(&v(0.0, 0.0, 50.0), bar * bar, &mut seen, 2));
    }

    /// `coverage_gaps` grid walk vs the full scan: bitwise-equal gaps
    /// on a healthy output, a dropped half (far queries fall back to
    /// the scan), and a spike output (grid build bails), with queries
    /// on, above, beside, and far from the surface.
    #[test]
    fn coverage_gaps_grid_matches_scan_bitwise() {
        let n = 24usize;
        let (mut vertices, _, quads) = plane_grid(n);
        // Wavy output so distances are not all axis-aligned zeros.
        for (i, p) in vertices.iter_mut().enumerate() {
            let z = 0.05 * ((i as f64) * 0.7).sin();
            *p = v(p.x(), p.y(), z);
        }
        let half: Vec<Vec<usize>> = quads
            .iter()
            .enumerate()
            .filter(|(c, _)| c % (n - 1) < n / 2)
            .map(|(_, q)| q.clone())
            .collect();
        let mut spiked = quads.clone();
        let mut spike_verts = vertices.clone();
        spike_verts.push(v(1000.0, -1000.0, 1000.0));
        spiked.push(vec![0, 1, n + 1, spike_verts.len() - 1]);
        let mut queries = Vec::new();
        for j in 0..40 {
            for i in 0..40 {
                let x = -1.3 + 2.6 * i as f64 / 39.0;
                let y = -1.3 + 2.6 * j as f64 / 39.0;
                let z = 0.4 * ((i * 7 + j * 3) as f64 * 0.37).sin();
                queries.push(v(x, y, z));
            }
        }
        queries.push(v(0.0, 0.0, 25.0));
        queries.push(v(f64::NAN, 0.0, 0.0));
        for (tag, verts, qs) in [
            ("healthy", &vertices, &quads),
            ("half", &vertices, &half),
            ("spike", &spike_verts, &spiked),
        ] {
            let got = AutoRemesher::coverage_gaps(&queries, verts, qs);
            let (fan, _) = CoverageIndex::fan(verts, qs);
            let tris: Vec<(usize, usize, usize, [f64; 6])> = fan
                .iter()
                .map(|&(a, b, c)| {
                    let (pa, pb, pc) = (&verts[a], &verts[b], &verts[c]);
                    (
                        a,
                        b,
                        c,
                        [
                            pa.x().min(pb.x()).min(pc.x()),
                            pa.y().min(pb.y()).min(pc.y()),
                            pa.z().min(pb.z()).min(pc.z()),
                            pa.x().max(pb.x()).max(pc.x()),
                            pa.y().max(pb.y()).max(pc.y()),
                            pa.z().max(pb.z()).max(pc.z()),
                        ],
                    )
                })
                .collect();
            for (k, q) in queries.iter().enumerate() {
                let want = AutoRemesher::scan_nearest_dist2(q, verts, &tris).sqrt();
                assert_eq!(
                    got[k].to_bits(),
                    want.to_bits(),
                    "{tag}: query {k} grid gap {} vs scan {}",
                    got[k],
                    want
                );
            }
        }
    }

    /// Grid-arm agreement: a healthy output above `COVERAGE_SCAN_TRIS`
    /// routes to `Grid` and matches brute force on quiet (full grid)
    /// and firing (dropped half) outputs — the shell walk + ring exit
    /// are an accelerator, not a second verdict.
    #[test]
    fn grid_path_matches_brute_force() {
        let n = 20usize;
        let (vertices, triangles, quads) = plane_grid(n);
        // Drop the right half of the cells (kept: 171 quads = 342
        // fan tris, still Grid): input columns past ~7 cells from the
        // kept edge sit beyond the bar as one connected patch.
        let half: Vec<Vec<usize>> = quads
            .iter()
            .enumerate()
            .filter(|(c, _)| c % (n - 1) < n / 2 - 1)
            .map(|(_, q)| q.clone())
            .collect();
        assert!(half.len() * 2 >= COVERAGE_SCAN_TRIS);
        for (tag, qs) in [("full-grid", &quads), ("half-grid", &half)] {
            let diag = AutoRemesher::bbox_diag(&vertices);
            let unit = diag / (qs.len() as f64).sqrt();
            let bar = AutoRemesher::COVERAGE_WIDTH_MULTIPLE * unit;
            assert!(
                matches!(
                    CoverageIndex::build(&vertices, qs, bar),
                    Some(CoverageIndex::Grid(_))
                ),
                "{tag}: healthy 256+ tri output must route to Grid"
            );
            let (failed, _, _) =
                assert_input_side_matches_brute_force(tag, &vertices, &triangles, &vertices, qs);
            assert_eq!(
                failed,
                tag == "half-grid",
                "{tag}: full grid quiet, dropped half fires"
            );
        }
    }
}
