use super::AutoRemesher;
use crate::vector3::Vector3;
use std::collections::HashMap;

/// Output-side proximity index for the input-side verdict: a
/// uniform grid over the fanned output quads, answering covered-or-not
/// within a squared bar with early exit (input-side coverage runs over
/// up to 150k input verts, where the `coverage_gaps` full scan is
/// seconds per attempt). Small or degenerate outputs take the `Scan`
/// arm instead: when the mean fan edge collapses (h -> 0 on
/// zero-area/repeated-corner tris), ring expansion would walk ~bar/h
/// ~= 1e12 empty rings per query — that hung `differential_replay`
/// before the fallback existed. Both arms test the same predicate
/// (`point_triangle_dist2 <= bar2` over the same fan), so Grid === Scan
/// === brute force; the grid is an accelerator, not a second verdict.
pub(crate) enum CoverageIndex<'v> {
    Grid(CoverageGrid<'v>),
    Scan {
        verts: &'v [Vector3],
        tris: Vec<(usize, usize, usize)>,
    },
}

/// Fan-tri counts below this scan faster than a grid can index (no
/// build cost, no degenerate-hang risk).
pub(crate) const COVERAGE_SCAN_TRIS: usize = 256;

/// Ring bound the grid arm guarantees: build routes to `Scan` unless
/// `h > bar / COVERAGE_MAX_RINGS`, so ring exit fires at `r < 64` and
/// every query terminates after a bounded shell walk.
const COVERAGE_MAX_RINGS: f64 = 64.0;

/// Absolute cell cap for grid build; beyond it the output is
/// degenerate at grid scale (giant tris over a collapsed index) and
/// the caller scans instead, staying exact.
const COVERAGE_MAX_CELLS: usize = 1_000_000;

impl<'v> CoverageIndex<'v> {
    /// Fan-triangulates `quads` (out-of-range corners skipped, never
    /// trusted — same rule as `coverage_gaps`) and indexes the fan.
    /// `None` = no valid fan triangle: every vert uncovered (the
    /// existing INFINITY verdict for an empty extraction).
    pub(crate) fn build(verts: &'v [Vector3], quads: &[Vec<usize>], bar: f64) -> Option<Self> {
        let (tris, total) = Self::fan(verts, quads);
        if tris.is_empty() {
            return None;
        }
        if tris.len() >= COVERAGE_SCAN_TRIS {
            let h = 2.0 * total / tris.len() as f64;
            // Degenerate guard (also catches h <= 0 and NaN): without
            // it an all-repeated-corner output pins h at ~0 and ring
            // exit needs bar/h ~= 1e12+ iterations per query.
            if h > bar / COVERAGE_MAX_RINGS {
                if let Some(grid) = CoverageGrid::build(verts, tris, h) {
                    return Some(Self::Grid(grid));
                }
                // Cell-cap bail: re-fan for the scan (O(quads), on a
                // path that then does O(verts x tris) scan work).
                return Some(Self::Scan {
                    verts,
                    tris: Self::fan(verts, quads).0,
                });
            }
        }
        Some(Self::Scan { verts, tris })
    }

    /// Fan triangulation + summed first-edge lengths (the grid cell
    /// size derives from the mean).
    pub(crate) fn fan(
        verts: &[Vector3],
        quads: &[Vec<usize>],
    ) -> (Vec<(usize, usize, usize)>, f64) {
        let mut tris = Vec::new();
        let mut total = 0.0;
        for q in quads {
            for k in 1..q.len().saturating_sub(1) {
                let (a, b, c) = (q[0], q[k], q[k + 1]);
                if a < verts.len() && b < verts.len() && c < verts.len() {
                    total += (verts[b] - verts[a]).length();
                    tris.push((a, b, c));
                }
            }
        }
        (tris, total)
    }

    /// Whether any output triangle passes within `bar2` (squared) of
    /// `p`. Deterministic: fan order (scan) or fixed ring order (grid),
    /// order-free boolean reduction.
    pub(crate) fn covered_within(
        &self,
        p: &Vector3,
        bar2: f64,
        seen: &mut [u32],
        stamp: u32,
    ) -> bool {
        match self {
            Self::Grid(grid) => grid.covered_within(p, bar2, seen, stamp),
            Self::Scan { verts, tris } => tris.iter().any(|&(a, b, c)| {
                AutoRemesher::point_triangle_dist2(p, &verts[a], &verts[b], &verts[c]) <= bar2
            }),
        }
    }

    pub(crate) fn tri_count(&self) -> usize {
        match self {
            Self::Grid(grid) => grid.tris.len(),
            Self::Scan { tris, .. } => tris.len(),
        }
    }
}

pub(crate) struct CoverageGrid<'v> {
    verts: &'v [Vector3],
    tris: Vec<(usize, usize, usize)>,
    cells: HashMap<[i64; 3], Vec<usize>>,
    h: f64,
}

impl<'v> CoverageGrid<'v> {
    /// Indexes pre-fanned `tris` with cell size `h` (`None` when the
    /// index would exceed `COVERAGE_MAX_CELLS`: the caller scans
    /// instead). Per-tri spans are tri-local, so with the caller's
    /// `h > bar/64` guarantee the ranges below are bounded and this
    /// always terminates.
    pub(crate) fn build(
        verts: &'v [Vector3],
        tris: Vec<(usize, usize, usize)>,
        h: f64,
    ) -> Option<Self> {
        let mut cells: HashMap<[i64; 3], Vec<usize>> = HashMap::new();
        for (t, &(a, b, c)) in tris.iter().enumerate() {
            if cells.len() > COVERAGE_MAX_CELLS {
                return None;
            }
            let (pa, pb, pc) = (&verts[a], &verts[b], &verts[c]);
            let lo = [
                (pa.x().min(pb.x()).min(pc.x()) / h).floor() as i64,
                (pa.y().min(pb.y()).min(pc.y()) / h).floor() as i64,
                (pa.z().min(pb.z()).min(pc.z()) / h).floor() as i64,
            ];
            let hi = [
                (pa.x().max(pb.x()).max(pc.x()) / h).floor() as i64,
                (pa.y().max(pb.y()).max(pc.y()) / h).floor() as i64,
                (pa.z().max(pb.z()).max(pc.z()) / h).floor() as i64,
            ];
            for i in lo[0]..=hi[0] {
                for j in lo[1]..=hi[1] {
                    for k in lo[2]..=hi[2] {
                        cells.entry([i, j, k]).or_default().push(t);
                    }
                }
            }
        }
        Some(Self {
            verts,
            tris,
            cells,
            h,
        })
    }

    /// Tests one cell's tris against `bar2` (stamp-deduped, no
    /// per-query allocation); true on the first tri within range.
    pub(crate) fn test_cell(
        &self,
        p: &Vector3,
        key: [i64; 3],
        bar2: f64,
        seen: &mut [u32],
        stamp: u32,
    ) -> bool {
        let Some(list) = self.cells.get(&key) else {
            return false;
        };
        for &t in list.iter() {
            if seen[t] == stamp {
                continue;
            }
            seen[t] = stamp;
            let (a, b, c) = self.tris[t];
            if AutoRemesher::point_triangle_dist2(p, &self.verts[a], &self.verts[b], &self.verts[c])
                <= bar2
            {
                return true;
            }
        }
        false
    }

    /// Whether any indexed triangle passes within `bar2` (squared) of
    /// `p`. Ring expansion mirrors `bench/score.py`'s `TriangleGrid`:
    /// after ring `r` with no hit, remaining cells sit beyond `r*h`
    /// from the query, so `(r*h)^2 >= bar2` proves uncovered. Pure
    /// function of its inputs (fixed shell order, order-free boolean
    /// reduction; the map is only looked up, never iterated).
    /// `seen`/`stamp` dedupe triangle tests across the rings of one
    /// query (stamp values only compared for equality).
    pub(crate) fn covered_within(
        &self,
        p: &Vector3,
        bar2: f64,
        seen: &mut [u32],
        stamp: u32,
    ) -> bool {
        let c = [
            (p.x() / self.h).floor() as i64,
            (p.y() / self.h).floor() as i64,
            (p.z() / self.h).floor() as i64,
        ];
        // Wrapping: a query past ~9e18 cells saturates its base key;
        // neighbors then alias arbitrary cells, but every tested tri is
        // still measured exactly, so a `true` stays sound and the ring
        // exit below still terminates the walk.
        let cell = |dx: i64, dy: i64, dz: i64| {
            [
                c[0].wrapping_add(dx),
                c[1].wrapping_add(dy),
                c[2].wrapping_add(dz),
            ]
        };
        let mut r: i64 = 0;
        loop {
            // Chebyshev shell == r, each cell once: full z-faces, then
            // the x/y face strips over the open z-interval (r == 0
            // visits the center cell once; at r > 0 the y strips run
            // the open x-interval so shared edges meet once).
            if r == 0 {
                if self.test_cell(p, c, bar2, seen, stamp) {
                    return true;
                }
            } else {
                for &dz in &[-r, r] {
                    for dx in -r..=r {
                        for dy in -r..=r {
                            if self.test_cell(p, cell(dx, dy, dz), bar2, seen, stamp) {
                                return true;
                            }
                        }
                    }
                }
                for dz in -(r - 1)..=(r - 1) {
                    for d in -r..=r {
                        if self.test_cell(p, cell(r, d, dz), bar2, seen, stamp)
                            || self.test_cell(p, cell(-r, d, dz), bar2, seen, stamp)
                        {
                            return true;
                        }
                    }
                    for d in -(r - 1)..=(r - 1) {
                        if self.test_cell(p, cell(d, r, dz), bar2, seen, stamp)
                            || self.test_cell(p, cell(d, -r, dz), bar2, seen, stamp)
                        {
                            return true;
                        }
                    }
                }
            }
            // Remaining rings sit beyond r*h from the query: a bar
            // inside that proves uncovered (score.py's exit rule).
            // Terminates: build guarantees h > bar/64, so this fires
            // at r < 64 after a bounded shell walk.
            if (r as f64 * self.h).powi(2) >= bar2 {
                return false;
            }
            r += 1;
        }
    }
}

/// Coverage retry outcome for one island, reported like a failed
/// island. Present only when the first attempt failed coverage:
/// `retries_made` counts the jitter retries run (1-3, stopping at the
/// first full-coverage result); `recovered` tells whether one covered
/// fully (else the fallback attempt was kept);
/// `initial_uncovered`/`final_uncovered`
/// count verts beyond the coverage width before/after on the firing
/// side: working-mesh verts normally, ORIGINAL input verts when
/// `input_side` is set (the input side fires only, see
/// `input_coverage_failed`). `kept_attempt` is the committed attempt:
/// the full-coverage winner when `recovered`, else the earliest
/// working-quiet attempt (attempt 0 when it passed working-side) so
/// the input side can never make working-side coverage worse than
/// unretried.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CoverageReport {
    pub island_index: usize,
    pub retries_made: usize,
    pub recovered: bool,
    pub initial_uncovered: usize,
    pub final_uncovered: usize,
    pub input_side: bool,
    pub kept_attempt: usize,
}

impl AutoRemesher {
    /// Coverage failure threshold (resolution-relative): an island
    /// fails coverage when at least [`Self::COVERAGE_MIN_REGION_VERTS`]
    /// working verts sit beyond [`Self::COVERAGE_WIDTH_MULTIPLE`] nominal
    /// quad widths (`diag/sqrt(nquads)`) from the extracted quads.
    /// Calibrated over the bench (10 cases: 0 verts beyond 3 widths),
    /// 24 noisy tiny seeds (1 fires), 359 suite island-runs (1 mid-size
    /// fixture fires) and beast@1000-native (7 dropped seeds: 67-156
    /// verts beyond 3 widths; covered seed: 0); see
    /// `docs/beast-knife-edge-bisection.md` and `docs/coverage-retry.md`.
    pub(crate) const COVERAGE_WIDTH_MULTIPLE: f64 = 3.0;

    /// Minimum uncovered working verts that count as a failed region
    /// (isolated spikes never fire the retry on their own).
    pub(crate) const COVERAGE_MIN_REGION_VERTS: usize = 25;

    /// Minimum CONNECTED uncovered verts (working-triangle-adjacent)
    /// that count as a failed small region: the per-region catcher
    /// for thin drops the count floor misses. Calibrated at 10
    /// (healthiest non-firing patch anywhere: 3; smallest genuine
    /// small drop: 10; bench unjittered: 0).
    pub(crate) const COVERAGE_MIN_PATCH_VERTS: usize = 10;

    /// Coverage retry seeds (deterministic jitter variants), tried in
    /// order; the first full-coverage result wins.
    pub(crate) const COVERAGE_RETRY_SEEDS: [u64; 3] = [1, 2, 3];

    /// Retry jitter amplitude, relative to the island working-mesh
    /// diagonal. Noise-floor jitter (1e-9, the `bench/noise.py` scale)
    /// cannot move the uv rounding that folds a dropped region (1e-6
    /// still fails); 1e-4 recovers 7/8 native-beast seeds while the
    /// partial fold resists; 1e-3 recovers 8/8 on the first retry with
    /// exact-zero residuals, and the recovered outputs sit inside the
    /// healthy tiling spread (dist_mean 0.82-1.09 vs 0.68-1.01
    /// unretried). See `docs/coverage-retry.md`.
    pub(crate) const COVERAGE_JITTER_AMPLITUDE: f64 = 1e-3;

    /// Squared distance from point `p` to triangle `(a, b, c)` (Ericson
    /// 5.1.5, f64). Total: degenerate triangles fall through to the
    /// vertex/edge regions.
    pub(crate) fn point_triangle_dist2(p: &Vector3, a: &Vector3, b: &Vector3, c: &Vector3) -> f64 {
        let abx = b.x() - a.x();
        let aby = b.y() - a.y();
        let abz = b.z() - a.z();
        let acx = c.x() - a.x();
        let acy = c.y() - a.y();
        let acz = c.z() - a.z();
        let apx = p.x() - a.x();
        let apy = p.y() - a.y();
        let apz = p.z() - a.z();
        let d1 = abx * apx + aby * apy + abz * apz;
        let d2 = acx * apx + acy * apy + acz * apz;
        if d1 <= 0.0 && d2 <= 0.0 {
            return apx * apx + apy * apy + apz * apz;
        }
        let bpx = p.x() - b.x();
        let bpy = p.y() - b.y();
        let bpz = p.z() - b.z();
        let d3 = abx * bpx + aby * bpy + abz * bpz;
        let d4 = acx * bpx + acy * bpy + acz * bpz;
        if d3 >= 0.0 && d4 <= d3 {
            return bpx * bpx + bpy * bpy + bpz * bpz;
        }
        let vc = d1 * d4 - d3 * d2;
        if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
            let v = d1 / (d1 - d3);
            let qx = a.x() + v * abx - p.x();
            let qy = a.y() + v * aby - p.y();
            let qz = a.z() + v * abz - p.z();
            return qx * qx + qy * qy + qz * qz;
        }
        let cpx = p.x() - c.x();
        let cpy = p.y() - c.y();
        let cpz = p.z() - c.z();
        let d5 = abx * cpx + aby * cpy + abz * cpz;
        let d6 = acx * cpx + acy * cpy + acz * cpz;
        if d6 >= 0.0 && d5 <= d6 {
            return cpx * cpx + cpy * cpy + cpz * cpz;
        }
        let vb = d5 * d2 - d1 * d6;
        if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
            let w = d2 / (d2 - d6);
            let qx = a.x() + w * acx - p.x();
            let qy = a.y() + w * acy - p.y();
            let qz = a.z() + w * acz - p.z();
            return qx * qx + qy * qy + qz * qz;
        }
        let va = d3 * d6 - d5 * d4;
        if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
            let cbx = c.x() - b.x();
            let cby = c.y() - b.y();
            let cbz = c.z() - b.z();
            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            let qx = b.x() + w * cbx - p.x();
            let qy = b.y() + w * cby - p.y();
            let qz = b.z() + w * cbz - p.z();
            return qx * qx + qy * qy + qz * qz;
        }
        let denom = 1.0 / (va + vb + vc);
        let v = vb * denom;
        let w = vc * denom;
        let qx = a.x() + abx * v + acx * w - p.x();
        let qy = a.y() + aby * v + acy * w - p.y();
        let qz = a.z() + abz * v + acz * w - p.z();
        qx * qx + qy * qy + qz * qz
    }

    /// Per-working-vertex distance to the extracted surface (quads
    /// triangulate as fans, matching `bench/score.py`): empty quads give
    /// infinity (total failure). Each quad triangle carries an AABB
    /// reject so covered verts skip far triangles cheaply. Pure function
    /// of its inputs (fixed iteration order, f64).
    pub(crate) fn coverage_gaps(
        working: &[Vector3],
        quad_vertices: &[Vector3],
        quads: &[Vec<usize>],
    ) -> Vec<f64> {
        if quads.is_empty() || quad_vertices.is_empty() {
            return vec![f64::INFINITY; working.len()];
        }
        // Flatten quad fans once, with AABBs.
        let mut tris: Vec<(usize, usize, usize, [f64; 6])> = Vec::new();
        for q in quads {
            for k in 1..q.len().saturating_sub(1) {
                let (a, b, c) = (q[0], q[k], q[k + 1]);
                if a >= quad_vertices.len() || b >= quad_vertices.len() || c >= quad_vertices.len()
                {
                    continue;
                }
                let pa = &quad_vertices[a];
                let pb = &quad_vertices[b];
                let pc = &quad_vertices[c];
                tris.push((
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
                ));
            }
        }
        if tris.is_empty() {
            return vec![f64::INFINITY; working.len()];
        }
        let mut gaps = Vec::with_capacity(working.len());
        for p in working.iter() {
            let mut best = f64::INFINITY;
            for (a, b, c, bb) in tris.iter() {
                // AABB reject against the running best.
                let dx = if p.x() < bb[0] {
                    bb[0] - p.x()
                } else if p.x() > bb[3] {
                    p.x() - bb[3]
                } else {
                    0.0
                };
                let dy = if p.y() < bb[1] {
                    bb[1] - p.y()
                } else if p.y() > bb[4] {
                    p.y() - bb[4]
                } else {
                    0.0
                };
                let dz = if p.z() < bb[2] {
                    bb[2] - p.z()
                } else if p.z() > bb[5] {
                    p.z() - bb[5]
                } else {
                    0.0
                };
                if dx * dx + dy * dy + dz * dz >= best {
                    continue;
                }
                let d2 = Self::point_triangle_dist2(
                    p,
                    &quad_vertices[*a],
                    &quad_vertices[*b],
                    &quad_vertices[*c],
                );
                if d2 < best {
                    best = d2;
                }
            }
            gaps.push(best.sqrt());
        }
        gaps
    }

    /// Bbox diagonal of `points` (0 when empty or fully degenerate);
    /// the coverage scale plus the retry-jitter scale.
    pub(crate) fn bbox_diag(points: &[Vector3]) -> f64 {
        if points.is_empty() {
            return 0.0;
        }
        let mut lo = Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut hi = Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for w in points.iter() {
            lo.set_x(lo.x().min(w.x()));
            lo.set_y(lo.y().min(w.y()));
            lo.set_z(lo.z().min(w.z()));
            hi.set_x(hi.x().max(w.x()));
            hi.set_y(hi.y().max(w.y()));
            hi.set_z(hi.z().max(w.z()));
        }
        (hi - lo).length()
    }

    /// Coverage verdict over per-vertex `gaps`: `(failed, uncovered)`,
    /// where `uncovered` counts verts beyond
    /// [`Self::COVERAGE_WIDTH_MULTIPLE`] nominal quad widths
    /// (`diag/sqrt(nquads)`). Fails on a severe miss (`uncovered` past
    /// [`Self::COVERAGE_MIN_REGION_VERTS`], anywhere) or a small
    /// connected drop (the largest working-triangle-adjacent patch
    /// beyond the bar holds [`Self::COVERAGE_MIN_PATCH_VERTS`]+ verts:
    /// the per-region catcher for thin features the count floor
    /// misses). Empty quads never fail here (the failed-island path
    /// owns them); degenerate inputs (zero quads, zero/NaN diag) report
    /// no failure.
    pub(crate) fn coverage_failed(
        gaps: &[f64],
        diag: f64,
        nquads: usize,
        triangles: &[Vec<usize>],
        nverts: usize,
    ) -> (bool, usize) {
        if nquads == 0 || !(diag > 0.0) {
            return (false, 0);
        }
        let unit = diag / (nquads as f64).sqrt();
        let bar = Self::COVERAGE_WIDTH_MULTIPLE * unit;
        let uncovered = gaps.iter().filter(|g| **g > bar).count();
        if uncovered >= Self::COVERAGE_MIN_REGION_VERTS {
            return (true, uncovered);
        }
        let patch = Self::largest_uncovered_patch(gaps, triangles, nverts, bar);
        (patch >= Self::COVERAGE_MIN_PATCH_VERTS, uncovered)
    }

    /// Largest connected set of verts with `gaps` beyond `bar`,
    /// adjacent over `triangles` (indices into `nverts` verts; out of
    /// range corners are skipped, never trusted).
    pub(crate) fn largest_uncovered_patch(
        gaps: &[f64],
        triangles: &[Vec<usize>],
        nverts: usize,
        bar: f64,
    ) -> usize {
        let mut adj: Vec<Vec<usize>> = vec![Vec::new(); nverts];
        for t in triangles.iter() {
            if t.len() == 3 && t[0] < nverts && t[1] < nverts && t[2] < nverts {
                for e in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                    adj[e.0].push(e.1);
                    adj[e.1].push(e.0);
                }
            }
        }
        let mut seen = vec![false; nverts];
        let mut best = 0usize;
        for i in 0..gaps.len().min(nverts) {
            if seen[i] || gaps[i] <= bar {
                continue;
            }
            let mut stack = vec![i];
            seen[i] = true;
            let mut size = 0usize;
            while let Some(u) = stack.pop() {
                size += 1;
                for &nb in adj[u].iter() {
                    if nb < gaps.len() && !seen[nb] && gaps[nb] > bar {
                        seen[nb] = true;
                        stack.push(nb);
                    }
                }
            }
            best = best.max(size);
        }
        best
    }

    /// Largest connected set of `beyond` verts over `triangles`
    /// (boolean twin of `largest_uncovered_patch`, which keys off gap
    /// values; kept separate so neither call path changes shape).
    pub(crate) fn connected_patch_size(
        beyond: &[bool],
        triangles: &[Vec<usize>],
        nverts: usize,
    ) -> usize {
        let mut adj: Vec<Vec<usize>> = vec![Vec::new(); nverts];
        for t in triangles.iter() {
            if t.len() == 3 && t[0] < nverts && t[1] < nverts && t[2] < nverts {
                for e in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                    adj[e.0].push(e.1);
                    adj[e.1].push(e.0);
                }
            }
        }
        let mut seen = vec![false; nverts];
        let mut best = 0usize;
        for i in 0..beyond.len().min(nverts) {
            if seen[i] || !beyond[i] {
                continue;
            }
            let mut stack = vec![i];
            seen[i] = true;
            let mut size = 0usize;
            while let Some(u) = stack.pop() {
                size += 1;
                for &nb in adj[u].iter() {
                    if nb < beyond.len() && !seen[nb] && beyond[nb] {
                        seen[nb] = true;
                        stack.push(nb);
                    }
                }
            }
            best = best.max(size);
        }
        best
    }

    /// Input-side coverage verdict: `(failed, uncovered, patch)`
    /// over the island's ORIGINAL input verts vs the extracted quads.
    /// Same 3-width bar as working-side, but connectivity-only (no
    /// anywhere count floor): input noise legitimately strands
    /// isolated verts beyond the bar (dragon: 123 singletons at every
    /// bar), while genuine drops are connected. Calibrated bench-quiet
    /// at seed 0 (worst healthy: dragon scatter; armadillo
    /// fingertip-class sits just inside at 0.93x — a downstream
    /// fidelity miss, not a drop). Quiet on empty quads (the
    /// failed-island path owns them).
    pub(crate) fn input_coverage_failed(
        input_vertices: &[Vector3],
        input_triangles: &[Vec<usize>],
        quad_vertices: &[Vector3],
        quads: &[Vec<usize>],
        diag: f64,
    ) -> (bool, usize, usize) {
        if quads.is_empty() || !(diag > 0.0) {
            return (false, 0, 0);
        }
        let unit = diag / (quads.len() as f64).sqrt();
        let bar = Self::COVERAGE_WIDTH_MULTIPLE * unit;
        let Some(index) = CoverageIndex::build(quad_vertices, quads, bar) else {
            return (true, input_vertices.len(), input_vertices.len());
        };
        let mut seen = vec![0u32; index.tri_count()];
        let mut beyond = vec![false; input_vertices.len()];
        let mut stamp: u32 = 1;
        for (i, p) in input_vertices.iter().enumerate() {
            stamp = stamp.wrapping_add(1).max(1);
            if !index.covered_within(p, bar * bar, &mut seen, stamp) {
                beyond[i] = true;
            }
        }
        let uncovered = beyond.iter().filter(|b| **b).count();
        let patch = Self::connected_patch_size(&beyond, input_triangles, input_vertices.len());
        (patch >= Self::COVERAGE_MIN_PATCH_VERTS, uncovered, patch)
    }

    /// SplitMix64 (Steele et al.): deterministic cross-platform u64 stream
    /// for retry jitter (wrapping arithmetic only).
    pub(crate) fn splitmix64(state: &mut u64) -> u64 {
        *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = *state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Jitters working vertices in place for a coverage retry: per-coord
    /// offsets uniform in `[-eps/2, eps/2)` with `eps =
    /// COVERAGE_JITTER_AMPLITUDE * island_diag`. Keyed on position bits
    /// (bitwise-duplicate vertices move together, preserving the weld)
    /// and the retry seed; pure function of (positions, diag, seed), no
    /// tables, identical on every platform (integer hash, one rounded
    /// multiply per coord).
    pub(crate) fn jitter_working_vertices(vertices: &mut [Vector3], island_diag: f64, seed: u64) {
        let eps = Self::COVERAGE_JITTER_AMPLITUDE * island_diag;
        if !(eps > 0.0) {
            return;
        }
        for v in vertices.iter_mut() {
            let mut state = v.x().to_bits()
                ^ v.y().to_bits().rotate_left(21)
                ^ v.z().to_bits().rotate_left(42)
                ^ seed.rotate_left(13)
                ^ 0x9E37_79B9_7F4A_7C15;
            let mut draw = || {
                let h = Self::splitmix64(&mut state);
                // Top 53 bits -> exact dyadic in [0, 1).
                (h >> 11) as f64 * 2.0f64.powi(-53)
            };
            let ox = (draw() - 0.5) * eps;
            let oy = (draw() - 0.5) * eps;
            let oz = (draw() - 0.5) * eps;
            v.set_data(v.x() + ox, v.y() + oy, v.z() + oz);
        }
    }
}
