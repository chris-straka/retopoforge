//! Quantization + structured fill for the patch back end.
//!
//! Each shared arc is quantized once to an integer edge count, then
//! every patch is filled with Coons grids that meet exactly on the
//! shared sides (Bi-MDF-family shape: one global integer per arc, no
//! per-patch T-junctions):
//!
//! - quad patches: direct Coons grid after opposite-side equalization
//!   (single deterministic priority pass; unresolvable pairs fall back
//!   to a diagonal triangle pair, which is rare and reported);
//! - n-gons (n >= 5): one greedy cut sequence planned from side-count
//!   arithmetic (first-valid-choice per level), then emitted
//!   infallibly — no backtracking (the old try-every-cut search was
//!   exponential in n and hung beast@1000);
//! - odd n-gons terminate in one small triangle (subdivided while its
//!   sides stay even);
//! - digons/monogons: split into triangles at shared grid points;
//! - closed/holed/inconsistent patches: per-face 3-quad subdivision,
//!   which keeps full coverage by construction.
//!
//! Hard caps keep every patch bounded: 128 plannable sides, 2048
//! cut-plan search nodes, and 16384 grid points per Coons piece —
//! past any of them, the patch takes the per-face fallback (counted
//! as `capped_patches`).
//!
//! All grid vertices are projected onto their own patch's triangles
//! through an exact closest-point index (grid + scan fallback), so
//! thin features can never snap across gaps. Shared sides weld by
//! key, giving a manifold mesh except along fallback borders and
//! conflict diagonals (both rare and reported).

use crate::patch_backend::layout::Layout;
use crate::surface_mesh::SurfaceMesh;
use crate::vector3::Vector3;
use std::cmp::Reverse;
use std::collections::{BTreeMap, BinaryHeap, HashMap};

/// Fill statistics for the phase report.
#[derive(Clone, Debug, Default)]
pub(crate) struct FillStats {
    pub patches: usize,
    pub usable_patches: usize,
    pub conflicted_quads: usize,
    pub triangles: usize,
    pub fallback_faces: usize,
    pub degenerate_skipped: usize,
    pub clamped_arcs: usize,
    pub capped_patches: usize,
}

/// Hard cap on n-gon sides entering the cut planner: past it the
/// patch is degenerate tracing and takes the per-face fallback, which
/// keeps full coverage by construction.
const FILL_MAX_PLAN_SIDES: usize = 128;

/// Hard attempt cap on cut-plan search nodes per patch (the planner
/// backtracks over rotations, so depth alone cannot bound it): past
/// it the patch takes the per-face fallback.
const FILL_MAX_PLAN_NODES: usize = 2048;

/// Hard cap on Coons grid points per piece ((a+1) x (b+1)): past it
/// the patch takes the per-face fallback instead of emitting a giant
/// grid (quantization is shared, so counts are never clamped here).
const FILL_MAX_GRID_POINTS: usize = 16384;

/// Whether a Coons piece over opposite counts (a, b) exceeds the grid
/// cap (saturating: huge counts cap, never overflow).
fn grid_over_cap(a: usize, b: usize) -> bool {
    a.saturating_add(1).saturating_mul(b.saturating_add(1)) > FILL_MAX_GRID_POINTS
}

/// Filled island: welded verts/faces plus per-vertex patch UVs.
pub(crate) struct FillOutput {
    pub verts: Vec<Vector3>,
    pub faces: Vec<Vec<usize>>,
    pub uv_patch: Vec<usize>,
    pub uv_local: Vec<(f64, f64)>,
    pub stats: FillStats,
}

/// Weld key: shared sides use arc keys (both patches agree), interior
/// points use patch-local keys.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
enum Key {
    /// Layout node (patch corner).
    Node(usize),
    /// Shared side grid point: (arc, index).
    SidePoint(usize, usize),
    /// Coons grid interior: (patch, piece, i, j).
    Grid(usize, usize, usize, usize),
    /// Cut polyline point: (patch, cut, index).
    CutPoint(usize, usize, usize),
    /// Split point off the shared grid (digon/monogon, k < 3):
    /// (patch, side position, slot).
    SplitPoint(usize, usize, usize),
    /// Triangle-subdivision interior edge point: (patch, edge id, index).
    TriEdge(usize, usize, usize),
    /// Fallback edge midpoint: ordered working-vertex pair.
    FallbackEdge(usize, usize),
    /// Fallback face centroid: working face.
    FallbackCenter(usize),
    /// Fallback corner: original working vertex.
    FallbackVert(usize),
}

/// Closest point on triangle (a, b, c) to p.
fn closest_on_triangle(p: &Vector3, a: &Vector3, b: &Vector3, c: &Vector3) -> Vector3 {
    let ab = *b - *a;
    let ac = *c - *a;
    let ap = *p - *a;
    let d1 = Vector3::dot_product(&ab, &ap);
    let d2 = Vector3::dot_product(&ac, &ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return *a;
    }
    let bp = *p - *b;
    let d3 = Vector3::dot_product(&ab, &bp);
    let d4 = Vector3::dot_product(&ac, &bp);
    if d3 >= 0.0 && d4 <= d3 {
        return *b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return *a + ab * v;
    }
    let cp = *p - *c;
    let d5 = Vector3::dot_product(&ab, &cp);
    let d6 = Vector3::dot_product(&ac, &cp);
    if d6 >= 0.0 && d5 <= d6 {
        return *c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return *a + ac * w;
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return *b + (*c - *b) * w;
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    *a + ab * v + ac * w
}

/// Exact closest-point index over one patch's faces (grid vertex
/// projection runs per Coons interior point, where the old per-query
/// full scan was O(faces) each). Ring expansion tracks the best
/// candidate and exits when `(r*h)^2 >= best_d2`, which proves exact
/// (remaining cells sit beyond `r*h`); ties keep the first tested
/// (fixed ring order, insertion-ordered cell lists), so queries are
/// deterministic. Small or degenerate face sets take the `Scan` arm:
/// when the mean edge collapses (h -> 0 on zero-area tris), ring
/// expansion would walk ~dist/h empty rings before its first hit.
enum PatchProjection<'a> {
    Grid(ProjectionGrid<'a>),
    Scan {
        verts: &'a [Vector3],
        tris: Vec<[usize; 3]>,
    },
}

/// Face counts below this scan faster than a grid can index.
const PROJECTION_SCAN_TRIS: usize = 64;

/// Absolute cell cap for projection-grid build; beyond it the face
/// set is degenerate at grid scale and the caller scans instead.
const PROJECTION_MAX_CELLS: usize = 1_000_000;

impl<'a> PatchProjection<'a> {
    /// Indexes `faces` (topology face ids). Never empty-armed: zero
    /// faces scan to the query point itself (the old behavior).
    fn build(topology: &SurfaceMesh, verts: &'a [Vector3], faces: &[usize]) -> Self {
        let mut tris = Vec::with_capacity(faces.len());
        let mut total = 0.0;
        for &face in faces {
            let tri = topology.triangle(face);
            if tri[0] < verts.len() && tri[1] < verts.len() && tri[2] < verts.len() {
                total += (verts[tri[1]] - verts[tri[0]]).length();
                tris.push(*tri);
            }
        }
        if tris.len() < PROJECTION_SCAN_TRIS {
            return Self::Scan { verts, tris };
        }
        let h = 2.0 * total / tris.len() as f64;
        let mut lo = Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut hi = Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for v in verts {
            lo.set_data(lo.x().min(v.x()), lo.y().min(v.y()), lo.z().min(v.z()));
            hi.set_data(hi.x().max(v.x()), hi.y().max(v.y()), hi.z().max(v.z()));
        }
        let diag = (hi - lo).length();
        // Degenerate guard (also catches h <= 0 and NaN): a collapsed
        // cell size would send ring expansion to billions of empty
        // rings before its first candidate.
        if !(h > 1e-9 * diag) {
            return Self::Scan { verts, tris };
        }
        match ProjectionGrid::build(verts, tris, h) {
            Some(grid) => Self::Grid(grid),
            // Cell-cap bail: re-collect for the scan (O(faces), on a
            // path that then does O(queries x faces) scan work).
            None => {
                let mut tris = Vec::with_capacity(faces.len());
                for &face in faces {
                    let tri = topology.triangle(face);
                    if tri[0] < verts.len() && tri[1] < verts.len() && tri[2] < verts.len() {
                        tris.push(*tri);
                    }
                }
                Self::Scan { verts, tris }
            }
        }
    }

    /// Closest point on the indexed faces to `point`. `seen`/`stamp`
    /// dedupe triangle tests across the rings of one grid query (the
    /// scan arm ignores them); `seen` must span the tri count.
    fn closest(&self, point: &Vector3, seen: &mut [u32], stamp: u32) -> Vector3 {
        match self {
            Self::Grid(grid) => grid.closest(point, seen, stamp),
            Self::Scan { verts, tris } => {
                let mut best = *point;
                let mut best_d2 = f64::INFINITY;
                for tri in tris {
                    let candidate =
                        closest_on_triangle(point, &verts[tri[0]], &verts[tri[1]], &verts[tri[2]]);
                    let d2 = (candidate - *point).length_squared();
                    if d2 < best_d2 {
                        best_d2 = d2;
                        best = candidate;
                    }
                }
                best
            }
        }
    }

    fn tri_count(&self) -> usize {
        match self {
            Self::Grid(grid) => grid.tris.len(),
            Self::Scan { tris, .. } => tris.len(),
        }
    }
}

struct ProjectionGrid<'a> {
    verts: &'a [Vector3],
    tris: Vec<[usize; 3]>,
    cells: HashMap<[i64; 3], Vec<u32>>,
    h: f64,
}

impl<'a> ProjectionGrid<'a> {
    /// Indexes pre-collected `tris` (`None` past
    /// `PROJECTION_MAX_CELLS`: the caller scans instead).
    fn build(verts: &'a [Vector3], tris: Vec<[usize; 3]>, h: f64) -> Option<Self> {
        let mut cells: HashMap<[i64; 3], Vec<u32>> = HashMap::new();
        for (t, tri) in tris.iter().enumerate() {
            if cells.len() > PROJECTION_MAX_CELLS {
                return None;
            }
            let (pa, pb, pc) = (&verts[tri[0]], &verts[tri[1]], &verts[tri[2]]);
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
                        cells.entry([i, j, k]).or_default().push(t as u32);
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

    /// Exact closest point: ring expansion tracking the best
    /// candidate; exits when `(r*h)^2 >= best_d2` (remaining rings sit
    /// beyond `r*h`, so no untested tri can win). Fixed shell order +
    /// strict improvement = deterministic.
    fn closest(&self, point: &Vector3, seen: &mut [u32], stamp: u32) -> Vector3 {
        let c = [
            (point.x() / self.h).floor() as i64,
            (point.y() / self.h).floor() as i64,
            (point.z() / self.h).floor() as i64,
        ];
        // Wrapping: a query past ~9e18 cells saturates its base key;
        // neighbors then alias arbitrary cells, but every tested tri is
        // still measured exactly, so the exit below still terminates
        // the walk.
        let cell = |dx: i64, dy: i64, dz: i64| {
            [
                c[0].wrapping_add(dx),
                c[1].wrapping_add(dy),
                c[2].wrapping_add(dz),
            ]
        };
        let mut best = *point;
        let mut best_d2 = f64::INFINITY;
        let mut r: i64 = 0;
        loop {
            // Chebyshev shell == r, each cell once (r == 0 visits the
            // center once; the y strips run the open x-interval so
            // shared edges meet once).
            if r == 0 {
                self.test_shell_cell(point, c, seen, stamp, &mut best, &mut best_d2);
            } else {
                for &dz in &[-r, r] {
                    for dx in -r..=r {
                        for dy in -r..=r {
                            self.test_shell_cell(
                                point,
                                cell(dx, dy, dz),
                                seen,
                                stamp,
                                &mut best,
                                &mut best_d2,
                            );
                        }
                    }
                }
                for dz in -(r - 1)..=(r - 1) {
                    for d in -r..=r {
                        self.test_shell_cell(
                            point,
                            cell(r, d, dz),
                            seen,
                            stamp,
                            &mut best,
                            &mut best_d2,
                        );
                        self.test_shell_cell(
                            point,
                            cell(-r, d, dz),
                            seen,
                            stamp,
                            &mut best,
                            &mut best_d2,
                        );
                    }
                    for d in -(r - 1)..=(r - 1) {
                        self.test_shell_cell(
                            point,
                            cell(d, r, dz),
                            seen,
                            stamp,
                            &mut best,
                            &mut best_d2,
                        );
                        self.test_shell_cell(
                            point,
                            cell(d, -r, dz),
                            seen,
                            stamp,
                            &mut best,
                            &mut best_d2,
                        );
                    }
                }
            }
            if (r as f64 * self.h).powi(2) >= best_d2 {
                return best;
            }
            r += 1;
        }
    }

    /// Tests one cell's tris, keeping the strictly closest (ties keep
    /// the first tested: fixed order, deterministic).
    #[allow(clippy::too_many_arguments)]
    fn test_shell_cell(
        &self,
        point: &Vector3,
        key: [i64; 3],
        seen: &mut [u32],
        stamp: u32,
        best: &mut Vector3,
        best_d2: &mut f64,
    ) {
        let Some(list) = self.cells.get(&key) else {
            return;
        };
        for &t in list {
            if seen[t as usize] == stamp {
                continue;
            }
            seen[t as usize] = stamp;
            let tri = self.tris[t as usize];
            let candidate = closest_on_triangle(
                point,
                &self.verts[tri[0]],
                &self.verts[tri[1]],
                &self.verts[tri[2]],
            );
            let d2 = (candidate - *point).length_squared();
            if d2 < *best_d2 {
                *best_d2 = d2;
                *best = candidate;
            }
        }
    }
}

/// Terminal kind of a cut plan: which loop arm the 5/6-side
/// remainder takes (the arm re-derives the planned rotation as its
/// first-valid choice, so the rotation itself needs no payload).
#[derive(Clone, Copy, Debug)]
enum NgonTerminal {
    Pent,
    Hex,
}

/// Union-find over count variables (original sides plus cut sides),
/// with one raw vote per variable for class resolution.
#[derive(Clone)]
struct CountClasses {
    parent: Vec<usize>,
    vote: Vec<f64>,
}

impl CountClasses {
    fn new(raw: &[f64]) -> Self {
        Self {
            parent: (0..raw.len()).collect(),
            vote: raw.to_vec(),
        }
    }

    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            let root = self.parent[self.parent[x]];
            self.parent[x] = root;
            x = root;
        }
        x
    }

    fn union(&mut self, a: usize, b: usize) {
        let ra = self.find(a);
        let rb = self.find(b);
        if ra != rb {
            self.parent[ra] = rb;
        }
    }

    /// Fresh cut variable inheriting its raw vote from `source`.
    fn fresh_cut(&mut self, source: usize) -> usize {
        let id = self.parent.len();
        self.parent.push(id);
        self.vote.push(self.vote[source]);
        id
    }
}

/// Resolve count classes to integers near their raw votes.
///
/// Each class needs one integer within `tol` of every member vote;
/// fixed pins (claimed arcs) must agree and lie in range. Picks the
/// in-range integer nearest the vote median (ties to the smaller),
/// or fails the sequence. `fixed` is per original side.
fn resolve_classes(
    classes: &mut CountClasses,
    n_original: usize,
    fixed: &[Option<usize>],
    tol: f64,
) -> Option<Vec<usize>> {
    let mut members: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for side in 0..n_original {
        let root = classes.find(side);
        members.entry(root).or_default().push(side);
    }
    // Cut votes attach to their class through their variable.
    let ballots: Vec<f64> = classes.vote.clone();
    let mut votes: BTreeMap<usize, Vec<f64>> = BTreeMap::new();
    for (var, vote) in ballots.into_iter().enumerate() {
        let root = classes.find(var);
        votes.entry(root).or_default().push(vote);
    }
    let mut assignment = vec![0usize; n_original];
    for (root, sides) in &members {
        let class_votes = &votes[root];
        let lo = class_votes.iter().map(|v| v - tol).fold(f64::MIN, f64::max);
        let hi = class_votes.iter().map(|v| v + tol).fold(f64::MAX, f64::min);
        let pins: Vec<usize> = sides.iter().filter_map(|&s| fixed[s]).collect();
        if !pins.is_empty() {
            if pins.iter().any(|&p| p != pins[0]) {
                return None;
            }
            let pin = pins[0] as f64;
            if pin < lo || pin > hi {
                return None;
            }
            for &s in sides {
                assignment[s] = pins[0];
            }
            continue;
        }
        let floor = lo.ceil().max(1.0) as usize;
        let ceil = hi.floor().min(256.0) as usize;
        if floor > ceil {
            return None;
        }
        let mut sorted = class_votes.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = sorted[sorted.len() / 2];
        let mut best = floor;
        let mut best_key = (f64::MAX, usize::MAX);
        for candidate in floor..=ceil {
            let key = ((candidate as f64 - median).abs(), candidate);
            if key.0 < best_key.0 || (key.0 == best_key.0 && key.1 < best_key.1) {
                best_key = key;
                best = candidate;
            }
        }
        for &s in sides {
            assignment[s] = best;
        }
    }
    Some(assignment)
}

/// Ear-cut search state: level side variables plus the recorded ear
/// quads (for the grid-cap check after resolution).
struct CountSearch<'a> {
    raw: &'a [f64],
    fixed: &'a [Option<usize>],
    tol: f64,
    budget: &'a mut usize,
    capped: &'a mut bool,
}

/// Solve integer side counts admitting a cut plan.
///
/// Searches ear-cut sequences like `plan_ngon_cuts`, but each
/// sequence induces equality classes over the side counts instead of
/// demanding exact pre-rounded equality; classes resolve to integers
/// within `tol` of their measured lengths (claimed arcs pin their
/// class). Returns the integer assignment for the original sides, or
/// None when no sequence resolves (caller falls back). `arc` maps
/// sides sharing one arc (pre-unioned: one variable).
fn solve_ngon_counts(
    raw: &[f64],
    arc: &[usize],
    arc_fixed: &[Option<usize>],
    tol: f64,
    budget: &mut usize,
    capped: &mut bool,
) -> Option<Vec<usize>> {
    let n = raw.len();
    debug_assert!(n >= 5 && arc.len() == n);
    let fixed: Vec<Option<usize>> = arc.iter().map(|&a| arc_fixed[a]).collect();
    // Sides sharing one arc are one variable: union them upfront.
    let mut classes = CountClasses::new(raw);
    for i in 0..n {
        for j in (i + 1)..n {
            if arc[i] == arc[j] {
                classes.union(i, j);
            }
        }
    }
    // Same-arc sides with conflicting pins are unplannable already
    // (no equalities yet, so only a pin conflict can fail here).
    {
        let mut probe = classes.clone();
        if resolve_classes(&mut probe, n, &fixed, tol).is_none() {
            return None;
        }
    }
    let mut search = CountSearch {
        raw,
        fixed: &fixed,
        tol,
        budget,
        capped,
    };
    let level: Vec<usize> = (0..n).collect();
    search.solve_level(level, classes, &mut Vec::new())
}

/// Trigon wheel splits: side `i` splits into `(a_i, b_i)` with
/// `b_i = a_{i+2}` (opposite-side equality around the three wheel
/// quads), giving `a_i = (n_i + n_{i+1} - n_{i+2}) / 2`. Requires
/// strict triangle inequalities plus an even sum (equivalently every
/// `a_i` a positive integer: a side of count 1 can never split).
/// Returns the `a` triple (spoke counts are `t_i = a_{i+1}`).
fn wheel_splits(counts: [usize; 3]) -> Option<[usize; 3]> {
    let n = [counts[0] as i64, counts[1] as i64, counts[2] as i64];
    let mut splits = [0usize; 3];
    for i in 0..3 {
        let twice = n[i] + n[(i + 1) % 3] - n[(i + 2) % 3];
        if twice < 2 || twice % 2 != 0 {
            return None;
        }
        splits[i] = (twice / 2) as usize;
    }
    Some(splits)
}

/// Grid caps on the three wheel quads (`a_i` by `t_i = a_{i+1}`).
fn wheel_over_cap(splits: [usize; 3]) -> bool {
    (0..3).any(|i| grid_over_cap(splits[i], splits[(i + 1) % 3]))
}

/// Quad-opposite pin targets for an arc: values pinned on the
/// opposite arcs of usable quads using `arc`. Later writers match
/// earlier pins across quads (a trigon side matching the pin across
/// its quad neighbor keeps that quad's pair equal instead of
/// deferring it). Duplicates kept (consensus weighs more).
fn quad_opposite_targets(layout: &Layout, arc_fixed: &[Option<usize>], arc: usize) -> Vec<usize> {
    let mut targets = Vec::new();
    for patch in &layout.patches {
        if !patch.usable || patch.sides.len() != 4 {
            continue;
        }
        for position in 0..4 {
            if patch.sides[position].arc != arc {
                continue;
            }
            let opposite = patch.sides[(position + 2) % 4].arc;
            if let Some(pin) = arc_fixed[opposite] {
                targets.push(pin);
            }
        }
    }
    targets
}

/// Pin-match bonus (L1 units): a quad-opposite pin match is worth
/// this much raw distance. Matching quads across saves conflicted
/// quads (2 tris each); the bonus bounds the reach (a match wins
/// only within bonus of the best unmatched L1).
const MATCH_BONUS: f64 = 3.0;

/// Candidate values for one side: the tol box plus any quad-opposite
/// pin targets (matching beyond tol is allowed; the bonus bounds how
/// far a match can pull). Sorted, deduped, clamped to [1, 256].
fn match_candidates(raw: f64, tol: f64, targets: &[usize]) -> Vec<usize> {
    let mut values: Vec<usize> = Vec::new();
    let lo = (raw - tol).ceil().max(1.0) as usize;
    let hi = (raw + tol).floor().min(256.0) as usize;
    if lo <= hi {
        values.extend(lo..=hi);
    }
    for &t in targets {
        if (1..=256).contains(&t) {
            values.push(t);
        }
    }
    values.sort_unstable();
    values.dedup();
    values
}

/// Trigon agreement: integer counts admitting a wheel fill near the
/// raw votes (claimed arcs pin; sides sharing one arc take one
/// value). Bounded local search over the tol box plus pin targets;
/// minimizes L1 minus bonus per quad-opposite pin match, ties to the
/// lexicographically smallest.
fn solve_trigon_counts(
    raw: &[f64; 3],
    arc: &[usize; 3],
    arc_fixed: &[Option<usize>],
    tol: f64,
    capped: &mut bool,
    layout: &Layout,
) -> Option<[usize; 3]> {
    let pinned: [Option<usize>; 3] = [arc_fixed[arc[0]], arc_fixed[arc[1]], arc_fixed[arc[2]]];
    let targets: [Vec<usize>; 3] = [
        quad_opposite_targets(layout, arc_fixed, arc[0]),
        quad_opposite_targets(layout, arc_fixed, arc[1]),
        quad_opposite_targets(layout, arc_fixed, arc[2]),
    ];
    let boxes: [Vec<usize>; 3] = [
        match pinned[0] {
            Some(pin) => vec![pin],
            None => match_candidates(raw[0], tol, &targets[0]),
        },
        match pinned[1] {
            Some(pin) => vec![pin],
            None => match_candidates(raw[1], tol, &targets[1]),
        },
        match pinned[2] {
            Some(pin) => vec![pin],
            None => match_candidates(raw[2], tol, &targets[2]),
        },
    ];
    if boxes.iter().any(Vec::is_empty) {
        return None;
    }
    let mut best: Option<[usize; 3]> = None;
    let mut best_score = f64::MAX;
    for &c0 in &boxes[0] {
        for &c1 in &boxes[1] {
            for &c2 in &boxes[2] {
                let candidate = [c0, c1, c2];
                // One arc, one count.
                if (arc[0] == arc[1] && c0 != c1)
                    || (arc[1] == arc[2] && c1 != c2)
                    || (arc[0] == arc[2] && c0 != c2)
                {
                    continue;
                }
                let Some(splits) = wheel_splits(candidate) else {
                    continue;
                };
                if wheel_over_cap(splits) {
                    *capped = true;
                    continue;
                }
                let matches: usize = candidate
                    .iter()
                    .enumerate()
                    .map(|(i, &c)| targets[i].iter().filter(|&&t| t == c).count())
                    .sum();
                let l1: f64 = candidate
                    .iter()
                    .enumerate()
                    .map(|(i, &c)| (c as f64 - raw[i]).abs())
                    .sum();
                // Minimize L1 minus bonus per match; ties (equal
                // scores) keep the lex smallest (ascending iteration
                // + strict improvement).
                let score = l1 - MATCH_BONUS * matches as f64;
                if score < best_score {
                    best_score = score;
                    best = Some(candidate);
                }
            }
        }
    }
    best
}

/// Trigon solve with tol escalation (same knob as the ngon solver).
fn solve_trigon_counts_best(
    raw: &[f64; 3],
    arc: &[usize; 3],
    arc_fixed: &[Option<usize>],
    capped: &mut bool,
    layout: &Layout,
) -> Option<[usize; 3]> {
    for tol in agreement_tols() {
        if let Some(assignment) = solve_trigon_counts(raw, arc, arc_fixed, tol, capped, layout) {
            return Some(assignment);
        }
    }
    None
}

/// Digon agreement: both sides even (halves `m` feed the two quads
/// `m0 x m1` / `m1 x m0`), near the raw votes, pins respected (an
/// odd pin fails the solve). Minimizes L1 minus bonus per
/// quad-opposite pin match (even targets only: odd is unmatchable).
/// Same tol escalation as the rest.
fn solve_digon_counts(
    raw: &[f64; 2],
    arc: &[usize; 2],
    arc_fixed: &[Option<usize>],
    tol: f64,
    capped: &mut bool,
    layout: &Layout,
) -> Option<[usize; 2]> {
    let pinned = [arc_fixed[arc[0]], arc_fixed[arc[1]]];
    let targets = [
        quad_opposite_targets(layout, arc_fixed, arc[0]),
        quad_opposite_targets(layout, arc_fixed, arc[1]),
    ];
    let mut boxes: [Vec<usize>; 2] = [Vec::new(), Vec::new()];
    for i in 0..2 {
        if let Some(pin) = pinned[i] {
            if pin % 2 != 0 {
                return None;
            }
            boxes[i] = vec![pin];
        } else {
            let mut values = match_candidates(raw[i], tol, &targets[i]);
            values.retain(|v| *v >= 2 && *v % 2 == 0);
            if values.is_empty() {
                return None;
            }
            boxes[i] = values;
        }
    }
    let mut best: Option<[usize; 2]> = None;
    let mut best_score = f64::MAX;
    for &c0 in &boxes[0] {
        for &c1 in &boxes[1] {
            if arc[0] != arc[1] || c0 == c1 {
                let (m0, m1) = (c0 / 2, c1 / 2);
                if grid_over_cap(m0, m1) {
                    *capped = true;
                } else {
                    let matches = targets[0].iter().filter(|&&t| t == c0).count()
                        + targets[1].iter().filter(|&&t| t == c1).count();
                    let l1 = (c0 as f64 - raw[0]).abs() + (c1 as f64 - raw[1]).abs();
                    let score = l1 - MATCH_BONUS * matches as f64;
                    if score < best_score {
                        best_score = score;
                        best = Some([c0, c1]);
                    }
                }
            }
        }
    }
    best
}

/// Digon solve with tol escalation.
fn solve_digon_counts_best(
    raw: &[f64; 2],
    arc: &[usize; 2],
    arc_fixed: &[Option<usize>],
    capped: &mut bool,
    layout: &Layout,
) -> Option<[usize; 2]> {
    for tol in agreement_tols() {
        if let Some(assignment) = solve_digon_counts(raw, arc, arc_fixed, tol, capped, layout) {
            return Some(assignment);
        }
    }
    None
}

/// Monogon agreement: the loop side splits into three wheel
/// subsides `(m0, m1, m2)` summing near the raw vote (pinned sums
/// respected). Sums matching a quad-opposite pin win, then nearest.
/// Bounded box around thirds (at most 125 triples).
fn solve_monogon_counts(
    raw: f64,
    arc: usize,
    arc_fixed: &[Option<usize>],
    tol: f64,
    capped: &mut bool,
    layout: &Layout,
) -> Option<[usize; 3]> {
    let pin = arc_fixed[arc];
    let targets = quad_opposite_targets(layout, arc_fixed, arc);
    let third = raw / 3.0;
    let (lo, hi) = (
        (third - tol).ceil().max(1.0) as usize,
        (third + tol).floor().min(256.0) as usize,
    );
    if lo > hi {
        return None;
    }
    let mut best: Option<[usize; 3]> = None;
    let mut best_matches = 0usize;
    let mut best_l1 = f64::MAX;
    for m0 in lo..=hi {
        for m1 in lo..=hi {
            for m2 in lo..=hi {
                let candidate = [m0, m1, m2];
                let sum: usize = candidate.iter().sum();
                if let Some(pin) = pin {
                    if sum != pin {
                        continue;
                    }
                } else if (sum as f64 - raw).abs() > tol {
                    continue;
                }
                let Some(splits) = wheel_splits(candidate) else {
                    continue;
                };
                if wheel_over_cap(splits) {
                    *capped = true;
                    continue;
                }
                let matches = targets.iter().filter(|&&t| t == sum).count();
                let l1 = (sum as f64 - raw).abs();
                let better = best.is_none()
                    || matches > best_matches
                    || (matches == best_matches && l1 < best_l1);
                if better {
                    best_matches = matches;
                    best_l1 = l1;
                    best = Some(candidate);
                }
            }
        }
    }
    best
}

/// Monogon solve with tol escalation. Returns the three subsides
/// (the side count is their sum).
fn solve_monogon_counts_best(
    raw: f64,
    arc: usize,
    arc_fixed: &[Option<usize>],
    capped: &mut bool,
    layout: &Layout,
) -> Option<[usize; 3]> {
    for tol in agreement_tols() {
        if let Some(assignment) = solve_monogon_counts(raw, arc, arc_fixed, tol, capped, layout) {
            return Some(assignment);
        }
    }
    None
}

/// Conflicted-quad diagonal rescue: free cut count making both
/// diagonal halves wheel-fillable, or None. Tries both diagonals
/// (parity is a coin flip per diagonal). Returns (use_02, cut).
fn solve_diag_cut(counts: [usize; 4]) -> Option<(bool, usize)> {
    let target = (counts[0] + counts[1] + counts[2] + counts[3]) as f64 / 4.0;
    for use_02 in [true, false] {
        let (a, b, c, d) = if use_02 {
            (counts[0], counts[1], counts[2], counts[3])
        } else {
            (counts[1], counts[2], counts[3], counts[0])
        };
        let lo = ((a as i64 - b as i64).abs().max((c as i64 - d as i64).abs()) + 1).max(1);
        let hi = ((a + b) as i64).min((c + d) as i64) - 1;
        if lo > hi {
            continue;
        }
        let mut best: Option<usize> = None;
        let mut best_key = (f64::MAX, usize::MAX);
        for cut in lo..=hi {
            let cut = cut as usize;
            let (Some(s0), Some(s1)) = (wheel_splits([a, b, cut]), wheel_splits([c, d, cut]))
            else {
                continue;
            };
            if wheel_over_cap(s0) || wheel_over_cap(s1) {
                continue;
            }
            let key = ((cut as f64 - target).abs(), cut);
            if key.0 < best_key.0 || (key.0 == best_key.0 && key.1 < best_key.1) {
                best_key = key;
                best = Some(cut);
            }
        }
        if let Some(cut) = best {
            return Some((use_02, cut));
        }
    }
    None
}

/// Corner-split rescue: rotation + cut count, or None. Rotation `r`:
/// cut from corner `r` to side `r+2` split at `n_r`, opposite side
/// `r+3` split at `n_{r+1}`: two quads `(a,b,h,cut)` /
/// `(h',k,k',cut)` with `h = a`, `cut = k = b` forced, needing
/// `n_{r+2} - n_r == n_{r+3} - n_{r+1} > 0`. First valid rotation.
fn solve_corner_split(counts: [usize; 4]) -> Option<(usize, usize)> {
    for r in 0..4 {
        let a = counts[r];
        let b = counts[(r + 1) % 4];
        let c = counts[(r + 2) % 4];
        let d = counts[(r + 3) % 4];
        if c <= a || d <= b || c - a != d - b {
            continue;
        }
        if grid_over_cap(a, b) || grid_over_cap(c - a, b) {
            continue;
        }
        return Some((r, b));
    }
    None
}

/// Corner-cut rescue: rotation + cut count, or None. Rotation `r`:
/// cut from corner `r` to side `r+2` split at `n_r` (forced): quad
/// `(n_r, n_{r+1}, h, cut)` plus trigon wheel `(n_{r+2} - n_r,
/// n_{r+3}, cut = n_{r+1})`. First valid rotation.
fn solve_corner_cut(counts: [usize; 4]) -> Option<(usize, usize)> {
    for r in 0..4 {
        let a = counts[r];
        let b = counts[(r + 1) % 4];
        let c = counts[(r + 2) % 4];
        let d = counts[(r + 3) % 4];
        if c <= a {
            continue;
        }
        let triple = [c - a, d, b];
        let Some(splits) = wheel_splits(triple) else {
            continue;
        };
        if grid_over_cap(a, b) || wheel_over_cap(splits) {
            continue;
        }
        return Some((r, b));
    }
    None
}

/// Agreement tolerances, tight first: most patches solve within 1
/// of their measured lengths; the remainder gets one looser pass
/// before the caller falls back.
fn agreement_tols() -> Vec<f64> {
    vec![1.0, 2.0]
}

/// Solve with tol escalation: try each tolerance in order with a
/// fresh budget, returning the first assignment found.
fn solve_ngon_counts_best(
    raw: &[f64],
    arc: &[usize],
    arc_fixed: &[Option<usize>],
    capped: &mut bool,
) -> Option<Vec<usize>> {
    for tol in agreement_tols() {
        let mut budget = FILL_MAX_PLAN_NODES;
        if let Some(assignment) = solve_ngon_counts(raw, arc, arc_fixed, tol, &mut budget, capped) {
            return Some(assignment);
        }
    }
    None
}

impl CountSearch<'_> {
    /// Recorded ear quad per cut: (a-var, b-var) for the cap check.
    fn solve_level(
        &mut self,
        level: Vec<usize>,
        classes: CountClasses,
        ears: &mut Vec<(usize, usize)>,
    ) -> Option<Vec<usize>> {
        if *self.budget == 0 {
            *self.capped = true;
            return None;
        }
        *self.budget -= 1;
        let n = level.len();
        if n == 5 {
            for i in 0..5 {
                let mut trial = classes.clone();
                // Quad sides (i+2, i+3, i+4): opposite equality.
                trial.union(level[(i + 2) % 5], level[(i + 4) % 5]);
                let mut trial_ears = ears.clone();
                trial_ears.push((level[(i + 2) % 5], level[(i + 3) % 5]));
                // Triangle (i, i+1, cut): the cut takes sides[1] of
                // the quad, so the wheel triple is (i, i+1, i+3).
                let wheel = [level[i], level[(i + 1) % 5], level[(i + 3) % 5]];
                if let Some(assign) = self.try_resolve(trial, &trial_ears, Some(wheel)) {
                    return Some(assign);
                }
            }
            return None;
        }
        if n == 6 {
            for i in 0..6 {
                let idx: Vec<usize> = (0..6).map(|k| level[(i + k) % 6]).collect();
                let mut trial = classes.clone();
                trial.union(idx[0], idx[2]);
                trial.union(idx[3], idx[5]);
                trial.union(idx[1], idx[4]);
                let mut trial_ears = ears.clone();
                trial_ears.push((idx[0], idx[1]));
                trial_ears.push((idx[3], idx[4]));
                if let Some(assign) = self.try_resolve(trial, &trial_ears, None) {
                    return Some(assign);
                }
            }
            return None;
        }
        for i in 0..n {
            let mut trial = classes.clone();
            trial.union(level[i], level[(i + 2) % n]);
            let cut = trial.fresh_cut(level[(i + 1) % n]);
            trial.union(cut, level[(i + 1) % n]);
            let mut rest = Vec::with_capacity(n - 2);
            for k in 3..n {
                rest.push(level[(i + k) % n]);
            }
            rest.push(cut);
            ears.push((level[i], level[(i + 1) % n]));
            if let Some(assign) = self.solve_level(rest, trial, ears) {
                return Some(assign);
            }
            ears.pop();
        }
        None
    }

    /// Resolve classes and verify grid caps on every recorded ear
    /// plus the optional pentagon wheel triple (variable ids).
    fn try_resolve(
        &mut self,
        mut classes: CountClasses,
        ears: &[(usize, usize)],
        wheel: Option<[usize; 3]>,
    ) -> Option<Vec<usize>> {
        let n = self.raw.len();
        let assignment = resolve_classes(&mut classes, n, self.fixed, self.tol)?;
        // Variable values: originals from the assignment, cuts from
        // their class root's original member.
        let mut values = vec![0usize; classes.parent.len()];
        for (side, &value) in assignment.iter().enumerate() {
            values[side] = value;
        }
        for var in n..values.len() {
            let root = classes.find(var);
            let member = (0..n).find(|&s| classes.find(s) == root);
            match member {
                Some(s) => values[var] = assignment[s],
                // Cut-only class (no original member): fall back to
                // its raw vote (cannot happen: cuts union with b).
                None => return None,
            }
        }
        for &(a, b) in ears {
            if grid_over_cap(values[a], values[b]) {
                *self.capped = true;
                return None;
            }
        }
        if let Some(w) = wheel {
            let triple = [values[w[0]], values[w[1]], values[w[2]]];
            match wheel_splits(triple) {
                Some(splits) if !wheel_over_cap(splits) => {}
                Some(_) => {
                    *self.capped = true;
                    return None;
                }
                None => return None,
            }
        }
        Some(assignment)
    }
}

/// First pentagon rotation whose cut yields a quad plus an
/// all-quad wheel triangle: quad sides (arc, arc, arc, cut) need
/// sides[0] == sides[2] with the cut taking sides[1], and the
/// triangle (i, i+1, cut) needs wheel splits. Over-cap rotations are
/// skipped (`capped`). Shared by the solver, the geometry planner,
/// and the emit arm, so plan and emit agree on the rotation.
fn first_pent_cut(counts: &[usize], capped: &mut bool) -> Option<usize> {
    debug_assert_eq!(counts.len(), 5);
    for i in 0..5 {
        let s0 = counts[(i + 2) % 5];
        let s1 = counts[(i + 3) % 5];
        let s2 = counts[(i + 4) % 5];
        if s0 != s2 {
            continue;
        }
        if grid_over_cap(s0, s1) {
            *capped = true;
            continue;
        }
        let triple = [counts[i], counts[(i + 1) % 5], s1];
        match wheel_splits(triple) {
            Some(splits) if !wheel_over_cap(splits) => return Some(i),
            Some(_) => {
                *capped = true;
            }
            None => {}
        }
    }
    None
}

/// First hexagon rotation whose halving cut yields two quads: cut
/// i -> i+3 needs a == c, d == f, and b == e (shared cut count).
/// Over-cap rotations are skipped (`capped`).
fn first_hex_cut(counts: &[usize], capped: &mut bool) -> Option<usize> {
    debug_assert_eq!(counts.len(), 6);
    for i in 0..6 {
        let sides = [
            counts[i],
            counts[(i + 1) % 6],
            counts[(i + 2) % 6],
            counts[(i + 3) % 6],
            counts[(i + 4) % 6],
            counts[(i + 5) % 6],
        ];
        if sides[0] != sides[2] || sides[3] != sides[5] || sides[1] != sides[4] {
            continue;
        }
        if grid_over_cap(sides[0], sides[1]) || grid_over_cap(sides[3], sides[4]) {
            *capped = true;
            continue;
        }
        return Some(i);
    }
    None
}

/// The fill context: one island's inputs plus the welded output.
struct Filler<'a> {
    topology: &'a SurfaceMesh,
    verts: &'a [Vector3],
    layout: &'a Layout,
    counts: Vec<usize>,
    verts_out: Vec<Vector3>,
    faces_out: Vec<Vec<usize>>,
    uv_patch: Vec<usize>,
    uv_local: Vec<(f64, f64)>,
    keys: BTreeMap<Key, usize>,
    stats: FillStats,
    degenerate_area: f64,
    next_tri_edge: usize,
    projection: PatchProjection<'a>,
    proj_seen: Vec<u32>,
    proj_stamp: u32,
}

impl<'a> Filler<'a> {
    fn emit(&mut self, key: Key, position: Vector3, patch: usize, uv: (f64, f64)) -> usize {
        if let Some(&id) = self.keys.get(&key) {
            return id;
        }
        let id = self.verts_out.len();
        self.keys.insert(key, id);
        self.verts_out.push(position);
        self.uv_patch.push(patch);
        self.uv_local.push(uv);
        id
    }

    /// Emit a face, skipping degenerate ones (fewer than 3 distinct
    /// corners or near-zero area).
    fn emit_face(&mut self, corners: &[usize]) {
        debug_assert!(corners.len() == 3 || corners.len() == 4);
        let mut distinct: Vec<usize> = corners.to_vec();
        distinct.sort_unstable();
        distinct.dedup();
        if distinct.len() < 3 {
            self.stats.degenerate_skipped += 1;
            return;
        }
        let positions: Vec<Vector3> = corners.iter().map(|&c| self.verts_out[c]).collect();
        let mut area = 0.0;
        for i in 1..positions.len() - 1 {
            let e0 = positions[i] - positions[0];
            let e1 = positions[i + 1] - positions[0];
            area += 0.5 * Vector3::cross_product(&e0, &e1).length();
        }
        if area <= self.degenerate_area {
            self.stats.degenerate_skipped += 1;
            return;
        }
        if corners.len() == 3 {
            self.stats.triangles += 1;
        }
        self.faces_out.push(corners.to_vec());
    }

    /// Rebuilds the projection index for a patch's faces:
    /// structured fills project onto their own patch only, so thin
    /// features can never snap across gaps.
    fn begin_patch(&mut self, patch_faces: &[usize]) {
        self.projection = PatchProjection::build(self.topology, self.verts, patch_faces);
        self.proj_seen = vec![0u32; self.projection.tri_count()];
        self.proj_stamp = 1;
    }

    /// Closest point on the current patch's triangles.
    fn project(&mut self, point: &Vector3) -> Vector3 {
        self.proj_stamp = self.proj_stamp.wrapping_add(1).max(1);
        self.projection
            .closest(point, &mut self.proj_seen, self.proj_stamp)
    }

    /// Resample an arc path to count+1 points by arc length.
    fn resample_arc(&self, arc: usize, count: usize) -> Vec<Vector3> {
        let path = &self.layout.graph.arcs[arc].path;
        let mut points: Vec<Vector3> = path.iter().map(|&v| self.verts[v]).collect();
        // Drop consecutive duplicates (looped paths repeat vertices).
        points.dedup_by(|b, a| (*b - *a).length_squared() == 0.0);
        if points.len() < 2 || count == 0 {
            return points;
        }
        let mut cumulative = vec![0.0; points.len()];
        for i in 1..points.len() {
            cumulative[i] = cumulative[i - 1] + (points[i] - points[i - 1]).length();
        }
        let total = cumulative[points.len() - 1];
        if total <= 0.0 {
            return vec![points[0]; count + 1];
        }
        let mut out = Vec::with_capacity(count + 1);
        let mut segment = 0;
        for i in 0..=count {
            let target = total * i as f64 / count as f64;
            while segment + 1 < points.len() - 1 && cumulative[segment + 1] < target {
                segment += 1;
            }
            let span = cumulative[segment + 1] - cumulative[segment];
            let t = if span > 0.0 {
                ((target - cumulative[segment]) / span).clamp(0.0, 1.0)
            } else {
                0.0
            };
            out.push(points[segment] + (points[segment + 1] - points[segment]) * t);
        }
        // Exact endpoints (shared node positions).
        out[0] = points[0];
        out[count] = points[points.len() - 1];
        out
    }

    /// Grid keys + positions for a patch side (loop order).
    fn side_sequence(&self, patch: usize, position: usize) -> (Vec<Key>, Vec<Vector3>) {
        let side = self.layout.patches[patch].sides[position];
        let count = self.counts[side.arc];
        let mut points = self.resample_arc(side.arc, count);
        let mut keys: Vec<Key> = (0..=count).map(|i| Key::SidePoint(side.arc, i)).collect();
        // Endpoints are corner nodes.
        let (entry, exit) = if side.forward {
            (
                self.layout.graph.arcs[side.arc].a,
                self.layout.graph.arcs[side.arc].b,
            )
        } else {
            (
                self.layout.graph.arcs[side.arc].b,
                self.layout.graph.arcs[side.arc].a,
            )
        };
        if !side.forward {
            // Loop order runs b -> a: reverse the resample (a -> b)
            // and the arc-indexed interior keys. The Node endpoints
            // are set AFTER the reversal: reversing them too swaps
            // entry/exit, keying the entry position under the exit
            // node (order-dependent node corruption + slivers).
            points.reverse();
            keys.reverse();
        }
        keys[0] = Key::Node(entry);
        keys[count] = Key::Node(exit);
        (keys, points)
    }

    /// Corner node id of a patch side's start (loop order).
    fn side_entry_node(&self, patch: usize, position: usize) -> usize {
        let side = self.layout.patches[patch].sides[position];
        if side.forward {
            self.layout.graph.arcs[side.arc].a
        } else {
            self.layout.graph.arcs[side.arc].b
        }
    }

    /// Coons grid over a quad with given side sequences (loop order)
    /// and opposite counts (a for sides 0/2, b for sides 1/3).
    /// `piece` distinguishes cut pieces within the patch for keys.
    fn fill_coons(
        &mut self,
        patch: usize,
        piece: usize,
        seqs: &[Vec<Key>; 4],
        positions: &[Vec<Vector3>; 4],
        a: usize,
        b: usize,
    ) {
        debug_assert_eq!(positions[0].len(), a + 1);
        debug_assert_eq!(positions[2].len(), a + 1);
        debug_assert_eq!(positions[1].len(), b + 1);
        debug_assert_eq!(positions[3].len(), b + 1);
        let corners = [
            positions[0][0],
            positions[1][0],
            positions[2][0],
            positions[3][0],
        ];
        let mut grid: Vec<Vec<usize>> = vec![vec![usize::MAX; b + 1]; a + 1];
        for i in 0..=a {
            for j in 0..=b {
                let on_s0 = j == 0;
                let on_s1 = i == a;
                let on_s2 = j == b;
                let on_s3 = i == 0;
                let (key, position) = if on_s0 && !on_s1 && !on_s3 {
                    (seqs[0][i], positions[0][i])
                } else if on_s2 && !on_s1 && !on_s3 {
                    (seqs[2][a - i], positions[2][a - i])
                } else if on_s1 && !on_s0 && !on_s2 {
                    (seqs[1][j], positions[1][j])
                } else if on_s3 && !on_s0 && !on_s2 {
                    (seqs[3][b - j], positions[3][b - j])
                } else if on_s0 || on_s1 || on_s2 || on_s3 {
                    // Corners.
                    let (key, position) = if i == 0 && j == 0 {
                        (seqs[0][0], positions[0][0])
                    } else if i == a && j == 0 {
                        (seqs[1][0], positions[1][0])
                    } else if i == a && j == b {
                        (seqs[2][0], positions[2][0])
                    } else {
                        (seqs[3][0], positions[3][0])
                    };
                    (key, position)
                } else {
                    // Bilinear Coons interior, projected to the patch.
                    let u = i as f64 / a as f64;
                    let v = j as f64 / b as f64;
                    let bottom = positions[0][i];
                    let top = positions[2][a - i];
                    let left = positions[3][b - j];
                    let right = positions[1][j];
                    let mut point = bottom * (1.0 - v) + top * v + left * (1.0 - u) + right * u
                        - corners[0] * (1.0 - u) * (1.0 - v)
                        - corners[1] * u * (1.0 - v)
                        - corners[2] * u * v
                        - corners[3] * (1.0 - u) * v;
                    point = self.project(&point);
                    (Key::Grid(patch, piece, i, j), point)
                };
                let uv = (i as f64 / a as f64, j as f64 / b as f64);
                grid[i][j] = self.emit(key, position, patch, uv);
            }
        }
        for i in 0..a {
            for j in 0..b {
                self.emit_face(&[
                    grid[i][j],
                    grid[i + 1][j],
                    grid[i + 1][j + 1],
                    grid[i][j + 1],
                ]);
            }
        }
    }

    /// Sorted patch-vertex adjacency for cut paths, built once per
    /// patch (every plan level shares it).
    fn cut_adjacency(&self, patch_faces: &[usize]) -> BTreeMap<usize, Vec<usize>> {
        let mut in_patch: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for &face in patch_faces {
            let tri = self.topology.triangle(face);
            for &v in tri {
                in_patch.entry(v).or_insert_with(Vec::new);
            }
            for i in 0..3 {
                let a = tri[i];
                let b = tri[(i + 1) % 3];
                in_patch.get_mut(&a).unwrap().push(b);
                in_patch.get_mut(&b).unwrap().push(a);
            }
        }
        for neighbors in in_patch.values_mut() {
            neighbors.sort_unstable();
            neighbors.dedup();
        }
        in_patch
    }

    /// Dijkstra vertex path across patch faces between two corners.
    ///
    /// Binary-heap open set keyed by (distance bits, vertex): distances
    /// are non-negative so the bit order matches the float order, and
    /// the vertex id breaks ties, keeping the path deterministic.
    /// Always returns a usable path (direct hop when disconnected).
    fn cut_path(
        &self,
        adjacency: &BTreeMap<usize, Vec<usize>>,
        from: usize,
        to: usize,
    ) -> Vec<usize> {
        let in_patch = adjacency;
        if !in_patch.contains_key(&from) || !in_patch.contains_key(&to) {
            return vec![from, to];
        }
        let mut dist: BTreeMap<usize, f64> = BTreeMap::new();
        let mut prev: BTreeMap<usize, usize> = BTreeMap::new();
        let mut heap: BinaryHeap<(Reverse<u64>, Reverse<usize>)> = BinaryHeap::new();
        dist.insert(from, 0.0);
        heap.push((Reverse(0u64), Reverse(from)));
        while let Some((Reverse(bits), Reverse(vertex))) = heap.pop() {
            if bits != dist[&vertex].to_bits() {
                continue;
            }
            if vertex == to {
                break;
            }
            let base = dist[&vertex];
            if let Some(neighbors) = in_patch.get(&vertex) {
                for &u in neighbors {
                    let step = (self.verts[u] - self.verts[vertex]).length();
                    let candidate = base + step;
                    if candidate < *dist.get(&u).unwrap_or(&f64::INFINITY) {
                        dist.insert(u, candidate);
                        prev.insert(u, vertex);
                        heap.push((Reverse(candidate.to_bits()), Reverse(u)));
                    }
                }
            }
        }
        if !dist.contains_key(&to) {
            return vec![from, to];
        }
        let mut path = vec![to];
        while *path.last().unwrap() != from {
            let last = *path.last().unwrap();
            match prev.get(&last) {
                Some(&p) => path.push(p),
                None => return vec![from, to],
            }
            if path.len() > in_patch.len() + 1 {
                return vec![from, to];
            }
        }
        path.reverse();
        path
    }

    /// Point sequence along a cut path resampled to count+1 points.
    fn cut_sequence(
        &mut self,
        patch: usize,
        cut: usize,
        path: &[usize],
        count: usize,
        forward: bool,
        from_node: usize,
        to_node: usize,
    ) -> Vec<Key> {
        let points: Vec<Vector3> = path.iter().map(|&v| self.verts[v]).collect();
        let mut cumulative = vec![0.0; points.len().max(1)];
        for i in 1..points.len() {
            cumulative[i] = cumulative[i - 1] + (points[i] - points[i - 1]).length();
        }
        let total = *cumulative.last().unwrap_or(&0.0);
        let mut keys = Vec::with_capacity(count + 1);
        for i in 0..=count {
            let target = if total > 0.0 {
                total * i as f64 / count.max(1) as f64
            } else {
                0.0
            };
            let mut segment = 0;
            while segment + 1 < points.len().saturating_sub(1) && cumulative[segment + 1] < target {
                segment += 1;
            }
            let mut point = if points.len() >= 2 {
                let span = cumulative[segment + 1] - cumulative[segment];
                let t = if span > 0.0 {
                    ((target - cumulative[segment]) / span).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                points[segment] + (points[segment + 1] - points[segment]) * t
            } else {
                points[0]
            };
            point = self.project(&point);
            // Endpoints are corner nodes (shared keys, not
            // patch-local cut keys): a cut endpoint duplicates its
            // Node position otherwise (677 dup groups on
            // beast@1000). Endpoints use exact working-vertex
            // positions (never projected approximations), so
            // first-writer order against the side grids is harmless.
            let (key, point) = if i == 0 {
                (Key::Node(from_node), self.verts[path[0]])
            } else if i == count {
                (Key::Node(to_node), self.verts[path[path.len() - 1]])
            } else {
                (
                    Key::CutPoint(patch, cut, if forward { i } else { count - i }),
                    point,
                )
            };
            let uv = (i as f64 / count.max(1) as f64, 0.5);
            self.emit(key, point, patch, uv);
            keys.push(key);
        }
        if !forward {
            keys.reverse();
        }
        keys
    }

    /// Cut from a corner node to a side-interior grid point (corner
    /// rescue templates): Dijkstra to the nearest arc-path working
    /// vertex, resampled, last point overwritten with the exact split
    /// position (keys: corner Node, interior CutPoints, split grid
    /// key). None only when the path degenerates (split nearest the
    /// start corner itself).
    fn cut_to_side_point(
        &mut self,
        patch: usize,
        cut: usize,
        adjacency: &BTreeMap<usize, Vec<usize>>,
        from_node: usize,
        side_arc: usize,
        split_key: Key,
        split_point: Vector3,
        count: usize,
    ) -> Option<(Vec<Key>, Vec<Vector3>)> {
        let path_verts = &self.layout.graph.arcs[side_arc].path;
        let mut nearest = usize::MAX;
        let mut nearest_dist = f64::MAX;
        for &v in path_verts {
            let d = (self.verts[v] - split_point).length_squared();
            if d < nearest_dist {
                nearest_dist = d;
                nearest = v;
            }
        }
        if nearest == usize::MAX {
            return None;
        }
        let from_vertex = self.layout.graph.nodes[from_node].vertex;
        if nearest == from_vertex {
            return None;
        }
        // Endpoint-nearest is fine (short sides): the path still has
        // length (from_node is never on this side) and the last
        // point is overwritten with the exact split position.
        let path = self.cut_path(adjacency, from_vertex, nearest);
        if path.len() < 2 {
            return None;
        }
        let points: Vec<Vector3> = path.iter().map(|&v| self.verts[v]).collect();
        let mut cumulative = vec![0.0; points.len()];
        for i in 1..points.len() {
            cumulative[i] = cumulative[i - 1] + (points[i] - points[i - 1]).length();
        }
        let total = cumulative[points.len() - 1];
        let mut keys = Vec::with_capacity(count + 1);
        let mut out = Vec::with_capacity(count + 1);
        for i in 0..=count {
            let target = if total > 0.0 {
                total * i as f64 / count.max(1) as f64
            } else {
                0.0
            };
            let mut segment = 0;
            while segment + 1 < points.len().saturating_sub(1) && cumulative[segment + 1] < target {
                segment += 1;
            }
            let mut point = if points.len() >= 2 {
                let span = cumulative[segment + 1] - cumulative[segment];
                let t = if span > 0.0 {
                    ((target - cumulative[segment]) / span).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                points[segment] + (points[segment + 1] - points[segment]) * t
            } else {
                points[0]
            };
            point = self.project(&point);
            let (key, point) = if i == 0 {
                // Exact node position (matches the side grids).
                (Key::Node(from_node), self.verts[from_vertex])
            } else if i == count {
                // Exact split position (welds with the side grid).
                (split_key, split_point)
            } else {
                (Key::CutPoint(patch, cut, i), point)
            };
            let uv = (i as f64 / count.max(1) as f64, 0.5);
            self.emit(key, point, patch, uv);
            keys.push(key);
            out.push(point);
        }
        Some((keys, out))
    }

    /// Greedy cut planner: pure side-count arithmetic deciding the
    /// single cut sequence (first-valid-choice at every level, loop
    /// order) before any geometry runs. Geometry never fails (Dijkstra
    /// always returns a path, Coons never fails), so planning success
    /// is exactly the old backtracking search's first success —
    /// without the exponential re-emission (n x (n-2) x ... Dijkstra +
    /// Coons runs, which hung beast@1000). `None` = no exact sequence
    /// or a cap bound: the caller falls back (`capped` tells which).
    ///
    /// Plan levels (cut rotation per level, loop order) append to
    /// `cuts`; the terminal is the 5/6-side kind the terminal branch
    /// takes. `budget` bounds total search nodes (the rotation search
    /// backtracks, so depth alone cannot bound it).
    fn plan_ngon_cuts(
        counts: &[usize],
        cuts: &mut Vec<usize>,
        capped: &mut bool,
        budget: &mut usize,
    ) -> Option<NgonTerminal> {
        if *budget == 0 {
            *capped = true;
            return None;
        }
        *budget -= 1;
        match counts.len() {
            5 => first_pent_cut(counts, capped).map(|_| NgonTerminal::Pent),
            6 => first_hex_cut(counts, capped).map(|_| NgonTerminal::Hex),
            n => {
                for i in 0..n {
                    let a = counts[i];
                    let b = counts[(i + 1) % n];
                    let c = counts[(i + 2) % n];
                    if a != c {
                        continue;
                    }
                    if grid_over_cap(a, b) {
                        *capped = true;
                        continue;
                    }
                    let mut rest = Vec::with_capacity(n - 2);
                    for k in 3..n {
                        rest.push(counts[(i + k) % n]);
                    }
                    rest.push(b.max(1));
                    cuts.push(i);
                    if let Some(terminal) = Self::plan_ngon_cuts(&rest, cuts, capped, budget) {
                        return Some(terminal);
                    }
                    cuts.pop();
                }
                None
            }
        }
    }

    /// Corner-cut fill of an n-gon (n >= 5) into Coons quads plus one
    /// triangle (odd n). Plans the single cut sequence first, then
    /// emits it infallibly (no tentative geometry, no rollback).
    /// Returns false when no exact cut sequence exists or a cap binds
    /// (caller runs the per-face fallback).
    ///
    /// `corners` are node ids in loop order, `sides` the side-grid key
    /// sequences between consecutive corners (lengths give counts+1).
    fn fill_ngon(
        &mut self,
        patch: usize,
        corners: &[usize],
        sides: &[Vec<Key>],
        side_points: &[Vec<Vector3>],
        patch_faces: &[usize],
        piece_base: &mut usize,
        cut_base: &mut usize,
        capped: &mut bool,
    ) -> bool {
        let n = corners.len();
        debug_assert!(n >= 5 && sides.len() == n);
        // fill_ngon keeps patch_faces for the one-shot adjacency
        // build (projection runs through begin_patch state instead).
        let adjacency = self.cut_adjacency(patch_faces);
        if n == 5 {
            // Pentagon: the planned rotation (shared predicate with
            // the solver) yields a quad plus an all-quad wheel
            // triangle. Bounded (1 Dijkstra, no recursion).
            let counts: Vec<usize> = sides.iter().map(|s| s.len() - 1).collect();
            let Some(i) = first_pent_cut(&counts, capped) else {
                return false;
            };
            // Wheel splits before any emission (no orphan quad when
            // the defensive check below disagrees with the planner).
            let triple = [counts[i], counts[(i + 1) % 5], counts[(i + 3) % 5]];
            let Some(splits) = wheel_splits(triple).filter(|s| !wheel_over_cap(*s)) else {
                return false;
            };
            let quad_corners = [
                corners[(i + 2) % 5],
                corners[(i + 3) % 5],
                corners[(i + 4) % 5],
                corners[i],
            ];
            let quad_sides = [
                sides[(i + 2) % 5].len() - 1,
                sides[(i + 3) % 5].len() - 1,
                sides[(i + 4) % 5].len() - 1,
            ];
            let cut_count = quad_sides[1].max(1);
            let from = self.layout.graph.nodes[quad_corners[3]].vertex;
            let to = self.layout.graph.nodes[quad_corners[0]].vertex;
            let path = self.cut_path(&adjacency, from, to);
            let cut = *cut_base;
            *cut_base += 1;
            let cut_keys = self.cut_sequence(
                patch,
                cut,
                &path,
                cut_count,
                true,
                quad_corners[3],
                quad_corners[0],
            );
            let cut_points: Vec<Vector3> = cut_keys
                .iter()
                .map(|k| self.verts_out[self.keys[k]])
                .collect();
            let piece = *piece_base;
            *piece_base += 1;
            self.fill_coons(
                patch,
                piece,
                &[
                    sides[(i + 2) % 5].clone(),
                    sides[(i + 3) % 5].clone(),
                    sides[(i + 4) % 5].clone(),
                    cut_keys.clone(),
                ],
                &[
                    side_points[(i + 2) % 5].clone(),
                    side_points[(i + 3) % 5].clone(),
                    side_points[(i + 4) % 5].clone(),
                    cut_points.clone(),
                ],
                quad_sides[0],
                quad_sides[1],
            );
            // The triangle (corners i, i+1, i+2) as an all-quad
            // wheel over its real side grids (the cut reversed into
            // loop order), so shared sides weld instead of cracking.
            let mut cut_rev = cut_keys.clone();
            cut_rev.reverse();
            let mut cut_pts_rev = cut_points.clone();
            cut_pts_rev.reverse();
            self.fill_wheel(
                patch,
                &[sides[i].clone(), sides[(i + 1) % 5].clone(), cut_rev],
                &[
                    side_points[i].clone(),
                    side_points[(i + 1) % 5].clone(),
                    cut_pts_rev,
                ],
                splits,
                piece_base,
                cut_base,
            );
            return true;
        }
        // n == 6: halving cut into two quads. Bounded (6
        // Dijkstras, no recursion) — not the old hang.
        if n == 6 {
            for i in 0..6 {
                let a = sides[i].len() - 1;
                let b = sides[(i + 1) % 6].len() - 1;
                let c = sides[(i + 2) % 6].len() - 1;
                let d = sides[(i + 3) % 6].len() - 1;
                let e = sides[(i + 4) % 6].len() - 1;
                let f = sides[(i + 5) % 6].len() - 1;
                // Cut i -> i+3: quads (a,b,c,cut) and (d,e,f,cut)
                // need a == c, d == f, and b == e (shared cut count).
                if a != c || d != f || b != e {
                    continue;
                }
                if grid_over_cap(a, b) || grid_over_cap(d, e) {
                    *capped = true;
                    continue;
                }
                let cut_count = b.max(1);
                let from = self.layout.graph.nodes[corners[i]].vertex;
                let to = self.layout.graph.nodes[corners[(i + 3) % 6]].vertex;
                let path = self.cut_path(&adjacency, from, to);
                let cut = *cut_base;
                *cut_base += 1;
                let cut_keys = self.cut_sequence(
                    patch,
                    cut,
                    &path,
                    cut_count,
                    true,
                    corners[i],
                    corners[(i + 3) % 6],
                );
                let cut_points: Vec<Vector3> = cut_keys
                    .iter()
                    .map(|k| self.verts_out[self.keys[k]])
                    .collect();
                let mut rev_keys = cut_keys.clone();
                rev_keys.reverse();
                let mut rev_points = cut_points.clone();
                rev_points.reverse();
                // The cut runs corner[i] -> corner[i+3]: piece 1's
                // side 3 must run corner[i+3] -> corner[i] (loop
                // order), piece 2's corner[i] -> corner[i+3]. (The
                // old swapped assignment collapsed corners once cut
                // endpoints became shared Node keys.)
                let piece = *piece_base;
                *piece_base += 1;
                self.fill_coons(
                    patch,
                    piece,
                    &[
                        sides[i].clone(),
                        sides[(i + 1) % 6].clone(),
                        sides[(i + 2) % 6].clone(),
                        rev_keys,
                    ],
                    &[
                        side_points[i].clone(),
                        side_points[(i + 1) % 6].clone(),
                        side_points[(i + 2) % 6].clone(),
                        rev_points,
                    ],
                    a,
                    b,
                );
                let piece = *piece_base;
                *piece_base += 1;
                self.fill_coons(
                    patch,
                    piece,
                    &[
                        sides[(i + 3) % 6].clone(),
                        sides[(i + 4) % 6].clone(),
                        sides[(i + 5) % 6].clone(),
                        cut_keys,
                    ],
                    &[
                        side_points[(i + 3) % 6].clone(),
                        side_points[(i + 4) % 6].clone(),
                        side_points[(i + 5) % 6].clone(),
                        cut_points,
                    ],
                    d,
                    e,
                );
                return true;
            }
            return false;
        }
        // n >= 7: plan the single cut sequence, then emit it
        // infallibly (no tentative geometry, no rollback). The old
        // backtracking search (try every cut, recurse, roll back) was
        // exponential in n with a Dijkstra + Coons fill per attempt —
        // that hung beast@1000.
        if n > FILL_MAX_PLAN_SIDES {
            *capped = true;
            return false;
        }
        let counts: Vec<usize> = sides.iter().map(|s| s.len() - 1).collect();
        let mut plan_cuts = Vec::new();
        let mut plan_capped = false;
        let mut budget = FILL_MAX_PLAN_NODES;
        if Self::plan_ngon_cuts(&counts, &mut plan_cuts, &mut plan_capped, &mut budget).is_none() {
            *capped |= plan_capped;
            return false;
        }
        *capped |= plan_capped;
        let mut level_corners = corners.to_vec();
        let mut level_sides = sides.to_vec();
        let mut level_points = side_points.to_vec();
        for &i in &plan_cuts {
            let m = level_corners.len();
            let a = level_sides[i].len() - 1;
            let b = level_sides[(i + 1) % m].len() - 1;
            let cut_count = b.max(1);
            let from = self.layout.graph.nodes[level_corners[i]].vertex;
            let to = self.layout.graph.nodes[level_corners[(i + 3) % m]].vertex;
            let path = self.cut_path(&adjacency, from, to);
            let cut = *cut_base;
            *cut_base += 1;
            let cut_keys = self.cut_sequence(
                patch,
                cut,
                &path,
                cut_count,
                true,
                level_corners[i],
                level_corners[(i + 3) % m],
            );
            let cut_points: Vec<Vector3> = cut_keys
                .iter()
                .map(|k| self.verts_out[self.keys[k]])
                .collect();
            // The cut runs corner[i] -> corner[i+3]: the piece's
            // side 3 must run corner[i+3] -> corner[i] (loop order),
            // so the piece takes the reversed cut and the remainder
            // (closing corner[i] -> corner[i+3]) takes it forward.
            let mut piece_keys = cut_keys.clone();
            piece_keys.reverse();
            let mut piece_points = cut_points.clone();
            piece_points.reverse();
            let piece = *piece_base;
            *piece_base += 1;
            self.fill_coons(
                patch,
                piece,
                &[
                    level_sides[i].clone(),
                    level_sides[(i + 1) % m].clone(),
                    level_sides[(i + 2) % m].clone(),
                    piece_keys,
                ],
                &[
                    level_points[i].clone(),
                    level_points[(i + 1) % m].clone(),
                    level_points[(i + 2) % m].clone(),
                    piece_points,
                ],
                a,
                b,
            );
            // Remainder: corners i+3..i+m-1, i with the cut closing it.
            let mut rest_corners = Vec::with_capacity(m - 2);
            let mut rest_sides = Vec::with_capacity(m - 2);
            let mut rest_points = Vec::with_capacity(m - 2);
            for k in 3..m {
                rest_corners.push(level_corners[(i + k) % m]);
                rest_sides.push(level_sides[(i + k) % m].clone());
                rest_points.push(level_points[(i + k) % m].clone());
            }
            rest_corners.push(level_corners[i]);
            rest_sides.push(cut_keys);
            rest_points.push(cut_points);
            level_corners = rest_corners;
            level_sides = rest_sides;
            level_points = rest_points;
        }
        // Terminal remainder (5/6 sides): recurse once; the loop arms
        // find the planned rotation (same first-valid choice), so this
        // is true by construction (verdict returned defensively).
        self.fill_ngon(
            patch,
            &level_corners,
            &level_sides,
            &level_points,
            patch_faces,
            piece_base,
            cut_base,
            capped,
        )
    }

    /// Subdivided triangle fill: 4-subtri recursion while all sides
    /// stay even (midpoints land on shared grids), else one face.
    fn fill_triangle(
        &mut self,
        patch: usize,
        seqs: &[Vec<Key>; 3],
        points: &[Vec<Vector3>; 3],
        depth: usize,
    ) {
        let counts = [seqs[0].len() - 1, seqs[1].len() - 1, seqs[2].len() - 1];
        let subdividable = depth < 3
            && counts.iter().all(|&c| c >= 2 && c % 2 == 0)
            && counts.iter().any(|&c| c >= 4);
        if !subdividable {
            let corners = [
                self.emit(seqs[0][0], points[0][0], patch, (0.0, 0.0)),
                self.emit(seqs[1][0], points[1][0], patch, (1.0, 0.0)),
                self.emit(seqs[2][0], points[2][0], patch, (0.5, 1.0)),
            ];
            self.emit_face(&corners);
            return;
        }
        // Midpoints are shared grid points (even counts).
        let halves = [counts[0] / 2, counts[1] / 2, counts[2] / 2];
        let mid_points = [
            points[0][halves[0]],
            points[1][halves[1]],
            points[2][halves[2]],
        ];
        // Inner edges between midpoints (patch-local).
        let inner: Vec<Vec<Key>> = (0..3)
            .map(|e| {
                let edge = self.next_tri_edge;
                self.next_tri_edge += 1;
                let count = halves[e];
                let a = mid_points[e];
                let b = mid_points[(e + 1) % 3];
                (0..=count)
                    .map(|i| {
                        let t = i as f64 / count.max(1) as f64;
                        let point = self.project(&(a + (b - a) * t));
                        let key = Key::TriEdge(patch, edge, i);
                        self.emit(key, point, patch, (t, 0.5));
                        key
                    })
                    .collect()
            })
            .collect();
        let inner_points: Vec<Vec<Vector3>> = inner
            .iter()
            .map(|seq| seq.iter().map(|k| self.verts_out[self.keys[k]]).collect())
            .collect();
        // Corner tri c (loop order): side-c half (corner to mid),
        // inner edge (mid[c] to mid[prev]), side-prev half (mid to
        // corner). inner[e] runs mid[e] -> mid[(e+1)%3], so the wanted
        // edge is inner[prev] reversed.
        for c in 0..3 {
            let side_a = seqs[c][..=halves[c]].to_vec();
            let points_a = points[c][..=halves[c]].to_vec();
            let prev = (c + 2) % 3;
            let side_b = seqs[prev][halves[prev]..].to_vec();
            let points_b = points[prev][halves[prev]..].to_vec();
            let mut inner_seq = inner[prev].clone();
            let mut inner_pts = inner_points[prev].clone();
            inner_seq.reverse();
            inner_pts.reverse();
            self.fill_triangle(
                patch,
                &[side_a, inner_seq, side_b],
                &[points_a, inner_pts, points_b],
                depth + 1,
            );
        }
        // Center triangle (midpoints).
        self.fill_triangle(
            patch,
            &[inner[0].clone(), inner[1].clone(), inner[2].clone()],
            &[
                inner_points[0].clone(),
                inner_points[1].clone(),
                inner_points[2].clone(),
            ],
            depth + 1,
        );
    }

    /// Wheel spoke from a side-grid split point to the patch center:
    /// straight chord, interior points projected (endpoints exact, so
    /// first-writer order against the side grids is harmless).
    fn spoke_sequence(
        &mut self,
        patch: usize,
        cut: usize,
        from_key: Key,
        from_point: Vector3,
        to_key: Key,
        to_point: Vector3,
        count: usize,
    ) -> (Vec<Key>, Vec<Vector3>) {
        let mut keys = Vec::with_capacity(count + 1);
        let mut points = Vec::with_capacity(count + 1);
        for i in 0..=count {
            let t = i as f64 / count.max(1) as f64;
            let (key, point) = if i == 0 {
                (from_key, from_point)
            } else if i == count {
                (to_key, to_point)
            } else {
                let point = self.project(&(from_point + (to_point - from_point) * t));
                (Key::CutPoint(patch, cut, i), point)
            };
            self.emit(key, point, patch, (t, 0.5));
            keys.push(key);
            points.push(point);
        }
        (keys, points)
    }

    /// All-quad wheel over 3 side seqs in loop order: center O plus 3
    /// spokes plus 3 Coons quads `(C_i, S_i, O, S_{i-1})`. `splits`
    /// from `wheel_splits` (shared with the solver, so plan and emit
    /// agree); spoke/center keys are patch-local cut points, split
    /// points reuse the shared side-grid keys.
    fn fill_wheel(
        &mut self,
        patch: usize,
        seqs: &[Vec<Key>; 3],
        points: &[Vec<Vector3>; 3],
        splits: [usize; 3],
        piece_base: &mut usize,
        cut_base: &mut usize,
    ) {
        let centroid = (points[0][0] + points[1][0] + points[2][0]) * (1.0 / 3.0);
        let center = self.project(&centroid);
        let center_cut = *cut_base;
        *cut_base += 1;
        let center_key = Key::CutPoint(patch, center_cut, 0);
        self.emit(center_key, center, patch, (0.5, 0.5));
        let mut spokes: Vec<(Vec<Key>, Vec<Vector3>)> = Vec::with_capacity(3);
        for i in 0..3 {
            let cut = *cut_base;
            *cut_base += 1;
            spokes.push(self.spoke_sequence(
                patch,
                cut,
                seqs[i][splits[i]],
                points[i][splits[i]],
                center_key,
                center,
                splits[(i + 1) % 3],
            ));
        }
        for i in 0..3 {
            let a = splits[i];
            let b = splits[(i + 1) % 3];
            let prev = (i + 2) % 3;
            let mut side2 = spokes[prev].0.clone();
            side2.reverse();
            let mut points2 = spokes[prev].1.clone();
            points2.reverse();
            let piece = *piece_base;
            *piece_base += 1;
            self.fill_coons(
                patch,
                piece,
                &[
                    seqs[i][..=a].to_vec(),
                    spokes[i].0.clone(),
                    side2,
                    seqs[prev][splits[prev]..].to_vec(),
                ],
                &[
                    points[i][..=a].to_vec(),
                    spokes[i].1.clone(),
                    points2,
                    points[prev][splits[prev]..].to_vec(),
                ],
                a,
                b,
            );
        }
    }

    /// All-quad digon: two quads `(A, M_0, O, M_1)` / `(B, M_1, O,
    /// M_0)` over the side midpoints (`halves` from the even-count
    /// solve), sharing spokes and center like the wheel.
    fn fill_digon_quads(
        &mut self,
        patch: usize,
        seqs: &[Vec<Key>; 2],
        points: &[Vec<Vector3>; 2],
        halves: [usize; 2],
        piece_base: &mut usize,
        cut_base: &mut usize,
    ) {
        let (m0, m1) = (halves[0], halves[1]);
        let center = self.project(&((points[0][0] + points[1][0]) * 0.5));
        let center_cut = *cut_base;
        *cut_base += 1;
        let center_key = Key::CutPoint(patch, center_cut, 0);
        self.emit(center_key, center, patch, (0.5, 0.5));
        let cut0 = *cut_base;
        *cut_base += 1;
        let cut1 = *cut_base;
        *cut_base += 1;
        let (spoke0_keys, spoke0_points) = self.spoke_sequence(
            patch,
            cut0,
            seqs[0][m0],
            points[0][m0],
            center_key,
            center,
            m1,
        );
        let (spoke1_keys, spoke1_points) = self.spoke_sequence(
            patch,
            cut1,
            seqs[1][m1],
            points[1][m1],
            center_key,
            center,
            m0,
        );
        let mut spoke0_rev = spoke0_keys.clone();
        spoke0_rev.reverse();
        let mut spoke0_rev_points = spoke0_points.clone();
        spoke0_rev_points.reverse();
        let mut spoke1_rev = spoke1_keys.clone();
        spoke1_rev.reverse();
        let mut spoke1_rev_points = spoke1_points.clone();
        spoke1_rev_points.reverse();
        for (quad_keys, quad_points, a, b) in [
            (
                [
                    seqs[0][..=m0].to_vec(),
                    spoke0_keys.clone(),
                    spoke1_rev.clone(),
                    seqs[1][m1..].to_vec(),
                ],
                [
                    points[0][..=m0].to_vec(),
                    spoke0_points.clone(),
                    spoke1_rev_points.clone(),
                    points[1][m1..].to_vec(),
                ],
                m0,
                m1,
            ),
            (
                [
                    seqs[1][..=m1].to_vec(),
                    spoke1_keys.clone(),
                    spoke0_rev.clone(),
                    seqs[0][m0..].to_vec(),
                ],
                [
                    points[1][..=m1].to_vec(),
                    spoke1_points.clone(),
                    spoke0_rev_points.clone(),
                    points[0][m0..].to_vec(),
                ],
                m1,
                m0,
            ),
        ] {
            let piece = *piece_base;
            *piece_base += 1;
            self.fill_coons(patch, piece, &quad_keys, &quad_points, a, b);
        }
    }

    /// Conflicted-quad diagonal rescue: free cut across `use_02 ? 02 :
    /// 13`, both halves wheel-filled. Splits re-derived (pure from
    /// counts); false only on a plan/emit disagreement (caller falls
    /// back to the diagonal pair), checked before any emission.
    fn fill_diag_wheels(
        &mut self,
        patch: usize,
        corners: &[usize; 4],
        seqs: &[Vec<Key>; 4],
        points: &[Vec<Vector3>; 4],
        counts: [usize; 4],
        use_02: bool,
        cut_count: usize,
        patch_faces: &[usize],
        piece_base: &mut usize,
        cut_base: &mut usize,
    ) -> bool {
        let (a, b, c, d) = if use_02 {
            (counts[0], counts[1], counts[2], counts[3])
        } else {
            (counts[1], counts[2], counts[3], counts[0])
        };
        let (Some(splits0), Some(splits1)) = (
            wheel_splits([a, b, cut_count]).filter(|s| !wheel_over_cap(*s)),
            wheel_splits([c, d, cut_count]).filter(|s| !wheel_over_cap(*s)),
        ) else {
            return false;
        };
        let (from_node, to_node) = if use_02 {
            (corners[0], corners[2])
        } else {
            (corners[1], corners[3])
        };
        let adjacency = self.cut_adjacency(patch_faces);
        let from = self.layout.graph.nodes[from_node].vertex;
        let to = self.layout.graph.nodes[to_node].vertex;
        let path = self.cut_path(&adjacency, from, to);
        let cut = *cut_base;
        *cut_base += 1;
        let cut_keys = self.cut_sequence(patch, cut, &path, cut_count, true, from_node, to_node);
        let cut_points: Vec<Vector3> = cut_keys
            .iter()
            .map(|k| self.verts_out[self.keys[k]])
            .collect();
        let mut cut_rev = cut_keys.clone();
        cut_rev.reverse();
        let mut cut_pts_rev = cut_points.clone();
        cut_pts_rev.reverse();
        let (tri0_seqs, tri0_points, tri1_seqs, tri1_points) = if use_02 {
            (
                [seqs[0].clone(), seqs[1].clone(), cut_rev],
                [points[0].clone(), points[1].clone(), cut_pts_rev.clone()],
                [cut_keys, seqs[2].clone(), seqs[3].clone()],
                [cut_points, points[2].clone(), points[3].clone()],
            )
        } else {
            (
                [seqs[1].clone(), seqs[2].clone(), cut_rev],
                [points[1].clone(), points[2].clone(), cut_pts_rev.clone()],
                [cut_keys, seqs[3].clone(), seqs[0].clone()],
                [cut_points, points[3].clone(), points[0].clone()],
            )
        };
        self.fill_wheel(
            patch,
            &tri0_seqs,
            &tri0_points,
            splits0,
            piece_base,
            cut_base,
        );
        self.fill_wheel(
            patch,
            &tri1_seqs,
            &tri1_points,
            splits1,
            piece_base,
            cut_base,
        );
        true
    }

    /// Corner-split rescue emit: cut from corner `r` to the side
    /// `r+2` split, two Coons quads. False (before any emission)
    /// when the cut degenerates.
    fn fill_corner_split(
        &mut self,
        patch: usize,
        corners: &[usize; 4],
        seqs: &[Vec<Key>; 4],
        points: &[Vec<Vector3>; 4],
        side_arcs: [usize; 4],
        rotation: usize,
        cut_count: usize,
        patch_faces: &[usize],
        piece_base: &mut usize,
        cut_base: &mut usize,
    ) -> bool {
        let r = rotation;
        let (s0, s1, s2, s3) = (
            &seqs[r],
            &seqs[(r + 1) % 4],
            &seqs[(r + 2) % 4],
            &seqs[(r + 3) % 4],
        );
        let (p0, p1, p2, p3) = (
            &points[r],
            &points[(r + 1) % 4],
            &points[(r + 2) % 4],
            &points[(r + 3) % 4],
        );
        let (a, b) = (s0.len() - 1, s1.len() - 1);
        let h = a;
        if s2.len() - 1 <= h || s3.len() - 1 <= b {
            return false;
        }
        // Opposite-equality recheck (plan/emit agreement).
        if s2.len() - 1 - h != s3.len() - 1 - b {
            return false;
        }
        let adjacency = self.cut_adjacency(patch_faces);
        let cut = *cut_base;
        *cut_base += 1;
        let Some((cut_keys, cut_points)) = self.cut_to_side_point(
            patch,
            cut,
            &adjacency,
            corners[r],
            side_arcs[(r + 2) % 4],
            s2[h],
            p2[h],
            cut_count,
        ) else {
            return false;
        };
        let mut cut_rev = cut_keys.clone();
        cut_rev.reverse();
        let mut cut_pts_rev = cut_points.clone();
        cut_pts_rev.reverse();
        let piece = *piece_base;
        *piece_base += 1;
        self.fill_coons(
            patch,
            piece,
            &[s0.clone(), s1.clone(), s2[..=h].to_vec(), cut_rev],
            &[p0.clone(), p1.clone(), p2[..=h].to_vec(), cut_pts_rev],
            a,
            b,
        );
        let hp = s2.len() - 1 - h;
        let piece = *piece_base;
        *piece_base += 1;
        self.fill_coons(
            patch,
            piece,
            &[
                s2[h..].to_vec(),
                s3[..=b].to_vec(),
                s3[b..].to_vec(),
                cut_keys,
            ],
            &[
                p2[h..].to_vec(),
                p3[..=b].to_vec(),
                p3[b..].to_vec(),
                cut_points,
            ],
            hp,
            b,
        );
        true
    }

    /// Corner-cut rescue emit: cut from corner `r` to the side `r+2`
    /// split, Coons quad plus trigon wheel. False (before any
    /// emission) on splits/cut disagreement.
    fn fill_corner_cut(
        &mut self,
        patch: usize,
        corners: &[usize; 4],
        seqs: &[Vec<Key>; 4],
        points: &[Vec<Vector3>; 4],
        side_arcs: [usize; 4],
        rotation: usize,
        cut_count: usize,
        patch_faces: &[usize],
        piece_base: &mut usize,
        cut_base: &mut usize,
    ) -> bool {
        let r = rotation;
        let (s0, s1, s2, s3) = (
            &seqs[r],
            &seqs[(r + 1) % 4],
            &seqs[(r + 2) % 4],
            &seqs[(r + 3) % 4],
        );
        let (p0, p1, p2, p3) = (
            &points[r],
            &points[(r + 1) % 4],
            &points[(r + 2) % 4],
            &points[(r + 3) % 4],
        );
        let (a, b) = (s0.len() - 1, s1.len() - 1);
        let h = a;
        if s2.len() - 1 <= h {
            return false;
        }
        let triple = [s2.len() - 1 - h, s3.len() - 1, cut_count];
        let Some(splits) = wheel_splits(triple).filter(|s| !wheel_over_cap(*s)) else {
            return false;
        };
        let adjacency = self.cut_adjacency(patch_faces);
        let cut = *cut_base;
        *cut_base += 1;
        let Some((cut_keys, cut_points)) = self.cut_to_side_point(
            patch,
            cut,
            &adjacency,
            corners[r],
            side_arcs[(r + 2) % 4],
            s2[h],
            p2[h],
            cut_count,
        ) else {
            return false;
        };
        let mut cut_rev = cut_keys.clone();
        cut_rev.reverse();
        let mut cut_pts_rev = cut_points.clone();
        cut_pts_rev.reverse();
        let piece = *piece_base;
        *piece_base += 1;
        self.fill_coons(
            patch,
            piece,
            &[s0.clone(), s1.clone(), s2[..=h].to_vec(), cut_rev],
            &[p0.clone(), p1.clone(), p2[..=h].to_vec(), cut_pts_rev],
            a,
            b,
        );
        self.fill_wheel(
            patch,
            &[s2[h..].to_vec(), s3.clone(), cut_keys],
            &[p2[h..].to_vec(), p3.clone(), cut_points],
            splits,
            piece_base,
            cut_base,
        );
        true
    }

    /// Fallback: greedy tri pairing (two tris one quad) plus
    /// 3-quad subdivision for unpaired leftovers (all-quad, covers
    /// every face). Manifold except at pair/subdiv interfaces
    /// (T-junctions there, few); cracked against structured
    /// neighbors (rare by construction).
    fn fill_fallback(&mut self, patch: usize, patch_faces: &[usize]) {
        self.stats.fallback_faces += patch_faces.len();
        if patch_faces.is_empty() {
            return;
        }
        // In-patch edge -> face indices.
        let mut edge_faces: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
        for (idx, &face) in patch_faces.iter().enumerate() {
            let tri = self.topology.triangle(face);
            for i in 0..3 {
                let (a, b) = (tri[i], tri[(i + 1) % 3]);
                edge_faces
                    .entry((a.min(b), a.max(b)))
                    .or_default()
                    .push(idx);
            }
        }
        // Neighbors sharing exactly one edge, with pair scores.
        let mut neighbors: Vec<Vec<(usize, PairScore)>> = vec![Vec::new(); patch_faces.len()];
        for (idx, &face) in patch_faces.iter().enumerate() {
            let tri = self.topology.triangle(face);
            let mut shared: BTreeMap<usize, Vec<(usize, usize)>> = BTreeMap::new();
            for i in 0..3 {
                let (a, b) = (tri[i], tri[(i + 1) % 3]);
                let key = (a.min(b), a.max(b));
                if let Some(faces) = edge_faces.get(&key) {
                    for &other in faces {
                        if other != idx {
                            shared.entry(other).or_default().push(key);
                        }
                    }
                }
            }
            for (other, edges) in shared {
                if edges.len() == 1 {
                    let score = self.pair_score(face, patch_faces[other], edges[0]);
                    neighbors[idx].push((other, score));
                }
            }
            neighbors[idx].sort_by_key(|&(other, _)| other);
        }
        let partners = greedy_pairs(&neighbors, false);
        // All-or-nothing per patch: mixed pair/subdiv interfaces
        // T-junction (each stranded leftover cracks against its
        // paired neighbors), so partially-paired patches subdivide
        // uniformly (manifold) and only fully-paired patches emit
        // pair quads.
        let complete = partners.iter().all(Option::is_some);
        for (idx, &face) in patch_faces.iter().enumerate() {
            if complete {
                if let Some(other) = partners[idx] {
                    if other > idx {
                        self.emit_paired_quad(patch, face, patch_faces[other]);
                    }
                }
            } else {
                self.emit_subdiv_tri(patch, face);
            }
        }
    }

    /// Pair quality for two tris sharing one edge: flatness (bentness
    /// of the dihedral), roundness (min union-quad corner), then
    /// shared-edge length. Degenerate tris score worst (never pair).
    fn pair_score(&self, face_a: usize, face_b: usize, edge: (usize, usize)) -> PairScore {
        let tri_a = self.topology.triangle(face_a);
        let tri_b = self.topology.triangle(face_b);
        let odd = |tri: &[usize]| -> usize {
            *tri.iter()
                .find(|&&v| v != edge.0 && v != edge.1)
                .unwrap_or(&tri[0])
        };
        let (odd_a, odd_b) = (odd(tri_a), odd(tri_b));
        let normal = |tri: &[usize]| -> Option<Vector3> {
            let e0 = self.verts[tri[1]] - self.verts[tri[0]];
            let e1 = self.verts[tri[2]] - self.verts[tri[0]];
            let n = Vector3::cross_product(&e0, &e1);
            if n.length() <= 1e-18 {
                None
            } else {
                Some(n.normalized())
            }
        };
        let (Some(na), Some(nb)) = (normal(tri_a), normal(tri_b)) else {
            return PairScore::worst();
        };
        let bent_um = ((1.0 - Vector3::dot_product(&na, &nb)) * 1e6).max(0.0) as u64;
        // Union quad loop: odd_a -> x -> odd_b -> y.
        let corners = [
            self.verts[odd_a],
            self.verts[edge.0],
            self.verts[odd_b],
            self.verts[edge.1],
        ];
        let mut min_deg = f64::MAX;
        for k in 0..4 {
            let angle = corner_angle_deg(&corners[(k + 3) % 4], &corners[k], &corners[(k + 1) % 4]);
            min_deg = min_deg.min(angle);
        }
        let edge_um = ((self.verts[edge.0] - self.verts[edge.1]).length() * 1e6) as u64;
        PairScore {
            bent_um,
            min_cdeg: (min_deg * 100.0) as i64,
            edge_um,
        }
    }

    /// One quad over two paired tris (corners only; the shared edge
    /// becomes the interior diagonal).
    fn emit_paired_quad(&mut self, patch: usize, face_a: usize, face_b: usize) {
        let tri_a = self.topology.triangle(face_a);
        let tri_b = self.topology.triangle(face_b);
        let edges_a = [
            (tri_a[0].min(tri_a[1]), tri_a[0].max(tri_a[1])),
            (tri_a[1].min(tri_a[2]), tri_a[1].max(tri_a[2])),
            (tri_a[2].min(tri_a[0]), tri_a[2].max(tri_a[0])),
        ];
        let edges_b = [
            (tri_b[0].min(tri_b[1]), tri_b[0].max(tri_b[1])),
            (tri_b[1].min(tri_b[2]), tri_b[1].max(tri_b[2])),
            (tri_b[2].min(tri_b[0]), tri_b[2].max(tri_b[0])),
        ];
        let shared: Vec<(usize, usize)> = edges_a
            .iter()
            .filter(|e| edges_b.contains(e))
            .copied()
            .collect();
        if shared.len() != 1 {
            // Degenerate adjacency (cannot happen: pairing requires
            // exactly one shared edge): subdivide both, stay all-quad.
            self.emit_subdiv_tri(patch, face_a);
            self.emit_subdiv_tri(patch, face_b);
            return;
        }
        let (x, y) = shared[0];
        let odd_a = tri_a
            .iter()
            .find(|&&v| v != x && v != y)
            .copied()
            .unwrap_or(tri_a[0]);
        let odd_b = tri_b
            .iter()
            .find(|&&v| v != x && v != y)
            .copied()
            .unwrap_or(tri_b[0]);
        let corners = [odd_a, x, odd_b, y]
            .map(|v| self.emit(Key::FallbackVert(v), self.verts[v], patch, (0.5, 0.5)));
        self.emit_face(&corners);
    }

    /// Per-face 3-quad subdivision (unpaired leftovers).
    fn emit_subdiv_tri(&mut self, patch: usize, face: usize) {
        let tri = self.topology.triangle(face);
        let (a, b, c) = (tri[0], tri[1], tri[2]);
        let centroid = Vector3::new(
            (self.verts[a].x() + self.verts[b].x() + self.verts[c].x()) / 3.0,
            (self.verts[a].y() + self.verts[b].y() + self.verts[c].y()) / 3.0,
            (self.verts[a].z() + self.verts[b].z() + self.verts[c].z()) / 3.0,
        );
        let center = self.emit(Key::FallbackCenter(face), centroid, patch, (0.5, 0.5));
        let edges = [(a, b), (b, c), (c, a)];
        let mut mids = [0usize; 3];
        for (i, &(u, v)) in edges.iter().enumerate() {
            let key = if u < v {
                Key::FallbackEdge(u, v)
            } else {
                Key::FallbackEdge(v, u)
            };
            let point = (self.verts[u] + self.verts[v]) * 0.5;
            mids[i] = self.emit(key, point, patch, (0.5, 0.5));
        }
        let corners =
            [a, b, c].map(|v| self.emit(Key::FallbackVert(v), self.verts[v], patch, (0.5, 0.5)));
        self.emit_face(&[corners[0], mids[0], center, mids[2]]);
        self.emit_face(&[corners[1], mids[1], center, mids[0]]);
        self.emit_face(&[corners[2], mids[2], center, mids[1]]);
    }
}

/// Corner angle at `b` (degrees) for 3D points.
fn corner_angle_deg(a: &Vector3, b: &Vector3, c: &Vector3) -> f64 {
    let v1 = *a - *b;
    let v2 = *c - *b;
    let n1 = v1.length();
    let n2 = v2.length();
    if n1 <= 0.0 || n2 <= 0.0 {
        return 0.0;
    }
    (Vector3::dot_product(&v1, &v2) / (n1 * n2))
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees()
}

/// Tri-pair quality: flatness, roundness, shared-edge length.
#[derive(Clone, Copy, Debug)]
struct PairScore {
    /// Dihedral bentness ((1 - cos) * 1e6; 0 flat). Ascending.
    bent_um: u64,
    /// Min union-quad corner (centidegrees). Descending.
    min_cdeg: i64,
    /// Shared-edge length (* 1e6). Ascending.
    edge_um: u64,
}

impl PairScore {
    fn worst() -> Self {
        Self {
            bent_um: u64::MAX,
            min_cdeg: i64::MIN,
            edge_um: u64::MAX,
        }
    }

    fn better_than(&self, other: &Self) -> bool {
        (self.bent_um, std::cmp::Reverse(self.min_cdeg), self.edge_um)
            < (
                other.bent_um,
                std::cmp::Reverse(other.min_cdeg),
                other.edge_um,
            )
    }
}

/// Greedy tri pairing: faces in order, each unpaired face takes its
/// best-scoring unpaired neighbor. Neighbor lists must be ascending
/// (strict improvement keeps the smallest index on score ties).
/// `degree_first` processes constrained faces (fewest neighbors)
/// first, stranding fewer (better cardinality, worse quality order).
/// Returns the partner index per face (None unpaired).
fn greedy_pairs(neighbors: &[Vec<(usize, PairScore)>], degree_first: bool) -> Vec<Option<usize>> {
    let mut partner: Vec<Option<usize>> = vec![None; neighbors.len()];
    let mut order: Vec<usize> = (0..neighbors.len()).collect();
    if degree_first {
        order.sort_by_key(|&face| (neighbors[face].len(), face));
    }
    for face in order {
        if partner[face].is_some() {
            continue;
        }
        let mut best: Option<(usize, PairScore)> = None;
        for &(other, score) in &neighbors[face] {
            if partner[other].is_some() {
                continue;
            }
            let better = match best {
                None => true,
                Some((_, current)) => score.better_than(&current),
            };
            if better {
                best = Some((other, score));
            }
        }
        if let Some((other, _)) = best {
            partner[face] = Some(other);
            partner[other] = Some(face);
        }
    }
    partner
}

/// Initial arc counts: arc length over local quad width. Returns
/// the counts plus the number clamped at the 256 cap.
/// Raw (unrounded) side counts: arc length over local quad width.
/// Clamped to [1, 256] like the integer counts (beyond 256 quads a
/// side means degenerate tracing). The agreement solver resolves
/// integers near these; `initial_counts` rounds them for patches
/// that take any counts (triangles, digons, monogons, unclaimed).
fn raw_counts(
    layout: &Layout,
    topology: &SurfaceMesh,
    verts: &[Vector3],
    face_width: &[f64],
) -> (Vec<f64>, usize) {
    // Edge -> faces for the local width lookup.
    let mut edge_faces: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
    for f in 0..topology.face_count() {
        let tri = topology.triangle(f);
        for i in 0..3 {
            let (a, b) = (tri[i], tri[(i + 1) % 3]);
            edge_faces.entry((a.min(b), a.max(b))).or_default().push(f);
        }
    }
    let mut clamped = 0usize;
    let counts = layout
        .graph
        .arcs
        .iter()
        .enumerate()
        .map(|(arc, _)| {
            let path = &layout.graph.arcs[arc].path;
            let mut length = 0.0;
            let mut width_sum = 0.0;
            let mut width_count = 0usize;
            for window in path.windows(2) {
                length += (verts[window[1]] - verts[window[0]]).length();
                if let Some(faces) =
                    edge_faces.get(&(window[0].min(window[1]), window[0].max(window[1])))
                {
                    for &face in faces {
                        width_sum += face_width[face];
                        width_count += 1;
                    }
                }
            }
            let width = if width_count > 0 {
                width_sum / width_count as f64
            } else {
                1.0
            };
            let width = width.max(1e-9);
            // Capped: a side beyond 256 quads means degenerate tracing,
            // and grids grow quadratically in the count.
            let raw = length / width;
            if raw > 256.0 {
                clamped += 1;
            }
            raw.clamp(1.0, 256.0)
        })
        .collect();
    (counts, clamped)
}

fn initial_counts(
    layout: &Layout,
    topology: &SurfaceMesh,
    verts: &[Vector3],
    face_width: &[f64],
) -> (Vec<usize>, usize, Vec<f64>) {
    let (raw, clamped) = raw_counts(layout, topology, verts, face_width);
    let counts = raw.iter().map(|r| r.round() as usize).collect();
    (counts, clamped, raw)
}

/// Conflicted-quad agreement: opposite sides agree (`n0 == n2`,
/// `n1 == n3`) near the raw votes, pins respected. Same class
/// machinery as the ngon solver, one fixed equality pattern.
fn solve_quad_counts(
    raw: &[f64; 4],
    arc: &[usize; 4],
    arc_fixed: &[Option<usize>],
    tol: f64,
    capped: &mut bool,
) -> Option<[usize; 4]> {
    let fixed: Vec<Option<usize>> = arc.iter().map(|&a| arc_fixed[a]).collect();
    let mut classes = CountClasses::new(raw);
    for i in 0..4 {
        for j in (i + 1)..4 {
            if arc[i] == arc[j] {
                classes.union(i, j);
            }
        }
    }
    classes.union(0, 2);
    classes.union(1, 3);
    let assignment = resolve_classes(&mut classes, 4, &fixed, tol)?;
    if grid_over_cap(assignment[0], assignment[1]) {
        *capped = true;
        return None;
    }
    Some([assignment[0], assignment[1], assignment[2], assignment[3]])
}

/// Quad solve with tol escalation (same knob as the other solvers).
fn solve_quad_counts_best(
    raw: &[f64; 4],
    arc: &[usize; 4],
    arc_fixed: &[Option<usize>],
    capped: &mut bool,
) -> Option<[usize; 4]> {
    for tol in agreement_tols() {
        if let Some(assignment) = solve_quad_counts(raw, arc, arc_fixed, tol, capped) {
            return Some(assignment);
        }
    }
    None
}

/// Priority propagation over quad patches: opposite sides agree on
/// one count (first writer wins, patch-id order). Returns the
/// per-patch deferred flags (both-set-unequal pairs, solved by
/// agreement instead) plus the per-arc claimed flags. Pre-claimed
/// arcs (constrained small patches solve first) seed the set flags
/// and propagate by copy, so propagation never overwrites them.
/// Fresh pairs minimize L1 to the raw votes minus bonus per
/// quad-opposite pin match (later quads match earlier pins across
/// their neighbors instead of forcing a third value that defers a
/// downstream quad).
fn propagate_counts(
    layout: &Layout,
    counts: &mut [usize],
    arc_fixed: &mut [Option<usize>],
    raw: &[f64],
) -> (Vec<bool>, Vec<bool>) {
    let mut set: Vec<bool> = arc_fixed.iter().map(Option::is_some).collect();
    let mut deferred = vec![false; layout.patches.len()];
    for (patch_id, patch) in layout.patches.iter().enumerate() {
        if !patch.usable || patch.sides.len() != 4 {
            continue;
        }
        for &(first, second) in &[(0usize, 2usize), (1usize, 3usize)] {
            let a = patch.sides[first].arc;
            let b = patch.sides[second].arc;
            match (set[a], set[b]) {
                (false, false) => {
                    let average = ((counts[a] + counts[b]) as f64 / 2.0).round().max(1.0) as usize;
                    let targets_a = quad_opposite_targets(layout, arc_fixed, a);
                    let targets_b = quad_opposite_targets(layout, arc_fixed, b);
                    let mut candidates = vec![average];
                    candidates.extend(targets_a.iter().copied());
                    candidates.extend(targets_b.iter().copied());
                    candidates.sort_unstable();
                    candidates.dedup();
                    let mut value = average;
                    // (score, distance-to-average, value): ascending.
                    let mut best_key = (f64::MAX, usize::MAX, usize::MAX);
                    for v in candidates {
                        if v == 0 || v > 256 {
                            continue;
                        }
                        let matches = targets_a.iter().filter(|&&t| t == v).count()
                            + targets_b.iter().filter(|&&t| t == v).count();
                        let l1 = (v as f64 - raw[a]).abs() + (v as f64 - raw[b]).abs();
                        let key = (l1 - MATCH_BONUS * matches as f64, v.abs_diff(average), v);
                        let better = key.0 < best_key.0
                            || (key.0 == best_key.0
                                && (key.1 < best_key.1
                                    || (key.1 == best_key.1 && key.2 < best_key.2)));
                        if better {
                            best_key = key;
                            value = v;
                        }
                    }
                    counts[a] = value;
                    counts[b] = value;
                    set[a] = true;
                    set[b] = true;
                }
                (true, false) => {
                    counts[b] = counts[a];
                    set[b] = true;
                }
                (false, true) => {
                    counts[a] = counts[b];
                    set[a] = true;
                }
                (true, true) => {
                    if counts[a] != counts[b] {
                        deferred[patch_id] = true;
                        break;
                    }
                }
            }
        }
    }
    // Merge propagation claims back (pre-claimed arcs keep their
    // values: copy arms only write the unset side).
    for (arc, &is_set) in set.iter().enumerate() {
        if is_set {
            arc_fixed[arc] = Some(counts[arc]);
        }
    }
    (deferred, set)
}

/// Unify face winding per connected component (BFS over shared
/// edges): patch loops orient arbitrarily, so grids come out mixed.
fn unify_winding(faces: &mut [Vec<usize>]) {
    // Edge -> faces using it (undirected).
    let mut edge_faces: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
    for (face_id, face) in faces.iter().enumerate() {
        for i in 0..face.len() {
            let a = face[i];
            let b = face[(i + 1) % face.len()];
            edge_faces
                .entry((a.min(b), a.max(b)))
                .or_default()
                .push(face_id);
        }
    }
    // Face adjacency through shared edges. Only the first two faces
    // per edge pair up (manifold assumption): non-manifold soup with
    // dozens of faces on one edge would blow up combinatorially, and
    // those faces orient through their other edges anyway.
    let mut adjacent: Vec<Vec<usize>> = vec![Vec::new(); faces.len()];
    for face_list in edge_faces.values() {
        if face_list.len() >= 2 {
            let (a, b) = (face_list[0], face_list[1]);
            adjacent[a].push(b);
            adjacent[b].push(a);
        }
    }
    for neighbors in &mut adjacent {
        neighbors.sort_unstable();
        neighbors.dedup();
    }
    let mut oriented = vec![false; faces.len()];
    for seed in 0..faces.len() {
        if oriented[seed] {
            continue;
        }
        oriented[seed] = true;
        let mut stack = vec![seed];
        while let Some(face_id) = stack.pop() {
            for &next in &adjacent[face_id] {
                if oriented[next] {
                    continue;
                }
                oriented[next] = true;
                // The shared edge must run opposite directions.
                let face = faces[face_id].clone();
                let other = faces[next].clone();
                let mut shared: Option<(usize, usize)> = None;
                'search: for i in 0..face.len() {
                    let a = face[i];
                    let b = face[(i + 1) % face.len()];
                    for j in 0..other.len() {
                        let c = other[j];
                        let d = other[(j + 1) % other.len()];
                        if (a == c && b == d) || (a == d && b == c) {
                            shared = Some((a, b));
                            break 'search;
                        }
                    }
                }
                if let Some((a, b)) = shared {
                    // Find the other face's direction on (a, b).
                    let mut same_direction = false;
                    for j in 0..other.len() {
                        if other[j] == a && other[(j + 1) % other.len()] == b {
                            same_direction = true;
                            break;
                        }
                    }
                    if same_direction {
                        faces[next].reverse();
                    }
                }
                stack.push(next);
            }
        }
    }
}

/// Phase-1 plan per patch: what phase 2 emits. Counts are final
/// after phase 1 (phase 2 never writes them), so shared arcs cannot
/// disagree across patches (the old interleave emitted early patches
/// before later patches re-solved their arcs: 12 cracked arcs on
/// beast@1000).
#[derive(Clone, Copy)]
enum Phase1Plan {
    /// Structured fill with the solved counts (quads route at emit:
    /// coons, diagonal-wheel rescue, or diagonal pair).
    Emit,
    /// Old triangle fill with the current counts (all-quad solve
    /// failed but triangles take any counts).
    Tri,
    /// Per-face fallback with a named reason.
    Fallback(&'static str),
}

/// Fill one island's layout.
pub(crate) fn fill_layout(
    topology: &SurfaceMesh,
    verts: &[Vector3],
    layout: &Layout,
    face_width: &[f64],
    degenerate_area: f64,
) -> FillOutput {
    let (mut counts, clamped_arcs, raw) = initial_counts(layout, topology, verts, face_width);
    // Claimed arcs pin the agreement solvers (constrained small
    // patches first, then quad propagation, then ngons last:
    // first-writer-wins in that order).
    let mut arc_fixed: Vec<Option<usize>> = vec![None; counts.len()];
    // Phase 1: solve every patch's counts (pure arithmetic on raw
    // votes; no geometry). Writes counts and claims; phase 2 only
    // reads.
    let mut plans = vec![Phase1Plan::Emit; layout.patches.len()];
    let mut solve_capped_patches = 0usize;
    // Phase 1a: constrained small patches claim first (their wheels
    // need exact inequalities; quads and ngons adapt via pins).
    for (patch_id, patch) in layout.patches.iter().enumerate() {
        if !patch.usable || patch.sides.is_empty() {
            plans[patch_id] = Phase1Plan::Fallback("fallback-unusable");
            continue;
        }
        if patch.sides.len() == 3 {
            let arc = [patch.sides[0].arc, patch.sides[1].arc, patch.sides[2].arc];
            let raw3 = [raw[arc[0]], raw[arc[1]], raw[arc[2]]];
            let mut capped = false;
            match solve_trigon_counts_best(&raw3, &arc, &arc_fixed, &mut capped, layout) {
                Some(assignment) => {
                    for (side, &value) in assignment.iter().enumerate() {
                        counts[arc[side]] = value;
                        arc_fixed[arc[side]] = Some(value);
                    }
                }
                None => {
                    plans[patch_id] = Phase1Plan::Tri;
                    if capped {
                        solve_capped_patches += 1;
                    }
                }
            }
        } else if patch.sides.len() == 2 {
            let arc = [patch.sides[0].arc, patch.sides[1].arc];
            let raw2 = [raw[arc[0]], raw[arc[1]]];
            let mut capped = false;
            match solve_digon_counts_best(&raw2, &arc, &arc_fixed, &mut capped, layout) {
                Some(assignment) => {
                    for (side, &value) in assignment.iter().enumerate() {
                        counts[arc[side]] = value;
                        arc_fixed[arc[side]] = Some(value);
                    }
                }
                None => {
                    plans[patch_id] = Phase1Plan::Tri;
                    if capped {
                        solve_capped_patches += 1;
                    }
                }
            }
        } else if patch.sides.len() <= 2 {
            let arc = patch.sides[0].arc;
            let mut capped = false;
            match solve_monogon_counts_best(raw[arc], arc, &arc_fixed, &mut capped, layout) {
                Some(subsides) => {
                    let sum: usize = subsides.iter().sum();
                    counts[arc] = sum;
                    arc_fixed[arc] = Some(sum);
                }
                None => {
                    plans[patch_id] = Phase1Plan::Tri;
                    if capped {
                        solve_capped_patches += 1;
                    }
                }
            }
        }
    }
    // Phase 1b: quad propagation over the small-patch claims.
    let (deferred, _claimed) = propagate_counts(layout, &mut counts, &mut arc_fixed, &raw);
    // Phase 1c: deferred quads (agreement) and ngons in patch-id
    // order. Ngons solve last: their equality-class search is the
    // most flexible guest.
    for (patch_id, patch) in layout.patches.iter().enumerate() {
        if !patch.usable || patch.sides.is_empty() {
            continue;
        }
        match patch.sides.len() {
            4 if deferred[patch_id] => {
                // Agreement rescue for the coons path (writes equal
                // opposites); failure stays optimistic (phase 2
                // routes to the diagonal-wheel rescue or the pair).
                let arc = [
                    patch.sides[0].arc,
                    patch.sides[1].arc,
                    patch.sides[2].arc,
                    patch.sides[3].arc,
                ];
                let raw4 = [raw[arc[0]], raw[arc[1]], raw[arc[2]], raw[arc[3]]];
                let mut capped = false;
                if let Some(assignment) =
                    solve_quad_counts_best(&raw4, &arc, &arc_fixed, &mut capped)
                {
                    for (side, &value) in assignment.iter().enumerate() {
                        counts[arc[side]] = value;
                        arc_fixed[arc[side]] = Some(value);
                    }
                } else if capped {
                    solve_capped_patches += 1;
                }
            }
            4 | 3 | 2 | 1 => {}
            _ => {
                let side_arcs: Vec<usize> = patch.sides.iter().map(|side| side.arc).collect();
                let side_raw: Vec<f64> = side_arcs.iter().map(|&arc| raw[arc]).collect();
                let mut solve_capped = false;
                let solved =
                    solve_ngon_counts_best(&side_raw, &side_arcs, &arc_fixed, &mut solve_capped);
                match solved {
                    Some(assignment) => {
                        for (side, &value) in assignment.iter().enumerate() {
                            counts[side_arcs[side]] = value;
                            arc_fixed[side_arcs[side]] = Some(value);
                        }
                    }
                    None => {
                        if solve_capped {
                            plans[patch_id] = Phase1Plan::Fallback("fallback-ngon-capped");
                            solve_capped_patches += 1;
                        } else {
                            plans[patch_id] = Phase1Plan::Fallback("fallback-ngon-noplan");
                        }
                    }
                }
            }
        }
    }
    let mut filler = Filler {
        topology,
        verts,
        layout,
        counts,
        verts_out: Vec::new(),
        faces_out: Vec::new(),
        uv_patch: Vec::new(),
        uv_local: Vec::new(),
        keys: BTreeMap::new(),
        stats: FillStats::default(),
        degenerate_area,
        next_tri_edge: 0,
        projection: PatchProjection::build(topology, verts, &[]),
        proj_seen: Vec::new(),
        proj_stamp: 1,
    };
    filler.stats.patches = layout.patches.len();
    filler.stats.clamped_arcs = clamped_arcs;
    filler.stats.capped_patches = solve_capped_patches;
    let debug = std::env::var_os("RETOPO_PATCH_DEBUG").is_some();
    // Phase 2: emit every patch from the final counts (read-only).
    for (patch_id, patch) in layout.patches.iter().enumerate() {
        if patch.usable && !patch.sides.is_empty() {
            filler.stats.usable_patches += 1;
        }
        if let Phase1Plan::Fallback(reason) = plans[patch_id] {
            if debug {
                eprintln!(
                    "patch {patch_id}: sides={} faces={} outcome={reason}",
                    patch.sides.len(),
                    patch.faces.len(),
                );
            }
            filler.fill_fallback(patch_id, &patch.faces);
            continue;
        }
        // Per-patch outcome for RETOPO_PATCH_DEBUG (quantization tuning).
        let mut outcome = "structured";
        // Structured fills project onto their own patch only (the
        // fallback never projects).
        filler.begin_patch(&patch.faces);
        match patch.sides.len() {
            4 => {
                let mut seqs: Vec<Vec<Key>> = Vec::new();
                let mut points: Vec<Vec<Vector3>> = Vec::new();
                for position in 0..4 {
                    let (keys, positions) = filler.side_sequence(patch_id, position);
                    seqs.push(keys);
                    points.push(positions);
                }
                let seqs4 = [
                    seqs[0].clone(),
                    seqs[1].clone(),
                    seqs[2].clone(),
                    seqs[3].clone(),
                ];
                let points4 = [
                    points[0].clone(),
                    points[1].clone(),
                    points[2].clone(),
                    points[3].clone(),
                ];
                let counts4 = [
                    seqs4[0].len() - 1,
                    seqs4[1].len() - 1,
                    seqs4[2].len() - 1,
                    seqs4[3].len() - 1,
                ];
                let (a, b) = (counts4[0], counts4[1]);
                if counts4[2] == a && counts4[3] == b {
                    if grid_over_cap(a, b) {
                        outcome = "fallback-capped-grid";
                        filler.stats.capped_patches += 1;
                        filler.fill_fallback(patch_id, &patch.faces);
                    } else {
                        filler.fill_coons(patch_id, 0, &seqs4, &points4, a, b);
                    }
                } else {
                    // Conflicted opposites: rescue ladder (all-quad),
                    // diagonal pair only as last resort. Each emitter
                    // verifies before emitting, so chaining is safe.
                    filler.stats.conflicted_quads += 1;
                    let corners4 = [
                        filler.side_entry_node(patch_id, 0),
                        filler.side_entry_node(patch_id, 1),
                        filler.side_entry_node(patch_id, 2),
                        filler.side_entry_node(patch_id, 3),
                    ];
                    let side_arcs4 = [
                        patch.sides[0].arc,
                        patch.sides[1].arc,
                        patch.sides[2].arc,
                        patch.sides[3].arc,
                    ];
                    let mut piece = 1usize;
                    let mut cut_base = 0usize;
                    let mut rescued: Option<&'static str> = None;
                    if rescued.is_none() {
                        if let Some((use_02, cut)) = solve_diag_cut(counts4) {
                            if filler.fill_diag_wheels(
                                patch_id,
                                &corners4,
                                &seqs4,
                                &points4,
                                counts4,
                                use_02,
                                cut,
                                &patch.faces,
                                &mut piece,
                                &mut cut_base,
                            ) {
                                rescued = Some("diagwheel");
                            }
                        }
                    }
                    if rescued.is_none() {
                        if let Some((rotation, cut)) = solve_corner_split(counts4) {
                            if filler.fill_corner_split(
                                patch_id,
                                &corners4,
                                &seqs4,
                                &points4,
                                side_arcs4,
                                rotation,
                                cut,
                                &patch.faces,
                                &mut piece,
                                &mut cut_base,
                            ) {
                                rescued = Some("cornersplit");
                            }
                        }
                    }
                    if rescued.is_none() {
                        if let Some((rotation, cut)) = solve_corner_cut(counts4) {
                            if filler.fill_corner_cut(
                                patch_id,
                                &corners4,
                                &seqs4,
                                &points4,
                                side_arcs4,
                                rotation,
                                cut,
                                &patch.faces,
                                &mut piece,
                                &mut cut_base,
                            ) {
                                rescued = Some("cornercut");
                            }
                        }
                    }
                    match rescued {
                        Some(kind) => {
                            outcome = kind;
                        }
                        None => {
                            outcome = "tri-quad";
                            fill_diagonal(&mut filler, patch_id);
                        }
                    }
                }
            }
            3 => {
                let mut seqs: Vec<Vec<Key>> = Vec::new();
                let mut points: Vec<Vec<Vector3>> = Vec::new();
                for position in 0..3 {
                    let (keys, positions) = filler.side_sequence(patch_id, position);
                    seqs.push(keys);
                    points.push(positions);
                }
                let seqs3 = [seqs[0].clone(), seqs[1].clone(), seqs[2].clone()];
                let points3 = [points[0].clone(), points[1].clone(), points[2].clone()];
                let wheel = match plans[patch_id] {
                    Phase1Plan::Emit => {
                        let counts = [seqs3[0].len() - 1, seqs3[1].len() - 1, seqs3[2].len() - 1];
                        wheel_splits(counts).filter(|s| !wheel_over_cap(*s))
                    }
                    _ => None,
                };
                match wheel {
                    Some(splits) => {
                        let mut piece = 1usize;
                        let mut cut = 0usize;
                        filler.fill_wheel(patch_id, &seqs3, &points3, splits, &mut piece, &mut cut);
                    }
                    None => {
                        outcome = if matches!(plans[patch_id], Phase1Plan::Emit) {
                            "tri-mismatch"
                        } else {
                            "tri-trigon"
                        };
                        filler.fill_triangle(patch_id, &seqs3, &points3, 0);
                    }
                }
            }
            2 => {
                let mut seqs: Vec<Vec<Key>> = Vec::new();
                let mut points: Vec<Vec<Vector3>> = Vec::new();
                for position in 0..2 {
                    let (keys, positions) = filler.side_sequence(patch_id, position);
                    seqs.push(keys);
                    points.push(positions);
                }
                let seqs2 = [seqs[0].clone(), seqs[1].clone()];
                let points2 = [points[0].clone(), points[1].clone()];
                let halves = match plans[patch_id] {
                    Phase1Plan::Emit => {
                        let n = [seqs2[0].len() - 1, seqs2[1].len() - 1];
                        (n[0] % 2 == 0
                            && n[1] % 2 == 0
                            && n[0] >= 2
                            && n[1] >= 2
                            && !grid_over_cap(n[0] / 2, n[1] / 2))
                        .then(|| [n[0] / 2, n[1] / 2])
                    }
                    _ => None,
                };
                match halves {
                    Some(halves) => {
                        let mut piece = 1usize;
                        let mut cut = 0usize;
                        filler.fill_digon_quads(
                            patch_id, &seqs2, &points2, halves, &mut piece, &mut cut,
                        );
                    }
                    None => {
                        outcome = if matches!(plans[patch_id], Phase1Plan::Emit) {
                            "tri-mismatch"
                        } else {
                            "tri-digon"
                        };
                        if !fill_digon(&mut filler, patch_id) {
                            outcome = "fallback-digon";
                            filler.fill_fallback(patch_id, &patch.faces);
                        }
                    }
                }
            }
            1 => {
                let (keys, positions) = filler.side_sequence(patch_id, 0);
                let wheel_data = match plans[patch_id] {
                    Phase1Plan::Emit => {
                        // Re-derive subsides summing to the solved
                        // count (any wheel-valid split works: the
                        // interior is patch-local, the side grids are
                        // shared by index).
                        let arc = patch.sides[0].arc;
                        let mut _capped = false;
                        solve_monogon_counts_best(raw[arc], arc, &arc_fixed, &mut _capped, layout)
                            .filter(|sub| sub.iter().sum::<usize>() == keys.len() - 1)
                            .and_then(|sub| {
                                wheel_splits(sub)
                                    .filter(|s| !wheel_over_cap(*s))
                                    .map(|splits| (sub, splits))
                            })
                    }
                    _ => None,
                };
                match wheel_data {
                    Some((sub, splits)) => {
                        let (u0, u1) = (sub[0], sub[1]);
                        let wseqs = [
                            keys[..=u0].to_vec(),
                            keys[u0..=u0 + u1].to_vec(),
                            keys[u0 + u1..].to_vec(),
                        ];
                        let wpoints = [
                            positions[..=u0].to_vec(),
                            positions[u0..=u0 + u1].to_vec(),
                            positions[u0 + u1..].to_vec(),
                        ];
                        let mut piece = 1usize;
                        let mut cut = 0usize;
                        filler.fill_wheel(patch_id, &wseqs, &wpoints, splits, &mut piece, &mut cut);
                    }
                    None => {
                        outcome = if matches!(plans[patch_id], Phase1Plan::Emit) {
                            "tri-mismatch"
                        } else {
                            "tri-monogon"
                        };
                        if !fill_monogon(&mut filler, patch_id) {
                            outcome = "fallback-monogon";
                            filler.fill_fallback(patch_id, &patch.faces);
                        }
                    }
                }
            }
            _ => {
                // Counts solved in phase 1; emit only. An emit
                // failure keeps the solved counts (neighbors already
                // agree) and falls back.
                let corners: Vec<usize> = (0..patch.sides.len())
                    .map(|position| filler.side_entry_node(patch_id, position))
                    .collect();
                let mut seqs: Vec<Vec<Key>> = Vec::new();
                let mut points: Vec<Vec<Vector3>> = Vec::new();
                for position in 0..patch.sides.len() {
                    let (keys, positions) = filler.side_sequence(patch_id, position);
                    seqs.push(keys);
                    points.push(positions);
                }
                let mut piece = 1usize;
                let mut cut = 0usize;
                let mut capped = false;
                if !filler.fill_ngon(
                    patch_id,
                    &corners,
                    &seqs,
                    &points,
                    &patch.faces,
                    &mut piece,
                    &mut cut,
                    &mut capped,
                ) {
                    if capped {
                        outcome = "fallback-ngon-capped";
                        filler.stats.capped_patches += 1;
                    } else {
                        outcome = "fallback-ngon-emitfail";
                    }
                    filler.fill_fallback(patch_id, &patch.faces);
                }
            }
        }
        if debug {
            eprintln!(
                "patch {patch_id}: sides={} faces={} outcome={outcome}",
                patch.sides.len(),
                patch.faces.len(),
            );
        }
    }
    let mut output = FillOutput {
        verts: filler.verts_out,
        faces: filler.faces_out,
        uv_patch: filler.uv_patch,
        uv_local: filler.uv_local,
        stats: filler.stats,
    };
    unify_winding(&mut output.faces);
    output
}

/// Diagonal triangle pair for a conflicted quad patch (rare): two
/// flat triangles over the corners. Cracked against gridded
/// neighbors by one sagitta; reported in the stats.
fn fill_diagonal(filler: &mut Filler, patch: usize) {
    let corners: Vec<usize> = (0..4).map(|p| filler.side_entry_node(patch, p)).collect();
    let positions: Vec<Vector3> = corners
        .iter()
        .map(|&node| filler.verts[filler.layout.graph.nodes[node].vertex])
        .collect();
    // Shorter diagonal (deterministic tie-break: 0-2).
    let diagonal_02 = (positions[2] - positions[0]).length_squared();
    let diagonal_13 = (positions[3] - positions[1]).length_squared();
    let ids: Vec<usize> = corners
        .iter()
        .enumerate()
        .map(|(i, &node)| filler.emit(Key::Node(node), positions[i], patch, (0.5, 0.5)))
        .collect();
    if diagonal_02 <= diagonal_13 {
        filler.emit_face(&[ids[0], ids[1], ids[2]]);
        filler.emit_face(&[ids[0], ids[2], ids[3]]);
    } else {
        filler.emit_face(&[ids[0], ids[1], ids[3]]);
        filler.emit_face(&[ids[1], ids[2], ids[3]]);
    }
}

/// Digon (two sides) to triangle via a midpoint split of the longest
/// side. On-grid when the side count allows (shared key, no crack);
/// chord midpoint otherwise (T-junction, no hole).
fn fill_digon(filler: &mut Filler, patch: usize) -> bool {
    let (keys0, points0) = filler.side_sequence(patch, 0);
    let (keys1, points1) = filler.side_sequence(patch, 1);
    let len0 = points0.len() - 1;
    let len1 = points1.len() - 1;
    let split_first = if len0 != len1 {
        len0 > len1
    } else {
        // Tie-break by 3D length, then side 0.
        let mut length0 = 0.0;
        for window in points0.windows(2) {
            length0 += (window[1] - window[0]).length();
        }
        let mut length1 = 0.0;
        for window in points1.windows(2) {
            length1 += (window[1] - window[0]).length();
        }
        length0 >= length1
    };
    let (split_keys, split_points, other_keys, other_points) = if split_first {
        (keys0, points0, keys1, points1)
    } else {
        (keys1, points1, keys0, points0)
    };
    let count = split_keys.len() - 1;
    let (mid_key, mid_point) = if count >= 2 {
        let j = count / 2;
        (split_keys[j], split_points[j])
    } else {
        // Single-segment side: chord midpoint (T-junction).
        let point = (split_points[0] + split_points[split_points.len() - 1]) * 0.5;
        let projected = filler.project(&point);
        (Key::SplitPoint(patch, 0, 0), projected)
    };
    // Triangle corners: entry of split side, midpoint, exit (= entry
    // of other side). Sides in loop order: first half, second half,
    // other side.
    let (tri_a_keys, tri_a_points) = if count >= 2 {
        let j = count / 2;
        (split_keys[..=j].to_vec(), split_points[..=j].to_vec())
    } else {
        (
            vec![split_keys[0], mid_key],
            vec![split_points[0], mid_point],
        )
    };
    let (tri_b_keys, tri_b_points) = if count >= 2 {
        let j = count / 2;
        (split_keys[j..].to_vec(), split_points[j..].to_vec())
    } else {
        (
            vec![mid_key, split_keys[count]],
            vec![mid_point, split_points[count]],
        )
    };
    filler.fill_triangle(
        patch,
        &[tri_a_keys, tri_b_keys, other_keys],
        &[tri_a_points, tri_b_points, other_points],
        0,
    );
    true
}

/// Monogon (one loop side) to triangle via two split points at thirds
/// (shared keys when they land on the grid).
fn fill_monogon(filler: &mut Filler, patch: usize) -> bool {
    let (keys, points) = filler.side_sequence(patch, 0);
    let count = keys.len() - 1;
    if count < 1 {
        return false;
    }
    // Fractions along the side polyline (path positions).
    let mut cumulative = vec![0.0; points.len()];
    for i in 1..points.len() {
        cumulative[i] = cumulative[i - 1] + (points[i] - points[i - 1]).length();
    }
    let total = cumulative[points.len() - 1];
    let at_fraction = |fraction: f64| -> (Key, Vector3) {
        // Snap to the grid when the fraction hits it exactly.
        let exact = fraction * count as f64;
        let rounded = exact.round();
        if (exact - rounded).abs() < 1e-9 {
            let index = (rounded as usize).min(count);
            return (keys[index], points[index]);
        }
        let target = total * fraction;
        let mut segment = 0;
        while segment + 1 < points.len().saturating_sub(1) && cumulative[segment + 1] < target {
            segment += 1;
        }
        let span = points[segment + 1] - points[segment];
        let width = cumulative[segment + 1] - cumulative[segment];
        let t = if width > 0.0 {
            ((target - cumulative[segment]) / width).clamp(0.0, 1.0)
        } else {
            0.0
        };
        (Key::SplitPoint(patch, 0, 0), points[segment] + span * t)
    };
    let (key_a, point_a) = at_fraction(1.0 / 3.0);
    let (key_b, mut point_b) = at_fraction(2.0 / 3.0);
    let key_b = if key_b == key_a {
        // Degenerate (tiny loop): nudge the second split off the first.
        point_b = point_a + (points[points.len() - 1] - points[0]) * 0.01;
        Key::SplitPoint(patch, 0, 1)
    } else {
        key_b
    };
    // Triangle sides along the loop: node -> A -> B -> node.
    let node_key = keys[0];
    filler.fill_triangle(
        patch,
        &[
            vec![node_key, key_a],
            vec![key_a, key_b],
            vec![key_b, node_key],
        ],
        &[
            vec![points[0], point_a],
            vec![point_a, point_b],
            vec![point_b, points[0]],
        ],
        0,
    );
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Noplan count vectors (scan-verified): exact wheel-aware
    /// planning fails on all of these; the class solver must fix them
    /// within tol of the measured lengths.
    const NOPLAN_VECTORS: &[&[usize]] = &[
        &[1, 2, 2, 2, 2],
        &[2, 1, 3, 3, 4],
        &[3, 2, 3, 4, 1],
        &[1, 4, 4, 4, 4],
        &[2, 3, 2, 3, 3],
        &[1, 1, 1, 2, 2, 2],
        &[1, 1, 2, 3, 2, 3],
        &[1, 1, 2, 2, 3, 3],
        &[1, 2, 3, 2, 2, 2, 2],
        &[1, 2, 2, 2, 2, 2, 2, 2],
    ];

    #[test]
    fn agreement_solver_fixes_noplan_vectors() {
        for vector in NOPLAN_VECTORS {
            let raw: Vec<f64> = vector.iter().map(|&c| c as f64).collect();
            let arc: Vec<usize> = (0..vector.len()).collect();
            let arc_fixed: Vec<Option<usize>> = vec![None; vector.len()];
            let mut capped = false;
            let solved = solve_ngon_counts_best(&raw, &arc, &arc_fixed, &mut capped);
            assert!(solved.is_some(), "unsolved at tol<=2: {vector:?}");
            let assignment = solved.unwrap();
            // Within the loosest tol of every measured length.
            for (side, &value) in assignment.iter().enumerate() {
                assert!(
                    (value as f64 - raw[side]).abs() <= 2.0,
                    "side {side} drifted: {value} vs raw {}",
                    raw[side]
                );
            }
            // The solved integers admit an exact cut plan (what the
            // emitter re-derives).
            let mut cuts = Vec::new();
            let mut emit_capped = false;
            let mut emit_budget = FILL_MAX_PLAN_NODES;
            let plan =
                Filler::plan_ngon_cuts(&assignment, &mut cuts, &mut emit_capped, &mut emit_budget);
            assert!(plan.is_some(), "no exact plan on solved {assignment:?}");
        }
    }

    #[test]
    fn agreement_solver_respects_pins() {
        // Hex [2,1,1,1,1,2]: pinning sides 0 and 1 to disagreeing
        // values kills every rotation; agreeing pins solve.
        let raw = vec![2.0, 1.0, 1.0, 1.0, 1.0, 2.0];
        let arc: Vec<usize> = (0..6).collect();
        let mut arc_fixed: Vec<Option<usize>> = vec![None; 6];
        arc_fixed[0] = Some(2);
        arc_fixed[1] = Some(5);
        let mut budget = FILL_MAX_PLAN_NODES;
        let mut capped = false;
        assert!(solve_ngon_counts(&raw, &arc, &arc_fixed, 1.0, &mut budget, &mut capped).is_none());
        arc_fixed[1] = Some(1);
        let mut budget = FILL_MAX_PLAN_NODES;
        let solved = solve_ngon_counts(&raw, &arc, &arc_fixed, 1.0, &mut budget, &mut capped);
        assert!(solved.is_some());
        let assignment = solved.unwrap();
        assert_eq!(assignment[0], 2);
        assert_eq!(assignment[1], 1);
    }

    #[test]
    fn agreement_solver_unions_shared_arcs() {
        // Pentagon with sides 0 and 2 on one arc: they are one
        // variable, so the (0,2) quad opposite pair agrees freely.
        let raw = vec![1.0, 2.0, 3.0, 2.0, 2.0];
        let arc = vec![0, 1, 0, 2, 3];
        let arc_fixed: Vec<Option<usize>> = vec![None; 4];
        let mut budget = FILL_MAX_PLAN_NODES;
        let mut capped = false;
        let solved = solve_ngon_counts(&raw, &arc, &arc_fixed, 2.0, &mut budget, &mut capped);
        assert!(solved.is_some());
        let assignment = solved.unwrap();
        assert_eq!(assignment[0], assignment[2]);
    }

    #[test]
    fn closest_point_basics() {
        let a = Vector3::new(0.0, 0.0, 0.0);
        let b = Vector3::new(1.0, 0.0, 0.0);
        let c = Vector3::new(0.0, 1.0, 0.0);
        let interior = closest_on_triangle(&Vector3::new(0.2, 0.2, 1.0), &a, &b, &c);
        assert!((interior.x() - 0.2).abs() < 1e-9);
        assert!((interior.y() - 0.2).abs() < 1e-9);
        assert!(interior.z().abs() < 1e-9);
        let vertex = closest_on_triangle(&Vector3::new(5.0, 0.0, 0.0), &a, &b, &c);
        assert!((vertex.x() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn projection_grid_matches_brute_force_distance() {
        // 10x10 plane grid (162 tris -> Grid arm): every query's
        // returned distance must equal the brute-force minimum
        // exactly (the minimum VALUE is order-free; tied points may
        // differ, so points are not compared).
        let n = 10usize;
        let mut verts = Vec::new();
        for j in 0..n {
            for i in 0..n {
                verts.push(Vector3::new(i as f64, j as f64, 0.0));
            }
        }
        let mut tris = Vec::new();
        for j in 0..n - 1 {
            for i in 0..n - 1 {
                let a = j * n + i;
                tris.push(vec![a, a + 1, a + n]);
                tris.push(vec![a + 1, a + n + 1, a + n]);
            }
        }
        let topology = SurfaceMesh::new(&verts, &tris);
        let faces: Vec<usize> = (0..topology.face_count()).collect();
        let index = PatchProjection::build(&topology, &verts, &faces);
        assert!(matches!(index, PatchProjection::Grid(_)));
        let mut seen = vec![0u32; index.tri_count()];
        // Deterministic query cloud: over, under, and off the grid.
        let mut state = 0x1234_5678_9abc_def1u64;
        let mut next = || {
            state = state
                .wrapping_add(0x9E37_79B9_7F4A_7C15)
                .wrapping_mul(0xBF58_476D_1CE4_E5B9);
            (state >> 11) as f64 * 2.0f64.powi(-53)
        };
        for q in 0..50 {
            let point = Vector3::new(next() * 12.0 - 1.5, next() * 12.0 - 1.5, next() * 4.0 - 2.0);
            let stamp = (q + 1) as u32;
            let got = index.closest(&point, &mut seen, stamp);
            let got_d2 = (got - point).length_squared();
            let mut best_d2 = f64::INFINITY;
            for face in &faces {
                let tri = topology.triangle(*face);
                let candidate =
                    closest_on_triangle(&point, &verts[tri[0]], &verts[tri[1]], &verts[tri[2]]);
                let d2 = (candidate - point).length_squared();
                if d2 < best_d2 {
                    best_d2 = d2;
                }
            }
            assert_eq!(
                got_d2, best_d2,
                "query {q}: grid distance must equal brute-force minimum"
            );
        }
    }

    #[test]
    fn grid_cap_boundaries() {
        // (a+1) x (b+1) > 16384, saturating (never panics).
        assert!(!grid_over_cap(127, 127));
        assert!(grid_over_cap(128, 128));
        assert!(grid_over_cap(0, 20_000));
        assert!(grid_over_cap(usize::MAX, 1));
        assert!(!grid_over_cap(0, 0));
    }

    #[test]
    fn planner_finds_first_valid_cut() {
        // i=0 mismatched (1 vs 2), i=1 cuts (2 == 2) with a remainder
        // terminating in a wheel-compatible pentagon.
        let mut cuts = Vec::new();
        let mut capped = false;
        let mut budget = FILL_MAX_PLAN_NODES;
        let terminal =
            Filler::plan_ngon_cuts(&[1, 2, 2, 2, 1, 2, 2], &mut cuts, &mut capped, &mut budget);
        assert!(matches!(terminal, Some(NgonTerminal::Pent)), "{terminal:?}");
        assert_eq!(cuts, vec![1]);
        assert!(!capped);
    }

    #[test]
    fn planner_terminates_in_hex() {
        // Uniform octagon: first cut at 0, hexagon terminal at 0.
        let mut cuts = Vec::new();
        let mut capped = false;
        let mut budget = FILL_MAX_PLAN_NODES;
        let terminal = Filler::plan_ngon_cuts(
            &[3, 3, 3, 3, 3, 3, 3, 3],
            &mut cuts,
            &mut capped,
            &mut budget,
        );
        assert!(matches!(terminal, Some(NgonTerminal::Hex)), "{terminal:?}");
        assert_eq!(cuts, vec![0]);
        assert!(!capped);
    }

    #[test]
    fn planner_rejects_unplannable() {
        // No rotation matches anywhere: None, and no cap bound.
        let mut cuts = Vec::new();
        let mut capped = false;
        let mut budget = FILL_MAX_PLAN_NODES;
        let terminal =
            Filler::plan_ngon_cuts(&[1, 2, 3, 4, 5, 6, 7], &mut cuts, &mut capped, &mut budget);
        assert!(terminal.is_none());
        assert!(cuts.is_empty());
        assert!(!capped);
    }

    #[test]
    fn planner_honors_node_budget() {
        // The same plannable input plans with budget and caps without.
        let counts = [3, 3, 3, 3, 3, 3, 3, 3];
        let mut cuts = Vec::new();
        let mut capped = false;
        // Two nodes plan this input (level + hex terminal); one node
        // starves.
        let mut budget = 1;
        let starved = Filler::plan_ngon_cuts(&counts, &mut cuts, &mut capped, &mut budget);
        assert!(starved.is_none());
        assert!(capped);
        let mut cuts = Vec::new();
        let mut capped = false;
        let mut budget = FILL_MAX_PLAN_NODES;
        let planned = Filler::plan_ngon_cuts(&counts, &mut cuts, &mut capped, &mut budget);
        assert!(planned.is_some());
        assert!(!capped);
    }

    #[test]
    fn pent_cut_skips_over_cap_rotation_then_succeeds() {
        // Rotation 0 matches counts but its quad is over-cap (skipped,
        // capped set); rotation 2 matches and fits.
        let mut capped = false;
        let rotation = first_pent_cut(&[2, 200, 200, 200, 200], &mut capped);
        assert_eq!(rotation, Some(2));
        assert!(capped);
        // All rotations over-cap: no plan, capped.
        let mut capped = false;
        assert_eq!(
            first_pent_cut(&[200, 200, 200, 200, 200], &mut capped),
            None
        );
        assert!(capped);
    }

    #[test]
    fn hex_cut_skips_over_cap_rotations() {
        let mut capped = false;
        assert_eq!(
            first_hex_cut(&[200, 200, 200, 200, 200, 200], &mut capped),
            None
        );
        assert!(capped);
        let mut capped = false;
        assert_eq!(first_hex_cut(&[3, 3, 3, 3, 3, 3], &mut capped), Some(0));
        assert!(!capped);
    }

    #[test]
    fn wheel_splits_accepts_valid_triples() {
        // Equilateral: three 1-wide quads around the center.
        assert_eq!(wheel_splits([2, 2, 2]), Some([1, 1, 1]));
        // (2,3,3): splits (1,2,1), spokes (2,1,1).
        assert_eq!(wheel_splits([2, 3, 3]), Some([1, 2, 1]));
        // Odd-sum and triangle violations fail.
        assert_eq!(wheel_splits([1, 1, 1]), None);
        assert_eq!(wheel_splits([1, 2, 2]), None);
        assert_eq!(wheel_splits([1, 2, 3]), None);
        assert_eq!(wheel_splits([1, 1, 2]), None);
        // A side of count 1 can never split.
        assert_eq!(wheel_splits([1, 3, 3]), None);
    }

    #[test]
    fn greedy_pairs_covers_chains_and_prefers_scores() {
        let flat = || PairScore {
            bent_um: 0,
            min_cdeg: 8900,
            edge_um: 1000,
        };
        let bent = || PairScore {
            bent_um: 500_000,
            min_cdeg: 3000,
            edge_um: 1000,
        };
        // Chain of 3: first pair wins, leftover alone.
        let neighbors = vec![
            vec![(1, flat())],
            vec![(0, flat()), (2, flat())],
            vec![(1, flat())],
        ];
        assert_eq!(
            greedy_pairs(&neighbors, false),
            vec![Some(1), Some(0), None]
        );
        // Score preference beats index order.
        let neighbors = vec![
            vec![(1, bent()), (2, flat())],
            vec![(0, bent())],
            vec![(0, flat())],
        ];
        assert_eq!(
            greedy_pairs(&neighbors, false),
            vec![Some(2), None, Some(0)]
        );
        // 4-cycle: all paired.
        let neighbors = vec![
            vec![(1, flat()), (3, flat())],
            vec![(0, flat()), (2, flat())],
            vec![(1, flat()), (3, flat())],
            vec![(0, flat()), (2, flat())],
        ];
        assert_eq!(
            greedy_pairs(&neighbors, false),
            vec![Some(1), Some(0), Some(3), Some(2)]
        );
        // Isolated face stays alone.
        let neighbors: Vec<Vec<(usize, PairScore)>> = vec![vec![]];
        assert_eq!(greedy_pairs(&neighbors, false), vec![None]);
        // Degree-first strands fewer: hub 0 with leaves 1..3 plus 4
        // behind leaf 1. Face order pairs (0,1) and strands 2,3,4;
        // degree order pairs (0,2) and (1,4), stranding only 3.
        let neighbors = vec![
            vec![(1, flat()), (2, flat()), (3, flat())],
            vec![(0, flat()), (4, flat())],
            vec![(0, flat())],
            vec![(0, flat())],
            vec![(1, flat())],
        ];
        assert_eq!(
            greedy_pairs(&neighbors, true),
            vec![Some(2), Some(4), Some(0), None, Some(1)]
        );
    }

    #[test]
    fn winding_unify_opposes_shared_edges() {
        // Second quad runs the shared edge 2->3 the same way as the
        // first; unification must flip exactly one of them.
        let mut faces = vec![vec![0, 1, 2, 3], vec![2, 3, 4, 5]];
        unify_winding(&mut faces);
        let follows = |face: &[usize], a: usize, b: usize| -> bool {
            face.iter()
                .position(|&v| v == a)
                .map(|i| face[(i + 1) % face.len()] == b)
                .unwrap_or(false)
        };
        assert_ne!(
            follows(&faces[0], 2, 3),
            follows(&faces[1], 2, 3),
            "shared edge runs opposite ways: {faces:?}"
        );
    }
}
