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
//! - even n-gons: recursive corner-to-corner cutting into Coons quads
//!   (exact opposite matches only, else the per-face fallback);
//! - odd n-gons: recursive cutting into Coons quads plus one small
//!   triangle (subdivided while its sides stay even);
//! - digons/monogons: split into triangles at shared grid points;
//! - closed/holed/inconsistent patches: per-face 3-quad subdivision,
//!   which keeps full coverage by construction.
//!
//! All grid vertices are projected onto their own patch's triangles,
//! so thin features can never snap across gaps. Shared sides weld by
//! key, giving a manifold mesh except along fallback borders and
//! conflict diagonals (both rare and reported).

use crate::patch_backend::layout::Layout;
use crate::surface_mesh::SurfaceMesh;
use crate::vector3::Vector3;
use std::cmp::Reverse;
use std::collections::{BTreeMap, BinaryHeap};

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

    /// Closest point on the patch's own triangles.
    fn project(&self, patch_faces: &[usize], point: &Vector3) -> Vector3 {
        let mut best = *point;
        let mut best_d2 = f64::INFINITY;
        for &face in patch_faces {
            let tri = self.topology.triangle(face);
            let candidate = closest_on_triangle(
                point,
                &self.verts[tri[0]],
                &self.verts[tri[1]],
                &self.verts[tri[2]],
            );
            let d2 = (candidate - *point).length_squared();
            if d2 < best_d2 {
                best_d2 = d2;
                best = candidate;
            }
        }
        best
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
            (self.layout.graph.arcs[side.arc].a, self.layout.graph.arcs[side.arc].b)
        } else {
            (self.layout.graph.arcs[side.arc].b, self.layout.graph.arcs[side.arc].a)
        };
        keys[0] = Key::Node(entry);
        keys[count] = Key::Node(exit);
        if !side.forward {
            points.reverse();
            keys.reverse();
        }
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
        patch_faces: &[usize],
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
                    let mut point = bottom * (1.0 - v)
                        + top * v + left * (1.0 - u)
                        + right * u
                        - corners[0] * (1.0 - u) * (1.0 - v)
                        - corners[1] * u * (1.0 - v)
                        - corners[2] * u * v
                        - corners[3] * (1.0 - u) * v;
                    point = self.project(patch_faces, &point);
                    (Key::Grid(patch, piece, i, j), point)
                };
                let uv = (i as f64 / a as f64, j as f64 / b as f64);
                grid[i][j] = self.emit(key, position, patch, uv);
            }
        }
        for i in 0..a {
            for j in 0..b {
                self.emit_face(&[grid[i][j], grid[i + 1][j], grid[i + 1][j + 1], grid[i][j + 1]]);
            }
        }
    }

    /// Dijkstra vertex path across patch faces between two corners.
    ///
    /// Binary-heap open set keyed by (distance bits, vertex): distances
    /// are non-negative so the bit order matches the float order, and
    /// the vertex id breaks ties, keeping the path deterministic.
    fn cut_path(&self, patch_faces: &[usize], from: usize, to: usize) -> Vec<usize> {
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
        patch_faces: &[usize],
        forward: bool,
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
            while segment + 1 < points.len().saturating_sub(1)
                && cumulative[segment + 1] < target
            {
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
            point = self.project(patch_faces, &point);
            let key = Key::CutPoint(patch, cut, if forward { i } else { count - i });
            let uv = (i as f64 / count.max(1) as f64, 0.5);
            self.emit(key, point, patch, uv);
            keys.push(key);
        }
        if !forward {
            keys.reverse();
        }
        keys
    }

    /// Recursive corner-cut fill of an n-gon (n >= 5) into Coons quads
    /// plus one triangle (odd n). Returns false when no exact cut
    /// sequence exists (caller runs the per-face fallback).
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
    ) -> bool {
        let n = corners.len();
        debug_assert!(n >= 5 && sides.len() == n);
        if n == 5 {
            // Pentagon: try all corner-to-second-neighbor cuts; each
            // yields a quad plus a triangle.
            for i in 0..5 {
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
                // Quad sides: arc, arc, arc, cut. Opposite equality
                // needs sides[0] == sides[2]; the cut takes sides[1].
                if quad_sides[0] != quad_sides[2] {
                    continue;
                }
                let cut_count = quad_sides[1].max(1);
                let from = self.layout.graph.nodes[quad_corners[3]].vertex;
                let to = self.layout.graph.nodes[quad_corners[0]].vertex;
                let path = self.cut_path(patch_faces, from, to);
                let cut = *cut_base;
                *cut_base += 1;
                let cut_keys =
                    self.cut_sequence(patch, cut, &path, cut_count, patch_faces, true);
                let cut_points: Vec<Vector3> =
                    cut_keys.iter().map(|k| self.verts_out[self.keys[k]]).collect();
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
                    patch_faces,
                );
                // The triangle (corners i, i+1, i+2) with its real
                // side grids, so shared sides weld instead of cracking.
                let mut cut_rev = cut_keys.clone();
                cut_rev.reverse();
                let mut cut_pts_rev = cut_points.clone();
                cut_pts_rev.reverse();
                self.fill_triangle(
                    patch,
                    &[
                        sides[i].clone(),
                        sides[(i + 1) % 5].clone(),
                        cut_rev,
                    ],
                    &[
                        side_points[i].clone(),
                        side_points[(i + 1) % 5].clone(),
                        cut_pts_rev,
                    ],
                    patch_faces,
                    0,
                );
                return true;
            }
            return false;
        }
        // n >= 6: cut off one quad (corners i..i+3 plus cut), recurse
        // on the (n-2)-gon. Both parities terminate (even reaches 4
        // via repeated cuts handled by the caller... only odd reaches
        // 5 here; even n == 6 splits into two quads below).
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
                let cut_count = b.max(1);
                let from = self.layout.graph.nodes[corners[i]].vertex;
                let to = self.layout.graph.nodes[corners[(i + 3) % 6]].vertex;
                let path = self.cut_path(patch_faces, from, to);
                let cut = *cut_base;
                *cut_base += 1;
                let cut_keys =
                    self.cut_sequence(patch, cut, &path, cut_count, patch_faces, true);
                let cut_points: Vec<Vector3> =
                    cut_keys.iter().map(|k| self.verts_out[self.keys[k]]).collect();
                let mut rev_keys = cut_keys.clone();
                rev_keys.reverse();
                let mut rev_points = cut_points.clone();
                rev_points.reverse();
                let piece = *piece_base;
                *piece_base += 1;
                self.fill_coons(
                    patch,
                    piece,
                    &[
                        sides[i].clone(),
                        sides[(i + 1) % 6].clone(),
                        sides[(i + 2) % 6].clone(),
                        cut_keys,
                    ],
                    &[
                        side_points[i].clone(),
                        side_points[(i + 1) % 6].clone(),
                        side_points[(i + 2) % 6].clone(),
                        cut_points,
                    ],
                    a,
                    b,
                    patch_faces,
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
                        rev_keys,
                    ],
                    &[
                        side_points[(i + 3) % 6].clone(),
                        side_points[(i + 4) % 6].clone(),
                        side_points[(i + 5) % 6].clone(),
                        rev_points,
                    ],
                    d,
                    e,
                    patch_faces,
                );
                return true;
            }
            return false;
        }
        // n >= 7: cut off one quad, recurse.
        for i in 0..n {
            let a = sides[i].len() - 1;
            let b = sides[(i + 1) % n].len() - 1;
            let c = sides[(i + 2) % n].len() - 1;
            if a != c {
                continue;
            }
            let cut_count = b.max(1);
            let from = self.layout.graph.nodes[corners[i]].vertex;
            let to = self.layout.graph.nodes[corners[(i + 3) % n]].vertex;
            let path = self.cut_path(patch_faces, from, to);
            // Tentatively fill the quad, then recurse on the rest. The
            // recursion owns later pieces/cuts, so a failed recursion
            // must roll back: snapshot the output lengths. (Emitted
            // vertices stay (harmless duplicates are impossible: keys
            // are unique), only faces roll back... vertices are keyed,
            // so keeping them is consistent.)
            let faces_before = self.faces_out.len();
            let cut = *cut_base;
            let cut_keys = self.cut_sequence(patch, cut, &path, cut_count, patch_faces, true);
            let cut_points: Vec<Vector3> =
                cut_keys.iter().map(|k| self.verts_out[self.keys[k]]).collect();
            let piece = *piece_base;
            self.fill_coons(
                patch,
                piece,
                &[
                    sides[i].clone(),
                    sides[(i + 1) % n].clone(),
                    sides[(i + 2) % n].clone(),
                    cut_keys.clone(),
                ],
                &[
                    side_points[i].clone(),
                    side_points[(i + 1) % n].clone(),
                    side_points[(i + 2) % n].clone(),
                    cut_points.clone(),
                ],
                a,
                b,
                patch_faces,
            );
            // Remainder: corners i+3..i+n-1, i with the cut closing it.
            let mut rest_corners = Vec::new();
            let mut rest_sides: Vec<Vec<Key>> = Vec::new();
            let mut rest_points: Vec<Vec<Vector3>> = Vec::new();
            for k in 3..n {
                rest_corners.push(corners[(i + k) % n]);
            }
            rest_corners.push(corners[i]);
            for k in 3..n {
                rest_sides.push(sides[(i + k) % n].clone());
                rest_points.push(side_points[(i + k) % n].clone());
            }
            let mut rev_keys = cut_keys;
            rev_keys.reverse();
            let mut rev_points = cut_points;
            rev_points.reverse();
            rest_sides.push(rev_keys);
            rest_points.push(rev_points);
            // The remainder has n-2 >= 5 sides; quad remainder (n == 6
            // handled above; n == 7 gives 5) recurses directly.
            *piece_base += 1;
            *cut_base += 1;
            let ok = if rest_corners.len() == 4 {
                // Direct quad: needs opposite equality on the nose.
                let lens: Vec<usize> = rest_sides.iter().map(|s| s.len() - 1).collect();
                if lens[0] == lens[2] && lens[1] == lens[3] {
                    let piece = *piece_base;
                    *piece_base += 1;
                    self.fill_coons(
                        patch,
                        piece,
                        &[
                            rest_sides[0].clone(),
                            rest_sides[1].clone(),
                            rest_sides[2].clone(),
                            rest_sides[3].clone(),
                        ],
                        &[
                            rest_points[0].clone(),
                            rest_points[1].clone(),
                            rest_points[2].clone(),
                            rest_points[3].clone(),
                        ],
                        lens[0],
                        lens[1],
                        patch_faces,
                    );
                    true
                } else {
                    false
                }
            } else {
                self.fill_ngon(
                    patch,
                    &rest_corners,
                    &rest_sides,
                    &rest_points,
                    patch_faces,
                    piece_base,
                    cut_base,
                )
            };
            if ok {
                return true;
            }
            self.faces_out.truncate(faces_before);
        }
        false
    }

    /// Subdivided triangle fill: 4-subtri recursion while all sides
    /// stay even (midpoints land on shared grids), else one face.
    fn fill_triangle(
        &mut self,
        patch: usize,
        seqs: &[Vec<Key>; 3],
        points: &[Vec<Vector3>; 3],
        patch_faces: &[usize],
        depth: usize,
    ) {
        let counts = [
            seqs[0].len() - 1,
            seqs[1].len() - 1,
            seqs[2].len() - 1,
        ];
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
                        let point = self.project(patch_faces, &(a + (b - a) * t));
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
                patch_faces,
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
            patch_faces,
            depth + 1,
        );
    }

    /// Per-face 3-quad subdivision fallback (all-quad, covers every
    /// face). Manifold inside the fallback region (shared edge keys);
    /// cracked against structured neighbors (rare by construction).
    fn fill_fallback(&mut self, patch: usize, patch_faces: &[usize]) {
        self.stats.fallback_faces += patch_faces.len();
        for &face in patch_faces {
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
            let corners = [a, b, c].map(|v| {
                self.emit(Key::FallbackVert(v), self.verts[v], patch, (0.5, 0.5))
            });
            self.emit_face(&[corners[0], mids[0], center, mids[2]]);
            self.emit_face(&[corners[1], mids[1], center, mids[0]]);
            self.emit_face(&[corners[2], mids[2], center, mids[1]]);
        }
    }
}

/// Initial arc counts: arc length over local quad width. Returns
/// the counts plus the number clamped at the 256 cap.
fn initial_counts(
    layout: &Layout,
    topology: &SurfaceMesh,
    verts: &[Vector3],
    face_width: &[f64],
) -> (Vec<usize>, usize) {
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
            let raw = (length / width).round();
            if raw > 256.0 {
                clamped += 1;
            }
            raw.clamp(1.0, 256.0) as usize
        })
        .collect();
    (counts, clamped)
}

/// Priority propagation over quad patches: opposite sides agree on
/// one count (first writer wins, patch-id order). Returns the
/// per-patch deferred flags (both-set-unequal pairs, filled by
/// diagonal instead).
fn propagate_counts(layout: &Layout, counts: &mut [usize]) -> Vec<bool> {
    let mut set = vec![false; counts.len()];
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
                    let value = ((counts[a] + counts[b]) as f64 / 2.0).round().max(1.0) as usize;
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
    deferred
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

/// Fill one island's layout.
pub(crate) fn fill_layout(
    topology: &SurfaceMesh,
    verts: &[Vector3],
    layout: &Layout,
    face_width: &[f64],
    degenerate_area: f64,
) -> FillOutput {
    let (mut counts, clamped_arcs) = initial_counts(layout, topology, verts, face_width);
    let deferred = propagate_counts(layout, &mut counts);
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
    };
    filler.stats.patches = layout.patches.len();
    filler.stats.clamped_arcs = clamped_arcs;
    for (patch_id, patch) in layout.patches.iter().enumerate() {
        if !patch.usable || patch.sides.is_empty() {
            filler.fill_fallback(patch_id, &patch.faces);
            continue;
        }
        filler.stats.usable_patches += 1;
        match patch.sides.len() {
            4 if !deferred[patch_id] => {
                let mut seqs: Vec<Vec<Key>> = Vec::new();
                let mut points: Vec<Vec<Vector3>> = Vec::new();
                for position in 0..4 {
                    let (keys, positions) = filler.side_sequence(patch_id, position);
                    seqs.push(keys);
                    points.push(positions);
                }
                let a = seqs[0].len() - 1;
                let b = seqs[1].len() - 1;
                if seqs[2].len() - 1 == a && seqs[3].len() - 1 == b {
                    filler.fill_coons(
                        patch_id,
                        0,
                        &[seqs[0].clone(), seqs[1].clone(), seqs[2].clone(), seqs[3].clone()],
                        &[
                            points[0].clone(),
                            points[1].clone(),
                            points[2].clone(),
                            points[3].clone(),
                        ],
                        a,
                        b,
                        &patch.faces,
                    );
                } else {
                    filler.stats.conflicted_quads += 1;
                    fill_diagonal(&mut filler, patch_id, &patch.faces);
                }
            }
            4 => {
                // Deferred by propagation: diagonal triangle pair.
                filler.stats.conflicted_quads += 1;
                fill_diagonal(&mut filler, patch_id, &patch.faces);
            }
            3 => {
                let mut seqs: Vec<Vec<Key>> = Vec::new();
                let mut points: Vec<Vec<Vector3>> = Vec::new();
                for position in 0..3 {
                    let (keys, positions) = filler.side_sequence(patch_id, position);
                    seqs.push(keys);
                    points.push(positions);
                }
                filler.fill_triangle(
                    patch_id,
                    &[seqs[0].clone(), seqs[1].clone(), seqs[2].clone()],
                    &[points[0].clone(), points[1].clone(), points[2].clone()],
                    &patch.faces,
                    0,
                );
            }
            2 => {
                if !fill_digon(&mut filler, patch_id, &patch.faces) {
                    filler.fill_fallback(patch_id, &patch.faces);
                }
            }
            1 => {
                if !fill_monogon(&mut filler, patch_id, &patch.faces) {
                    filler.fill_fallback(patch_id, &patch.faces);
                }
            }
            _ => {
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
                if !filler.fill_ngon(
                    patch_id,
                    &corners,
                    &seqs,
                    &points,
                    &patch.faces,
                    &mut piece,
                    &mut cut,
                ) {
                    filler.fill_fallback(patch_id, &patch.faces);
                }
            }
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
fn fill_diagonal(filler: &mut Filler, patch: usize, patch_faces: &[usize]) {
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
    let _ = patch_faces;
}

/// Digon (two sides) to triangle via a midpoint split of the longest
/// side. On-grid when the side count allows (shared key, no crack);
/// chord midpoint otherwise (T-junction, no hole).
fn fill_digon(filler: &mut Filler, patch: usize, patch_faces: &[usize]) -> bool {
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
        let projected = filler.project(patch_faces, &point);
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
        patch_faces,
        0,
    );
    true
}

/// Monogon (one loop side) to triangle via two split points at thirds
/// (shared keys when they land on the grid).
fn fill_monogon(filler: &mut Filler, patch: usize, patch_faces: &[usize]) -> bool {
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
        patch_faces,
        0,
    );
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn closest_point_basics() {
        let a = Vector3::new(0.0, 0.0, 0.0);
        let b = Vector3::new(1.0, 0.0, 0.0);
        let c = Vector3::new(0.0, 1.0, 0.0);
        let interior = closest_on_triangle(
            &Vector3::new(0.2, 0.2, 1.0),
            &a,
            &b,
            &c,
        );
        assert!((interior.x() - 0.2).abs() < 1e-9);
        assert!((interior.y() - 0.2).abs() < 1e-9);
        assert!(interior.z().abs() < 1e-9);
        let vertex = closest_on_triangle(&Vector3::new(5.0, 0.0, 0.0), &a, &b, &c);
        assert!((vertex.x() - 1.0).abs() < 1e-9);
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
