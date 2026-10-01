//! Port of `core/symmetry.*` (`retopo.core.symmetry`).
//!
//! Line-by-line mirror: vote-based dominant-plane detection, support
//! scoring, point/direction mirroring, cross-field symmetrization and
//! vertex snapping. All vector math goes through [`crate::vector3`]
//! (FMA-exact, bitwise against the C++ build), so this file only has to
//! get its own scalar expressions right.
//!
//! FMA transcription: Clang fuses `2.0 * offset - x` to
//! `fma(offset, 2.0, -x)` at -O3 (verified in IR, brew Clang 23, ARM64 —
//! the only multiply-add form in this module; `0.5 * (a + b)` does not
//! fuse since the multiply consumes the add, and neither does the
//! ring-stop square). The product `offset * 2.0` is an exact power-of-two
//! scaling, so fused and unfused agree on all finite inputs except
//! overflow-adjacent extremes; the port still transcribes the fused form
//! with explicit [`f64::mul_add`] so it is bit-identical even there.
//!
//! Deliberate restructures (mesh_separator precedent, commented at the
//! site): the `npos` sentinel is `Option<usize>`; the `PointGrid` cell
//! map is a Rust `HashMap` with the default hasher instead of
//! `unordered_map` with FNV-1a (the grid only ever does point lookups,
//! never iteration, so bucket order cannot affect results).

use crate::vector3::Vector3;
use std::collections::HashMap;

/// Mirror plane: axis 0 = X, 1 = Y, 2 = Z; -1 = no plane (mirrors
/// `AutoRemesher::SymmetryPlane`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SymmetryPlane {
    /// Mirror axis: 0 = X, 1 = Y, 2 = Z; -1 = no plane.
    pub axis: i32,
    /// Plane position: points with p[axis] == offset lie on the plane.
    pub offset: f64,
    /// Detection support: fraction of the input vertices that have a
    /// mirrored partner within tolerance (1.0 for a perfectly mirrored
    /// mesh).
    pub score: f64,
}

impl Default for SymmetryPlane {
    #[inline]
    fn default() -> Self {
        Self {
            axis: -1,
            offset: 0.0,
            score: 0.0,
        }
    }
}

impl SymmetryPlane {
    /// Mirrors `SymmetryPlane::valid`.
    #[inline]
    #[must_use]
    pub fn valid(&self) -> bool {
        self.axis >= 0 && self.axis < 3
    }
}

/// Uniform-grid cell key (mirrors the anonymous-namespace `CellKey`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
struct CellKey {
    x: i64,
    y: i64,
    z: i64,
}

/// Uniform-grid nearest lookup over the ORIGINAL point positions (mirrors
/// the anonymous-namespace `PointGrid`). Queries expand ring by ring and
/// stop once the closest unvisited ring is farther than the best hit, so
/// near-symmetric partners resolve in ring 0-2 while a `max_radius <= 0`
/// query still degenerates to a global nearest search.
///
/// Deliberate restructure: `std::unordered_map` with FNV-1a becomes a Rust
/// `HashMap` with the default hasher. `nearest` only does point lookups
/// (`find`), never iteration, so bucket order cannot affect results; the
/// per-cell index vectors keep C++ insertion order.
struct PointGrid<'a> {
    points: &'a [Vector3],
    cell_size: f64,
    cells: HashMap<CellKey, Vec<usize>>,
}

impl<'a> PointGrid<'a> {
    fn new(points: &'a [Vector3], cell_size: f64) -> Self {
        let cell_size = if cell_size > 0.0 { cell_size } else { 1e-9 };
        let mut cells = HashMap::new();
        cells.reserve(points.len() * 2 + 1);
        let mut grid = Self {
            points,
            cell_size,
            cells,
        };
        for i in 0..points.len() {
            grid.cells
                .entry(grid.cell_of(&points[i]))
                .or_default()
                .push(i);
        }
        grid
    }

    /// Mirrors `PointGrid::nearest` (`npos` is `None` here).
    fn nearest(&self, query: &Vector3, max_radius: f64) -> Option<usize> {
        if self.points.is_empty() {
            return None;
        }
        let center = self.cell_of(query);
        let mut best: Option<usize> = None;
        let mut best_distance_squared = if max_radius > 0.0 {
            max_radius * max_radius
        } else {
            f64::INFINITY
        };
        // The grid spans at most points.size() occupied cells along any
        // axis, so this many rings always cover the whole grid.
        let max_ring = self.points.len() as i64 + 1;
        for ring in 0..=max_ring {
            // Closest possible distance to any cell strictly outside the
            // rings visited so far is (ring - 1) * cellSize; stop once the
            // best hit beats it.
            if ring > 0 {
                let stop = (ring - 1) as f64 * self.cell_size;
                if stop * stop >= best_distance_squared {
                    break;
                }
            }
            if max_radius > 0.0 && (ring - 1) as f64 * self.cell_size > max_radius {
                break;
            }
            // Visit only the shell of the cube: at least one coordinate
            // offset has magnitude `ring`.
            for dx in -ring..=ring {
                for dy in -ring..=ring {
                    for dz in -ring..=ring {
                        if dx.abs().max(dy.abs()).max(dz.abs()) != ring {
                            continue;
                        }
                        let Some(bucket) = self.cells.get(&CellKey {
                            x: center.x + dx,
                            y: center.y + dy,
                            z: center.z + dz,
                        }) else {
                            continue;
                        };
                        for &index in bucket {
                            let distance_squared = (self.points[index] - *query).length_squared();
                            if distance_squared < best_distance_squared {
                                best_distance_squared = distance_squared;
                                best = Some(index);
                            }
                        }
                    }
                }
            }
            if best_distance_squared <= 0.0 {
                break;
            }
        }
        best
    }

    fn has_near(&self, query: &Vector3, radius: f64) -> bool {
        self.nearest(query, radius).is_some()
    }

    fn cell_of(&self, point: &Vector3) -> CellKey {
        // `as i64` saturates on overflow where the C++ cast is UB; the
        // oracle keeps inputs in the finite moderate range where both
        // truncate identically.
        CellKey {
            x: (point.x() / self.cell_size).floor() as i64,
            y: (point.y() / self.cell_size).floor() as i64,
            z: (point.z() / self.cell_size).floor() as i64,
        }
    }
}

/// `std::min<double>` exactly: `(b < a) ? b : a` (differs from
/// `f64::min` on NaN inputs, which the oracle excludes but the mirror
/// still gets right).
#[inline]
fn cxx_min(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

/// `std::max<double>` exactly: `(a < b) ? b : a`.
#[inline]
fn cxx_max(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}

fn bounding_box(points: &[Vector3], lower: &mut Vector3, upper: &mut Vector3) {
    *lower = points[0];
    *upper = points[0];
    for point in points {
        for i in 0..3 {
            lower[i] = cxx_min(lower[i], point[i]);
            upper[i] = cxx_max(upper[i], point[i]);
        }
    }
}

fn bounding_diagonal(points: &[Vector3]) -> f64 {
    if points.is_empty() {
        return 0.0;
    }
    let mut lower = Vector3::default();
    let mut upper = Vector3::default();
    bounding_box(points, &mut lower, &mut upper);
    (upper - lower).length()
}

// Cell sized for surface-distributed points: the mean neighbor spacing of
// N points over an area ~ D^2 scales as D/sqrt(N).
fn partner_cell_size(points: &[Vector3], diagonal: f64) -> f64 {
    if points.is_empty() || diagonal <= 0.0 {
        return 1e-9;
    }
    cxx_max(4.0 * diagonal / (points.len() as f64).sqrt(), 1e-9)
}

/// Mirror-symmetry constraints (mirrors `AutoRemesher::Symmetry`).
pub struct Symmetry;

impl Symmetry {
    /// Mirrors `Symmetry::mirrorPoint`. The axis expression is the fused
    /// `fma(offset, 2.0, -x)` form — see the module docs.
    #[inline]
    #[must_use]
    pub fn mirror_point(point: &Vector3, plane: &SymmetryPlane) -> Vector3 {
        let mut mirrored = *point;
        if plane.valid() {
            let axis = plane.axis as usize;
            mirrored[axis] = plane.offset.mul_add(2.0, -point[axis]);
        }
        mirrored
    }

    /// Mirrors `Symmetry::mirrorDirection`.
    #[inline]
    #[must_use]
    pub fn mirror_direction(direction: &Vector3, plane: &SymmetryPlane) -> Vector3 {
        let mut mirrored = *direction;
        if plane.valid() {
            let axis = plane.axis as usize;
            mirrored[axis] = -direction[axis];
        }
        mirrored
    }

    /// Fraction of vertices whose mirror across the plane lands within
    /// `tolerance` of another vertex (mirrors `Symmetry::scorePlane`).
    pub fn score_plane(vertices: &[Vector3], axis: i32, offset: f64, tolerance: f64) -> f64 {
        if vertices.is_empty() || axis < 0 || axis > 2 || tolerance <= 0.0 {
            return 0.0;
        }
        let plane = SymmetryPlane {
            axis,
            offset,
            score: 0.0,
        };
        let grid = PointGrid::new(vertices, tolerance);
        let mut hits = 0usize;
        for vertex in vertices {
            if grid.has_near(&Self::mirror_point(vertex, &plane), tolerance) {
                hits += 1;
            }
        }
        hits as f64 / vertices.len() as f64
    }

    /// Vote-based dominant-plane detection over the X, Y and Z
    /// axis-aligned planes through the bounding-box center. Ties prefer Y,
    /// then Z, then X: organic sculpts are Y-symmetric in practice. Always
    /// returns a valid plane for a non-empty input; the caller gates on
    /// `score` (mirrors `Symmetry::detectPlane`).
    pub fn detect_plane(vertices: &[Vector3]) -> SymmetryPlane {
        let mut best = SymmetryPlane::default();
        if vertices.is_empty() {
            return best;
        }
        let mut lower = Vector3::default();
        let mut upper = Vector3::default();
        bounding_box(vertices, &mut lower, &mut upper);
        let diagonal = (upper - lower).length();
        let tolerance = cxx_max(0.01 * diagonal, 1e-9);
        // Ties prefer Y, then Z, then X (strictly-greater keeps the first best).
        for axis in [1, 2, 0] {
            let offset = 0.5 * (lower[axis as usize] + upper[axis as usize]);
            let score = Self::score_plane(vertices, axis, offset, tolerance);
            if score > best.score {
                best = SymmetryPlane {
                    axis,
                    offset,
                    score,
                };
            }
        }
        best
    }

    /// Fixed-axis plane at the bounding-box center, with its support score
    /// (mirrors `Symmetry::fixedPlane`).
    pub fn fixed_plane(vertices: &[Vector3], axis: i32) -> SymmetryPlane {
        let mut plane = SymmetryPlane::default();
        if vertices.is_empty() || axis < 0 || axis > 2 {
            return plane;
        }
        let mut lower = Vector3::default();
        let mut upper = Vector3::default();
        bounding_box(vertices, &mut lower, &mut upper);
        let diagonal = (upper - lower).length();
        let tolerance = cxx_max(0.01 * diagonal, 1e-9);
        plane.axis = axis;
        plane.offset = 0.5 * (lower[axis as usize] + upper[axis as usize]);
        plane.score = Self::score_plane(vertices, axis, plane.offset, tolerance);
        plane
    }

    /// Mirror-average a per-face cross field: each face is averaged (under
    /// the cross's 4-way symmetry) with the mirrored field of the face
    /// nearest to its mirrored centroid. Faces without a partner are left
    /// unchanged (mirrors `Symmetry::symmetrizeFrameField`).
    pub fn symmetrize_frame_field(
        vertices: &[Vector3],
        triangles: &[Vec<usize>],
        field: &mut Vec<Vector3>,
        plane: &SymmetryPlane,
    ) {
        if !plane.valid() || triangles.is_empty() || field.len() != triangles.len() {
            return;
        }

        let mut centroids = vec![Vector3::default(); triangles.len()];
        let mut normals = vec![Vector3::default(); triangles.len()];
        for f in 0..triangles.len() {
            let triangle = &triangles[f];
            if triangle.len() < 3 {
                continue;
            }
            let a = vertices[triangle[0]];
            let b = vertices[triangle[1]];
            let c = vertices[triangle[2]];
            centroids[f] = (a + b + c) / 3.0;
            normals[f] = Vector3::normal(&a, &b, &c);
        }
        let diagonal = bounding_diagonal(&centroids);
        let grid = PointGrid::new(&centroids, partner_cell_size(&centroids, diagonal));

        // Match a mirrored tangent vector against the cross it is averaged into:
        // project onto the face tangent plane, then pick the k*90-degree rotation
        // about the normal closest to the face's own field direction.
        let match_cross = |own: &Vector3, mirrored: &Vector3, normal: &Vector3| {
            let axis = normal.normalized();
            let mut tangent = *mirrored - axis * Vector3::dot_product(mirrored, &axis);
            if tangent.length() <= 1e-12 {
                return Vector3::default();
            }
            tangent.normalize();
            let own_direction = own.normalized();
            let perpendicular = Vector3::cross_product(&axis, &tangent);
            let mut best = tangent;
            let mut best_dot = Vector3::dot_product(&tangent, &own_direction);
            let candidates = [perpendicular, -tangent, -perpendicular];
            for candidate in &candidates {
                let dot = Vector3::dot_product(candidate, &own_direction);
                if dot > best_dot {
                    best_dot = dot;
                    best = *candidate;
                }
            }
            best
        };

        let mut done = vec![false; triangles.len()];
        for f in 0..triangles.len() {
            if done[f] || field[f].length() <= 1e-12 || normals[f].length() <= 1e-12 {
                continue;
            }
            let partner = grid.nearest(&Self::mirror_point(&centroids[f], plane), 0.0);
            let Some(partner) = partner else {
                continue;
            };
            if normals[partner].length() <= 1e-12 {
                continue;
            }
            if partner == f {
                // A face straddling the plane mirrors onto itself: average its
                // field with its own mirror so the cross is plane-symmetric.
                let matched = match_cross(
                    &field[f],
                    &Self::mirror_direction(&field[f], plane),
                    &normals[f],
                );
                if matched.length() <= 1e-12 {
                    done[f] = true;
                    continue;
                }
                field[f] = (field[f].normalized() + matched).normalized();
                done[f] = true;
                continue;
            }
            if done[partner] || field[partner].length() <= 1e-12 {
                continue;
            }
            let matched = match_cross(
                &field[f],
                &Self::mirror_direction(&field[partner], plane),
                &normals[f],
            );
            if matched.length() <= 1e-12 {
                continue;
            }
            let averaged = (field[f].normalized() + matched).normalized();
            if averaged.length() <= 1e-12 {
                continue;
            }
            field[f] = averaged;
            // The partner gets the exact mirror, re-projected onto its own
            // tangent plane (its plane is the mirror of this face's plane).
            let axis = normals[partner].normalized();
            let mut mirrored = Self::mirror_direction(&averaged, plane);
            mirrored = mirrored - axis * Vector3::dot_product(&mirrored, &axis);
            if mirrored.length() > 1e-12 {
                field[partner] = mirrored.normalized();
            }
            done[f] = true;
            done[partner] = true;
        }
    }

    /// Snap vertices to exact mirror symmetry: every vertex ends up either
    /// exactly mirrored by a partner or exactly on the plane (mirrors
    /// `Symmetry::symmetrizeVertices`).
    pub fn symmetrize_vertices(vertices: &mut Vec<Vector3>, plane: &SymmetryPlane) {
        if !plane.valid() || vertices.is_empty() {
            return;
        }
        let axis = plane.axis as usize;
        let diagonal = bounding_diagonal(vertices);
        // Pairing runs on the original positions: the grid is built once and the
        // averaged positions are written into a separate buffer, so every vertex
        // pairs against the unmoved mesh.
        let grid = PointGrid::new(vertices, partner_cell_size(vertices, diagonal));
        let original = vertices.clone();
        let mut snapped = vertices.clone();
        let mut done = vec![false; vertices.len()];
        for i in 0..original.len() {
            if done[i] {
                continue;
            }
            let partner = grid.nearest(&Self::mirror_point(&original[i], plane), 0.0);
            let Some(partner) = partner else {
                done[i] = true;
                continue;
            };
            if partner == i {
                snapped[i][axis] = plane.offset;
                done[i] = true;
                continue;
            }
            if done[partner] {
                // The partner already belongs to an exact pair; coincide with its
                // mirror rather than breaking that pair's exactness.
                snapped[i] = Self::mirror_point(&snapped[partner], plane);
                done[i] = true;
                continue;
            }
            // Exact mirror pair: shared coordinates are averaged, the axis
            // coordinates are mirrored about the plane through their midpoint.
            let mut first = original[i];
            let mut second = original[partner];
            for c in 0..3 {
                if c == axis {
                    continue;
                }
                let mean = 0.5 * (first[c] + second[c]);
                first[c] = mean;
                second[c] = mean;
            }
            // Fused inner form, like mirror_point (see the module docs).
            first[axis] = 0.5 * (first[axis] + plane.offset.mul_add(2.0, -second[axis]));
            second[axis] = plane.offset.mul_add(2.0, -first[axis]);
            snapped[i] = first;
            snapped[partner] = second;
            done[i] = true;
            done[partner] = true;
        }
        *vertices = snapped;
    }
}
