//! Port of `core/density.*` (`retopo.core.density`).
//!
//! Line-by-line mirror: same items in the same order, same thresholds
//! ([`Density::MIN_MULTIPLIER`] / [`Density::MAX_MULTIPLIER`]).
//!
//! FP notes: `density.cpp` has no multiply-add site of its own — every FP
//! op flows through [`Vector3`] (`length_squared`, `cross_product`,
//! `length`), whose Clang FMA fusion is already transcribed with explicit
//! `mul_add` in [`crate::vector3`], so this module needs none. The two
//! `std::min`/`std::max` call sites use [`cxx_min`]/[`cxx_max`], which
//! replicate the `(b < a) ? b : a` / `(a < b) ? b : a` NaN propagation
//! (`f64::min`/`f64::max` return the non-NaN operand instead, which
//! diverges on NaN coordinates).
//!
//! Out-of-contract inputs: cell coordinates that overflow `i64` (huge
//! coordinates with a ~1e-9 cell) are UB in C++ (`long long`
//! conversion); the port saturates (`as i64`, matching ARM64 `fcvtzs`).
//! The oracle stays in range.

use crate::vector3::Vector3;
use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hash, Hasher};

/// Local density control (mirrors `AutoRemesher::Density`: a stateless
/// namespace class, so every item is associated).
pub struct Density;

impl Density {
    /// Supported per-vertex density multiplier range. 1.0 leaves a region
    /// unchanged; values outside the range clamp to it.
    pub const MIN_MULTIPLIER: f64 = 0.25;
    /// Supported per-vertex density multiplier range. 1.0 leaves a region
    /// unchanged; values outside the range clamp to it.
    pub const MAX_MULTIPLIER: f64 = 4.0;

    /// Clamp every entry to [`MIN_MULTIPLIER`](Self::MIN_MULTIPLIER)..
    /// [`MAX_MULTIPLIER`](Self::MAX_MULTIPLIER) (non-finite values become
    /// 1.0). An empty or all-1.0 field normalizes to empty, which is the
    /// OFF state: callers skip all density work when the result is empty, so
    /// the pipeline stays bit-identical to a run without any field.
    #[must_use]
    #[allow(clippy::manual_clamp)] // Port mirrors the C++ branch ladder; `clamp` differs on NaN.
    pub fn normalize_field(field: &[f64]) -> Vec<f64> {
        if field.is_empty() {
            return Vec::new();
        }
        let mut normalized = Vec::with_capacity(field.len());
        let mut uniform = true;
        for &value in field {
            let mut value = value;
            if !value.is_finite() {
                value = 1.0;
            } else if value < Self::MIN_MULTIPLIER {
                value = Self::MIN_MULTIPLIER;
            } else if value > Self::MAX_MULTIPLIER {
                value = Self::MAX_MULTIPLIER;
            }
            normalized.push(value);
            if value != 1.0 {
                uniform = false;
            }
        }
        if uniform {
            return Vec::new();
        }
        normalized
    }

    /// Target edge-length scale for a density multiplier: quads-per-area
    /// scale as 1/h^2, so d times the quads need edges 1/sqrt(d) as long.
    #[must_use]
    pub fn edge_scale_for(density: f64) -> f64 {
        if !density.is_finite() || density <= 0.0 {
            return 1.0;
        }
        1.0 / density.sqrt()
    }

    /// Nearest-neighbor resample of a per-vertex scalar field onto a new
    /// point set (used to carry the density mask across decimation and
    /// isotropic remeshing, which both retopologize the island). Returns
    /// uniform 1.0 on any size mismatch so the caller normalizes back to OFF.
    #[must_use]
    pub fn resample_nearest(
        src_positions: &[Vector3],
        src_field: &[f64],
        dst_positions: &[Vector3],
    ) -> Vec<f64> {
        let mut resampled = vec![1.0; dst_positions.len()];
        if src_positions.is_empty()
            || src_field.len() != src_positions.len()
            || dst_positions.is_empty()
        {
            return resampled;
        }
        // Cell sized for surface-distributed points: the mean neighbor spacing of
        // N points over an area ~ D^2 scales as D/sqrt(N).
        let diagonal = bounding_diagonal(src_positions);
        let cell_size = if diagonal > 0.0 {
            cxx_max(4.0 * diagonal / (src_positions.len() as f64).sqrt(), 1e-9)
        } else {
            1e-9
        };
        let grid = PointGrid::new(src_positions, cell_size);
        for (i, dst) in dst_positions.iter().enumerate() {
            if let Some(hit) = grid.nearest(dst) {
                resampled[i] = src_field[hit];
            }
        }
        resampled
    }

    /// Multiply each face scaling entry by the edge scale of its averaged
    /// vertex density, then renormalize all entries so SUM A_f/m_f^2 is
    /// preserved: quads move into dense regions without changing the total
    /// budget. No-op on any size mismatch.
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // Port mirrors the C++ negated comparison; `!(a<b)` differs from `a>=b` on NaN.
    pub fn apply_to_scaling_field(
        vertices: &[Vector3],
        triangles: &[Vec<usize>],
        density_per_vertex: &[f64],
        face_scaling: &mut [f64],
    ) {
        if density_per_vertex.len() != vertices.len()
            || face_scaling.len() != triangles.len()
            || vertices.is_empty()
            || triangles.is_empty()
        {
            return;
        }
        let mut face_areas = vec![0.0; triangles.len()];
        for (i, triangle) in triangles.iter().enumerate() {
            if triangle.len() >= 3 {
                let e0 = vertices[triangle[1]] - vertices[triangle[0]];
                let e1 = vertices[triangle[2]] - vertices[triangle[0]];
                face_areas[i] = 0.5 * Vector3::cross_product(&e0, &e1).length();
            }
        }
        let mut budget_before = 0.0;
        for i in 0..triangles.len() {
            let m = face_scaling[i];
            if m > 0.0 {
                budget_before += face_areas[i] / (m * m);
            }
        }
        if !(budget_before > 0.0) || !budget_before.is_finite() {
            return;
        }
        for (i, triangle) in triangles.iter().enumerate() {
            let mut face_density = 0.0;
            for &v in triangle {
                face_density += if v < density_per_vertex.len() {
                    density_per_vertex[v]
                } else {
                    1.0
                };
            }
            face_density /= triangle.len() as f64;
            let scale = Self::edge_scale_for(face_density);
            if scale > 0.0 && scale.is_finite() {
                face_scaling[i] *= scale;
            }
        }
        let mut budget_after = 0.0;
        for i in 0..triangles.len() {
            let m = face_scaling[i];
            if m > 0.0 && m.is_finite() {
                budget_after += face_areas[i] / (m * m);
            }
        }
        if !(budget_after > 0.0) || !budget_after.is_finite() {
            return;
        }
        let rescale = (budget_after / budget_before).sqrt();
        if rescale > 0.0 && rescale.is_finite() {
            for m in face_scaling.iter_mut() {
                *m *= rescale;
            }
        }
    }
}

/// `std::min<double>` semantics: `(b < a) ? b : a` — a NaN `a` stays NaN.
/// (`f64::min` returns the non-NaN operand instead.)
#[inline]
fn cxx_min(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

/// `std::max<double>` semantics: `(a < b) ? b : a` — a NaN `a` stays NaN.
/// (`f64::max` returns the non-NaN operand instead.)
#[inline]
fn cxx_max(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}

#[derive(Clone, Copy, PartialEq, Eq)]
struct CellKey {
    x: i64,
    y: i64,
    z: i64,
}

impl Hash for CellKey {
    fn hash<H: Hasher>(&self, state: &mut H) {
        // Little-endian bytes, x then y then z — the same byte order
        // `CellKeyHash` mixes in density.cpp.
        state.write_i64(self.x);
        state.write_i64(self.y);
        state.write_i64(self.z);
    }
}

/// FNV-1a over the key bytes: the same hash `density.cpp` uses
/// (`CellKeyHash`), and far cheaper than `RandomState` SipHash on this
/// lookup-hot path (measured ~4x on the timing loop before the switch).
/// Default is the FNV offset basis (a fresh hasher has hashed nothing).
#[derive(Clone, Copy)]
struct FnvHasher(u64);

impl Default for FnvHasher {
    fn default() -> Self {
        Self(1469598103934665603)
    }
}

impl Hasher for FnvHasher {
    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 ^= b as u64;
            self.0 = self.0.wrapping_mul(1099511628211);
        }
    }

    fn finish(&self) -> u64 {
        self.0
    }
}

type CellMap = HashMap<CellKey, Vec<usize>, BuildHasherDefault<FnvHasher>>;

// Uniform-grid exact nearest lookup. Queries expand ring by ring and stop
// once the closest unvisited ring is farther than the best hit, so
// near-coincident points (decimated/remeshed verts on the same surface)
// resolve in the first rings while far queries still terminate.
//
// Only point lookups observe the map (never iteration order), so the
// hash function cannot affect outputs; FNV-1a mirrors C++ bit for bit.
struct PointGrid<'a> {
    points: &'a [Vector3],
    cell_size: f64,
    cells: CellMap,
}

impl<'a> PointGrid<'a> {
    fn new(points: &'a [Vector3], cell_size: f64) -> Self {
        let cell_size = if cell_size > 0.0 { cell_size } else { 1e-9 };
        // Borrowck restructure: `cell_of` takes the size explicitly instead
        // of `&self` so the entry map can be filled without aliasing.
        let mut cells: CellMap =
            HashMap::with_capacity_and_hasher(points.len() * 2 + 1, Default::default());
        for (i, point) in points.iter().enumerate() {
            cells
                .entry(Self::cell_of(point, cell_size))
                .or_default()
                .push(i);
        }
        Self {
            points,
            cell_size,
            cells,
        }
    }

    fn nearest(&self, query: &Vector3) -> Option<usize> {
        if self.points.is_empty() {
            return None;
        }
        let center = Self::cell_of(query, self.cell_size);
        let mut best: Option<usize> = None;
        let mut best_distance_squared = f64::INFINITY;
        let max_ring = self.points.len() as i64 + 1;
        let mut ring: i64 = 0;
        while ring <= max_ring {
            if ring > 0 {
                // Same evaluation order as C++: one multiply, then squared.
                let d = (ring - 1) as f64 * self.cell_size;
                if d * d >= best_distance_squared {
                    break;
                }
            }
            for dx in -ring..=ring {
                for dy in -ring..=ring {
                    for dz in -ring..=ring {
                        if dx.abs().max(dy.abs()).max(dz.abs()) != ring {
                            continue;
                        }
                        // Wrapping: out-of-range cell coordinates are UB in
                        // C++ (signed overflow); wrapping matches release
                        // builds in practice and never panics.
                        let key = CellKey {
                            x: center.x.wrapping_add(dx),
                            y: center.y.wrapping_add(dy),
                            z: center.z.wrapping_add(dz),
                        };
                        let Some(indices) = self.cells.get(&key) else {
                            continue;
                        };
                        for &index in indices {
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
            ring += 1;
        }
        best
    }

    fn cell_of(point: &Vector3, cell_size: f64) -> CellKey {
        CellKey {
            x: (point.x() / cell_size).floor() as i64,
            y: (point.y() / cell_size).floor() as i64,
            z: (point.z() / cell_size).floor() as i64,
        }
    }
}

fn bounding_diagonal(points: &[Vector3]) -> f64 {
    let mut lower = points[0];
    let mut upper = points[0];
    for point in points {
        for i in 0..3 {
            lower[i] = cxx_min(lower[i], point[i]);
            upper[i] = cxx_max(upper[i], point[i]);
        }
    }
    (upper - lower).length()
}
