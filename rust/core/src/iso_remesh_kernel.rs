//! Port of `thirdparty/isotropicremesher/` (AABB tree, halfedge mesh, remesher
//! kernel) backing [`crate::isotropic_remesher`].
//!
//! Line-by-line structural mirror: same types, same functions in the same
//! order, same thresholds (`1e-6` degenerate area factor, `20` leaf size,
//! `4/5`/`4/3` edge bounds, `0.86602540378` equilateral factor). Deliberate
//! restructures, all proven by the differential oracle
//! (`rust/core/tests/isotropic_remesher_diff.rs`):
//! - Pointer soup -> index arenas: vertices/faces/halfedges/tree-nodes live
//!   in `Vec`s and link by index. Allocation order (append-only, removal by
//!   flag) matches the C++ linked lists, so traversal order is identical.
//!   Links that are never null after linking (`next`/`previous`/`start`/
//!   `left`) are plain `usize`; nullable links (`opposite`, `first`,
//!   list ends) are `Option<usize>`.
//! - `std::unordered_map` (halfedge pairing) -> `BTreeMap`, `std::set`
//!   (collapse neighbor sets) -> `BTreeSet`: the C++ iteration order is
//!   address/hash dependent, but both uses are order-free (symmetric
//!   opposite-linking, intersection *count*), so the ordered maps give the
//!   same result deterministically.
//! - `iterateVertexHalfedges` + `std::function` handlers -> collect-then-visit
//!   (`collect_vertex_halfedges` + plain loops): every handler either only
//!   reads (early exit on the first `false` is result-identical) or only
//!   rewrites `start_vertex` (which the traversal never follows), and no
//!   handler mutates the followed links mid-iteration, so the visit sequence
//!   and the outcomes are unchanged.
//! - The AABB tree owns its box list (moved in at construction) instead of
//!   borrowing the remesher's member: the boxes are only read through the
//!   tree after construction, so this is behavior-identical and avoids a
//!   self-referential struct.
//! - Non-triangle input faces are skipped (counted, see `build_anomalies`,
//!   instead of the C++ per-face stderr line) instead of leaving an orphan
//!   face behind: the C++ null-derefs on the orphan in
//!   `averageEdgeLength` before producing any output, so no observable
//!   behavior changes (out-of-contract input).
//! - The `bottomVertex` repair in `collapse_edge` checks `bottomFace` twice,
//!   mirroring the apparent C++ typo (`topFace` missing) exactly.
//!
//! FP audit (mandatory for this geometry code): all three thirdparty TUs were
//! compiled to LLVM IR with the project's Release flags (`-O3
//! -funroll-loops`, brew Clang 23.1.2, ARM64) and every `llvm.fma`/`fmuladd`
//! was mapped back to source. Every fused site sits inside the thirdparty
//! `vector3.h` inlines (`lengthSquared`, `length`, `normalized`,
//! `normalize`, `crossProduct`, `dotProduct`, `normal`) whose formulas are
//! character-identical to `core/vector3.cppm` — using
//! [`crate::vector3::Vector3`] inherits the exact `mul_add` transcription.
//! Zero kernel-level fusion: the PN midpoint, the `+= n*area*corner`
//! accumulations, `intersect_segment_and_plane`'s final add and
//! `project_point_on_line` all codegen as separate `fmul`/`fadd`/`fsub`/
//! `fdiv` (verified in IR; `/8.0` becomes `*0.125`, exact). `std::pow(x, 2)`
//! lowers to `x*x`; the flip `deviation` goes `sitofp`/`fmul`/`fptosi` and
//! is mirrored in `f64`.
//!
//! Not ported (unused by the remesh path, verified by call-site grep):
//! thirdparty `vector2.h`, `Vector3::project`, `intersectSegmentAndTriangle`,
//! elementwise `operator*`/`operator/` — `crate::vector2`/`crate::vector3`
//! cover what little of that surface the kernel needs.

use crate::progress::ProgressHandler;
use crate::vector3::Vector3;
use std::cmp::Ordering;
use std::collections::btree_map::Entry;
use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Thirdparty Vector3 extras used by the remesh path
// (core::Vector3 has no equivalent; IR-verified unfused, plain operators).
// ---------------------------------------------------------------------------

/// Mirrors thirdparty `Vector3::containsNan`.
#[inline]
#[must_use]
fn contains_nan(v: &Vector3) -> bool {
    v[0].is_nan() || v[1].is_nan() || v[2].is_nan()
}

/// Mirrors thirdparty `Vector3::containsInf`.
#[inline]
#[must_use]
fn contains_inf(v: &Vector3) -> bool {
    v[0].is_infinite() || v[1].is_infinite() || v[2].is_infinite()
}

/// Mirrors thirdparty `Vector3::intersectSegmentAndPlane` (`vector3.h:278`):
/// `Some` intersection point, `None` when parallel or outside the segment.
/// The C++ out-param becomes the return value; order of checks is unchanged.
fn intersect_segment_and_plane(
    segment_point0: &Vector3,
    segment_point1: &Vector3,
    point_on_plane: &Vector3,
    plane_normal: &Vector3,
) -> Option<Vector3> {
    let u = *segment_point1 - *segment_point0;
    let w = *segment_point0 - *point_on_plane;
    let d = Vector3::dot_product(plane_normal, &u);
    let n = Vector3::dot_product(&-*plane_normal, &w);
    if d.abs() <= f64::EPSILON {
        return None;
    }
    let s = n / d;
    if s < 0.0 || s > 1.0 || s.is_nan() || s.is_infinite() {
        return None;
    }
    Some(*segment_point0 + s * u)
}

/// Mirrors thirdparty `Vector3::projectPointOnLine` (`vector3.h:323`).
fn project_point_on_line(
    point: &Vector3,
    line_point_a: &Vector3,
    line_point_b: &Vector3,
) -> Vector3 {
    let a_to_point = *point - *line_point_a;
    let a_to_b = *line_point_b - *line_point_a;
    *line_point_a
        + Vector3::dot_product(&a_to_point, &a_to_b) / Vector3::dot_product(&a_to_b, &a_to_b)
            * a_to_b
}

/// `std::max(double, double)` semantics (`(a < b) ? b : a`): unlike
/// `f64::max`, a leading NaN propagates. Only `is_triangle_degenerate` needs
/// this (NaN edge lengths from non-finite input).
#[inline]
#[must_use]
fn std_max(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}

// ---------------------------------------------------------------------------
// AxisAlignedBoudingBox (spelling corrected from the C++ typo)
// ---------------------------------------------------------------------------

/// Axis-aligned bounding box (mirrors `AxisAlignedBoudingBox`).
#[derive(Clone, Debug)]
pub struct AxisAlignedBoundingBox {
    min: Vector3,
    max: Vector3,
    sum: Vector3,
    num: usize,
    center: Vector3,
}

impl Default for AxisAlignedBoundingBox {
    fn default() -> Self {
        Self {
            min: Vector3::new(f64::MAX, f64::MAX, f64::MAX),
            max: Vector3::new(f64::MIN, f64::MIN, f64::MIN),
            sum: Vector3::default(),
            num: 0,
            center: Vector3::default(),
        }
    }
}

impl AxisAlignedBoundingBox {
    /// Mirrors `update` (max check first, then min, then the sum).
    pub fn update(&mut self, vertex: &Vector3) {
        for i in 0..3 {
            if vertex[i] > self.max[i] {
                self.max[i] = vertex[i];
            }
            if vertex[i] < self.min[i] {
                self.min[i] = vertex[i];
            }
            self.sum[i] += vertex[i];
        }
        self.num += 1;
    }

    /// Mirrors `updateCenter` (`m_center = m_sum /= (float)m_num`): the count
    /// narrows through `f32` before the division, and `sum` is divided too.
    pub fn update_center(&mut self) {
        if 0 == self.num {
            return;
        }
        self.sum /= self.num as f32 as f64;
        self.center = self.sum;
    }

    #[inline]
    #[must_use]
    pub fn center(&self) -> &Vector3 {
        &self.center
    }

    #[inline]
    #[must_use]
    pub fn lower_bound(&self) -> &Vector3 {
        &self.min
    }

    #[inline]
    #[must_use]
    pub fn upper_bound(&self) -> &Vector3 {
        &self.max
    }

    #[inline]
    pub fn lower_bound_mut(&mut self) -> &mut Vector3 {
        &mut self.min
    }

    #[inline]
    pub fn upper_bound_mut(&mut self) -> &mut Vector3 {
        &mut self.max
    }

    /// Mirrors `intersectWithAt` (unused by the remesh path; kept for a
    /// complete mirror). The 4-element sort uses `<`-with-Equal-fallback so
    /// non-finite bounds stay ordered instead of tripping `partial_cmp`.
    pub fn intersect_with_at(&self, other: &Self, result: &mut Self) -> bool {
        let other_min = other.lower_bound();
        let other_max = other.upper_bound();
        for i in 0..3 {
            if self.min[i] <= other_max[i] && self.max[i] >= other_min[i] {
                continue;
            }
            return false;
        }
        for i in 0..3 {
            let mut points = [self.min[i], other_max[i], self.max[i], other_min[i]];
            points.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
            result.lower_bound_mut()[i] = points[1];
            result.upper_bound_mut()[i] = points[2];
        }
        true
    }

    /// Mirrors `intersectWith`.
    #[must_use]
    pub fn intersect_with(&self, other: &Self) -> bool {
        let other_min = other.lower_bound();
        let other_max = other.upper_bound();
        for i in 0..3 {
            if self.min[i] <= other_max[i] && self.max[i] >= other_min[i] {
                continue;
            }
            return false;
        }
        true
    }
}

// ---------------------------------------------------------------------------
// AxisAlignedBoudingBoxTree
// ---------------------------------------------------------------------------

/// Tree node (mirrors `AxisAlignedBoudingBoxTree::Node`; children are arena
/// indices, `None` for a leaf).
#[derive(Clone, Debug, Default)]
pub struct AabbTreeNode {
    bounding_box: AxisAlignedBoundingBox,
    center: Vector3,
    box_indices: Vec<usize>,
    left: Option<usize>,
    right: Option<usize>,
}

impl AabbTreeNode {
    #[inline]
    #[must_use]
    fn is_leaf(&self) -> bool {
        self.left.is_none() && self.right.is_none()
    }
}

/// Bounding-box tree over the input triangles (mirrors
/// `AxisAlignedBoudingBoxTree`). Restructure: owns its box list (moved in at
/// construction) instead of borrowing the caller's member; nodes live in an
/// arena instead of a pointer tree.
#[derive(Clone, Debug, Default)]
pub struct AxisAlignedBoundingBoxTree {
    boxes: Vec<AxisAlignedBoundingBox>,
    nodes: Vec<AabbTreeNode>,
    box_indices_order_list: Vec<usize>,
    spans: [(usize, f32); 3],
}

impl AxisAlignedBoundingBoxTree {
    /// Mirrors `m_leafMaxNodeSize`.
    const LEAF_MAX_NODE_SIZE: usize = 20;

    /// Mirrors the constructor (boxes moved in; the root center accumulates
    /// the box centers and divides by the `(float)` count).
    #[must_use]
    pub fn new(
        boxes: Vec<AxisAlignedBoundingBox>,
        box_indices: Vec<usize>,
        outter_box: AxisAlignedBoundingBox,
    ) -> Self {
        let mut tree = Self {
            boxes,
            nodes: Vec::new(),
            box_indices_order_list: Vec::new(),
            spans: [(0, 0.0); 3],
        };
        let mut root = AabbTreeNode {
            bounding_box: outter_box,
            box_indices,
            ..Default::default()
        };
        if !root.box_indices.is_empty() {
            for box_index in root.box_indices.clone() {
                root.center += tree.boxes[box_index].center;
            }
            root.center /= root.box_indices.len() as f32 as f64;
        }
        tree.nodes.push(root);
        tree.split_node(0);
        tree
    }

    /// Root node index (mirrors `root()`; the root is always node 0).
    #[inline]
    #[must_use]
    pub fn root(&self) -> usize {
        0
    }

    /// Mirrors `boxes()`.
    #[inline]
    #[must_use]
    pub fn boxes(&self) -> &[AxisAlignedBoundingBox] {
        &self.boxes
    }

    /// Mirrors `test` (the destructor/`deleteNode` pair has no equivalent:
    /// the arena drops as one).
    pub fn test(
        &self,
        first: usize,
        second_tree: &Self,
        second: usize,
        pairs: &mut Vec<(usize, usize)>,
    ) {
        self.test_nodes(first, second_tree, second, pairs);
    }

    /// Mirrors `testNodes`.
    fn test_nodes(
        &self,
        first: usize,
        second_tree: &Self,
        second: usize,
        pairs: &mut Vec<(usize, usize)>,
    ) {
        if self.nodes[first]
            .bounding_box
            .intersect_with(&second_tree.nodes[second].bounding_box)
        {
            if self.nodes[first].is_leaf() {
                if second_tree.nodes[second].is_leaf() {
                    for &a in &self.nodes[first].box_indices {
                        for &b in &second_tree.nodes[second].box_indices {
                            if self.boxes[a].intersect_with(&second_tree.boxes[b]) {
                                pairs.push((a, b));
                            }
                        }
                    }
                } else {
                    let left = second_tree.nodes[second].left;
                    let right = second_tree.nodes[second].right;
                    // Internal nodes always have both children (set together
                    // in `split_node`); the debug assert documents it.
                    debug_assert!(left.is_some() && right.is_some());
                    if let (Some(left), Some(right)) = (left, right) {
                        self.test_nodes(first, second_tree, left, pairs);
                        self.test_nodes(first, second_tree, right, pairs);
                    }
                }
            } else if second_tree.nodes[second].is_leaf() {
                let left = self.nodes[first].left;
                let right = self.nodes[first].right;
                debug_assert!(left.is_some() && right.is_some());
                if let (Some(left), Some(right)) = (left, right) {
                    self.test_nodes(left, second_tree, second, pairs);
                    self.test_nodes(right, second_tree, second, pairs);
                }
            } else if self.nodes[first].box_indices.len()
                < second_tree.nodes[second].box_indices.len()
            {
                let left = second_tree.nodes[second].left;
                let right = second_tree.nodes[second].right;
                debug_assert!(left.is_some() && right.is_some());
                if let (Some(left), Some(right)) = (left, right) {
                    self.test_nodes(first, second_tree, left, pairs);
                    self.test_nodes(first, second_tree, right, pairs);
                }
            } else {
                let left = self.nodes[first].left;
                let right = self.nodes[first].right;
                debug_assert!(left.is_some() && right.is_some());
                if let (Some(left), Some(right)) = (left, right) {
                    self.test_nodes(left, second_tree, second, pairs);
                    self.test_nodes(right, second_tree, second, pairs);
                }
            }
        }
    }

    /// Mirrors `splitNode` exactly, including the scratch-list fill order
    /// (left side downward, right side upward), the empty-side rebalance,
    /// the `(float)` center divisions, and the left-before-right recursion.
    fn split_node(&mut self, node: usize) {
        let box_indices = self.nodes[node].box_indices.clone();
        if box_indices.len() <= Self::LEAF_MAX_NODE_SIZE {
            return;
        }
        let lower = *self.nodes[node].bounding_box.lower_bound();
        let upper = *self.nodes[node].bounding_box.upper_bound();
        for i in 0..3 {
            self.spans[i] = (i, (upper[i] - lower[i]) as f32);
        }
        // `std::max_element` keeps the FIRST maximum (`>` comparison).
        let mut longest_axis = 0;
        for i in 1..3 {
            if self.spans[i].1 > self.spans[longest_axis].1 {
                longest_axis = i;
            }
        }
        let split_point = self.nodes[node].center[longest_axis];
        let left = self.nodes.len();
        self.nodes.push(AabbTreeNode::default());
        let right = self.nodes.len();
        self.nodes.push(AabbTreeNode::default());
        self.nodes[node].left = Some(left);
        self.nodes[node].right = Some(right);
        self.box_indices_order_list
            .resize(box_indices.len() + box_indices.len() + 2, 0);
        let mut left_offset = box_indices.len();
        let mut right_offset = box_indices.len() - 1;
        let mut left_count = 0;
        let mut right_count = 0;
        for &box_index in &box_indices {
            let center = self.boxes[box_index].center[longest_axis];
            if center < split_point {
                left_offset -= 1;
                self.box_indices_order_list[left_offset] = box_index;
                left_count += 1;
            } else {
                right_offset += 1;
                self.box_indices_order_list[right_offset] = box_index;
                right_count += 1;
            }
        }

        // The C++ also trims the surviving side's count here; only the
        // counts feeding `middle` below are kept (the trimmed values are
        // never read afterwards).
        if 0 == left_count {
            left_count = right_count / 2;
            left_offset = right_offset - box_indices.len() + 1;
        } else if 0 == right_count {
            left_count -= left_count / 2;
            right_offset = left_offset + box_indices.len() - 1;
        }

        let middle = left_offset + left_count - 1;

        for i in left_offset..=middle {
            let box_index = self.box_indices_order_list[i];
            let lower = *self.boxes[box_index].lower_bound();
            let upper = *self.boxes[box_index].upper_bound();
            let center = *self.boxes[box_index].center();
            self.nodes[left].bounding_box.update(&lower);
            self.nodes[left].bounding_box.update(&upper);
            self.nodes[left].box_indices.push(box_index);
            self.nodes[left].center += center;
        }

        for i in middle + 1..=right_offset {
            let box_index = self.box_indices_order_list[i];
            let lower = *self.boxes[box_index].lower_bound();
            let upper = *self.boxes[box_index].upper_bound();
            let center = *self.boxes[box_index].center();
            self.nodes[right].bounding_box.update(&lower);
            self.nodes[right].bounding_box.update(&upper);
            self.nodes[right].box_indices.push(box_index);
            self.nodes[right].center += center;
        }

        let left_len = self.nodes[left].box_indices.len();
        self.nodes[left].center /= left_len as f32 as f64;
        self.split_node(left);

        let right_len = self.nodes[right].box_indices.len();
        self.nodes[right].center /= right_len as f32 as f64;
        self.split_node(right);
    }
}

// ---------------------------------------------------------------------------
// IsotropicHalfedgeMesh
// ---------------------------------------------------------------------------

/// Placeholder for links that are set right after allocation (never read
/// while unset on any path; indexing it would panic, where the C++ would
/// null-deref).
const UNSET_LINK: usize = usize::MAX;

/// Mesh vertex (mirrors `IsotropicHalfedgeMesh::Vertex`; the leading
/// underscores on the computed fields are dropped: `_valence` -> `valence`,
/// `_isBoundary` -> `is_boundary`, `_normal` -> `normal`, `_smoothNormal` ->
/// `smooth_normal`). Fields are public like the C++ struct's.
#[derive(Clone, Debug)]
pub struct IsoVertex {
    pub position: Vector3,
    pub first_halfedge: Option<usize>,
    pub previous_vertex: Option<usize>,
    pub next_vertex: Option<usize>,
    pub initial_faces: i32,
    pub valence: i32,
    pub is_boundary: bool,
    pub normal: Vector3,
    pub smooth_normal: Vector3,
    pub removed: bool,
    pub debug_index: usize,
    pub output_index: usize,
    pub featured: bool,
    pub target_edge_length: f64,
}

impl Default for IsoVertex {
    fn default() -> Self {
        Self {
            position: Vector3::default(),
            first_halfedge: None,
            previous_vertex: None,
            next_vertex: None,
            initial_faces: 0,
            valence: -1,
            is_boundary: false,
            normal: Vector3::default(),
            smooth_normal: Vector3::default(),
            removed: false,
            debug_index: 0,
            output_index: 0,
            featured: false,
            target_edge_length: 0.0,
        }
    }
}

/// Mesh face (mirrors `IsotropicHalfedgeMesh::Face`). `halfedge` is a plain
/// index: every recorded face gets one immediately (non-triangles are
/// skipped, see the module docs).
#[derive(Clone, Debug)]
pub struct IsoFace {
    pub halfedge: usize,
    pub previous_face: Option<usize>,
    pub next_face: Option<usize>,
    pub normal: Vector3,
    pub removed: bool,
    pub debug_index: usize,
}

impl Default for IsoFace {
    fn default() -> Self {
        Self {
            halfedge: UNSET_LINK,
            previous_face: None,
            next_face: None,
            normal: Vector3::default(),
            removed: false,
            debug_index: 0,
        }
    }
}

/// Mesh halfedge (mirrors `IsotropicHalfedgeMesh::Halfedge`). `start_vertex`,
/// `left_face`, `next_halfedge` and `previous_halfedge` are plain indices
/// (always set by the link step before any read); only `opposite_halfedge`
/// is nullable in the C++ and stays `Option` here.
#[derive(Clone, Debug)]
pub struct IsoHalfedge {
    pub start_vertex: usize,
    pub left_face: usize,
    pub next_halfedge: usize,
    pub previous_halfedge: usize,
    pub opposite_halfedge: Option<usize>,
    pub feature_state: i32,
    pub debug_index: usize,
}

impl Default for IsoHalfedge {
    fn default() -> Self {
        Self {
            start_vertex: UNSET_LINK,
            left_face: UNSET_LINK,
            next_halfedge: UNSET_LINK,
            previous_halfedge: UNSET_LINK,
            opposite_halfedge: None,
            feature_state: -1,
            debug_index: 0,
        }
    }
}

/// A triangle is degenerate when its area is negligible compared to the
/// square of its longest edge (mirrors the file-local
/// `isTriangleDegenerate`; `std::max` nesting and NaN semantics preserved).
fn is_triangle_degenerate(a: &Vector3, b: &Vector3, c: &Vector3) -> bool {
    let longest_edge_squared = std_max(
        std_max((*b - *a).length_squared(), (*c - *b).length_squared()),
        (*a - *c).length_squared(),
    );
    if longest_edge_squared <= 0.0 {
        return true;
    }
    Vector3::area(a, b, c) <= 1e-6 * longest_edge_squared
}

/// Halfedge triangle mesh (mirrors `IsotropicHalfedgeMesh`). Restructure:
/// arena indices instead of pointers (see the module docs).
#[derive(Clone, Debug, Default)]
pub struct IsotropicHalfedgeMesh {
    vertices: Vec<IsoVertex>,
    faces: Vec<IsoFace>,
    halfedges: Vec<IsoHalfedge>,
    first_vertex: Option<usize>,
    last_vertex: Option<usize>,
    first_face: Option<usize>,
    last_face: Option<usize>,
    debug_vertex_index: usize,
    debug_face_index: usize,
    debug_halfedge_index: usize,
    /// Input faces the constructor skipped for not being triangles.
    non_triangle_faces: usize,
    /// Directed edges seen twice (non-manifold or inconsistently
    /// wound input); the first halfedge keeps the map slot.
    repeated_halfedges: usize,
}

impl IsotropicHalfedgeMesh {
    /// Construction anomalies `(non_triangle_faces, repeated_halfedges)`.
    /// The C++ printed one stderr line per occurrence; the counts let
    /// callers report them once, and only when asked to (`--verbose`).
    #[must_use]
    pub fn build_anomalies(&self) -> (usize, usize) {
        (self.non_triangle_faces, self.repeated_halfedges)
    }

    #[inline]
    fn make_halfedge_key(first: usize, second: usize) -> u64 {
        ((first as u64) << 32) | (second as u64)
    }

    #[inline]
    fn swap_halfedge_key(key: u64) -> u64 {
        Self::make_halfedge_key((key & 0xffff_ffff) as usize, (key >> 32) as usize)
    }

    /// Element accessors (the C++ wrapper reaches the same public fields
    /// through raw pointers).
    #[inline]
    #[must_use]
    pub fn vertex(&self, index: usize) -> &IsoVertex {
        &self.vertices[index]
    }

    #[inline]
    pub fn vertex_mut(&mut self, index: usize) -> &mut IsoVertex {
        &mut self.vertices[index]
    }

    #[inline]
    #[must_use]
    pub fn face(&self, index: usize) -> &IsoFace {
        &self.faces[index]
    }

    #[inline]
    #[must_use]
    pub fn halfedge(&self, index: usize) -> &IsoHalfedge {
        &self.halfedges[index]
    }

    /// Mirrors the constructor. Restructure: non-triangle faces are skipped
    /// outright (the C++ orphans them and then null-derefs in
    /// `averageEdgeLength`, so no observable output exists for them);
    /// `BTreeMap` replaces `unordered_map` (order-free use, see module docs).
    #[must_use]
    pub fn new(vertices: &[Vector3], faces: &[Vec<usize>]) -> Self {
        let mut mesh = Self::default();
        let mut halfedge_vertices = Vec::with_capacity(vertices.len());
        for position in vertices {
            let vertex = mesh.new_vertex();
            mesh.vertices[vertex].position = *position;
            halfedge_vertices.push(vertex);
        }

        let mut halfedge_map: BTreeMap<u64, usize> = BTreeMap::new();
        for indices in faces {
            if 3 != indices.len() {
                mesh.non_triangle_faces += 1;
                continue;
            }
            let face = mesh.new_face();
            let mut halfedges = Vec::with_capacity(3);
            for i in 0..3 {
                let j = (i + 1) % 3;
                let first = indices[i];
                let second = indices[j];

                let vertex = halfedge_vertices[first];
                mesh.vertices[vertex].initial_faces += 1;

                let halfedge = mesh.new_halfedge();
                mesh.halfedges[halfedge].start_vertex = vertex;
                mesh.halfedges[halfedge].left_face = face;

                if mesh.faces[face].halfedge == UNSET_LINK {
                    mesh.faces[face].halfedge = halfedge;
                }
                if mesh.vertices[vertex].first_halfedge.is_none() {
                    mesh.vertices[vertex].first_halfedge = Some(halfedge);
                }

                halfedges.push(halfedge);

                // `std::unordered_map::insert` keeps the FIRST halfedge on a
                // duplicate key (failed inserts change nothing); `entry`
                // preserves that (`insert` would overwrite with the last).
                match halfedge_map.entry(Self::make_halfedge_key(first, second)) {
                    Entry::Vacant(slot) => {
                        slot.insert(halfedge);
                    }
                    Entry::Occupied(_) => {
                        mesh.repeated_halfedges += 1;
                    }
                }
            }
            mesh.link_face_halfedges(&halfedges);
        }
        let keys: Vec<u64> = halfedge_map.keys().cloned().collect();
        for key in keys {
            let halfedge = halfedge_map[&key];
            if let Some(&opposite) = halfedge_map.get(&Self::swap_halfedge_key(key)) {
                mesh.halfedges[halfedge].opposite_halfedge = Some(opposite);
                mesh.halfedges[opposite].opposite_halfedge = Some(halfedge);
            }
        }

        let mut vertex = mesh.first_vertex;
        while let Some(index) = vertex {
            vertex = mesh.vertices[index].next_vertex;
            let (valence, is_boundary) = mesh.vertex_valence(index);
            mesh.vertices[index].is_boundary = is_boundary;
            mesh.vertices[index].valence = valence as i32;
            if is_boundary {
                if mesh.vertices[index].valence == mesh.vertices[index].initial_faces + 1 {
                    continue;
                }
            } else if mesh.vertices[index].valence == mesh.vertices[index].initial_faces {
                continue;
            }
            mesh.vertices[index].featured = true;
        }
        mesh
    }

    /// Mirrors `linkFaceHalfedges`.
    fn link_face_halfedges(&mut self, halfedges: &[usize]) {
        for i in 0..halfedges.len() {
            let j = (i + 1) % halfedges.len();
            self.halfedges[halfedges[i]].next_halfedge = halfedges[j];
            self.halfedges[halfedges[j]].previous_halfedge = halfedges[i];
        }
    }

    /// Mirrors `updateFaceHalfedgesLeftFace`.
    fn update_face_halfedges_left_face(&mut self, halfedges: &[usize], left_face: usize) {
        for &halfedge in halfedges {
            self.halfedges[halfedge].left_face = left_face;
        }
    }

    /// Mirrors `linkHalfedgePair`.
    fn link_halfedge_pair(&mut self, first: Option<usize>, second: Option<usize>) {
        if let Some(first) = first {
            self.halfedges[first].opposite_halfedge = second;
        }
        if let Some(second) = second {
            self.halfedges[second].opposite_halfedge = first;
        }
    }

    /// Mirrors `averageEdgeLength`.
    #[must_use]
    pub fn average_edge_length(&self) -> f64 {
        let mut total_length = 0.0;
        let mut halfedge_count = 0usize;
        let mut face = self.first_face;
        while let Some(index) = face {
            face = self.faces[index].next_face;
            let start_halfedge = self.faces[index].halfedge;
            let mut halfedge = start_halfedge;
            loop {
                let next_halfedge = self.halfedges[halfedge].next_halfedge;
                total_length += (self.vertices[self.halfedges[halfedge].start_vertex].position
                    - self.vertices[self.halfedges[next_halfedge].start_vertex].position)
                    .length();
                halfedge_count += 1;
                halfedge = next_halfedge;
                if halfedge == start_halfedge {
                    break;
                }
            }
        }
        if 0 == halfedge_count {
            return 0.0;
        }
        total_length / halfedge_count as f64
    }

    /// Mirrors `newFace`.
    fn new_face(&mut self) -> usize {
        let face = self.faces.len();
        self.faces.push(IsoFace::default());
        self.debug_face_index += 1;
        self.faces[face].debug_index = self.debug_face_index;
        if let Some(last) = self.last_face {
            self.faces[last].next_face = Some(face);
            self.faces[face].previous_face = Some(last);
        } else {
            self.first_face = Some(face);
        }
        self.last_face = Some(face);
        face
    }

    /// Mirrors `newVertex`.
    fn new_vertex(&mut self) -> usize {
        let vertex = self.vertices.len();
        self.vertices.push(IsoVertex::default());
        self.debug_vertex_index += 1;
        self.vertices[vertex].debug_index = self.debug_vertex_index;
        if let Some(last) = self.last_vertex {
            self.vertices[last].next_vertex = Some(vertex);
            self.vertices[vertex].previous_vertex = Some(last);
        } else {
            self.first_vertex = Some(vertex);
        }
        self.last_vertex = Some(vertex);
        vertex
    }

    /// Mirrors `newHalfedge`.
    fn new_halfedge(&mut self) -> usize {
        let halfedge = self.halfedges.len();
        self.halfedges.push(IsoHalfedge::default());
        self.debug_halfedge_index += 1;
        self.halfedges[halfedge].debug_index = self.debug_halfedge_index;
        halfedge
    }

    /// Mirrors `moveToNextFace` (`nullptr` becomes `None`).
    #[must_use]
    pub fn move_to_next_face(&self, face: Option<usize>) -> Option<usize> {
        let mut current = match face {
            None => {
                let first = self.first_face?;
                if !self.faces[first].removed {
                    return Some(first);
                }
                first
            }
            Some(index) => index,
        };
        loop {
            current = self.faces[current].next_face?;
            if !self.faces[current].removed {
                return Some(current);
            }
        }
    }

    /// Mirrors `moveToNextVertex`.
    #[must_use]
    pub fn move_to_next_vertex(&self, vertex: Option<usize>) -> Option<usize> {
        let mut current = match vertex {
            None => {
                let first = self.first_vertex?;
                if !self.vertices[first].removed {
                    return Some(first);
                }
                first
            }
            Some(index) => index,
        };
        loop {
            current = self.vertices[current].next_vertex?;
            if !self.vertices[current].removed {
                return Some(current);
            }
        }
    }

    /// Outgoing halfedges around `vertex` in `iterateVertexHalfedges` visit
    /// order (restructure: collected up front so handlers can mutate through
    /// `&mut self`; the followed links are never mutated mid-iteration, so
    /// the sequence matches the C++ exactly).
    fn collect_vertex_halfedges(&self, vertex: usize) -> Vec<usize> {
        let mut out = Vec::new();
        let start = match self.vertices[vertex].first_halfedge {
            None => return out,
            Some(halfedge) => halfedge,
        };
        let mut loop_halfedge = start;
        loop {
            out.push(loop_halfedge);
            match self.halfedges[loop_halfedge].opposite_halfedge {
                None => {
                    loop_halfedge = start;
                    loop {
                        let previous = self.halfedges[loop_halfedge].previous_halfedge;
                        match self.halfedges[previous].opposite_halfedge {
                            None => break,
                            Some(opposite) => {
                                loop_halfedge = opposite;
                                out.push(loop_halfedge);
                                if loop_halfedge == start {
                                    break;
                                }
                            }
                        }
                    }
                    break;
                }
                Some(opposite) => {
                    loop_halfedge = self.halfedges[opposite].next_halfedge;
                    if loop_halfedge == start {
                        break;
                    }
                }
            }
        }
        out
    }

    /// Mirrors `breakFace`. Restructure: the two out-param halfedge lists are
    /// returned (both call sites pass fresh vectors).
    fn break_face(
        &mut self,
        left_old_face: usize,
        halfedge: usize,
        break_point_vertex: usize,
    ) -> (Vec<usize>, Vec<usize>) {
        let left_face_halfedges = [
            self.halfedges[halfedge].previous_halfedge,
            halfedge,
            self.halfedges[halfedge].next_halfedge,
        ];

        let left_new_face = self.new_face();
        self.faces[left_new_face].halfedge = left_face_halfedges[2];
        self.halfedges[left_face_halfedges[2]].left_face = left_new_face;

        let new0 = self.new_halfedge();
        let new1 = self.new_halfedge();
        let left_new_face_halfedges = vec![new0, new1, left_face_halfedges[2]];
        self.link_face_halfedges(&left_new_face_halfedges);
        self.update_face_halfedges_left_face(&left_new_face_halfedges, left_new_face);
        self.halfedges[new0].start_vertex = self.halfedges[left_face_halfedges[0]].start_vertex;
        self.halfedges[new1].start_vertex = break_point_vertex;

        let new2 = self.new_halfedge();
        let left_old_face_halfedges = vec![left_face_halfedges[0], halfedge, new2];
        self.link_face_halfedges(&left_old_face_halfedges);
        self.update_face_halfedges_left_face(&left_old_face_halfedges, left_old_face);
        self.halfedges[new2].start_vertex = break_point_vertex;

        self.vertices[break_point_vertex].first_halfedge = Some(new1);

        self.link_halfedge_pair(
            Some(left_new_face_halfedges[0]),
            Some(left_old_face_halfedges[2]),
        );

        self.faces[left_old_face].halfedge = left_old_face_halfedges[0];
        (left_new_face_halfedges, left_old_face_halfedges)
    }

    /// Edge midpoint on the PN triangle patch (mirrors the inline formula in
    /// `breakEdge`/`collapseEdge`; same expression as
    /// `IsoRemeshKernel::pn_triangle_edge_midpoint`).
    fn pn_edge_midpoint(p1: &Vector3, p2: &Vector3, n1: &Vector3, n2: &Vector3) -> Vector3 {
        let d12 = Vector3::dot_product(&(*p2 - *p1), n1);
        let d21 = Vector3::dot_product(&(*p1 - *p2), n2);
        let p210 = (2.0 * *p1 + *p2 - d12 * *n1) / 3.0;
        let p120 = (2.0 * *p2 + *p1 - d21 * *n2) / 3.0;
        (*p1 + *p2 + 3.0 * (p210 + p120)) / 8.0
    }

    /// Mirrors `breakEdge`.
    pub fn break_edge(&mut self, halfedge: usize) {
        let left_old_face = self.halfedges[halfedge].left_face;
        let opposite_halfedge = self.halfedges[halfedge].opposite_halfedge;
        let right_old_face = opposite_halfedge.map(|o| self.halfedges[o].left_face);

        let break_point_vertex = self.new_vertex();

        let start_vertex = self.halfedges[halfedge].start_vertex;
        let next_halfedge = self.halfedges[halfedge].next_halfedge;
        let next_vertex = self.halfedges[next_halfedge].start_vertex;
        if !self.vertices[start_vertex].smooth_normal.is_zero()
            && !self.vertices[next_vertex].smooth_normal.is_zero()
        {
            let p1 = self.vertices[start_vertex].position;
            let p2 = self.vertices[next_vertex].position;
            let n1 = self.vertices[start_vertex].smooth_normal;
            let n2 = self.vertices[next_vertex].smooth_normal;
            self.vertices[break_point_vertex].position = Self::pn_edge_midpoint(&p1, &p2, &n1, &n2);
            self.vertices[break_point_vertex].smooth_normal = (n1 + n2).normalized();
        } else {
            self.vertices[break_point_vertex].position =
                (self.vertices[start_vertex].position + self.vertices[next_vertex].position) * 0.5;
        }

        self.vertices[break_point_vertex].featured =
            self.vertices[start_vertex].featured && self.vertices[next_vertex].featured;

        let start_target = self.vertices[start_vertex].target_edge_length;
        let next_target = self.vertices[next_vertex].target_edge_length;
        self.vertices[break_point_vertex].target_edge_length =
            if start_target > 0.0 && next_target > 0.0 {
                (start_target + next_target) * 0.5
            } else {
                start_target + next_target
            };

        let (left_new_face_halfedges, left_old_face_halfedges) =
            self.break_face(left_old_face, halfedge, break_point_vertex);

        if let (Some(right_old_face), Some(opposite_halfedge)) = (right_old_face, opposite_halfedge)
        {
            let (right_new_face_halfedges, right_old_face_halfedges) =
                self.break_face(right_old_face, opposite_halfedge, break_point_vertex);
            self.link_halfedge_pair(
                Some(left_old_face_halfedges[1]),
                Some(right_new_face_halfedges[1]),
            );
            self.link_halfedge_pair(
                Some(left_new_face_halfedges[1]),
                Some(right_old_face_halfedges[1]),
            );
        }
    }

    /// Mirrors `collectVerticesAroundVertex` (`std::set` becomes `BTreeSet`).
    fn collect_vertices_around_vertex(&self, vertex: usize, vertices: &mut BTreeSet<usize>) {
        for halfedge in self.collect_vertex_halfedges(vertex) {
            let next = self.halfedges[halfedge].next_halfedge;
            vertices.insert(self.halfedges[next].start_vertex);
        }
    }

    /// Mirrors `vertexValence` (the `bool*` out-param becomes the second
    /// return value; every caller presets it to `false`, so this is
    /// equivalent). Note the boundary double-count of the start halfedge.
    pub fn vertex_valence(&self, vertex: usize) -> (usize, bool) {
        let start_halfedge = match self.vertices[vertex].first_halfedge {
            None => return (0, false),
            Some(halfedge) => halfedge,
        };

        let mut valence = 0;
        let mut is_boundary = false;

        let mut loop_halfedge = start_halfedge;
        loop {
            valence += 1;
            match self.halfedges[loop_halfedge].opposite_halfedge {
                None => {
                    is_boundary = true;
                    loop_halfedge = start_halfedge;
                    loop {
                        valence += 1;
                        let previous = self.halfedges[loop_halfedge].previous_halfedge;
                        match self.halfedges[previous].opposite_halfedge {
                            None => break,
                            Some(opposite) => {
                                loop_halfedge = opposite;
                                if loop_halfedge == start_halfedge {
                                    break;
                                }
                            }
                        }
                    }
                    break;
                }
                Some(opposite) => {
                    loop_halfedge = self.halfedges[opposite].next_halfedge;
                    if loop_halfedge == start_halfedge {
                        break;
                    }
                }
            }
        }

        (valence, is_boundary)
    }

    /// Mirrors `testLengthSquaredAroundVertex` (`true` = some neighbor would
    /// exceed the limit, i.e. the collapse is rejected).
    fn test_length_squared_around_vertex(
        &self,
        vertex: usize,
        target: &Vector3,
        max_edge_length_squared: f64,
    ) -> bool {
        for halfedge in self.collect_vertex_halfedges(vertex) {
            let next = self.halfedges[halfedge].next_halfedge;
            if (self.vertices[self.halfedges[next].start_vertex].position - *target)
                .length_squared()
                > max_edge_length_squared
            {
                return true;
            }
        }
        false
    }

    /// Mirrors `pointerVertexToNewVertex`.
    fn pointer_vertex_to_new_vertex(&mut self, vertex: usize, replacement: usize) {
        for halfedge in self.collect_vertex_halfedges(vertex) {
            self.halfedges[halfedge].start_vertex = replacement;
        }
    }

    /// Mirrors `testCollapseWouldFoldOrDegenerate` (`true` = reject).
    fn test_collapse_would_fold_or_degenerate(
        &self,
        vertex: usize,
        other_vertex: usize,
        collapse_to: &Vector3,
        removed_face_one: usize,
        removed_face_two: usize,
    ) -> bool {
        for halfedge in self.collect_vertex_halfedges(vertex) {
            let face = self.halfedges[halfedge].left_face;
            if face == removed_face_one || face == removed_face_two {
                continue;
            }

            let corner_vertices = [
                self.halfedges[self.halfedges[halfedge].previous_halfedge].start_vertex,
                self.halfedges[halfedge].start_vertex,
                self.halfedges[self.halfedges[halfedge].next_halfedge].start_vertex,
            ];
            let mut old_positions = [Vector3::default(); 3];
            let mut new_positions = [Vector3::default(); 3];
            for i in 0..3 {
                old_positions[i] = self.vertices[corner_vertices[i]].position;
                new_positions[i] =
                    if corner_vertices[i] == vertex || corner_vertices[i] == other_vertex {
                        *collapse_to
                    } else {
                        self.vertices[corner_vertices[i]].position
                    };
            }

            if is_triangle_degenerate(&new_positions[0], &new_positions[1], &new_positions[2]) {
                return true;
            }

            let old_normal =
                Vector3::normal(&old_positions[0], &old_positions[1], &old_positions[2]);
            let new_normal =
                Vector3::normal(&new_positions[0], &new_positions[1], &new_positions[2]);
            if old_normal.is_zero() {
                continue;
            }
            if Vector3::dot_product(&old_normal, &new_normal) <= 0.0 {
                return true;
            }
        }
        false
    }

    /// Mirrors `testMoveWouldDegenerate` (`true` = reject).
    fn test_move_would_degenerate(&self, vertex: usize, target: &Vector3) -> bool {
        for halfedge in self.collect_vertex_halfedges(vertex) {
            let previous = self.halfedges[halfedge].previous_halfedge;
            let next = self.halfedges[halfedge].next_halfedge;
            if is_triangle_degenerate(
                &self.vertices[self.halfedges[previous].start_vertex].position,
                target,
                &self.vertices[self.halfedges[next].start_vertex].position,
            ) {
                return true;
            }
        }
        false
    }

    /// Mirrors `relaxVertex`.
    pub fn relax_vertex(&mut self, vertex: usize) {
        if self.vertices[vertex].is_boundary || self.vertices[vertex].valence <= 0 {
            return;
        }

        let mut position = Vector3::default();
        let mut count = 0usize;
        for halfedge in self.collect_vertex_halfedges(vertex) {
            let next = self.halfedges[halfedge].next_halfedge;
            position += self.vertices[self.halfedges[next].start_vertex].position;
            count += 1;
        }

        if 0 == count {
            return;
        }

        position /= count as f64;

        let normal = self.vertices[vertex].normal;
        let projected_position = project_point_on_line(
            &self.vertices[vertex].position,
            &position,
            &(position + normal),
        );
        if contains_nan(&projected_position) || contains_inf(&projected_position) {
            // Averaging the one ring of a vertex whose triangles already lie
            // on a line squashes them completely, so leave such a vertex
            // alone.
            if !self.test_move_would_degenerate(vertex, &position) {
                self.vertices[vertex].position = position;
            }
            return;
        }

        if self.test_move_would_degenerate(vertex, &projected_position) {
            return;
        }

        self.vertices[vertex].position = projected_position;
    }

    /// Mirrors `isVertexPairConnected`.
    fn is_vertex_pair_connected(&self, first: usize, second: usize) -> bool {
        for halfedge in self.collect_vertex_halfedges(first) {
            // Both other corners of every incident face are visited, so
            // boundary vertices report their last neighbor too.
            let previous = self.halfedges[halfedge].previous_halfedge;
            let next = self.halfedges[halfedge].next_halfedge;
            if self.halfedges[next].start_vertex == second
                || self.halfedges[previous].start_vertex == second
            {
                return true;
            }
        }
        false
    }

    /// Valence deviation (mirrors the `deviation` lambda in `flipEdge`).
    /// The C++ `std::pow(int, 2)` lowers to `sitofp`+`fmul` (verified in IR),
    /// and the sums truncate back to `int` via `fptosi`; both are mirrored
    /// in `f64` here.
    fn valence_deviation(valence: i32, is_boundary: bool) -> f64 {
        let d = (valence - if is_boundary { 4 } else { 6 }) as f64;
        d * d
    }

    /// Mirrors `flipEdge`.
    pub fn flip_edge(&mut self, halfedge: usize) -> bool {
        let opposite = match self.halfedges[halfedge].opposite_halfedge {
            None => return false,
            Some(opposite) => opposite,
        };

        let top_vertex = self.halfedges[self.halfedges[halfedge].previous_halfedge].start_vertex;
        let bottom_vertex = self.halfedges[self.halfedges[opposite].previous_halfedge].start_vertex;

        let left_vertex = self.halfedges[halfedge].start_vertex;
        let right_vertex = self.halfedges[opposite].start_vertex;

        let (valence, is_left_boundary) = self.vertex_valence(left_vertex);
        let left_valence = valence as i32;
        if left_valence <= 3 {
            return false;
        }

        let (valence, is_right_boundary) = self.vertex_valence(right_vertex);
        let right_valence = valence as i32;
        if right_valence <= 3 {
            return false;
        }

        let (valence, is_top_boundary) = self.vertex_valence(top_vertex);
        let top_valence = valence as i32;

        let (valence, is_bottom_boundary) = self.vertex_valence(bottom_vertex);
        let bottom_valence = valence as i32;

        let old_deviation = (Self::valence_deviation(top_valence, is_top_boundary)
            + Self::valence_deviation(bottom_valence, is_bottom_boundary)
            + Self::valence_deviation(left_valence, is_left_boundary)
            + Self::valence_deviation(right_valence, is_right_boundary))
            as i32;
        let new_deviation = (Self::valence_deviation(top_valence + 1, is_top_boundary)
            + Self::valence_deviation(bottom_valence + 1, is_bottom_boundary)
            + Self::valence_deviation(left_valence - 1, is_left_boundary)
            + Self::valence_deviation(right_valence - 1, is_right_boundary))
            as i32;

        if new_deviation >= old_deviation {
            return false;
        }

        // The flipped edge connects the two apexes. When they are already
        // connected, the flip would create a second edge between the same
        // pair of vertices, which makes the mesh non-manifold.
        if self.is_vertex_pair_connected(top_vertex, bottom_vertex) {
            return false;
        }

        // Reject flips which fold the two triangles over each other or
        // collapse one of them onto a line, both of which the valence
        // criterion above is blind to.
        let top_position = self.vertices[top_vertex].position;
        let bottom_position = self.vertices[bottom_vertex].position;
        let left_position = self.vertices[left_vertex].position;
        let right_position = self.vertices[right_vertex].position;

        if is_triangle_degenerate(&top_position, &left_position, &bottom_position)
            || is_triangle_degenerate(&bottom_position, &right_position, &top_position)
        {
            return false;
        }

        let old_top_normal = Vector3::normal(&top_position, &left_position, &right_position);
        let old_bottom_normal = Vector3::normal(&bottom_position, &right_position, &left_position);
        let new_top_normal = Vector3::normal(&top_position, &left_position, &bottom_position);
        let new_bottom_normal = Vector3::normal(&bottom_position, &right_position, &top_position);

        if Vector3::dot_product(&new_top_normal, &old_top_normal) <= 0.0
            || Vector3::dot_product(&new_top_normal, &old_bottom_normal) <= 0.0
            || Vector3::dot_product(&new_bottom_normal, &old_top_normal) <= 0.0
            || Vector3::dot_product(&new_bottom_normal, &old_bottom_normal) <= 0.0
        {
            return false;
        }

        let top_face = self.halfedges[halfedge].left_face;
        let bottom_face = self.halfedges[opposite].left_face;

        if self.vertices[left_vertex].first_halfedge == Some(halfedge) {
            self.vertices[left_vertex].first_halfedge =
                Some(self.halfedges[opposite].next_halfedge);
        }

        if self.vertices[right_vertex].first_halfedge == Some(opposite) {
            self.vertices[right_vertex].first_halfedge =
                Some(self.halfedges[halfedge].next_halfedge);
        }

        let halfedge_next = self.halfedges[halfedge].next_halfedge;
        let opposite_next = self.halfedges[opposite].next_halfedge;
        self.halfedges[halfedge_next].left_face = bottom_face;
        self.halfedges[opposite_next].left_face = top_face;

        self.halfedges[halfedge].start_vertex = bottom_vertex;
        self.halfedges[opposite].start_vertex = top_vertex;

        self.faces[top_face].halfedge = halfedge;
        self.faces[bottom_face].halfedge = opposite;

        let new_left_halfedges = [
            self.halfedges[halfedge].previous_halfedge,
            self.halfedges[opposite].next_halfedge,
            halfedge,
        ];
        let new_right_halfedges = [
            self.halfedges[opposite].previous_halfedge,
            self.halfedges[halfedge].next_halfedge,
            opposite,
        ];

        self.link_face_halfedges(&new_left_halfedges);
        self.link_face_halfedges(&new_right_halfedges);

        true
    }

    /// Mirrors `collapseEdge`.
    pub fn collapse_edge(&mut self, halfedge: usize, max_edge_length_squared: f64) -> bool {
        // Collapsing a boundary edge would need the face on the other side,
        // which a boundary edge does not have.
        let opposite = match self.halfedges[halfedge].opposite_halfedge {
            None => return false,
            Some(opposite) => opposite,
        };

        let top_vertex = self.halfedges[self.halfedges[halfedge].previous_halfedge].start_vertex;
        let bottom_vertex = self.halfedges[self.halfedges[opposite].previous_halfedge].start_vertex;

        if top_vertex == bottom_vertex {
            return false;
        }

        if self.vertices[top_vertex].featured {
            return false;
        }

        if self.vertices[bottom_vertex].featured {
            return false;
        }

        let start_vertex = self.halfedges[halfedge].start_vertex;
        let next_halfedge = self.halfedges[halfedge].next_halfedge;
        let next_vertex = self.halfedges[next_halfedge].start_vertex;
        let collapse_to = if !self.vertices[start_vertex].smooth_normal.is_zero()
            && !self.vertices[next_vertex].smooth_normal.is_zero()
        {
            let p1 = self.vertices[start_vertex].position;
            let p2 = self.vertices[next_vertex].position;
            let n1 = self.vertices[start_vertex].smooth_normal;
            let n2 = self.vertices[next_vertex].smooth_normal;
            Self::pn_edge_midpoint(&p1, &p2, &n1, &n2)
        } else {
            (self.vertices[start_vertex].position + self.vertices[next_vertex].position) * 0.5
        };

        if self.test_length_squared_around_vertex(
            start_vertex,
            &collapse_to,
            max_edge_length_squared,
        ) {
            return false;
        }
        if self.test_length_squared_around_vertex(
            next_vertex,
            &collapse_to,
            max_edge_length_squared,
        ) {
            return false;
        }

        let mut neighbor_vertices = BTreeSet::new();
        let mut other_neighbor_vertices = BTreeSet::new();
        self.collect_vertices_around_vertex(start_vertex, &mut neighbor_vertices);
        self.collect_vertices_around_vertex(next_vertex, &mut other_neighbor_vertices);
        // Only the intersection size is used (`std::set_intersection` +
        // `size() > 2` in the C++).
        if neighbor_vertices
            .intersection(&other_neighbor_vertices)
            .count()
            > 2
        {
            return false;
        }

        let left_vertex = start_vertex;
        let right_vertex = self.halfedges[opposite].start_vertex;
        let top_face = self.halfedges[halfedge].left_face;
        let bottom_face = self.halfedges[opposite].left_face;

        // Moving both endpoints to the collapse point may turn a surviving
        // neighbor triangle inside out or squash it onto a line, which
        // leaves a fold behind that no later stage can undo.
        if self.test_collapse_would_fold_or_degenerate(
            left_vertex,
            right_vertex,
            &collapse_to,
            top_face,
            bottom_face,
        ) {
            return false;
        }
        if self.test_collapse_would_fold_or_degenerate(
            right_vertex,
            left_vertex,
            &collapse_to,
            top_face,
            bottom_face,
        ) {
            return false;
        }

        self.pointer_vertex_to_new_vertex(left_vertex, right_vertex);

        // The `if let` guards below mirror raw-pointer dereferences: an edge
        // endpoint always has incident halfedges, so these are `Some` on
        // every reachable path.
        if let Some(first) = self.vertices[right_vertex].first_halfedge {
            let left_face = self.halfedges[first].left_face;
            if top_face == left_face || bottom_face == left_face {
                let previous = self.halfedges[opposite].previous_halfedge;
                self.vertices[right_vertex].first_halfedge =
                    self.halfedges[previous].opposite_halfedge;
            }
        }
        if let Some(first) = self.vertices[top_vertex].first_halfedge {
            let left_face = self.halfedges[first].left_face;
            if top_face == left_face || bottom_face == left_face {
                let next = self.halfedges[halfedge].next_halfedge;
                self.vertices[top_vertex].first_halfedge = self.halfedges[next].opposite_halfedge;
            }
        }
        if let Some(first) = self.vertices[bottom_vertex].first_halfedge {
            let left_face = self.halfedges[first].left_face;
            // Mirrored exactly: the C++ checks `bottomFace` twice here
            // (`topFace` missing, apparent typo).
            #[allow(clippy::eq_op)]
            if bottom_face == left_face || bottom_face == left_face {
                let next = self.halfedges[opposite].next_halfedge;
                self.vertices[bottom_vertex].first_halfedge =
                    self.halfedges[next].opposite_halfedge;
            }
        }

        let prev_opp = self.halfedges[self.halfedges[halfedge].previous_halfedge].opposite_halfedge;
        let next_opp = self.halfedges[self.halfedges[halfedge].next_halfedge].opposite_halfedge;
        self.link_halfedge_pair(prev_opp, next_opp);
        let prev_opp = self.halfedges[self.halfedges[opposite].previous_halfedge].opposite_halfedge;
        let next_opp = self.halfedges[self.halfedges[opposite].next_halfedge].opposite_halfedge;
        self.link_halfedge_pair(prev_opp, next_opp);

        self.vertices[right_vertex].position = collapse_to;

        self.vertices[left_vertex].removed = true;
        self.faces[top_face].removed = true;
        self.faces[bottom_face].removed = true;

        true
    }

    /// Mirrors `updateVertexValences`.
    pub fn update_vertex_valences(&mut self) {
        for index in 0..self.vertices.len() {
            if self.vertices[index].removed {
                continue;
            }
            let (valence, is_boundary) = self.vertex_valence(index);
            self.vertices[index].is_boundary = is_boundary;
            self.vertices[index].valence = valence as i32;
        }
    }

    /// Mirrors `updateTriangleNormals`.
    pub fn update_triangle_normals(&mut self) {
        for index in 0..self.faces.len() {
            if self.faces[index].removed {
                continue;
            }
            let start_halfedge = self.faces[index].halfedge;
            let previous = self.halfedges[start_halfedge].previous_halfedge;
            let next = self.halfedges[start_halfedge].next_halfedge;
            self.faces[index].normal = Vector3::normal(
                &self.vertices[self.halfedges[previous].start_vertex].position,
                &self.vertices[self.halfedges[start_halfedge].start_vertex].position,
                &self.vertices[self.halfedges[next].start_vertex].position,
            );
        }
    }

    /// Mirrors `updateVertexNormals`.
    pub fn update_vertex_normals(&mut self) {
        for vertex in &mut self.vertices {
            if vertex.removed {
                continue;
            }
            vertex.normal = Vector3::default();
        }

        for index in 0..self.faces.len() {
            if self.faces[index].removed {
                continue;
            }
            let start_halfedge = self.faces[index].halfedge;
            let previous = self.halfedges[start_halfedge].previous_halfedge;
            let next = self.halfedges[start_halfedge].next_halfedge;
            let face_normal = self.faces[index].normal
                * Vector3::area(
                    &self.vertices[self.halfedges[previous].start_vertex].position,
                    &self.vertices[self.halfedges[start_halfedge].start_vertex].position,
                    &self.vertices[self.halfedges[next].start_vertex].position,
                );
            let corners = [
                self.halfedges[previous].start_vertex,
                self.halfedges[start_halfedge].start_vertex,
                self.halfedges[next].start_vertex,
            ];
            for corner in corners {
                self.vertices[corner].normal += face_normal;
            }
        }

        for vertex in &mut self.vertices {
            if vertex.removed {
                continue;
            }
            vertex.normal.normalize();
        }
    }

    /// Mirrors `featureHalfedge`.
    fn feature_halfedge(&mut self, halfedge: usize, radians: f64) {
        if -1 != self.halfedges[halfedge].feature_state {
            return;
        }

        let opposite = match self.halfedges[halfedge].opposite_halfedge {
            None => {
                let start_vertex = self.halfedges[halfedge].start_vertex;
                self.vertices[start_vertex].featured = true;
                self.halfedges[halfedge].feature_state = 1;
                return;
            }
            Some(opposite) => opposite,
        };
        let left_face = self.halfedges[halfedge].left_face;
        let opposite_left_face = self.halfedges[opposite].left_face;

        if Vector3::angle(
            &self.faces[left_face].normal,
            &self.faces[opposite_left_face].normal,
        ) >= radians
        {
            self.halfedges[halfedge].feature_state = 1;
            self.halfedges[opposite].feature_state = 1;
            let start_vertex = self.halfedges[halfedge].start_vertex;
            let opposite_start_vertex = self.halfedges[opposite].start_vertex;
            self.vertices[start_vertex].featured = true;
            self.vertices[opposite_start_vertex].featured = true;
            return;
        }

        self.halfedges[halfedge].feature_state = 0;
        self.halfedges[opposite].feature_state = 0;
    }

    /// Mirrors `featureBoundaries`.
    pub fn feature_boundaries(&mut self) {
        for index in 0..self.faces.len() {
            if self.faces[index].removed {
                continue;
            }
            let start_halfedge = self.faces[index].halfedge;
            let previous = self.halfedges[start_halfedge].previous_halfedge;
            let next = self.halfedges[start_halfedge].next_halfedge;
            if self.halfedges[previous].opposite_halfedge.is_none() {
                let start_vertex = self.halfedges[previous].start_vertex;
                self.vertices[start_vertex].featured = true;
            }
            if self.halfedges[start_halfedge].opposite_halfedge.is_none() {
                let start_vertex = self.halfedges[start_halfedge].start_vertex;
                self.vertices[start_vertex].featured = true;
            }
            if self.halfedges[next].opposite_halfedge.is_none() {
                let start_vertex = self.halfedges[next].start_vertex;
                self.vertices[start_vertex].featured = true;
            }
        }
    }

    /// Mirrors `featureEdges`.
    pub fn feature_edges(&mut self, radians: f64) {
        for index in 0..self.faces.len() {
            if self.faces[index].removed {
                continue;
            }
            let start_halfedge = self.faces[index].halfedge;
            let previous = self.halfedges[start_halfedge].previous_halfedge;
            let next = self.halfedges[start_halfedge].next_halfedge;
            self.feature_halfedge(previous, radians);
            self.feature_halfedge(start_halfedge, radians);
            self.feature_halfedge(next, radians);
        }
    }
}

// ---------------------------------------------------------------------------
// IsotropicRemesher (thirdparty kernel)
// ---------------------------------------------------------------------------

/// Isotropic remesher kernel (mirrors thirdparty `IsotropicRemesher`).
/// Borrows the input mesh; owns the working state (halfedge mesh, AABB tree,
/// normals, smooth-subdivision copies).
pub struct IsoRemeshKernel<'a> {
    vertices: &'a [Vector3],
    triangles: &'a [Vec<usize>],
    triangle_normals: Vec<Vector3>,
    halfedge_mesh: IsotropicHalfedgeMesh,
    bounding_box_tree: Option<AxisAlignedBoundingBoxTree>,
    sharp_edge_threshold_radians: f64,
    target_edge_length: f64,
    initial_average_edge_length: f64,
    target_triangle_count: usize,
    vertex_target_edge_lengths: Option<&'a [f64]>,
    smooth_normal_degrees: f64,
    smooth_vertex_normals: Vec<Vector3>,
    smooth_vertices: Vec<Vector3>,
    smooth_triangles: Vec<Vec<usize>>,
    smooth_triangle_normals: Vec<Vector3>,
    progress_handler: Option<ProgressHandler>,
}

impl<'a> IsoRemeshKernel<'a> {
    /// Mirrors the constructor (builds the halfedge mesh and records the
    /// initial average edge length).
    #[must_use]
    pub fn new(vertices: &'a [Vector3], triangles: &'a [Vec<usize>]) -> Self {
        let halfedge_mesh = IsotropicHalfedgeMesh::new(vertices, triangles);
        let initial_average_edge_length = halfedge_mesh.average_edge_length();
        Self {
            vertices,
            triangles,
            triangle_normals: Vec::new(),
            halfedge_mesh,
            bounding_box_tree: None,
            sharp_edge_threshold_radians: 0.0,
            target_edge_length: 0.0,
            initial_average_edge_length,
            target_triangle_count: 0,
            vertex_target_edge_lengths: None,
            smooth_normal_degrees: 0.0,
            smooth_vertex_normals: Vec::new(),
            smooth_vertices: Vec::new(),
            smooth_triangles: Vec::new(),
            smooth_triangle_normals: Vec::new(),
            progress_handler: None,
        }
    }

    /// Halfedge-mesh construction anomalies, see
    /// [`IsotropicHalfedgeMesh::build_anomalies`].
    #[must_use]
    pub fn build_anomalies(&self) -> (usize, usize) {
        self.halfedge_mesh.build_anomalies()
    }

    /// Mirrors `initialAverageEdgeLength`.
    #[inline]
    #[must_use]
    pub fn initial_average_edge_length(&self) -> f64 {
        self.initial_average_edge_length
    }

    /// Mirrors `setSharpEdgeIncludedAngle`.
    pub fn set_sharp_edge_included_angle(&mut self, degrees: f64) {
        self.sharp_edge_threshold_radians = (180.0 - degrees) * (PI / 180.0);
    }

    /// Mirrors `setTargetEdgeLength`.
    pub fn set_target_edge_length(&mut self, edge_length: f64) {
        self.target_edge_length = edge_length;
    }

    /// Mirrors `setVertexTargetEdgeLengths`.
    pub fn set_vertex_target_edge_lengths(&mut self, target_lengths: &'a [f64]) {
        self.vertex_target_edge_lengths = Some(target_lengths);
    }

    /// Mirrors `setTargetTriangleCount`.
    pub fn set_target_triangle_count(&mut self, triangle_count: usize) {
        self.target_triangle_count = triangle_count;
    }

    /// Mirrors `setSmoothNormalDegrees`.
    pub fn set_smooth_normal_degrees(&mut self, degrees: f64) {
        self.smooth_normal_degrees = degrees;
    }

    /// Mirrors `setProgressHandler`.
    pub fn set_progress_handler(&mut self, handler: ProgressHandler) {
        self.progress_handler = Some(handler);
    }

    /// Takes the progress handler back out (no C++ counterpart: `Box<dyn
    /// Fn>` is not cloneable like `std::function`, so the wrapper round-
    /// trips the handler through the kernel instead of copying it).
    pub(crate) fn take_progress_handler(&mut self) -> Option<ProgressHandler> {
        self.progress_handler.take()
    }

    /// Progress report (mirrors the `report` lambda in `remesh`).
    fn report(&self, fraction: f32, name: &str) {
        if let Some(handler) = &self.progress_handler {
            handler(fraction, name);
        }
    }

    /// Per-pass fraction (mirrors the `reportPass` lambda: `0.25 + 0.75 *
    /// pass / passCount` in `f32`, `1.0` when there are no passes).
    fn pass_fraction(pass: usize, pass_count: usize) -> f32 {
        if 0 == pass_count {
            1.0
        } else {
            0.25 + 0.75 * (pass as f32) / (pass_count as f32)
        }
    }

    /// Mirrors `pnTriangleEdgeMidpoint` (same expression as the mesh-local
    /// `pn_edge_midpoint`; kept as a separate associated function like the
    /// two separate C++ copies).
    fn pn_triangle_edge_midpoint(
        p1: &Vector3,
        p2: &Vector3,
        n1: &Vector3,
        n2: &Vector3,
    ) -> Vector3 {
        let d12 = Vector3::dot_product(&(*p2 - *p1), n1);
        let d21 = Vector3::dot_product(&(*p1 - *p2), n2);
        let p210 = (2.0 * *p1 + *p2 - d12 * *n1) / 3.0;
        let p120 = (2.0 * *p2 + *p1 - d21 * *n2) / 3.0;
        (*p1 + *p2 + 3.0 * (p210 + p120)) / 8.0
    }

    /// Corner angle at vertex `vi` of triangle `tri` (mirrors the three-way
    /// branch duplicated in both accumulation loops of
    /// `computeSmoothVertexNormals`).
    fn corner_angle_at_vertex(vertices: &[Vector3], tri: &[usize], vi: usize) -> f64 {
        if tri[0] == vi {
            Vector3::angle(
                &(vertices[tri[1]] - vertices[tri[0]]),
                &(vertices[tri[2]] - vertices[tri[0]]),
            )
        } else if tri[1] == vi {
            Vector3::angle(
                &(vertices[tri[0]] - vertices[tri[1]]),
                &(vertices[tri[2]] - vertices[tri[1]]),
            )
        } else {
            Vector3::angle(
                &(vertices[tri[0]] - vertices[tri[2]]),
                &(vertices[tri[1]] - vertices[tri[2]]),
            )
        }
    }

    /// Mirrors `computeSmoothVertexNormals` (both accumulation loops kept
    /// literal; only the corner-angle branch is factored).
    fn compute_smooth_vertex_normals(&mut self) {
        // Mirrors `m_smoothVertexNormals.assign(size, Vector3())`.
        self.smooth_vertex_normals.clear();
        self.smooth_vertex_normals
            .resize(self.vertices.len(), Vector3::default());
        let vertices = self.vertices;
        let triangles = self.triangles;

        // Compute face normals and build adjacency
        let mut face_normals = Vec::with_capacity(triangles.len());
        let mut faces_around_vertex: Vec<Vec<usize>> = Vec::new();
        faces_around_vertex.resize(vertices.len(), Vec::new());
        for tri in triangles {
            face_normals.push(Vector3::normal(
                &vertices[tri[0]],
                &vertices[tri[1]],
                &vertices[tri[2]],
            ));
            let fi = face_normals.len() - 1;
            for j in 0..3 {
                faces_around_vertex[tri[j]].push(fi);
            }
        }

        let threshold_radians = self.smooth_normal_degrees * PI / 180.0;

        for vi in 0..vertices.len() {
            let incident_faces = faces_around_vertex[vi].clone();
            if incident_faces.is_empty() {
                continue;
            }

            // Compute angle-area weighted normal from all incident faces
            // within the angle threshold
            let mut weighted_normal = Vector3::default();
            for &fi in &incident_faces {
                let tri = &triangles[fi];
                let area = Vector3::area(&vertices[tri[0]], &vertices[tri[1]], &vertices[tri[2]]);

                // Find the corner angle at this vertex for this triangle
                let corner_angle = Self::corner_angle_at_vertex(vertices, tri, vi);

                weighted_normal += face_normals[fi] * area * corner_angle;
            }

            if !weighted_normal.is_zero() {
                weighted_normal.normalize();
            }

            // If threshold > 0, refine by excluding faces whose normals
            // deviate too far from the initial weighted average
            if threshold_radians > 0.0 && !weighted_normal.is_zero() {
                let mut refined_normal = Vector3::default();
                for &fi in &incident_faces {
                    let angle = Vector3::angle(&face_normals[fi], &weighted_normal);
                    if angle <= threshold_radians {
                        let tri = &triangles[fi];
                        let area =
                            Vector3::area(&vertices[tri[0]], &vertices[tri[1]], &vertices[tri[2]]);
                        let corner_angle = Self::corner_angle_at_vertex(vertices, tri, vi);
                        refined_normal += face_normals[fi] * area * corner_angle;
                    }
                }
                if !refined_normal.is_zero() {
                    self.smooth_vertex_normals[vi] = refined_normal.normalized();
                }
            } else if !weighted_normal.is_zero() {
                self.smooth_vertex_normals[vi] = weighted_normal;
            }
        }
    }

    /// Mirrors `subdivideMeshWithPNTriangles`.
    fn subdivide_mesh_with_pn_triangles(&mut self) {
        self.smooth_vertices.clear();
        self.smooth_triangles.clear();
        self.smooth_triangle_normals.clear();

        if self.smooth_vertex_normals.len() != self.vertices.len() {
            return;
        }

        // Copy original vertices as the base set
        self.smooth_vertices = self.vertices.to_vec();

        for index in 0..self.triangles.len() {
            let tri = self.triangles[index].clone();
            let (i0, i1, i2) = (tri[0], tri[1], tri[2]);

            let p0 = self.vertices[i0];
            let p1 = self.vertices[i1];
            let p2 = self.vertices[i2];
            let n0 = self.smooth_vertex_normals[i0];
            let n1 = self.smooth_vertex_normals[i1];
            let n2 = self.smooth_vertex_normals[i2];

            // Compute PN Triangle edge midpoints
            let e01 = Self::pn_triangle_edge_midpoint(&p0, &p1, &n0, &n1);
            let e12 = Self::pn_triangle_edge_midpoint(&p1, &p2, &n1, &n2);
            let e20 = Self::pn_triangle_edge_midpoint(&p2, &p0, &n2, &n0);

            // Add new vertices
            let ie01 = self.smooth_vertices.len();
            self.smooth_vertices.push(e01);
            let ie12 = self.smooth_vertices.len();
            self.smooth_vertices.push(e12);
            let ie20 = self.smooth_vertices.len();
            self.smooth_vertices.push(e20);

            // Four sub-triangles
            let sub_tris = [
                [i0, ie01, ie20],
                [i1, ie12, ie01],
                [i2, ie20, ie12],
                [ie01, ie12, ie20],
            ];

            for sub_tri in &sub_tris {
                self.smooth_triangles
                    .push(vec![sub_tri[0], sub_tri[1], sub_tri[2]]);
                self.smooth_triangle_normals.push(Vector3::normal(
                    &self.smooth_vertices[sub_tri[0]],
                    &self.smooth_vertices[sub_tri[1]],
                    &self.smooth_vertices[sub_tri[2]],
                ));
            }
        }
    }

    /// Mirrors `addTriagleToAxisAlignedBoundingBox` (spelling kept at the
    /// call sites' level; the typo is the C++ member's name).
    fn add_triangle_to_axis_aligned_bounding_box(
        &self,
        triangle: &[usize],
        bbox: &mut AxisAlignedBoundingBox,
    ) {
        for i in 0..3 {
            bbox.update(&self.vertices[triangle[i]]);
        }
    }

    /// Mirrors `buildAxisAlignedBoundingBoxTree`.
    fn build_axis_aligned_bounding_box_tree(&mut self) {
        let mut triangle_boxes = Vec::with_capacity(self.triangles.len());
        for index in 0..self.triangles.len() {
            let mut bbox = AxisAlignedBoundingBox::default();
            let tri = self.triangles[index].clone();
            self.add_triangle_to_axis_aligned_bounding_box(&tri, &mut bbox);
            bbox.update_center();
            triangle_boxes.push(bbox);
        }

        let face_indices: Vec<usize> = (0..triangle_boxes.len()).collect();

        let mut group_box = AxisAlignedBoundingBox::default();
        for &i in &face_indices {
            let tri = self.triangles[i].clone();
            self.add_triangle_to_axis_aligned_bounding_box(&tri, &mut group_box);
        }
        group_box.update_center();

        self.bounding_box_tree = Some(AxisAlignedBoundingBoxTree::new(
            triangle_boxes,
            face_indices,
            group_box,
        ));
    }

    /// Mirrors `remesh`.
    pub fn remesh(&mut self, iteration: usize) {
        self.report(0.0, "Building bounding volume tree");
        self.triangle_normals = Vec::new();
        let vertices = self.vertices;
        let triangles = self.triangles;
        for tri in triangles {
            self.triangle_normals.push(Vector3::normal(
                &vertices[tri[0]],
                &vertices[tri[1]],
                &vertices[tri[2]],
            ));
        }

        let mut target_length = if self.target_edge_length > 0.0 {
            self.target_edge_length
        } else {
            self.initial_average_edge_length
        };

        if self.target_triangle_count > 0 {
            let mut total_area = 0.0;
            for tri in triangles {
                total_area +=
                    Vector3::area(&vertices[tri[0]], &vertices[tri[1]], &vertices[tri[2]]);
            }
            let triangle_area = total_area / self.target_triangle_count as f64;
            target_length = (triangle_area / (0.86602540378 * 0.5)).sqrt();
        }

        let min_target_length = 4.0 / 5.0 * target_length;
        let max_target_length = 4.0 / 3.0 * target_length;

        let min_target_length_squared = min_target_length * min_target_length;
        let max_target_length_squared = max_target_length * max_target_length;

        // If smooth normal threshold is set, compute smooth normals and
        // subdivide the input mesh using PN Triangle evaluation so that
        // the projection step projects onto a smooth curved surface.
        if self.smooth_normal_degrees > 0.0 {
            // Compute per-vertex smooth normals on the input mesh
            self.compute_smooth_vertex_normals();

            // Set smooth normals on the halfedge mesh vertices
            let mut vertex = self.halfedge_mesh.move_to_next_vertex(None);
            let mut vi = 0;
            while let Some(current) = vertex {
                if vi >= self.smooth_vertex_normals.len() {
                    break;
                }
                let normal = self.smooth_vertex_normals[vi];
                self.halfedge_mesh.vertex_mut(current).smooth_normal = normal;
                vertex = self.halfedge_mesh.move_to_next_vertex(Some(current));
                vi += 1;
            }

            // Subdivide the input mesh using PN Triangle evaluation
            self.subdivide_mesh_with_pn_triangles();

            // Build AABB tree from the subdivided (smooth) mesh
            let mut triangle_boxes = Vec::with_capacity(self.smooth_triangles.len());
            for index in 0..self.smooth_triangles.len() {
                let tri = self.smooth_triangles[index].clone();
                let mut bbox = AxisAlignedBoundingBox::default();
                for j in 0..3 {
                    bbox.update(&self.smooth_vertices[tri[j]]);
                }
                bbox.update_center();
                triangle_boxes.push(bbox);
            }
            let face_indices: Vec<usize> = (0..triangle_boxes.len()).collect();
            let mut group_box = AxisAlignedBoundingBox::default();
            for &i in &face_indices {
                let tri = self.smooth_triangles[i].clone();
                for j in 0..3 {
                    group_box.update(&self.smooth_vertices[tri[j]]);
                }
            }
            group_box.update_center();
            self.bounding_box_tree = Some(AxisAlignedBoundingBoxTree::new(
                triangle_boxes,
                face_indices,
                group_box,
            ));
        } else {
            self.build_axis_aligned_bounding_box_tree();
        }

        // Apply per-vertex target edge lengths if provided
        if let Some(target_lengths) = self.vertex_target_edge_lengths {
            let mut vertex = self.halfedge_mesh.move_to_next_vertex(None);
            let mut vi = 0;
            while let Some(current) = vertex {
                if vi >= target_lengths.len() {
                    break;
                }
                self.halfedge_mesh.vertex_mut(current).target_edge_length = target_lengths[vi];
                vertex = self.halfedge_mesh.move_to_next_vertex(Some(current));
                vi += 1;
            }
        }

        self.report(0.15, "Splitting long edges");
        let mut skip_split_once = true;
        if self.sharp_edge_threshold_radians > 0.0 {
            self.split_long_edges(max_target_length_squared);
            self.halfedge_mesh.update_triangle_normals();
            self.halfedge_mesh
                .feature_edges(self.sharp_edge_threshold_radians);
        } else {
            self.split_long_edges(max_target_length_squared);
            self.halfedge_mesh.feature_boundaries();
        }

        // The five passes below repeat once per iteration, so the remaining
        // 0.25..1 splits evenly across all of them.
        let pass_count = iteration * 5;
        let mut pass = 0;
        for _ in 0..iteration {
            self.report(
                Self::pass_fraction(pass, pass_count),
                "Splitting long edges",
            );
            pass += 1;
            if skip_split_once {
                skip_split_once = false;
            } else {
                self.split_long_edges(max_target_length_squared);
            }
            self.report(
                Self::pass_fraction(pass, pass_count),
                "Collapsing short edges",
            );
            pass += 1;
            self.collapse_short_edges(min_target_length_squared, max_target_length_squared);
            self.report(Self::pass_fraction(pass, pass_count), "Flipping edges");
            pass += 1;
            self.flip_edges();
            self.report(Self::pass_fraction(pass, pass_count), "Shifting vertices");
            pass += 1;
            self.shift_vertices();
            self.report(Self::pass_fraction(pass, pass_count), "Projecting vertices");
            pass += 1;
            self.project_vertices();
        }
        self.report(1.0, "");
    }

    /// Mirrors `remeshedHalfedgeMesh`.
    #[must_use]
    pub fn remeshed_halfedge_mesh(&mut self) -> &mut IsotropicHalfedgeMesh {
        &mut self.halfedge_mesh
    }

    /// Mirrors `splitLongEdges` (`std::pow(x, 2)` lowers to `x*x`; verified
    /// in IR).
    fn split_long_edges(&mut self, max_edge_length_squared: f64) {
        let mut face = self.halfedge_mesh.move_to_next_face(None);
        while let Some(current) = face {
            let start_halfedge = self.halfedge_mesh.face(current).halfedge;
            face = self.halfedge_mesh.move_to_next_face(Some(current));
            let mut halfedge = start_halfedge;
            loop {
                let next_halfedge = self.halfedge_mesh.halfedge(halfedge).next_halfedge;
                let start_vertex = self.halfedge_mesh.halfedge(halfedge).start_vertex;
                let next_vertex = self.halfedge_mesh.halfedge(next_halfedge).start_vertex;
                let length_squared = (self.halfedge_mesh.vertex(start_vertex).position
                    - self.halfedge_mesh.vertex(next_vertex).position)
                    .length_squared();
                let mut edge_max_len_sq = max_edge_length_squared;
                let t0 = self.halfedge_mesh.vertex(start_vertex).target_edge_length;
                let t1 = self.halfedge_mesh.vertex(next_vertex).target_edge_length;
                if t0 > 0.0 && t1 > 0.0 {
                    let edge_target = (t0 + t1) * 0.5;
                    let edge_max = 4.0 / 3.0 * edge_target;
                    edge_max_len_sq = edge_max * edge_max;
                }
                if length_squared > edge_max_len_sq {
                    self.halfedge_mesh.break_edge(halfedge);
                    break;
                }
                halfedge = next_halfedge;
                if halfedge == start_halfedge {
                    break;
                }
            }
        }
    }

    /// Mirrors `collapseShortEdges`.
    fn collapse_short_edges(&mut self, min_edge_length_squared: f64, max_edge_length_squared: f64) {
        let mut face = self.halfedge_mesh.move_to_next_face(None);
        while let Some(current) = face {
            if self.halfedge_mesh.face(current).removed {
                face = self.halfedge_mesh.move_to_next_face(Some(current));
                continue;
            }
            let start_halfedge = self.halfedge_mesh.face(current).halfedge;
            face = self.halfedge_mesh.move_to_next_face(Some(current));
            let mut halfedge = start_halfedge;
            loop {
                let next_halfedge = self.halfedge_mesh.halfedge(halfedge).next_halfedge;
                let start_vertex = self.halfedge_mesh.halfedge(halfedge).start_vertex;
                let next_vertex = self.halfedge_start_vertex(next_halfedge);
                let length_squared = (self.halfedge_mesh.vertex(start_vertex).position
                    - self.halfedge_mesh.vertex(next_vertex).position)
                    .length_squared();
                let mut edge_min_len_sq = min_edge_length_squared;
                let mut edge_max_len_sq = max_edge_length_squared;
                let t0 = self.halfedge_mesh.vertex(start_vertex).target_edge_length;
                let t1 = self.halfedge_mesh.vertex(next_vertex).target_edge_length;
                if t0 > 0.0 && t1 > 0.0 {
                    let edge_target = (t0 + t1) * 0.5;
                    let edge_min = 4.0 / 5.0 * edge_target;
                    let edge_max = 4.0 / 3.0 * edge_target;
                    edge_min_len_sq = edge_min * edge_min;
                    edge_max_len_sq = edge_max * edge_max;
                }
                if length_squared < edge_min_len_sq
                    && !self.halfedge_mesh.vertex(start_vertex).featured
                    && !self.halfedge_mesh.vertex(next_vertex).featured
                    && self.halfedge_mesh.collapse_edge(halfedge, edge_max_len_sq)
                {
                    break;
                }
                halfedge = next_halfedge;
                if halfedge == start_halfedge {
                    break;
                }
            }
        }
    }

    /// Start vertex of `halfedge` (tiny accessor to keep the call sites
    /// readable; the C++ chains `->` directly).
    fn halfedge_start_vertex(&self, halfedge: usize) -> usize {
        self.halfedge_mesh.halfedge(halfedge).start_vertex
    }

    /// Mirrors `flipEdges`.
    fn flip_edges(&mut self) {
        let mut face = self.halfedge_mesh.move_to_next_face(None);
        while let Some(current) = face {
            let start_halfedge = self.halfedge_mesh.face(current).halfedge;
            face = self.halfedge_mesh.move_to_next_face(Some(current));
            let mut halfedge = start_halfedge;
            loop {
                let next_halfedge = self.halfedge_mesh.halfedge(halfedge).next_halfedge;
                if self
                    .halfedge_mesh
                    .halfedge(halfedge)
                    .opposite_halfedge
                    .is_some()
                    && self.halfedge_mesh.flip_edge(halfedge)
                {
                    break;
                }
                halfedge = next_halfedge;
                if halfedge == start_halfedge {
                    break;
                }
            }
        }
    }

    /// Mirrors `shiftVertices`.
    fn shift_vertices(&mut self) {
        self.halfedge_mesh.update_vertex_valences();
        self.halfedge_mesh.update_triangle_normals();
        self.halfedge_mesh.update_vertex_normals();

        let mut vertex = self.halfedge_mesh.move_to_next_vertex(None);
        while let Some(current) = vertex {
            vertex = self.halfedge_mesh.move_to_next_vertex(Some(current));
            self.halfedge_mesh.relax_vertex(current);
        }
    }

    /// Mirrors `projectVertices`.
    fn project_vertices(&mut self) {
        // Use subdivided (smooth) mesh data for projection when available
        // (three independent emptiness checks, as in the C++).
        let use_smooth_vertices = !self.smooth_vertices.is_empty();
        let use_smooth_triangles = !self.smooth_triangles.is_empty();
        let use_smooth_normals = !self.smooth_triangle_normals.is_empty();
        let proj_vertices: &[Vector3] = if use_smooth_vertices {
            &self.smooth_vertices
        } else {
            self.vertices
        };
        let proj_triangles: &[Vec<usize>] = if use_smooth_triangles {
            &self.smooth_triangles
        } else {
            self.triangles
        };
        let proj_normals: &[Vector3] = if use_smooth_normals {
            &self.smooth_triangle_normals
        } else {
            &self.triangle_normals
        };

        let mut vertex = self.halfedge_mesh.move_to_next_vertex(None);
        while let Some(current) = vertex {
            vertex = self.halfedge_mesh.move_to_next_vertex(Some(current));
            if self.halfedge_mesh.vertex(current).featured {
                continue;
            }

            let start_halfedge = match self.halfedge_mesh.vertex(current).first_halfedge {
                None => continue,
                Some(halfedge) => halfedge,
            };

            let mut ray_boxes = vec![AxisAlignedBoundingBox::default()];
            ray_boxes[0].update(&self.halfedge_mesh.vertex(current).position);

            let mut loop_halfedge = start_halfedge;
            loop {
                let next = self.halfedge_mesh.halfedge(loop_halfedge).next_halfedge;
                let position = self
                    .halfedge_mesh
                    .vertex(self.halfedge_mesh.halfedge(next).start_vertex)
                    .position;
                ray_boxes[0].update(&position);
                match self.halfedge_mesh.halfedge(loop_halfedge).opposite_halfedge {
                    None => {
                        loop_halfedge = start_halfedge;
                        loop {
                            let previous =
                                self.halfedge_mesh.halfedge(loop_halfedge).previous_halfedge;
                            let position = self
                                .halfedge_mesh
                                .vertex(self.halfedge_start_vertex(previous))
                                .position;
                            ray_boxes[0].update(&position);
                            match self.halfedge_mesh.halfedge(previous).opposite_halfedge {
                                None => break,
                                Some(opposite) => {
                                    loop_halfedge = opposite;
                                    if loop_halfedge == start_halfedge {
                                        break;
                                    }
                                }
                            }
                        }
                        break;
                    }
                    Some(opposite) => {
                        loop_halfedge = self.halfedge_mesh.halfedge(opposite).next_halfedge;
                        if loop_halfedge == start_halfedge {
                            break;
                        }
                    }
                }
            }

            let bounding_box_size = *ray_boxes[0].upper_bound() - *ray_boxes[0].lower_bound();
            let normal = self.halfedge_mesh.vertex(current).normal;
            let segment =
                normal * (bounding_box_size[0] + bounding_box_size[1] + bounding_box_size[2]);

            let outter_box = ray_boxes[0].clone();
            let test_tree = AxisAlignedBoundingBoxTree::new(ray_boxes, vec![0], outter_box);
            let mut pairs: Vec<(usize, usize)> = Vec::new();
            // The tree is always built before the passes in `remesh` (both
            // branches assign it); the C++ derefs the pointer directly.
            if let Some(tree) = &self.bounding_box_tree {
                let root = tree.root();
                let test_root = test_tree.root();
                tree.test(root, &test_tree, test_root, &mut pairs);
            }

            let mut hits: Vec<(Vector3, f64)> = Vec::new();
            let position = self.halfedge_mesh.vertex(current).position;

            for &(first, _) in &pairs {
                // The segment runs both ways along the normal and reaches
                // well past the one ring, so on thin parts it also hits the
                // surface facing the other way. Landing there tears the
                // neighborhood open.
                if Vector3::dot_product(&proj_normals[first], &normal) <= 0.0 {
                    continue;
                }
                let triangle = &proj_triangles[first];
                let triangle_positions = [
                    proj_vertices[triangle[0]],
                    proj_vertices[triangle[1]],
                    proj_vertices[triangle[2]],
                ];
                if let Some(intersection) = intersect_segment_and_plane(
                    &(position - segment),
                    &(position + segment),
                    &triangle_positions[0],
                    &proj_normals[first],
                ) {
                    let edge_normals = [
                        Vector3::normal(
                            &intersection,
                            &triangle_positions[0],
                            &triangle_positions[1],
                        ),
                        Vector3::normal(
                            &intersection,
                            &triangle_positions[1],
                            &triangle_positions[2],
                        ),
                        Vector3::normal(
                            &intersection,
                            &triangle_positions[2],
                            &triangle_positions[0],
                        ),
                    ];
                    if Vector3::dot_product(&edge_normals[0], &edge_normals[1]) > 0.0
                        && Vector3::dot_product(&edge_normals[0], &edge_normals[2]) > 0.0
                    {
                        hits.push((intersection, (position - intersection).length_squared()));
                    }
                }
            }

            if !hits.is_empty() {
                // `std::min_element` keeps the FIRST minimum (`<`).
                let mut best = 0;
                for i in 1..hits.len() {
                    if hits[i].1 < hits[best].1 {
                        best = i;
                    }
                }
                self.halfedge_mesh.vertex_mut(current).position = hits[best].0;
            }
        }
    }
}
