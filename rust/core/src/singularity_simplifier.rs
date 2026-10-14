//! Port of `core/singularitysimplifier.*`
//! (`retopo.core.singularity_simplifier`).
//!
//! Line-by-line mirror of `AutoRemesher::SingularitySimplifier`: same
//! functions in the same order, same thresholds (`1e-12` unit fallback,
//! `1e-10` smoother stop, `256`/`40*r^2` iteration cap, `6`/`4`/`6`
//! distance/round/margin defaults, `90` sharp degrees), same `lround`
//! quarter-turn rule, same BFS/pairing greedy loop.
//!
//! Deliberate restructures, each noted at the site:
//! - the C++ `countAffected` lambda is a free function: the closure would
//!   hold `&self` across the angle mutations between its two calls;
//! - the round-loop `Candidate` struct lives at module scope (Rust items
//!   cannot be declared in a loop body as in C++);
//! - the `ball` BFS lambda captures a copied `&SurfaceMesh` instead of
//!   `&self`, so later `&mut` passes need no reborrow dance;
//! - the null-`m_field` case is unrepresentable (`field` is `&mut [Vector3]`,
//!   never null): `simplify` early-returns on an empty slice and
//!   `vertex_charges` returns zeros on a length mismatch, exactly the two
//!   defined C++ branches; a non-empty wrong-length slice indexes out of
//!   bounds and panics where the C++ reads out of bounds (both out of
//!   contract, the Rust side loudly instead of UB).
//!
//! FMA audit (Release IR, `llvm.fmuladd`, per
//! `docs/rust-port-conventions.md`; brew Clang 23.1.2, ARM64, project flags
//! `-O3 -funroll-loops`): 50 fused calls in the TU, 44 inside the
//! `vector3.cppm` inlines (already replicated by the FMA-exact [`Vector3`]
//! port, which this module uses exclusively) and 6 at
//! `singularitysimplifier.cpp:377` — the smoother accumulation, unrolled
//! 3x, two fusions per corner: `T - jump*K` fuses as
//! `fma(-jump, PI/2, T)` and `sum += w*X` as `fma(w, X, sum)`, both
//! mirrored with explicit [`f64::mul_add`] below. Notably UNfused (plain
//! operators, verified absent from the IR): the `u - dot*normal`
//! projections (lines 53/55), every `(a - b - c) / (PI/2)` quarter-turn
//! quotient, and the `cos*U + sin*V` field rebuild arithmetic (line 399).
//! The rebuild's adjacent `std::cos`/`std::sin` calls ARE fused by the
//! backend into one `sincos` libm call (absent from IR, present in
//! disasm: `___sincos_stret`), whose sine differs by 1 ulp from
//! standalone `sin` on some inputs — mirrored with an explicit `sincos`
//! binding, see the call site.
//! `std::lround` lowers to `llvm.lround` (half away from zero, like
//! [`f64::round`]); `std::max(maxChange, fabs)` lowers to
//! `fcmp olt` + select (NaN keeps the accumulator, mirrored with an
//! explicit comparison since [`f64::max`] instead drops NaN).
//!
//! Order-dependence note: the per-round candidate list is sorted by hop
//! count with `std::sort`, whose tie order is unspecified. The pinned
//! libc++ (brew LLVM 23.1.2, `__algorithm/sort.h`) sorts ranges shorter
//! than 24 elements with stable insertion sort / stable sort3-5 networks,
//! so Rust's stable [`slice::sort_by`] reproduces the C++ order exactly
//! whenever every round sees fewer than 24 candidates. Past that (dense
//! singularity clusters) the greedy pairing may legitimately differ; the
//! differential oracle keeps such stress meshes in a structural-only
//! section and tripwires the exact section on the dumped before-count.
//!
//! [`Vector3`]: crate::vector3::Vector3

use crate::surface_mesh::SurfaceMesh;
use crate::vector3::Vector3;
use std::collections::{BTreeSet, VecDeque};
use std::f64::consts::PI;

// Joint sine/cosine comes from the shared `crate::double_utils::joint_sin_cos`
// (audited once there): the C++ backend fuses this rebuild's adjacent
// `std::cos`/`std::sin` calls into one `sincos` libm call, whose sine
// differs by 1 ulp from standalone `sin` on some inputs.

/// Mirrors the anonymous-namespace `unit` (by value: `Vector3` is `Copy`,
/// so this compiles to the same loads as the C++ const refs).
#[inline]
fn unit(value: Vector3, fallback: Vector3) -> Vector3 {
    if value.length() < 1e-12 {
        fallback.normalized()
    } else {
        value.normalized()
    }
}

#[derive(Clone, Copy)]
struct Frame {
    u: Vector3,
    v: Vector3,
}

fn frame_for_face(mesh: &SurfaceMesh, face: usize) -> Frame {
    let normal = unit(mesh.face_normal(face), Vector3::new(0.0, 0.0, 1.0));
    let mut u = mesh.edge_vector(3 * face);
    // FMA audit: the C++ keeps these projections unfused (separate
    // fmul/fsub in IR), so the plain operators below are exact.
    u = u - Vector3::dot_product(&u, &normal) * normal;
    u = unit(
        u,
        if normal.x().abs() < 0.9 {
            Vector3::new(1.0, 0.0, 0.0)
        } else {
            Vector3::new(0.0, 1.0, 0.0)
        },
    );
    u = u - Vector3::dot_product(&u, &normal) * normal;
    u = unit(u, Vector3::new(1.0, 0.0, 0.0));
    Frame {
        u,
        v: Vector3::cross_product(&normal, &u),
    }
}

fn angle_in_frame(vector: &Vector3, frame: &Frame) -> f64 {
    Vector3::dot_product(vector, &frame.v).atan2(Vector3::dot_product(vector, &frame.u))
}

fn quarter_turn(mesh: &SurfaceMesh, corner: usize, frames: &[Frame], field_angles: &[f64]) -> i32 {
    let opposite = mesh.opposite_corner(corner);
    if opposite == SurfaceMesh::NPOS {
        return 0;
    }
    let f = mesh.corner_face(corner);
    let g = mesh.corner_face(opposite);
    let edge = mesh.edge_vector(corner);
    let connection = angle_in_frame(&edge, &frames[g]) - angle_in_frame(&edge, &frames[f]);
    // `std::lround` is half-away-from-zero, exactly `f64::round`; the
    // values here are small, so the float->int cast never saturates.
    let turns = ((field_angles[g] - field_angles[f] - connection) / (PI / 2.0)).round() as i32;
    ((turns % 4) + 4) % 4
}

/// Counts nonzero charges over the affected set (mirrors the C++
/// `countAffected` lambda; a free function because the closure form would
/// hold `&self` across the angle mutations between its two calls).
fn count_affected(charges: &[i32], affected: &[bool]) -> usize {
    let mut count = 0;
    for (v, &is_affected) in affected.iter().enumerate() {
        if is_affected && charges[v] != 0 {
            count += 1;
        }
    }
    count
}

/// Round-loop pairing candidate (mirrors the C++ block-local struct,
/// lifted to module scope).
struct Candidate {
    a: usize,
    b: usize,
    hops: usize,
}

/// Dipole canceller over a per-face cross field (mirrors
/// `AutoRemesher::SingularitySimplifier`).
pub struct SingularitySimplifier<'a> {
    mesh: &'a SurfaceMesh,
    field: &'a mut [Vector3],
    maximum_pair_distance: usize,
    maximum_rounds: usize,
    region_margin: usize,
    sharp_edge_degrees: f64,
    singularity_count_before: usize,
    singularity_count_after: usize,
    cancelled_pair_count: usize,
    angles: Vec<f64>,
    connection: Vec<f64>,
    mismatch: Vec<i32>,
    sharp_corner: Vec<bool>,
    frame_u: Vec<Vector3>,
    frame_v: Vec<Vector3>,
}

impl<'a> SingularitySimplifier<'a> {
    /// Mirrors the C++ constructor (stores the mesh and field references;
    /// the caches are built by [`Self::simplify`]).
    pub fn new(mesh: &'a SurfaceMesh, field: &'a mut [Vector3]) -> Self {
        Self {
            mesh,
            field,
            maximum_pair_distance: 6,
            maximum_rounds: 4,
            region_margin: 6,
            sharp_edge_degrees: 90.0,
            singularity_count_before: 0,
            singularity_count_after: 0,
            cancelled_pair_count: 0,
            angles: Vec::new(),
            connection: Vec::new(),
            mismatch: Vec::new(),
            sharp_corner: Vec::new(),
            frame_u: Vec::new(),
            frame_v: Vec::new(),
        }
    }

    pub fn set_maximum_pair_distance(&mut self, hops: usize) {
        self.maximum_pair_distance = hops;
    }

    pub fn set_sharp_edge_degrees(&mut self, degrees: f64) {
        self.sharp_edge_degrees = degrees;
    }

    pub fn vertex_charges(&self) -> Vec<i32> {
        let mut result = vec![0i32; self.mesh.vertex_count()];
        if self.field.len() != self.mesh.face_count() {
            return result;
        }
        // simplify() is an inner loop that calls this several times per cancelled
        // pair, so only pay for the per-face frames when the cached corner
        // mismatches are missing and quarterTurn actually needs them.
        let use_cached_mismatch = self.mismatch.len() == self.mesh.corner_count();
        let mut frames: Vec<Frame> = Vec::new();
        let mut field_angles: Vec<f64> = Vec::new();
        if !use_cached_mismatch {
            frames.reserve(self.mesh.face_count());
            field_angles.reserve(self.mesh.face_count());
            for f in 0..self.mesh.face_count() {
                frames.push(frame_for_face(self.mesh, f));
                let back = frames.len() - 1;
                field_angles.push(angle_in_frame(&self.field[f], &frames[back]));
            }
        }
        for v in 0..self.mesh.vertex_count() {
            let corners = self.mesh.corners_around_vertex(v);
            if corners.is_empty() {
                continue;
            }
            let mut c = corners[0];
            let start = c;
            let mut sum = 0i32;
            let mut steps = 0usize;
            loop {
                steps += 1;
                if steps > self.mesh.corner_count() {
                    sum = 0;
                    break;
                }
                sum += if use_cached_mismatch {
                    self.mismatch[c]
                } else {
                    quarter_turn(self.mesh, c, &frames, &field_angles)
                };
                let opposite = self.mesh.opposite_corner(c);
                if opposite == SurfaceMesh::NPOS {
                    sum = 0;
                    break;
                }
                c = self.mesh.next_corner(opposite);
                if self.mesh.corner_vertex(c) != v {
                    sum = 0;
                    break;
                }
                if c == start {
                    break;
                }
            }
            if c == start {
                result[v] = ((sum % 4) + 4) % 4;
            }
        }
        result
    }

    pub fn singularity_count(&self) -> usize {
        let mut result = 0;
        for charge in self.vertex_charges() {
            if charge != 0 {
                result += 1;
            }
        }
        result
    }

    pub fn singularity_count_before(&self) -> usize {
        self.singularity_count_before
    }

    pub fn singularity_count_after(&self) -> usize {
        self.singularity_count_after
    }

    pub fn cancelled_pair_count(&self) -> usize {
        self.cancelled_pair_count
    }

    fn build_frames_and_connection(&mut self) {
        let faces = self.mesh.face_count();
        let corners = self.mesh.corner_count();
        self.frame_u.resize(faces, Vector3::default());
        self.frame_v.resize(faces, Vector3::default());
        self.angles.resize(faces, 0.0);
        for f in 0..faces {
            let frame = frame_for_face(self.mesh, f);
            self.frame_u[f] = frame.u;
            self.frame_v[f] = frame.v;
            self.angles[f] = Vector3::dot_product(&self.field[f], &frame.v)
                .atan2(Vector3::dot_product(&self.field[f], &frame.u));
        }
        // `assign` semantics (clear + refill): a second simplify() call
        // must not see stale corners.
        self.connection.clear();
        self.connection.resize(corners, 0.0);
        self.mismatch.clear();
        self.mismatch.resize(corners, 0);
        self.sharp_corner.clear();
        self.sharp_corner.resize(corners, false);
        for c in 0..corners {
            let other = self.mesh.opposite_corner(c);
            if other == SurfaceMesh::NPOS {
                continue;
            }
            let f = self.mesh.corner_face(c);
            let g = self.mesh.corner_face(other);
            let edge = self.mesh.edge_vector(c);
            self.connection[c] = Vector3::dot_product(&edge, &self.frame_v[g])
                .atan2(Vector3::dot_product(&edge, &self.frame_u[g]))
                - Vector3::dot_product(&edge, &self.frame_v[f])
                    .atan2(Vector3::dot_product(&edge, &self.frame_u[f]));
            self.sharp_corner[c] =
                self.mesh.normal_angle(c) >= self.sharp_edge_degrees * PI / 180.0;
        }
        let every: Vec<usize> = (0..faces).collect();
        self.update_mismatches(&every);
    }

    fn update_mismatches(&mut self, faces: &[usize]) {
        for &f in faces {
            for c in (3 * f)..(3 * f + 3) {
                let other = self.mesh.opposite_corner(c);
                if other == SurfaceMesh::NPOS {
                    continue;
                }
                let g = self.mesh.corner_face(other);
                let turns = ((self.angles[g] - (self.angles[f] + self.connection[c])) / (PI / 2.0))
                    .round() as i32;
                self.mismatch[c] = ((turns % 4) + 4) % 4;
                self.mismatch[other] = (4 - self.mismatch[c]) % 4;
            }
        }
    }

    fn facets_around_vertex(&self, vertex: usize) -> Vec<usize> {
        let incident = self.mesh.corners_around_vertex(vertex);
        if incident.is_empty() {
            return Vec::new();
        }
        let mut result = Vec::new();
        let mut c = incident[0];
        let start = c;
        loop {
            if result.len() > self.mesh.corner_count() {
                return Vec::new();
            }
            result.push(self.mesh.corner_face(c));
            let other = self.mesh.opposite_corner(c);
            if other == SurfaceMesh::NPOS {
                return Vec::new();
            }
            c = self.mesh.next_corner(other);
            if self.mesh.corner_vertex(c) != vertex {
                return Vec::new();
            }
            if c == start {
                break;
            }
        }
        result
    }

    fn corner_along_edge(&self, from: usize, to: usize) -> usize {
        for &c in self.mesh.corners_around_vertex(from) {
            if self.mesh.corner_vertex(self.mesh.next_corner(c)) == to
                && self.mesh.opposite_corner(c) != SurfaceMesh::NPOS
            {
                return c;
            }
        }
        SurfaceMesh::NPOS
    }

    fn path_between(
        &self,
        first: usize,
        second: usize,
        in_region: &[bool],
        free_faces: &[usize],
    ) -> Vec<usize> {
        let mut is_free = vec![false; self.mesh.face_count()];
        for &f in free_faces {
            is_free[f] = true;
        }
        let usable = |vertex: usize| {
            let fan = self.facets_around_vertex(vertex);
            if fan.is_empty() {
                return false;
            }
            for &f in &fan {
                if !in_region[f] {
                    return false;
                }
            }
            true
        };
        let crossable = |corner: usize| {
            let other = self.mesh.opposite_corner(corner);
            other != SurfaceMesh::NPOS
                && is_free[self.mesh.corner_face(corner)]
                && is_free[self.mesh.corner_face(other)]
        };
        let mut previous = vec![SurfaceMesh::NPOS; self.mesh.vertex_count()];
        let mut queue = VecDeque::new();
        previous[first] = first;
        queue.push_back(first);
        while let Some(v) = queue.pop_front() {
            if v == second {
                let mut path = Vec::new();
                let mut at = second;
                while at != first {
                    path.push(at);
                    at = previous[at];
                }
                path.push(first);
                path.reverse();
                return path;
            }
            for &c in self.mesh.corners_around_vertex(v) {
                let next = self.mesh.corner_vertex(self.mesh.next_corner(c));
                if previous[next] != SurfaceMesh::NPOS || !crossable(c) {
                    continue;
                }
                if next != second && !usable(next) {
                    continue;
                }
                previous[next] = v;
                queue.push_back(next);
            }
        }
        Vec::new()
    }

    fn cancel_pair(&mut self, first: usize, second: usize, hops: usize) -> bool {
        let radius = hops + self.region_margin;
        let mesh = self.mesh;
        let ball = |seeds: &[usize]| {
            let mut distance = vec![SurfaceMesh::NPOS; mesh.face_count()];
            let mut queue = VecDeque::new();
            for &f in seeds {
                if distance[f] == SurfaceMesh::NPOS {
                    distance[f] = 0;
                    queue.push_back(f);
                }
            }
            while let Some(f) = queue.pop_front() {
                if distance[f] >= radius {
                    continue;
                }
                for c in (3 * f)..(3 * f + 3) {
                    let other = mesh.opposite_corner(c);
                    if other == SurfaceMesh::NPOS {
                        continue;
                    }
                    let g = mesh.corner_face(other);
                    if distance[g] == SurfaceMesh::NPOS {
                        distance[g] = distance[f] + 1;
                        queue.push_back(g);
                    }
                }
            }
            distance
        };
        let first_fan = self.facets_around_vertex(first);
        let second_fan = self.facets_around_vertex(second);
        if first_fan.is_empty() || second_fan.is_empty() {
            return false;
        }
        let a = ball(&first_fan);
        let b = ball(&second_fan);
        let mut in_region = vec![false; self.mesh.face_count()];
        let mut region: Vec<usize> = Vec::new();
        let mut free_faces: Vec<usize> = Vec::new();
        for f in 0..self.mesh.face_count() {
            if a[f] != SurfaceMesh::NPOS && b[f] != SurfaceMesh::NPOS {
                in_region[f] = true;
                region.push(f);
            }
        }
        if region.is_empty() {
            return false;
        }
        for &f in &region {
            let mut frozen = false;
            for c in (3 * f)..(3 * f + 3) {
                let other = self.mesh.opposite_corner(c);
                if self.sharp_corner[c]
                    || other == SurfaceMesh::NPOS
                    || !in_region[self.mesh.corner_face(other)]
                {
                    frozen = true;
                    break;
                }
            }
            if !frozen {
                free_faces.push(f);
            }
        }
        if free_faces.is_empty() {
            return false;
        }
        for v in [first, second] {
            for f in self.facets_around_vertex(v) {
                if !in_region[f] {
                    return false;
                }
            }
        }
        let path = self.path_between(first, second, &in_region, &free_faces);
        if path.len() < 2 {
            return false;
        }
        let mut affected = vec![false; self.mesh.vertex_count()];
        for &f in &region {
            for c in (3 * f)..(3 * f + 3) {
                affected[self.mesh.corner_vertex(c)] = true;
            }
        }
        let before = count_affected(&self.vertex_charges(), &affected);
        let mut saved: Vec<f64> = Vec::with_capacity(free_faces.len());
        for &f in &free_faces {
            saved.push(self.angles[f]);
        }

        let mut jump = vec![0i32; self.mesh.corner_count()];
        for &f in &region {
            for c in (3 * f)..(3 * f + 3) {
                let other = self.mesh.opposite_corner(c);
                if other == SurfaceMesh::NPOS {
                    continue;
                }
                jump[c] = ((self.angles[self.mesh.corner_face(other)]
                    - self.angles[f]
                    - self.connection[c])
                    / (PI / 2.0))
                    .round() as i32;
            }
        }
        let carried = (((self.vertex_charges()[first] + 1) % 4) + 4) % 4 - 1;
        for i in 0..path.len() - 1 {
            let corner = self.corner_along_edge(path[i], path[i + 1]);
            if corner == SurfaceMesh::NPOS {
                return false;
            }
            let other = self.mesh.opposite_corner(corner);
            jump[corner] -= carried;
            jump[other] += carried;
        }

        for _ in 0..256usize.max(40 * radius * radius) {
            let mut max_change = 0.0;
            for &f in &free_faces {
                let mut sum = 0.0;
                let mut sw = 0.0;
                for c in (3 * f)..(3 * f + 3) {
                    let other = self.mesh.opposite_corner(c);
                    if other == SurfaceMesh::NPOS {
                        continue;
                    }
                    let g = self.mesh.corner_face(other);
                    let w = self.mesh.edge_vector(c).length();
                    // FMA audit: the C++ fuses this line into two fmuladds
                    // per corner (verified in Release IR):
                    // `fma(-jump, PI/2, angles[g] - connection[c])` for the
                    // parenthesized value, then `fma(w, value, sum)` for the
                    // accumulation. `sw += w` stays a plain fadd.
                    let t = self.angles[g] - self.connection[c];
                    let x = (-(jump[c] as f64)).mul_add(PI / 2.0, t);
                    sum = w.mul_add(x, sum);
                    sw += w;
                }
                if sw > 0.0 {
                    let value = sum / sw;
                    // `std::max(maxChange, fabs)` is `fcmp olt` + select in
                    // IR (NaN keeps the accumulator); `f64::max` would drop
                    // NaN instead, so mirror the comparison explicitly.
                    let delta = (value - self.angles[f]).abs();
                    max_change = if max_change < delta {
                        delta
                    } else {
                        max_change
                    };
                    self.angles[f] = value;
                }
            }
            if max_change < 1e-10 {
                break;
            }
        }
        self.update_mismatches(&region);
        let after = count_affected(&self.vertex_charges(), &affected);
        let charges = self.vertex_charges();
        if charges[first] != 0 || charges[second] != 0 || after + 2 > before {
            for (i, &f) in free_faces.iter().enumerate() {
                self.angles[f] = saved[i];
            }
            self.update_mismatches(&region);
            return false;
        }
        // FP audit: the C++ keeps this rebuild's arithmetic unfused (plain
        // fmul/fadd in IR and machine code, verified in disasm) — but the
        // backend fuses the adjacent `std::cos`/`std::sin` calls into a
        // single `sincos` libm call (`___sincos_stret` in the binary), whose
        // sine differs by 1 ulp from standalone `sin` on some inputs (case
        // 11 face 15: `0x...6158` vs `0x...6159`). The shared
        // `crate::double_utils::joint_sin_cos` binds that same entry
        // point, matching the C++ values bitwise.
        for &f in &free_faces {
            let (s, c) = crate::double_utils::joint_sin_cos(self.angles[f]);
            self.field[f] = c * self.frame_u[f] + s * self.frame_v[f];
        }
        true
    }

    pub fn simplify(&mut self) {
        if self.field.is_empty() {
            return;
        }
        self.build_frames_and_connection();
        self.cancelled_pair_count = 0;
        let initial = self.vertex_charges();
        self.singularity_count_before = 0;
        for i in initial {
            if i != 0 {
                self.singularity_count_before += 1;
            }
        }
        self.singularity_count_after = self.singularity_count_before;
        for _ in 0..self.maximum_rounds {
            let charges = self.vertex_charges();
            let mut singular: Vec<usize> = Vec::new();
            for (v, &charge) in charges.iter().enumerate() {
                if charge != 0 {
                    singular.push(v);
                }
            }
            if singular.len() < 2 {
                break;
            }
            let mut owner = vec![SurfaceMesh::NPOS; self.mesh.face_count()];
            let mut distance = vec![0usize; self.mesh.face_count()];
            let mut queue = VecDeque::new();
            let mut encounters: Vec<(usize, usize)> = Vec::new();
            for (i, &s) in singular.iter().enumerate() {
                for f in self.facets_around_vertex(s) {
                    if owner[f] == SurfaceMesh::NPOS {
                        owner[f] = i;
                        queue.push_back(f);
                    } else {
                        encounters.push((owner[f], i));
                    }
                }
            }
            let mut candidates: Vec<Candidate> = Vec::new();
            let mut seen = BTreeSet::new();
            let maximum_pair_distance = self.maximum_pair_distance;
            let mut consider = |a: usize, b: usize, d: usize| {
                if a == b
                    || d > maximum_pair_distance
                    || (charges[singular[a]] + charges[singular[b]]) % 4 != 0
                {
                    return;
                }
                let key = if a < b { (a, b) } else { (b, a) };
                if seen.insert(key) {
                    candidates.push(Candidate {
                        a: key.0,
                        b: key.1,
                        hops: d,
                    });
                }
            };
            for &(enc_first, enc_second) in &encounters {
                consider(enc_first, enc_second, 0);
            }
            while let Some(f) = queue.pop_front() {
                for c in (3 * f)..(3 * f + 3) {
                    let other = self.mesh.opposite_corner(c);
                    if other == SurfaceMesh::NPOS {
                        continue;
                    }
                    let g = self.mesh.corner_face(other);
                    if owner[g] == SurfaceMesh::NPOS {
                        if distance[f] + 1 > maximum_pair_distance {
                            continue;
                        }
                        owner[g] = owner[f];
                        distance[g] = distance[f] + 1;
                        queue.push_back(g);
                    } else {
                        consider(owner[f], owner[g], distance[f] + distance[g]);
                    }
                }
            }
            // Stable sort: identical to the pinned libc++ `std::sort` for
            // the < 24-candidate regime (see the module docs); past that
            // the C++ tie order is unspecified anyway.
            candidates.sort_by_key(|x| x.hops);
            let mut used = vec![false; singular.len()];
            let mut cancelled = 0usize;
            for c in &candidates {
                if !used[c.a]
                    && !used[c.b]
                    && self.cancel_pair(singular[c.a], singular[c.b], c.hops)
                {
                    used[c.a] = true;
                    used[c.b] = true;
                    cancelled += 1;
                }
            }
            if cancelled == 0 {
                break;
            }
            self.cancelled_pair_count += cancelled;
        }
        self.singularity_count_after = self.singularity_count();
    }
}
