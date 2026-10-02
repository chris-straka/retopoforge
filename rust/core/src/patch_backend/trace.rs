//! Motorcycle separatrix tracing for the patch back end.
//!
//! Clean-room patch layout in the QuadWild/QGP family shape: from each
//! cross-field singularity, trace one ray per sector along mesh edges
//! (edge-walking keeps every arc on vertices, so arc crossings are
//! exact and need no epsilon logic). Rays run to completion —
//! singularities, mesh boundary, loops, or dead ends — and CROSS other
//! arcs instead of stopping at them, so the layout has no T-junctions:
//! every interior node is a crossing (valence 4, regular) or a
//! singularity. The traced graph partitions the working mesh into
//! patches in [`crate::patch_backend::layout`].
//!
//! Determinism: all iteration is over sorted containers or vertex
//! order, and every geometric choice has a vertex-id tie-break, so the
//! same input yields the same graph on every run and platform.

use crate::surface_mesh::SurfaceMesh;
use crate::vector3::Vector3;
use std::collections::BTreeMap;
use std::collections::BTreeSet;
use std::f64::consts::PI;

/// Layout node: an arc endpoint or crossing.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum NodeKind {
    /// Cross-field singularity (valence 2/3/5/6).
    Singularity,
    /// Two arcs crossing (valence 4).
    Crossing,
    /// Ray reached the mesh boundary.
    Boundary,
    /// Ray ran out of forward edges (or hit the step cap).
    DeadEnd,
    /// Artificial seed on singularity-free islands.
    Seed,
    /// Ray closed a loop away from any node.
    Loop,
}

/// Layout node: always on a working-mesh vertex.
#[derive(Clone, Copy, Debug)]
pub(crate) struct TraceNode {
    pub vertex: usize,
    pub kind: NodeKind,
}

/// One traced arc: a vertex chain between two nodes.
#[derive(Clone, Debug)]
pub(crate) struct TraceArc {
    pub a: usize,
    pub b: usize,
    pub path: Vec<usize>,
}

/// Traced separatrix graph.
#[derive(Clone, Debug, Default)]
pub(crate) struct ArcGraph {
    pub nodes: Vec<TraceNode>,
    pub arcs: Vec<TraceArc>,
}

#[cfg(test)]
impl ArcGraph {
    /// Polyline length of an arc in mesh units.
    pub(crate) fn arc_length(&self, arc: usize, verts: &[Vector3]) -> f64 {
        let mut length = 0.0;
        let path = &self.arcs[arc].path;
        for w in path.windows(2) {
            length += (verts[w[1]] - verts[w[0]]).length();
        }
        length
    }
}

/// Faces around a vertex in cyclic order (one ring walk).
///
/// Returns corner ids in ring order plus whether the ring closed (open
/// at a mesh boundary otherwise). Deterministic: starts from the
/// smallest corner id.
fn ordered_ring(topology: &SurfaceMesh, vertex: usize) -> (Vec<usize>, bool) {
    let corners = topology.corners_around_vertex(vertex);
    if corners.is_empty() {
        return (Vec::new(), false);
    }
    let start = *corners.iter().min().unwrap();
    // Walk to the ring start (a boundary corner, or back to `start`
    // when the ring is closed). Bounded: pathological vertices must
    // terminate even if the predecessor chain cycles.
    let mut first = start;
    for _ in 0..corners.len() + 1 {
        let opposite = topology.opposite_corner(topology.previous_corner(first));
        if opposite == SurfaceMesh::NPOS {
            break;
        }
        let next = topology.next_corner(opposite);
        if topology.corner_vertex(next) != vertex || next == start {
            break;
        }
        first = next;
        if first == start {
            break;
        }
    }
    let mut ring = vec![first];
    let mut closed = false;
    loop {
        let last = *ring.last().unwrap();
        let opposite = topology.opposite_corner(last);
        if opposite == SurfaceMesh::NPOS {
            break;
        }
        let next = topology.next_corner(opposite);
        if topology.corner_vertex(next) != vertex {
            break;
        }
        if next == first {
            closed = true;
            break;
        }
        if ring.len() > corners.len() + 1 {
            break;
        }
        ring.push(next);
    }
    (ring, closed)
}

/// Rotate vector `v` around unit axis `axis` by `angle` (radians).
fn rotate_around_axis(v: &Vector3, axis: &Vector3, angle: f64) -> Vector3 {
    let cos = angle.cos();
    let sin = angle.sin();
    let dot = Vector3::dot_product(axis, v);
    let cross = Vector3::cross_product(axis, v);
    Vector3::new(
        v.x() * cos + cross.x() * sin + axis.x() * dot * (1.0 - cos),
        v.y() * cos + cross.y() * sin + axis.y() * dot * (1.0 - cos),
        v.z() * cos + cross.z() * sin + axis.z() * dot * (1.0 - cos),
    )
}

/// Signed quarter-turn sum of the cross field around a vertex.
///
/// Parallel-transports the first face's field vector around the ring
/// and accumulates the signed mismatch (in units of pi/2) against each
/// face's own field vector. The sign convention matches positive
/// index: a valence-3 singularity (index +1/4) sums to +1.
fn winding_sum(topology: &SurfaceMesh, verts: &[Vector3], field: &[Vector3], vertex: usize) -> i32 {
    let (ring, closed) = ordered_ring(topology, vertex);
    if ring.len() < 2 || !closed {
        return 0;
    }
    let faces: Vec<usize> = ring.iter().map(|&c| topology.corner_face(c)).collect();
    let mut transported = field[faces[0]];
    let mut total = 0i32;
    for i in 0..faces.len() {
        let f_next = faces[(i + 1) % faces.len()];
        // Shared edge: (vertex, w) for some w; rotate about it by the
        // signed dihedral from faces[i] to f_next.
        let n0 = topology.face_normal(faces[i]);
        let n1 = topology.face_normal(f_next);
        let axis_unnorm = Vector3::cross_product(&n0, &n1);
        let sin_d = axis_unnorm.length();
        // Edge direction (vertex -> w): the corner's next vertex.
        let corner = ring[i];
        let next_corner = topology.next_corner(corner);
        let w = topology.corner_vertex(next_corner);
        let mut axis = verts[w] - verts[vertex];
        let axis_len = axis.length();
        if axis_len <= 0.0 {
            return 0;
        }
        axis = Vector3::new(
            axis.x() / axis_len,
            axis.y() / axis_len,
            axis.z() / axis_len,
        );
        // Orient the axis so the rotation carries n0 to n1.
        let mut angle = sin_d.atan2(Vector3::dot_product(&n0, &n1).clamp(-1.0, 1.0));
        let trial = rotate_around_axis(&n0, &axis, angle);
        let flipped = rotate_around_axis(&n0, &axis, -angle);
        if (trial - n1).length_squared() > (flipped - n1).length_squared() {
            angle = -angle;
        }
        transported = rotate_around_axis(&transported, &axis, angle);
        // Signed mismatch vs f_next's field, in quarter turns.
        let target = field[f_next];
        let u = transported;
        let v = Vector3::cross_product(&n1, &u);
        let x = Vector3::dot_product(&target, &u);
        let y = Vector3::dot_product(&target, &v);
        let mismatch = y.atan2(x) / (PI / 2.0);
        // Wrap to [-2, 2].
        let mut m = mismatch.round() as i32 % 4;
        if m > 2 {
            m -= 4;
        } else if m < -2 {
            m += 4;
        }
        total += m;
        transported = target;
    }
    total
}

/// Ray count for a singularity charge: 1 -> 3, 3 -> 5, 2 -> measured
/// (2 for index +1/2, 6 for index -1/2, 4 when the measurement is
/// degenerate). Any count yields a valid layout; the true sector
/// count only improves corner alignment.
fn ray_count_for(
    topology: &SurfaceMesh,
    verts: &[Vector3],
    field: &[Vector3],
    vertex: usize,
    charge: i32,
) -> usize {
    match charge {
        1 => 3,
        3 => 5,
        2 => {
            let sum = winding_sum(topology, verts, field, vertex);
            if sum > 0 {
                2
            } else if sum < 0 {
                6
            } else {
                4
            }
        }
        _ => 0,
    }
}

/// Orthonormal tangent basis at a vertex normal.
fn tangent_basis(normal: &Vector3) -> (Vector3, Vector3) {
    let helper = if normal.x().abs() < 0.9 {
        Vector3::new(1.0, 0.0, 0.0)
    } else {
        Vector3::new(0.0, 1.0, 0.0)
    };
    let mut t1 = Vector3::cross_product(normal, &helper);
    if t1.length_squared() <= 1e-24 {
        t1 = Vector3::new(1.0, 0.0, 0.0);
    } else {
        t1.normalize();
    }
    let t2 = Vector3::cross_product(normal, &t1);
    (t1, t2)
}

/// Seed directions for a singularity: one field-aligned unit tangent
/// per sector, spread evenly around the vertex.
///
/// Per neighboring face, takes the cross arm best aligned with the
/// radial direction, transports it to the vertex tangent plane, sorts
/// the arms by angle, and picks evenly spaced quantiles.
fn seed_directions(
    topology: &SurfaceMesh,
    verts: &[Vector3],
    field: &[Vector3],
    vertex_normals: &[Vector3],
    vertex: usize,
    count: usize,
) -> Vec<Vector3> {
    let (ring, _) = ordered_ring(topology, vertex);
    let normal = vertex_normals[vertex];
    if normal.length_squared() <= 1e-24 || ring.is_empty() || count == 0 {
        return Vec::new();
    }
    let (t1, t2) = tangent_basis(&normal);
    let mut arms: Vec<(f64, Vector3)> = Vec::new();
    for &corner in &ring {
        let face = topology.corner_face(corner);
        let tri = topology.triangle(face);
        let centroid = Vector3::new(
            (verts[tri[0]].x() + verts[tri[1]].x() + verts[tri[2]].x()) / 3.0,
            (verts[tri[0]].y() + verts[tri[1]].y() + verts[tri[2]].y()) / 3.0,
            (verts[tri[0]].z() + verts[tri[1]].z() + verts[tri[2]].z()) / 3.0,
        );
        let face_normal = topology.face_normal(face);
        let mut radial = centroid - verts[vertex];
        let along = Vector3::dot_product(&radial, &face_normal);
        radial = radial - face_normal * along;
        if radial.length_squared() <= 1e-24 {
            continue;
        }
        radial.normalize();
        let u = field[face];
        let v = Vector3::cross_product(&face_normal, &u);
        let candidates = [u, v, u * -1.0, v * -1.0];
        let mut best = candidates[0];
        let mut best_dot = f64::NEG_INFINITY;
        for candidate in &candidates {
            let dot = Vector3::dot_product(candidate, &radial);
            if dot > best_dot {
                best_dot = dot;
                best = *candidate;
            }
        }
        // Transport to the vertex tangent plane.
        let along_v = Vector3::dot_product(&best, &normal);
        let mut flat = best - normal * along_v;
        if flat.length_squared() <= 1e-18 {
            continue;
        }
        flat.normalize();
        let angle = Vector3::dot_product(&flat, &t2).atan2(Vector3::dot_product(&flat, &t1));
        arms.push((angle, flat));
    }
    if arms.is_empty() {
        return Vec::new();
    }
    arms.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    (0..count).map(|i| arms[i * arms.len() / count].1).collect()
}

/// The motorcycle tracer: owns the graph plus the lookup tables.
struct Tracer<'a> {
    topology: &'a SurfaceMesh,
    verts: &'a [Vector3],
    field: &'a [Vector3],
    vertex_normals: Vec<Vector3>,
    neighbors: Vec<Vec<usize>>,
    boundary: BTreeSet<usize>,
    graph: ArcGraph,
    node_of_vertex: BTreeMap<usize, usize>,
    /// Vertices lying on arcs: vertex -> list of (arc, path index).
    /// Nodes have entries too (as endpoints).
    arc_membership: BTreeMap<usize, Vec<(usize, usize)>>,
    max_steps: usize,
}

/// Minimum forward alignment (dot product) for walk steps after the
/// first. Negative would allow reversals; zero keeps rays moving with
/// the field while tolerating near-right-angle turns on coarse spans.
const DOT_MIN: f64 = -0.1;

impl<'a> Tracer<'a> {
    fn new(topology: &'a SurfaceMesh, verts: &'a [Vector3], field: &'a [Vector3]) -> Self {
        let n = topology.vertex_count();
        let mut vertex_normals = vec![Vector3::default(); n];
        for f in 0..topology.face_count() {
            let tri = topology.triangle(f);
            let e0 = verts[tri[0]] - verts[tri[2]];
            let e1 = verts[tri[1]] - verts[tri[2]];
            let face_normal = Vector3::cross_product(&e0, &e1);
            for &v in tri {
                vertex_normals[v] += face_normal;
            }
        }
        for normal in &mut vertex_normals {
            if normal.length_squared() > 1e-30 {
                normal.normalize();
            }
        }
        let mut neighbors: Vec<Vec<usize>> = vec![Vec::new(); n];
        for f in 0..topology.face_count() {
            let tri = topology.triangle(f);
            for i in 0..3 {
                let a = tri[i];
                let b = tri[(i + 1) % 3];
                neighbors[a].push(b);
                neighbors[b].push(a);
            }
        }
        for ring in &mut neighbors {
            ring.sort_unstable();
            ring.dedup();
        }
        let mut boundary = BTreeSet::new();
        for c in 0..topology.corner_count() {
            if topology.is_boundary_corner(c) {
                boundary.insert(topology.corner_vertex(c));
                boundary.insert(topology.corner_vertex(topology.next_corner(c)));
            }
        }
        let max_steps = 4 * n.max(64);
        Self {
            topology,
            verts,
            field,
            vertex_normals,
            neighbors,
            boundary,
            graph: ArcGraph::default(),
            node_of_vertex: BTreeMap::new(),
            arc_membership: BTreeMap::new(),
            max_steps,
        }
    }

    fn add_node(&mut self, vertex: usize, kind: NodeKind) -> usize {
        if let Some(&id) = self.node_of_vertex.get(&vertex) {
            return id;
        }
        let id = self.graph.nodes.len();
        self.graph.nodes.push(TraceNode { vertex, kind });
        self.node_of_vertex.insert(vertex, id);
        id
    }

    fn register_arc(&mut self, a: usize, b: usize, path: Vec<usize>) -> usize {
        let id = self.graph.arcs.len();
        for (index, &vertex) in path.iter().enumerate() {
            self.arc_membership
                .entry(vertex)
                .or_default()
                .push((id, index));
        }
        self.graph.arcs.push(TraceArc { a, b, path });
        id
    }

    /// Split arc `arc` at path index `at` (interior) into two arcs
    /// meeting at a new crossing node. Returns the new node id.
    ///
    /// The caller guarantees `at` is interior and `path[at]` is the
    /// crossing vertex; out-of-range calls are ignored defensively.
    fn split_arc(&mut self, arc: usize, at: usize, vertex: usize) -> Option<usize> {
        let (a, b, path) = {
            let existing = &self.graph.arcs[arc];
            (existing.a, existing.b, existing.path.clone())
        };
        if at == 0 || at + 1 >= path.len() || path[at] != vertex {
            return None;
        }
        let node = self.add_node(vertex, NodeKind::Crossing);
        let second_id = self.graph.arcs.len();
        // Remap every (arc, index) membership entry along the path:
        // head indices keep their arc, tail indices move to the new
        // arc, and the split vertex terminates both. Per-entry (not
        // per-vertex) so repeated vertices in looped paths keep each
        // occurrence registered.
        for (index, &v) in path.iter().enumerate() {
            let Some(entries) = self.arc_membership.get_mut(&v) else {
                continue;
            };
            let mut moved: Vec<(usize, usize)> = Vec::new();
            entries.retain(|&(aid, idx)| {
                if aid != arc || idx != index {
                    return true;
                }
                if index < at {
                    moved.push((arc, idx));
                } else if index == at {
                    moved.push((arc, at));
                    moved.push((second_id, 0));
                } else {
                    moved.push((second_id, idx - at));
                }
                false
            });
            entries.extend(moved.iter().copied());
        }
        self.graph.arcs[arc] = TraceArc {
            a,
            b: node,
            path: path[..=at].to_vec(),
        };
        self.graph.arcs.push(TraceArc {
            a: node,
            b,
            path: path[at..].to_vec(),
        });
        Some(node)
    }

    /// Best outgoing edge from `v` along `direction`: (neighbor, dot).
    /// Deterministic: highest dot wins, lowest vertex id breaks ties.
    fn best_step(
        &self,
        v: usize,
        direction: &Vector3,
        prev: Option<usize>,
    ) -> Option<(usize, f64)> {
        let normal = self.vertex_normals[v];
        if normal.length_squared() <= 1e-24 {
            return None;
        }
        let mut best: Option<(usize, f64)> = None;
        for &u in &self.neighbors[v] {
            if Some(u) == prev {
                continue;
            }
            let mut edge = self.verts[u] - self.verts[v];
            let along = Vector3::dot_product(&edge, &normal);
            edge = edge - normal * along;
            if edge.length_squared() <= 1e-24 {
                continue;
            }
            edge.normalize();
            let dot = Vector3::dot_product(&edge, direction);
            let replace = match best {
                None => true,
                Some((bu, bdot)) => dot > bdot || (dot == bdot && u < bu),
            };
            if replace {
                best = Some((u, dot));
            }
        }
        best
    }

    /// Transport a tangent direction from vertex `from` to vertex `to`
    /// (projection onto the target tangent plane). Falls back to the
    /// edge direction when the projection degenerates.
    fn transport(&self, direction: &Vector3, from: usize, to: usize) -> Option<Vector3> {
        let normal = self.vertex_normals[to];
        if normal.length_squared() <= 1e-24 {
            return None;
        }
        let along = Vector3::dot_product(direction, &normal);
        let mut flat = *direction - normal * along;
        if flat.length_squared() > 1e-18 {
            flat.normalize();
            return Some(flat);
        }
        let mut edge = self.verts[to] - self.verts[from];
        let along_edge = Vector3::dot_product(&edge, &normal);
        edge = edge - normal * along_edge;
        if edge.length_squared() <= 1e-24 {
            return None;
        }
        edge.normalize();
        Some(edge)
    }

    /// Trace one ray from an origin node along a seed direction.
    ///
    /// Terminal arrivals (other nodes, boundary, loops, dead ends) end
    /// the ray; interior points of existing arcs are CROSSED (the
    /// crossed arc splits, the ray continues), so no T-junctions form.
    fn trace_ray(&mut self, origin_node: usize, origin_vertex: usize, seed: Vector3) {
        let Some(first) = self.best_step(origin_vertex, &seed, None) else {
            return;
        };
        let mut direction = match self.transport(&seed, origin_vertex, first.0) {
            Some(d) => d,
            None => return,
        };
        let mut path = vec![origin_vertex];
        let mut path_set: BTreeSet<usize> = BTreeSet::from([origin_vertex]);
        // (vertex, previous-vertex) pairs traversed by this ray.
        let mut visited: BTreeSet<(usize, usize)> = BTreeSet::new();
        // Path indices where this ray crosses an existing arc.
        let mut crossings: Vec<usize> = Vec::new();
        let mut vertex = origin_vertex;
        let mut prev: Option<usize> = None;
        let mut first_step = true;
        let end_node: usize;
        loop {
            let Some((next, dot)) = self.best_step(vertex, &direction, prev) else {
                end_node = self.add_node(vertex, NodeKind::DeadEnd);
                break;
            };
            if !first_step && dot <= DOT_MIN {
                end_node = self.add_node(vertex, NodeKind::DeadEnd);
                break;
            }
            first_step = false;
            // Arrival checks, in order: origin loop, own tail, nodes,
            // arc crossings, boundary.
            if next == origin_vertex && path.len() > 2 {
                path.push(next);
                end_node = origin_node;
                break;
            }
            if visited.contains(&(next, vertex)) || path_set.contains(&next) {
                // Own-tail loop: cut at the first occurrence so the
                // loop span becomes a clean loop arc (node to itself)
                // instead of a lollipop fraction no side can span.
                if let Some(first) = path.iter().position(|&v| v == next) {
                    crossings.push(first);
                }
                path.push(next);
                end_node = self.add_node(next, NodeKind::Loop);
                break;
            }
            if let Some(&node) = self.node_of_vertex.get(&next) {
                path.push(next);
                end_node = node;
                break;
            }
            if self.arc_membership.contains_key(&next) {
                // Crossing: split crossed arcs until `next` terminates
                // every arc through it (endpoints are nodes, already
                // handled above). Each split converts one interior
                // occurrence into endpoints, so the loop terminates.
                let mut crossed_any = false;
                loop {
                    let entries = self.arc_membership.get(&next).cloned().unwrap_or_default();
                    let mut progressed = false;
                    for (arc, index) in entries {
                        if self.split_arc(arc, index, next).is_some() {
                            crossed_any = true;
                            progressed = true;
                            break;
                        }
                    }
                    if !progressed {
                        break;
                    }
                }
                if crossed_any {
                    // The split created the crossing node; record this
                    // path index so the ray splits here too at the end.
                    crossings.push(path.len());
                }
                // Continue through (crossing completion).
            }
            if self.boundary.contains(&next) {
                path.push(next);
                end_node = self.add_node(next, NodeKind::Boundary);
                break;
            }
            visited.insert((next, vertex));
            prev = Some(vertex);
            vertex = next;
            path.push(vertex);
            path_set.insert(vertex);
            match self.transport(&direction, prev.unwrap(), vertex) {
                Some(d) => direction = d,
                None => {
                    end_node = self.add_node(vertex, NodeKind::DeadEnd);
                    break;
                }
            }
            if path.len() > self.max_steps {
                end_node = self.add_node(vertex, NodeKind::DeadEnd);
                break;
            }
        }
        if path.len() < 2 {
            return;
        }
        // Split the ray at its crossings: each span becomes its own arc
        // so every crossing node has valence 4 (two arcs through).
        let mut cuts: Vec<usize> = vec![0];
        cuts.extend(crossings.iter().copied());
        cuts.push(path.len() - 1);
        cuts.sort_unstable();
        cuts.dedup();
        let mut prev_node = origin_node;
        for window in cuts.windows(2) {
            let (start, end) = (window[0], window[1]);
            if end <= start {
                continue;
            }
            let span = path[start..=end].to_vec();
            let end_is_final = end == path.len() - 1;
            let node = if end_is_final {
                end_node
            } else {
                self.add_node(span[span.len() - 1], NodeKind::Crossing)
            };
            if span.len() >= 2 {
                self.register_arc(prev_node, node, span);
            }
            prev_node = node;
        }
    }

    /// Farthest-vertex seed pass for islands whose singularity tracing
    /// produced no arcs (e.g. a torus has no singularities to start
    /// from). Up to three seeds, each emitting four rays.
    fn seed_pass(&mut self) {
        // BFS distances over the vertex graph from all arc vertices.
        let mut seeds: Vec<usize> = Vec::new();
        for _ in 0..3 {
            let mut dist: Vec<i64> = vec![-1; self.topology.vertex_count()];
            let mut queue: Vec<usize> = Vec::new();
            if self.arc_membership.is_empty() && seeds.is_empty() {
                dist[0] = 0;
                queue.push(0);
            } else {
                for &v in self.arc_membership.keys().chain(seeds.iter()) {
                    if dist[v] < 0 {
                        dist[v] = 0;
                        queue.push(v);
                    }
                }
            }
            let mut head = 0;
            while head < queue.len() {
                let v = queue[head];
                head += 1;
                for &u in &self.neighbors[v] {
                    if dist[u] < 0 {
                        dist[u] = dist[v] + 1;
                        queue.push(u);
                    }
                }
            }
            let mut best_vertex = 0;
            let mut best_dist = -1;
            for (v, &d) in dist.iter().enumerate() {
                if d > best_dist {
                    best_dist = d;
                    best_vertex = v;
                }
            }
            if best_dist <= 0 {
                break;
            }
            seeds.push(best_vertex);
            let node = self.add_node(best_vertex, NodeKind::Seed);
            // Four rays along the local cross.
            let corners = self.topology.corners_around_vertex(best_vertex);
            if corners.is_empty() {
                continue;
            }
            let face = self.topology.corner_face(corners[0]);
            let normal = self.topology.face_normal(face);
            let u = self.field[face];
            let v = Vector3::cross_product(&normal, &u);
            for seed in [u, v, u * -1.0, v * -1.0] {
                self.trace_ray(node, best_vertex, seed);
            }
            if !self.graph.arcs.is_empty() {
                break;
            }
        }
    }
}

/// Trace the separatrix graph for one working-mesh island.
pub(crate) fn trace_separatrices(
    topology: &SurfaceMesh,
    verts: &[Vector3],
    field: &[Vector3],
    charges: &[i32],
) -> ArcGraph {
    let mut tracer = Tracer::new(topology, verts, field);
    // Register singularity nodes in vertex order.
    let mut singularities: Vec<usize> = Vec::new();
    for (v, &charge) in charges.iter().enumerate() {
        if charge != 0 {
            singularities.push(v);
            tracer.add_node(v, NodeKind::Singularity);
        }
    }
    for &vertex in &singularities {
        let charge = charges[vertex];
        let count = ray_count_for(topology, verts, field, vertex, charge);
        if count == 0 {
            continue;
        }
        let seeds = seed_directions(
            topology,
            verts,
            field,
            &tracer.vertex_normals.clone(),
            vertex,
            count,
        );
        let origin_node = tracer.node_of_vertex[&vertex];
        for seed in seeds {
            tracer.trace_ray(origin_node, vertex, seed);
        }
    }
    if tracer.graph.arcs.is_empty() {
        tracer.seed_pass();
    }
    tracer.graph
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Closed diamond (octahedron).
    fn diamond() -> (Vec<Vector3>, Vec<Vec<usize>>) {
        let vertices = vec![
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
            Vector3::new(0.0, 0.0, -1.0),
        ];
        let triangles = vec![
            vec![0, 1, 2],
            vec![0, 2, 3],
            vec![0, 3, 4],
            vec![0, 4, 1],
            vec![5, 2, 1],
            vec![5, 3, 2],
            vec![5, 4, 3],
            vec![5, 1, 4],
        ];
        (vertices, triangles)
    }

    /// Flat hexagon fan: center + 6 ring verts, 6 tris (has boundary).
    fn fan() -> (Vec<Vector3>, Vec<Vec<usize>>) {
        let mut vertices = vec![Vector3::new(0.0, 0.0, 0.0)];
        for i in 0..6 {
            let angle = i as f64 * PI / 3.0;
            vertices.push(Vector3::new(angle.cos(), angle.sin(), 0.0));
        }
        let triangles: Vec<Vec<usize>> = (0..6).map(|i| vec![0, 1 + i, 1 + (i + 1) % 6]).collect();
        (vertices, triangles)
    }

    fn uniform_field(topology: &SurfaceMesh) -> Vec<Vector3> {
        // Constant +X field projected onto each face tangent plane.
        let mut field = Vec::new();
        for f in 0..topology.face_count() {
            let normal = topology.face_normal(f);
            let mut d = Vector3::new(1.0, 0.0, 0.0);
            let along = Vector3::dot_product(&d, &normal);
            d = d - normal * along;
            if d.length_squared() < 1e-12 {
                d = Vector3::new(0.0, 1.0, 0.0);
                let along = Vector3::dot_product(&d, &normal);
                d = d - normal * along;
            }
            d.normalize();
            field.push(d);
        }
        field
    }

    #[test]
    fn ring_walk_orders_diamond_apex() {
        let (vertices, triangles) = diamond();
        let topology = SurfaceMesh::new(&vertices, &triangles);
        let (ring, closed) = ordered_ring(&topology, 0);
        assert!(closed);
        assert_eq!(ring.len(), 4);
        // Consecutive corners share an edge through the vertex.
        for w in ring.windows(2) {
            let opposite = topology.opposite_corner(w[0]);
            assert_ne!(opposite, SurfaceMesh::NPOS);
            assert_eq!(topology.next_corner(opposite), w[1]);
        }
    }

    #[test]
    fn singularity_free_fan_gets_seed_arcs() {
        let (vertices, triangles) = fan();
        let topology = SurfaceMesh::new(&vertices, &triangles);
        let field = uniform_field(&topology);
        let charges = vec![0; vertices.len()];
        let graph = trace_separatrices(&topology, &vertices, &field, &charges);
        assert!(
            !graph.arcs.is_empty(),
            "seed pass must emit arcs on a singularity-free island"
        );
        // Every arc path is a valid vertex chain.
        for (arc_id, arc) in graph.arcs.iter().enumerate() {
            assert!(arc.path.len() >= 2);
            assert_eq!(graph.nodes[arc.a].vertex, arc.path[0]);
            assert_eq!(graph.nodes[arc.b].vertex, arc.path[arc.path.len() - 1]);
            assert!(graph.arc_length(arc_id, &vertices) > 0.0);
        }
    }

    #[test]
    fn tracing_is_deterministic() {
        let (vertices, triangles) = fan();
        let topology = SurfaceMesh::new(&vertices, &triangles);
        let field = uniform_field(&topology);
        let charges = vec![0; vertices.len()];
        let first = trace_separatrices(&topology, &vertices, &field, &charges);
        let second = trace_separatrices(&topology, &vertices, &field, &charges);
        assert_eq!(first.arcs.len(), second.arcs.len());
        assert_eq!(first.nodes.len(), second.nodes.len());
        for (a, b) in first.arcs.iter().zip(second.arcs.iter()) {
            assert_eq!(a.path, b.path);
        }
    }
}
