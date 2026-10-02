//! Patch partition for the patch back end.
//!
//! Flood-fills the working-mesh faces through the dual graph with the
//! traced arcs as walls, then reads each patch's boundary loop as an
//! ordered side list over shared arcs. Every side is used by all
//! adjacent patches with one grid ([`crate::patch_backend::fill`]), so
//! shared sides weld into manifold edges by construction.
//!
//! Patches the structured fill cannot take (closed, holed, or
//! corner-inconsistent, all rare) are flagged for the per-face
//! fallback, which keeps full coverage by construction.

use crate::patch_backend::trace::ArcGraph;
use crate::patch_backend::trace::NodeKind;
use crate::surface_mesh::SurfaceMesh;
use std::collections::BTreeMap;

/// One patch side: an arc traversed forward (a -> b) or backward.
#[derive(Clone, Copy, Debug)]
pub(crate) struct PatchSide {
    pub arc: usize,
    pub forward: bool,
}

/// One patch: a face set plus its ordered sides.
#[derive(Clone, Debug)]
pub(crate) struct Patch {
    pub faces: Vec<usize>,
    pub sides: Vec<PatchSide>,
    /// False for closed/holed/inconsistent patches, which take the
    /// per-face fallback instead of the structured fill.
    pub usable: bool,
}

/// Partition result. `graph` extends the traced graph with boundary
/// arcs (closed meshes gain none), so side arc ids index it directly.
pub(crate) struct Layout {
    pub patches: Vec<Patch>,
    pub graph: ArcGraph,
}

/// Undirected mesh edge key.
fn edge_key(a: usize, b: usize) -> (usize, usize) {
    if a < b { (a, b) } else { (b, a) }
}

/// Build the patch layout for one working-mesh island.
pub(crate) fn build_layout(topology: &SurfaceMesh, traced: &ArcGraph) -> Layout {
    // Edge -> faces.
    let mut edge_faces: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
    for f in 0..topology.face_count() {
        let tri = topology.triangle(f);
        for i in 0..3 {
            edge_faces
                .entry(edge_key(tri[i], tri[(i + 1) % 3]))
                .or_default()
                .push(f);
        }
    }
    let mut graph = traced.clone();
    // Dual-edge walls: (min-face, max-face) -> blocking arc (lowest id
    // when two arcs share an edge; the other span is then unreferenced
    // and ignored, which keeps side ownership unambiguous).
    let mut blocked: BTreeMap<(usize, usize), usize> = BTreeMap::new();
    for (arc_id, arc) in graph.arcs.iter().enumerate() {
        for window in arc.path.windows(2) {
            let faces = edge_faces.get(&edge_key(window[0], window[1]));
            let Some(faces) = faces else { continue };
            if faces.len() == 2 {
                let key = (faces[0].min(faces[1]), faces[0].max(faces[1]));
                blocked.entry(key).or_insert(arc_id);
            }
        }
    }
    // Flood fill through unblocked duals; patch ids in face order.
    let face_count = topology.face_count();
    let mut patch_of_face: Vec<usize> = vec![usize::MAX; face_count];
    let mut patch_faces: Vec<Vec<usize>> = Vec::new();
    // Face -> (neighbor face, shared edge key) adjacency.
    let mut dual: Vec<Vec<(usize, (usize, usize))>> = vec![Vec::new(); face_count];
    for faces in edge_faces.values() {
        if faces.len() == 2 {
            let (f, g) = (faces[0], faces[1]);
            let key = (f.min(g), f.max(g));
            dual[f].push((g, key));
            dual[g].push((f, key));
        }
    }
    for f in 0..face_count {
        if patch_of_face[f] != usize::MAX {
            continue;
        }
        let id = patch_faces.len();
        let mut stack = vec![f];
        patch_of_face[f] = id;
        let mut faces = Vec::new();
        while let Some(current) = stack.pop() {
            faces.push(current);
            for &(next, key) in &dual[current] {
                if blocked.contains_key(&key) || patch_of_face[next] != usize::MAX {
                    continue;
                }
                patch_of_face[next] = id;
                stack.push(next);
            }
        }
        faces.sort_unstable();
        patch_faces.push(faces);
    }
    // Boundary loops and sides per patch.
    let mut patches: Vec<Patch> = Vec::new();
    for faces in &patch_faces {
        patches.push(patch_from_faces(
            topology,
            faces,
            &patch_of_face,
            &blocked,
            &mut graph,
        ));
    }
    Layout { patches, graph }
}

/// Read one patch's boundary loop as an ordered side list.
///
/// Appends boundary arcs to `graph` as needed. Returns
/// `usable = false` (faces kept for the fallback) when the patch is
/// closed, holed, or corner-inconsistent.
fn patch_from_faces(
    topology: &SurfaceMesh,
    faces: &[usize],
    patch_of_face: &[usize],
    blocked: &BTreeMap<(usize, usize), usize>,
    graph: &mut ArcGraph,
) -> Patch {
    let patch_id = patch_of_face[faces[0]];
    // Boundary corners: face in patch, opposite outside or missing.
    let mut boundary_corners: Vec<usize> = Vec::new();
    for &face in faces {
        for local in 0..3 {
            let corner = 3 * face + local;
            let opposite = topology.opposite_corner(corner);
            if opposite == SurfaceMesh::NPOS {
                boundary_corners.push(corner);
            } else if patch_of_face[topology.corner_face(opposite)] != patch_id {
                boundary_corners.push(corner);
            }
        }
    }
    if boundary_corners.is_empty() {
        // Closed patch (no arcs around it): fallback covers it.
        if std::env::var_os("RETOPO_PATCH_DEBUG").is_some() {
            eprintln!("unusable: faces={} reason=closed", faces.len());
        }
        return Patch {
            faces: faces.to_vec(),
            sides: Vec::new(),
            usable: false,
        };
    }
    // Link corners into loops: each loop edge ends where the next starts.
    let mut by_start: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for &corner in &boundary_corners {
        by_start
            .entry(topology.corner_vertex(corner))
            .or_default()
            .push(corner);
    }
    for corners in by_start.values_mut() {
        corners.sort_unstable();
    }
    let mut remaining: BTreeMap<usize, usize> = BTreeMap::new();
    for &corner in &boundary_corners {
        remaining.insert(corner, corner);
    }
    let mut loops: Vec<Vec<usize>> = Vec::new();
    let mut simple = true;
    while let Some(&first) = remaining.keys().next() {
        let mut current = first;
        let mut walk: Vec<usize> = Vec::new();
        loop {
            remaining.remove(&current);
            walk.push(current);
            let end = topology.corner_vertex(topology.next_corner(current));
            let Some(candidates) = by_start.get(&end) else {
                simple = false;
                break;
            };
            let mut next: Option<usize> = None;
            for &candidate in candidates {
                if remaining.contains_key(&candidate) {
                    if next.is_some() {
                        // Non-manifold boundary pinch: ambiguous link.
                        simple = false;
                    } else {
                        next = Some(candidate);
                    }
                }
            }
            match next {
                Some(n) => {
                    current = n;
                    if walk.len() > boundary_corners.len() + 1 {
                        simple = false;
                        break;
                    }
                }
                None => break,
            }
        }
        // A loop must close back at its start vertex.
        let closed = topology.corner_vertex(topology.next_corner(*walk.last().unwrap()))
            == topology.corner_vertex(first);
        if !closed {
            simple = false;
        }
        loops.push(walk);
    }
    if !simple || loops.len() != 1 {
        // Pinched boundary or holes: fallback covers it.
        if std::env::var_os("RETOPO_PATCH_DEBUG").is_some() {
            eprintln!(
                "unusable: faces={} reason={} loops={}",
                faces.len(),
                if simple { "holed" } else { "pinched" },
                loops.len()
            );
        }
        return Patch {
            faces: faces.to_vec(),
            sides: Vec::new(),
            usable: false,
        };
    }
    let raw_boundary = &loops[0];
    // Map each loop edge to its blocking arc (or boundary run).
    #[derive(Clone, Copy, PartialEq, Eq)]
    enum EdgeOwner {
        Arc(usize),
        OpenBoundary,
    }
    let mut owners: Vec<EdgeOwner> = Vec::with_capacity(raw_boundary.len());
    for &corner in raw_boundary {
        let face = topology.corner_face(corner);
        let opposite = topology.opposite_corner(corner);
        if opposite == SurfaceMesh::NPOS {
            owners.push(EdgeOwner::OpenBoundary);
            continue;
        }
        let other = topology.corner_face(opposite);
        let key = (face.min(other), face.max(other));
        match blocked.get(&key) {
            Some(&arc) => owners.push(EdgeOwner::Arc(arc)),
            // Unblocked dual between different patches: cannot happen
            // (the flood would have joined them); treat as inconsistent.
            None => {
                if std::env::var_os("RETOPO_PATCH_DEBUG").is_some() {
                    eprintln!("unusable: faces={} reason=unblocked-dual", faces.len());
                }
                return Patch {
                    faces: faces.to_vec(),
                    sides: Vec::new(),
                    usable: false,
                };
            }
        }
    }
    // Rotate the loop to a run boundary: the walk starts at an
    // arbitrary corner, and linear compression would otherwise split
    // a run that wraps around the loop start (every one of beast's
    // 251 run-not-endpoint failures was this). Single-owner loops
    // rotate to the arc's endpoint vertex instead.
    let mut boundary = raw_boundary.clone();
    if owners.iter().all(|&o| o == owners[0]) {
        if let EdgeOwner::Arc(arc) = owners[0] {
            let endpoint = graph.arcs[arc].path[0];
            if let Some(pos) = boundary
                .iter()
                .position(|&c| topology.corner_vertex(c) == endpoint)
            {
                boundary.rotate_left(pos);
                owners.rotate_left(pos);
            } else {
                if std::env::var_os("RETOPO_PATCH_DEBUG").is_some() {
                    eprintln!(
                        "unusable: faces={} reason=loop-misses-endpoint",
                        faces.len()
                    );
                }
                return Patch {
                    faces: faces.to_vec(),
                    sides: Vec::new(),
                    usable: false,
                };
            }
        }
    } else if let Some(pos) = owners.iter().position(|&o| o != owners[0]) {
        boundary.rotate_left(pos);
        owners.rotate_left(pos);
    }
    // Compress runs; boundary runs become boundary arcs.
    let mut sides: Vec<PatchSide> = Vec::new();
    let mut run_start = 0;
    while run_start < boundary.len() {
        let mut run_end = run_start + 1;
        while run_end < boundary.len() && owners[run_end] == owners[run_start] {
            run_end += 1;
        }
        match owners[run_start] {
            EdgeOwner::Arc(arc) => {
                let start_vertex = topology.corner_vertex(boundary[run_start]);
                let forward = if graph.arcs[arc].path[0] == start_vertex {
                    true
                } else if graph.arcs[arc].path[graph.arcs[arc].path.len() - 1] == start_vertex {
                    false
                } else {
                    // Run end is not an arc endpoint: inconsistent.
                    if std::env::var_os("RETOPO_PATCH_DEBUG").is_some() {
                        eprintln!(
                            "unusable: faces={} reason=run-not-endpoint run_start={run_start} n_runs_boundary={} wrap_same={} arclen={}",
                            faces.len(),
                            boundary.len(),
                            owners[0] == owners[boundary.len() - 1],
                            graph.arcs[arc].path.len()
                        );
                    }
                    return Patch {
                        faces: faces.to_vec(),
                        sides: Vec::new(),
                        usable: false,
                    };
                };
                // The run must span the whole arc (region changes along
                // an arc happen only at nodes, which split arcs).
                let mut chain: Vec<usize> = Vec::with_capacity(run_end - run_start + 1);
                chain.push(start_vertex);
                for corner in &boundary[run_start..run_end] {
                    chain.push(topology.corner_vertex(topology.next_corner(*corner)));
                }
                let mut expected = graph.arcs[arc].path.clone();
                if !forward {
                    expected.reverse();
                }
                if chain != expected {
                    if std::env::var_os("RETOPO_PATCH_DEBUG").is_some() {
                        eprintln!("unusable: faces={} reason=run-not-full-arc", faces.len());
                    }
                    return Patch {
                        faces: faces.to_vec(),
                        sides: Vec::new(),
                        usable: false,
                    };
                }
                sides.push(PatchSide { arc, forward });
            }
            EdgeOwner::OpenBoundary => {
                let mut chain: Vec<usize> = Vec::with_capacity(run_end - run_start + 1);
                chain.push(topology.corner_vertex(boundary[run_start]));
                for corner in &boundary[run_start..run_end] {
                    chain.push(topology.corner_vertex(topology.next_corner(*corner)));
                }
                let a = add_boundary_node(graph, chain[0]);
                let b = add_boundary_node(graph, chain[chain.len() - 1]);
                let arc = graph.arcs.len();
                graph
                    .arcs
                    .push(crate::patch_backend::trace::TraceArc { a, b, path: chain });
                sides.push(PatchSide { arc, forward: true });
            }
        }
        run_start = run_end;
    }
    if sides.is_empty() {
        return Patch {
            faces: faces.to_vec(),
            sides: Vec::new(),
            usable: false,
        };
    }
    // Consecutive sides must share their corner node.
    for i in 0..sides.len() {
        let current = sides[i];
        let next = sides[(i + 1) % sides.len()];
        let exit = if current.forward {
            graph.arcs[current.arc].b
        } else {
            graph.arcs[current.arc].a
        };
        let entry = if next.forward {
            graph.arcs[next.arc].a
        } else {
            graph.arcs[next.arc].b
        };
        if exit != entry {
            if std::env::var_os("RETOPO_PATCH_DEBUG").is_some() {
                eprintln!("unusable: faces={} reason=corner-mismatch", faces.len());
            }
            return Patch {
                faces: faces.to_vec(),
                sides: Vec::new(),
                usable: false,
            };
        }
    }
    Patch {
        faces: faces.to_vec(),
        sides,
        usable: true,
    }
}

/// Fetch or create a boundary node for a vertex.
fn add_boundary_node(graph: &mut ArcGraph, vertex: usize) -> usize {
    for (id, node) in graph.nodes.iter().enumerate() {
        if node.vertex == vertex {
            return id;
        }
    }
    let id = graph.nodes.len();
    graph.nodes.push(crate::patch_backend::trace::TraceNode {
        vertex,
        kind: NodeKind::Boundary,
    });
    id
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::patch_backend::trace::TraceArc;
    use crate::patch_backend::trace::TraceNode;
    use crate::vector3::Vector3;

    /// Flat hexagon fan (center 0, ring 1..6).
    fn fan() -> (Vec<Vector3>, Vec<Vec<usize>>) {
        use std::f64::consts::PI;
        let mut vertices = vec![Vector3::new(0.0, 0.0, 0.0)];
        for i in 0..6 {
            let angle = i as f64 * PI / 3.0;
            vertices.push(Vector3::new(angle.cos(), angle.sin(), 0.0));
        }
        let triangles: Vec<Vec<usize>> = (0..6).map(|i| vec![0, 1 + i, 1 + (i + 1) % 6]).collect();
        (vertices, triangles)
    }

    #[test]
    fn empty_graph_on_open_fan_yields_usable_monogon() {
        let (vertices, triangles) = fan();
        let topology = SurfaceMesh::new(&vertices, &triangles);
        let graph = ArcGraph::default();
        let layout = build_layout(&topology, &graph);
        assert_eq!(layout.patches.len(), 1);
        assert!(layout.patches[0].usable);
        assert_eq!(layout.patches[0].sides.len(), 1);
        assert_eq!(layout.patches[0].faces.len(), triangles.len());
    }

    #[test]
    fn empty_graph_on_closed_diamond_yields_unusable_patch() {
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
        let topology = SurfaceMesh::new(&vertices, &triangles);
        let graph = ArcGraph::default();
        let layout = build_layout(&topology, &graph);
        assert_eq!(layout.patches.len(), 1);
        assert!(!layout.patches[0].usable);
        assert_eq!(layout.patches[0].faces.len(), triangles.len());
    }

    #[test]
    fn diameter_arc_splits_fan_into_two_patches() {
        let (vertices, triangles) = fan();
        let topology = SurfaceMesh::new(&vertices, &triangles);
        // Arc along vertices 1 -> 0 -> 4 (a diameter): walls across the
        // duals it crosses split the fan into two 3-face patches.
        let graph = ArcGraph {
            nodes: vec![
                TraceNode {
                    vertex: 1,
                    kind: NodeKind::Boundary,
                },
                TraceNode {
                    vertex: 0,
                    kind: NodeKind::Crossing,
                },
                TraceNode {
                    vertex: 4,
                    kind: NodeKind::Boundary,
                },
            ],
            arcs: vec![
                TraceArc {
                    a: 0,
                    b: 1,
                    path: vec![1, 0],
                },
                TraceArc {
                    a: 1,
                    b: 2,
                    path: vec![0, 4],
                },
            ],
        };
        let layout = build_layout(&topology, &graph);
        assert_eq!(layout.patches.len(), 2);
        for patch in &layout.patches {
            assert_eq!(patch.faces.len(), 3);
        }
        // Both patches usable: two diameter spans + boundary run each.
        assert!(layout.patches.iter().all(|p| p.usable));
        for patch in &layout.patches {
            assert_eq!(patch.sides.len(), 3, "triangle: {patch:?}");
        }
    }
}
