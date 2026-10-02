use super::QuadExtractor;

// ---------------------------------------------------------------------------
// Sorted per-round indexes for the fixpoint passes.
// ---------------------------------------------------------------------------
//
// The collapse/merge/cleanup passes rebuild adjacency maps over all faces
// every round. `BTreeMap` builds malloc per entry (150k+ allocs/round at
// 50k faces); these sorted-vector indexes collect (key, value) pairs in
// face order, sort once, and group — identical keys, values, and order to
// the `BTreeMap` builds (stable sort over face-ordered pairs keeps
// face-index vecs ascending; neighbor pairs dedup to the same sets),
// with a handful of allocations per round. Lookups are binary searches
// with identical hit/miss/value behavior. Equivalence is proved by
// `fixpoint_indexes_match_btree` plus the pipeline bitwise checks.

/// Sorted `(edge, face)` groups: identical keys, face-index vecs
/// (ascending face order), and group order to the per-round
/// `BTreeMap<(usize, usize), Vec<usize>>` build.
pub(crate) struct EdgeFaceIndex {
    edges: Vec<(usize, usize)>,
    starts: Vec<usize>,
    faces: Vec<usize>,
}

impl EdgeFaceIndex {
    pub(crate) fn build(polygons: &[Vec<usize>]) -> Self {
        let mut pairs: Vec<((usize, usize), usize)> = Vec::new();
        for (face_index, face) in polygons.iter().enumerate() {
            for i in 0..face.len() {
                pairs.push((
                    QuadExtractor::edge_of(face[i], face[(i + 1) % face.len()]),
                    face_index,
                ));
            }
        }
        // Stable: pairs start in face order, so each edge's group keeps
        // ascending face order exactly like the serial `push` build.
        pairs.sort_by(|a, b| a.0.cmp(&b.0));
        let mut edges = Vec::new();
        let mut starts = Vec::new();
        let mut faces = Vec::new();
        let mut i = 0;
        while i < pairs.len() {
            let edge = pairs[i].0;
            edges.push(edge);
            starts.push(faces.len());
            while i < pairs.len() && pairs[i].0 == edge {
                faces.push(pairs[i].1);
                i += 1;
            }
        }
        starts.push(faces.len());
        Self {
            edges,
            starts,
            faces,
        }
    }

    /// Faces incident to `edge` in ascending face order (`None` when absent).
    pub(crate) fn get(&self, edge: &(usize, usize)) -> Option<&[usize]> {
        let group = self.edges.binary_search(edge).ok()?;
        Some(&self.faces[self.starts[group]..self.starts[group + 1]])
    }

    /// `(edge, faces)` groups in ascending edge order.
    pub(crate) fn groups(&self) -> impl Iterator<Item = ((usize, usize), &[usize])> + '_ {
        self.edges.iter().enumerate().map(|(group, edge)| {
            (
                *edge,
                &self.faces[self.starts[group]..self.starts[group + 1]],
            )
        })
    }
}

/// Sorted `(vertex, neighbor)` groups, deduped: identical keys and
/// ascending neighbor order to the per-round
/// `BTreeMap<usize, BTreeSet<usize>>` build.
pub(crate) struct NeighborIndex {
    verts: Vec<usize>,
    starts: Vec<usize>,
    neighbors: Vec<usize>,
}

impl NeighborIndex {
    pub(crate) fn build(polygons: &[Vec<usize>]) -> Self {
        let mut pairs: Vec<(usize, usize)> = Vec::new();
        for face in polygons {
            for i in 0..face.len() {
                let j = (i + 1) % face.len();
                pairs.push((face[i], face[j]));
                pairs.push((face[j], face[i]));
            }
        }
        pairs.sort();
        pairs.dedup();
        let mut verts = Vec::new();
        let mut starts = Vec::new();
        let mut neighbors = Vec::new();
        let mut i = 0;
        while i < pairs.len() {
            let vert = pairs[i].0;
            verts.push(vert);
            starts.push(neighbors.len());
            while i < pairs.len() && pairs[i].0 == vert {
                neighbors.push(pairs[i].1);
                i += 1;
            }
        }
        starts.push(neighbors.len());
        Self {
            verts,
            starts,
            neighbors,
        }
    }

    /// Neighbors of `vertex` in ascending order (`None` when absent).
    pub(crate) fn get(&self, vertex: &usize) -> Option<&[usize]> {
        let group = self.verts.binary_search(vertex).ok()?;
        Some(&self.neighbors[self.starts[group]..self.starts[group + 1]])
    }

    /// Neighbors of `vertex`, or an empty slice when absent (every face
    /// vertex is present by construction; the empty case is unreachable
    /// but total, unlike `BTreeMap` indexing).
    pub(crate) fn get_or_empty(&self, vertex: &usize) -> &[usize] {
        self.get(vertex).unwrap_or(&[])
    }
}

/// Sorted `(vertex, face count)` runs: identical keys and counts to the
/// per-round `BTreeMap<usize, usize>` vertex-face-count build.
pub(crate) struct FaceCountIndex {
    verts: Vec<usize>,
    counts: Vec<usize>,
}

impl FaceCountIndex {
    pub(crate) fn build(polygons: &[Vec<usize>]) -> Self {
        let mut verts: Vec<usize> = Vec::new();
        for face in polygons {
            verts.extend(face.iter().copied());
        }
        verts.sort_unstable();
        let mut keys = Vec::new();
        let mut counts = Vec::new();
        let mut i = 0;
        while i < verts.len() {
            let vert = verts[i];
            let mut count = 0;
            while i < verts.len() && verts[i] == vert {
                count += 1;
                i += 1;
            }
            keys.push(vert);
            counts.push(count);
        }
        Self {
            verts: keys,
            counts,
        }
    }

    pub(crate) fn get(&self, vertex: &usize) -> Option<&usize> {
        let slot = self.verts.binary_search(vertex).ok()?;
        Some(&self.counts[slot])
    }
}

/// The isoline a connection was cut from: which uv coordinate is held
/// constant, which integer value it is held at, and which triangle produced
/// the segment.
#[derive(Clone, Copy, Debug)]
pub(crate) struct ConnectionInfo {
    pub(crate) triangle_index: usize,
    pub(crate) coord_index: i32,
    pub(crate) integer: i32,
}
