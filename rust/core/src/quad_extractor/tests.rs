use super::types::{EdgeFaceIndex, FaceCountIndex, NeighborIndex};
use super::*;

/// The sorted per-round indexes reproduce the `BTreeMap` builds
/// exactly: identical keys, values, group order, and lookup behavior
/// (hits, misses, and face-list order) over polygon soups including
/// degenerate faces.
#[test]
fn fixpoint_indexes_match_btree() {
    let cases: Vec<Vec<Vec<usize>>> = vec![
        vec![],
        vec![vec![0, 1, 2]],
        vec![vec![0, 1, 2, 3], vec![3, 2, 4], vec![4, 5, 6, 7, 8]],
        // Shared edges in both windings, repeated vertices, a
        // two-sided sliver, and an unused vertex id (9) for misses.
        vec![
            vec![0, 1, 2, 3],
            vec![3, 2, 1, 0],
            vec![2, 3, 4],
            vec![4, 4, 5],
            vec![6, 7],
            vec![7, 6, 7],
        ],
        // A grid-ish strip (every interior edge shared by two faces).
        (0..40)
            .map(|r| (0..4).map(|c| r * 4 + c).collect::<Vec<_>>())
            .collect(),
    ];
    for (case_index, polygons) in cases.iter().enumerate() {
        let context = format!("case {case_index}");
        // Reference: the exact serial `BTreeMap` builds the passes used.
        let mut edge_faces: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
        let mut vertex_neighbors: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
        let mut vertex_face_counts: BTreeMap<usize, usize> = BTreeMap::new();
        for (face_index, face) in polygons.iter().enumerate() {
            for i in 0..face.len() {
                let j = (i + 1) % face.len();
                edge_faces
                    .entry(QuadExtractor::edge_of(face[i], face[j]))
                    .or_default()
                    .push(face_index);
                vertex_neighbors.entry(face[i]).or_default().insert(face[j]);
                vertex_neighbors.entry(face[j]).or_default().insert(face[i]);
            }
            for vertex in face {
                *vertex_face_counts.entry(*vertex).or_default() += 1;
            }
        }
        // Edge groups: same keys, same ascending face vecs, same order.
        let edge_index = EdgeFaceIndex::build(polygons);
        let expected_groups: Vec<((usize, usize), Vec<usize>)> = edge_faces.into_iter().collect();
        let index_groups: Vec<((usize, usize), Vec<usize>)> = edge_index
            .groups()
            .map(|(edge, faces)| (edge, faces.to_vec()))
            .collect();
        assert_eq!(index_groups, expected_groups, "groups {context}");
        // Edge lookups: same hits (order included) and misses.
        let mut probe_edges: Vec<(usize, usize)> =
            index_groups.iter().map(|(edge, _)| *edge).collect();
        probe_edges.extend([(0, 9), (9, 9), (100, 200)]);
        for edge in &probe_edges {
            let expected = expected_groups
                .iter()
                .find(|(key, _)| key == edge)
                .map(|(_, faces)| faces.as_slice());
            assert_eq!(edge_index.get(edge), expected, "edge {edge:?} {context}");
        }
        // Neighbor lookups: same sets (ascending) and misses.
        let neighbor_index = NeighborIndex::build(polygons);
        for vertex in 0..12 {
            let expected: Option<Vec<usize>> = vertex_neighbors
                .get(&vertex)
                .map(|set| set.iter().copied().collect());
            assert_eq!(
                neighbor_index.get(&vertex).map(|s| s.to_vec()),
                expected,
                "neighbors {vertex} {context}"
            );
            assert_eq!(
                neighbor_index.get_or_empty(&vertex),
                expected.as_deref().unwrap_or(&[]),
                "neighbors-or-empty {vertex} {context}"
            );
        }
        // Face counts: same keys, counts, and misses.
        let count_index = FaceCountIndex::build(polygons);
        for vertex in 0..12 {
            assert_eq!(
                count_index.get(&vertex).copied(),
                vertex_face_counts.get(&vertex).copied(),
                "counts {vertex} {context}"
            );
        }
    }
}
