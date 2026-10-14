use super::QuadExtractor;
use crate::vector3::Vector3;
use std::collections::{BTreeMap, BTreeSet};

impl<'a> QuadExtractor<'a> {
    pub(crate) fn simplify_graph(graph: &mut BTreeMap<usize, BTreeSet<usize>>) {
        loop {
            let mut delay_pairs: BTreeMap<usize, (usize, usize)> = BTreeMap::new();
            // Restructure: the C++ erases vertices while iterating the map.
            // Erasure keeps survivors in order, and rewiring happens after
            // the scan, so scanning a snapshot and erasing as the scan
            // goes visits the same vertices with the same neighbor pairs.
            let mut snapshot = Vec::new();
            for (vertex, neighbors) in graph.iter() {
                if neighbors.len() != 2 {
                    continue;
                }
                let mut it = neighbors.iter();
                // Invariant: two neighbors (checked above); the fallback is
                // unreachable.
                if let (Some(first), Some(second)) = (it.next(), it.next()) {
                    snapshot.push((*vertex, (*first, *second)));
                }
            }
            for (vertex, (first_neighbor, second_neighbor)) in snapshot {
                if delay_pairs.contains_key(&first_neighbor)
                    || delay_pairs.contains_key(&second_neighbor)
                {
                    continue;
                }
                // Each vertex is visited (hence scheduled) at most once per
                // round, so this insert never overwrites.
                delay_pairs.insert(vertex, (first_neighbor, second_neighbor));
                graph.remove(&vertex);
            }
            if delay_pairs.is_empty() {
                break;
            }
            for (vertex, (first, second)) in &delay_pairs {
                if let Some(neighbors) = graph.get_mut(first) {
                    neighbors.remove(vertex);
                    neighbors.insert(*second);
                } else {
                    // The key is absent, so build its set directly.
                    graph.insert(*first, BTreeSet::from([*second]));
                }
                if let Some(neighbors) = graph.get_mut(second) {
                    neighbors.remove(vertex);
                    neighbors.insert(*first);
                } else {
                    graph.insert(*second, BTreeSet::from([*first]));
                }
            }
        }
    }

    pub(crate) fn remove_single_endpoints(
        _cross_points: &mut [Vector3],
        edge_connect_map: &mut BTreeMap<usize, BTreeSet<usize>>,
    ) -> bool {
        let mut removed = false;
        let mut endpoints = Vec::new();
        for (vertex, neighbors) in edge_connect_map.iter() {
            if neighbors.len() != 1 {
                continue;
            }
            endpoints.push(*vertex);
        }
        for endpoint in endpoints {
            let mut loop_index = endpoint;
            while let Some(neighbors) = edge_connect_map.get(&loop_index) {
                if neighbors.len() != 1 {
                    break;
                }
                // Invariant: exactly one neighbor (checked above).
                let Some(neighbor) = neighbors.iter().next().copied() else {
                    break;
                };
                edge_connect_map.remove(&loop_index);
                removed = true;
                if let Some(neighbor_set) = edge_connect_map.get_mut(&neighbor) {
                    neighbor_set.remove(&loop_index);
                } else {
                    break;
                }
                loop_index = neighbor;
            }
        }
        removed
    }

    pub(crate) fn collapse_triangles(
        cross_points: &mut [Vector3],
        edge_connect_map: &mut BTreeMap<usize, BTreeSet<usize>>,
    ) -> bool {
        let mut triangles = BTreeSet::new();
        for (level0, neighbors) in edge_connect_map.iter() {
            let level0 = *level0;
            for level1 in neighbors {
                let Some(find_level2) = edge_connect_map.get(level1) else {
                    continue;
                };
                for level2 in find_level2 {
                    if level0 == *level2 {
                        continue;
                    }
                    let Some(find_level3) = edge_connect_map.get(level2) else {
                        continue;
                    };
                    if !find_level3.contains(&level0) {
                        continue;
                    }
                    let mut sorted = [level0, *level1, *level2];
                    sorted.sort_unstable();
                    triangles.insert((sorted[0], sorted[1], sorted[2]));
                }
            }
        }

        if triangles.is_empty() {
            return false;
        }

        // Collapse one edge per triangle, the shortest one. Merging every
        // connected triangle into a single point instead would drag all of
        // their neighbors onto that point and leave a star of slivers
        // behind it
        let mut collapsed = false;
        for triangle in &triangles {
            let corners = [triangle.0, triangle.1, triangle.2];
            let mut shortest_edge = (0, 0);
            let mut shortest_length = f64::MAX;
            let mut still_a_triangle = true;
            for i in 0..3 {
                let j = (i + 1) % 3;
                match edge_connect_map.get(&corners[i]) {
                    Some(neighbors) if neighbors.contains(&corners[j]) => {}
                    _ => {
                        still_a_triangle = false;
                        break;
                    }
                }
                let length = (cross_points[corners[i]] - cross_points[corners[j]]).length();
                if length < shortest_length {
                    shortest_length = length;
                    shortest_edge = (corners[i], corners[j]);
                }
            }
            // An earlier collapse may have already taken this one apart
            if !still_a_triangle {
                continue;
            }
            Self::collapse_edge(cross_points, edge_connect_map, shortest_edge);
            collapsed = true;
        }

        collapsed
    }

    pub(crate) fn collapse_short_edges(
        cross_points: &mut [Vector3],
        edge_connect_map: &mut BTreeMap<usize, BTreeSet<usize>>,
    ) -> bool {
        let mut total_length = 0.0;
        let mut edge_count = 0;
        let mut edge_lengths = BTreeMap::new();
        for (point, neighbors) in edge_connect_map.iter() {
            for neighbor in neighbors {
                if edge_lengths.contains_key(&(*neighbor, *point)) {
                    continue;
                }
                let edge_length = (cross_points[*point] - cross_points[*neighbor]).length();
                total_length += edge_length;
                edge_lengths.insert((*point, *neighbor), edge_length);
                edge_count += 1;
            }
        }
        if 0 == edge_count {
            return false;
        }
        let average_edge_length = total_length / edge_count as f64;
        let collapsed_length = average_edge_length * 0.01;
        let mut collapsed = false;
        for (edge, length) in &edge_lengths {
            if *length > collapsed_length {
                continue;
            }
            Self::collapse_edge(cross_points, edge_connect_map, *edge);
            collapsed = true;
        }
        collapsed
    }

    fn collapse_edge(
        cross_points: &mut [Vector3],
        edge_connect_map: &mut BTreeMap<usize, BTreeSet<usize>>,
        edge: (usize, usize),
    ) {
        let has_forward = edge_connect_map
            .get(&edge.1)
            .is_some_and(|neighbors| neighbors.contains(&edge.0));
        if !has_forward {
            return;
        }
        let has_backward = edge_connect_map
            .get(&edge.0)
            .is_some_and(|neighbors| neighbors.contains(&edge.1));
        if !has_backward {
            return;
        }
        // Copy of a non-empty set (it holds `edge.1`, checked above), so
        // the copy keeps its order and count on both sides.
        let Some(first_neighbors) = edge_connect_map.get(&edge.0).cloned() else {
            return;
        };
        cross_points[edge.1] = (cross_points[edge.0] + cross_points[edge.1]) * 0.5;
        for neighbor in &first_neighbors {
            if *neighbor == edge.1 {
                continue;
            }
            edge_connect_map
                .entry(edge.1)
                .or_default()
                .insert(*neighbor);
            edge_connect_map
                .entry(*neighbor)
                .or_default()
                .insert(edge.1);
            edge_connect_map
                .entry(*neighbor)
                .or_default()
                .remove(&edge.0);
        }
        edge_connect_map.remove(&edge.0);
        edge_connect_map.entry(edge.1).or_default().remove(&edge.0);
        let second_empty = edge_connect_map
            .get(&edge.1)
            .is_some_and(|set| set.is_empty());
        if second_empty {
            edge_connect_map.remove(&edge.1);
        }
    }

    fn ring_face_normal(points: &[Vector3], corners: &[usize]) -> Vector3 {
        let mut center = Vector3::default();
        for corner in corners {
            center += points[*corner];
        }
        center /= corners.len() as f64;
        let mut normals = Vector3::default();
        for i in 0..corners.len() {
            normals += Vector3::normal(
                &points[corners[i % corners.len()]],
                &points[corners[(i + 1) % corners.len()]],
                &center,
            );
        }
        normals.normalized()
    }

    fn ring_side(
        points: &[Vector3],
        triangle_normals: &BTreeMap<usize, Vector3>,
        corners: &[usize],
    ) -> i32 {
        let ring_normal = Self::ring_face_normal(points, corners);
        let mut original_normal = Vector3::default();
        for it in corners {
            // C++ `operator[]` read: every corner is present (the map is
            // built over all points), and the table is never iterated, so
            // the default-on-miss value below is exactly equivalent.
            original_normal += triangle_normals.get(it).copied().unwrap_or_default();
        }
        let dot = Vector3::dot_product(&ring_normal, &original_normal.normalized());
        const DOT_THRESHOLD: f64 = 0.259; // > 75 or < 105 degrees
        if dot > DOT_THRESHOLD {
            1
        } else if dot < -DOT_THRESHOLD {
            -1
        } else {
            0
        }
    }

    /// One face-emission attempt (mirrors the five copy-pasted
    /// corner/side/halfedge blocks of `extractMesh`, one per face size):
    /// skip used corners, orient by ring side, skip used halfedges, then
    /// record the face, its corners (both windings) and its halfedges.
    pub(crate) fn try_add_face(
        points: &[Vector3],
        triangle_normals: &BTreeMap<usize, Vector3>,
        corners: &mut BTreeSet<(usize, usize, usize)>,
        half_edges: &mut BTreeSet<(usize, usize)>,
        quads: &mut Vec<Vec<usize>>,
        face: &[usize],
    ) {
        if Self::face_corner_exists(corners, face) {
            return;
        }
        let side = Self::ring_side(points, triangle_normals, face);
        if side > 0 {
            if Self::face_half_edge_exists(half_edges, face) {
                return;
            }
            quads.push(face.to_vec());
            Self::add_face_corners(corners, face);
            Self::add_face_half_edges(half_edges, face);
        } else if side < 0 {
            let reversed: Vec<usize> = face.iter().rev().copied().collect();
            if Self::face_half_edge_exists(half_edges, &reversed) {
                return;
            }
            quads.push(reversed.clone());
            Self::add_face_corners(corners, &reversed);
            Self::add_face_half_edges(half_edges, &reversed);
        }
    }

    fn corner_used(
        corners: &BTreeSet<(usize, usize, usize)>,
        previous: usize,
        current: usize,
        next: usize,
    ) -> bool {
        if corners.contains(&(previous, current, next)) {
            return true;
        }
        if corners.contains(&(next, current, previous)) {
            return true;
        }
        false
    }

    fn face_corner_exists(corners: &BTreeSet<(usize, usize, usize)>, vertices: &[usize]) -> bool {
        for i in 0..vertices.len() {
            let j = (i + 1) % vertices.len();
            let k = (i + 2) % vertices.len();
            if Self::corner_used(corners, vertices[i], vertices[j], vertices[k]) {
                return true;
            }
        }
        false
    }

    fn add_face_corners(corners: &mut BTreeSet<(usize, usize, usize)>, vertices: &[usize]) {
        for i in 0..vertices.len() {
            let j = (i + 1) % vertices.len();
            let k = (i + 2) % vertices.len();
            corners.insert((vertices[i], vertices[j], vertices[k]));
            corners.insert((vertices[k], vertices[j], vertices[i]));
        }
    }

    fn face_half_edge_exists(half_edges: &BTreeSet<(usize, usize)>, vertices: &[usize]) -> bool {
        for i in 0..vertices.len() {
            let j = (i + 1) % vertices.len();
            if half_edges.contains(&(vertices[i], vertices[j])) {
                return true;
            }
        }
        false
    }

    fn add_face_half_edges(half_edges: &mut BTreeSet<(usize, usize)>, vertices: &[usize]) {
        for i in 0..vertices.len() {
            let j = (i + 1) % vertices.len();
            half_edges.insert((vertices[i], vertices[j]));
        }
    }
}
