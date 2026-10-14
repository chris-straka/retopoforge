//! Port of `core/meshseparator.*`: orientation-sensitive connected-component
//! split of a face soup into islands.
//!
//! Line-by-line mirror of the C++ implementation: same packed directed-edge
//! table with the same `BTreeMap` fallback, same flood-fill visit order, so
//! island order and face order within islands match the C++ exactly.

use std::collections::{BTreeMap, VecDeque};

/// Largest vertex index that fits in the packed directed-edge key
/// (mirrors `maximumPackableVertexIndex`).
const MAXIMUM_PACKABLE_VERTEX_INDEX: usize = 0xffff_ffff;

/// A directed edge packed into one key (mirrors `packDirectedEdge`).
#[inline]
fn pack_directed_edge(from: usize, to: usize) -> u64 {
    ((from as u64) << 32) | (to as u64)
}

/// Sorted `(packed edge, face)` table (mirrors
/// `buildPackedEdgeToFaceTable`). Returns `false` when a vertex index does
/// not fit in 32 bits, in which case the caller falls back to the tree map.
fn build_packed_edge_to_face_table(faces: &[Vec<usize>], table: &mut Vec<(u64, usize)>) -> bool {
    let mut edge_count = 0;
    for face in faces {
        edge_count += face.len();
    }
    table.clear();
    table.reserve(edge_count);
    for (index, face) in faces.iter().enumerate() {
        for i in 0..face.len() {
            let j = (i + 1) % face.len();
            if face[i] > MAXIMUM_PACKABLE_VERTEX_INDEX || face[j] > MAXIMUM_PACKABLE_VERTEX_INDEX {
                return false;
            }
            table.push((pack_directed_edge(face[i], face[j]), index));
        }
    }
    // Sorting by (edge, face) leaves the highest face index for a repeated
    // directed edge last, which is the face the map form keeps: it assigns
    // in ascending face order and lets the last write win.
    // Deliberate restructure note: the C++ uses tbb::parallel_sort; this is
    // a single-threaded sort (no parallel-sort crate available offline).
    // Same total order on (edge, face), so the same entry wins.
    table.sort();
    true
}

/// Highest face index holding directed edge `(from, to)` (mirrors
/// `findFaceOfDirectedEdge`, whose `SIZE_MAX` sentinel is `None` here).
fn find_face_of_directed_edge(table: &[(u64, usize)], from: usize, to: usize) -> Option<usize> {
    let key = pack_directed_edge(from, to);
    // partition_point mirrors the std::upper_bound with the key<entry
    // comparator: first entry strictly greater than the key.
    let ub = table.partition_point(|entry| entry.0 <= key);
    if ub == 0 {
        return None;
    }
    let (found_key, found_face) = table[ub - 1];
    if found_key != key {
        return None;
    }
    Some(found_face)
}

/// Face-soup island splitter (mirrors `AutoRemesher::MeshSeparator`).
pub struct MeshSeparator;

impl MeshSeparator {
    /// Splits `faces` into orientation-connected islands, appended to
    /// `islands` (mirrors `splitToIslands`, which also appends).
    pub fn split_to_islands(faces: &[Vec<usize>], islands: &mut Vec<Vec<Vec<usize>>>) {
        let mut packed_edge_to_face = Vec::new();
        let mut edge_to_face_map = BTreeMap::new();
        let packed = build_packed_edge_to_face_table(faces, &mut packed_edge_to_face);
        if !packed {
            Self::build_edge_to_face_map(faces, &mut edge_to_face_map);
        }

        let opposite_face_of = |from: usize, to: usize| -> Option<usize> {
            if packed {
                find_face_of_directed_edge(&packed_edge_to_face, from, to)
            } else {
                edge_to_face_map.get(&(from, to)).copied()
            }
        };

        // The flood fill stays serial and visits faces in the same order as
        // before, so the islands and the faces inside them come out unchanged.
        let mut processed_faces = vec![false; faces.len()];
        let mut wait_faces: VecDeque<usize> = VecDeque::new();
        for index_in_group in 0..faces.len() {
            if processed_faces[index_in_group] {
                continue;
            }
            wait_faces.push_back(index_in_group);
            let mut island = Vec::new();
            while let Some(index) = wait_faces.pop_front() {
                if processed_faces[index] {
                    continue;
                }
                let face = &faces[index];
                for i in 0..face.len() {
                    let j = (i + 1) % face.len();
                    let Some(opposite_face) = opposite_face_of(face[j], face[i]) else {
                        continue;
                    };
                    wait_faces.push_back(opposite_face);
                }
                island.push(faces[index].clone());
                processed_faces[index] = true;
            }
            if island.is_empty() {
                continue;
            }
            islands.push(island);
        }
    }

    /// Directed-edge -> owning face map, last write wins (mirrors
    /// `buildEdgeToFaceMap`, including the clear).
    pub fn build_edge_to_face_map(
        faces: &[Vec<usize>],
        edge_to_face_map: &mut BTreeMap<(usize, usize), usize>,
    ) {
        edge_to_face_map.clear();
        for (index, face) in faces.iter().enumerate() {
            for i in 0..face.len() {
                let j = (i + 1) % face.len();
                edge_to_face_map.insert((face[i], face[j]), index);
            }
        }
    }
}
