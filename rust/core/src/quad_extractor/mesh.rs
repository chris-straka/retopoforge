use super::QuadExtractor;
use super::geom::{lerp_vec2, lerp_vec3};
use super::types::ConnectionInfo;
use crate::double_utils::is_zero;
use crate::position_key::PositionKey;
use crate::vector2::Vector2;
use crate::vector3::Vector3;
use std::collections::{BTreeMap, BTreeSet};

impl<'a> QuadExtractor<'a> {
    pub(crate) fn extract_mesh(
        &mut self,
        points: Vec<Vector3>,
        point_source_triangles: Vec<usize>,
        edge_connect_map: BTreeMap<usize, BTreeSet<usize>>,
    ) {
        // Never iterated (insert + reads only).
        let mut triangle_normals: BTreeMap<usize, Vector3> = BTreeMap::new();
        for (point_index, source) in point_source_triangles.iter().enumerate() {
            let triangle_vertices = &self.triangles[*source];
            let triangle_normal = Vector3::normal(
                &self.vertices[triangle_vertices[0]],
                &self.vertices[triangle_vertices[1]],
                &self.vertices[triangle_vertices[2]],
            );
            // Fresh `point_index` keys: this insert never overwrites.
            triangle_normals.insert(point_index, triangle_normal);
        }

        let mut corners = BTreeSet::new();
        let triangle_round = 4;
        for round in 0..5 {
            for (level0, _) in edge_connect_map.iter() {
                let level0 = *level0;
                let Some(find_level1) = edge_connect_map.get(&level0) else {
                    continue;
                };
                let triangle_vertices = &self.triangles[point_source_triangles[level0]];
                // Dead in the C++ too (computed, never read); kept so the
                // indexing side effects stay identical.
                let _triangle_normal = Vector3::normal(
                    &self.vertices[triangle_vertices[0]],
                    &self.vertices[triangle_vertices[1]],
                    &self.vertices[triangle_vertices[2]],
                );
                // Restructure: neighbor levels are copied out level by level
                // (the C++ nests iterators seven deep); the values and the
                // visit order are unchanged.
                let level1s: Vec<usize> = find_level1.iter().copied().collect();
                for level1 in level1s {
                    let Some(find_level2) = edge_connect_map.get(&level1) else {
                        continue;
                    };
                    if self.half_edges.contains(&(level0, level1))
                        && self.half_edges.contains(&(level1, level0))
                    {
                        continue;
                    }
                    let level2s: Vec<usize> = find_level2.iter().copied().collect();
                    for level2 in level2s {
                        if level0 == level2 {
                            continue;
                        }
                        let Some(find_level3) = edge_connect_map.get(&level2) else {
                            continue;
                        };
                        if self.half_edges.contains(&(level1, level2))
                            && self.half_edges.contains(&(level2, level1))
                        {
                            continue;
                        }
                        let level3s: Vec<usize> = find_level3.iter().copied().collect();
                        for level3 in level3s {
                            if level0 == level3 {
                                if triangle_round == round {
                                    Self::try_add_face(
                                        &points,
                                        &triangle_normals,
                                        &mut corners,
                                        &mut self.half_edges,
                                        &mut self.remeshed_polygons,
                                        &[level0, level1, level2],
                                    );
                                    break;
                                }
                            } else if triangle_round == round {
                                break;
                            }
                            if level1 == level3 || level0 == level3 {
                                continue;
                            }
                            let Some(find_level4) = edge_connect_map.get(&level3) else {
                                continue;
                            };
                            if self.half_edges.contains(&(level2, level3))
                                && self.half_edges.contains(&(level3, level2))
                            {
                                continue;
                            }
                            let level4s: Vec<usize> = find_level4.iter().copied().collect();
                            for level4 in level4s {
                                if level0 != level4 {
                                    if level2 == level4 || level1 == level4 {
                                        continue;
                                    }
                                    if round < 1 {
                                        continue;
                                    }
                                    let Some(find_level5) = edge_connect_map.get(&level4) else {
                                        continue;
                                    };
                                    if self.half_edges.contains(&(level3, level4))
                                        && self.half_edges.contains(&(level4, level3))
                                    {
                                        continue;
                                    }
                                    let level5s: Vec<usize> = find_level5.iter().copied().collect();
                                    for level5 in level5s {
                                        if level0 != level5 {
                                            if level3 == level5
                                                || level2 == level5
                                                || level1 == level5
                                            {
                                                continue;
                                            }
                                            if round < 2 {
                                                continue;
                                            }
                                            let Some(find_level6) = edge_connect_map.get(&level5)
                                            else {
                                                continue;
                                            };
                                            if self.half_edges.contains(&(level4, level5))
                                                && self.half_edges.contains(&(level5, level4))
                                            {
                                                continue;
                                            }
                                            let level6s: Vec<usize> =
                                                find_level6.iter().copied().collect();
                                            for level6 in level6s {
                                                if level0 != level6 {
                                                    if level4 == level6
                                                        || level3 == level6
                                                        || level2 == level6
                                                        || level1 == level6
                                                    {
                                                        continue;
                                                    }
                                                    if round < 3 {
                                                        continue;
                                                    }
                                                    let Some(find_level7) =
                                                        edge_connect_map.get(&level6)
                                                    else {
                                                        continue;
                                                    };
                                                    if self.half_edges.contains(&(level5, level6))
                                                        && self
                                                            .half_edges
                                                            .contains(&(level6, level5))
                                                    {
                                                        continue;
                                                    }
                                                    let level7s: Vec<usize> =
                                                        find_level7.iter().copied().collect();
                                                    for level7 in level7s {
                                                        if level0 != level7 {
                                                            continue;
                                                        }
                                                        if 3 != round {
                                                            break;
                                                        }
                                                        Self::try_add_face(
                                                            &points,
                                                            &triangle_normals,
                                                            &mut corners,
                                                            &mut self.half_edges,
                                                            &mut self.remeshed_polygons,
                                                            &[
                                                                level0, level1, level2, level3,
                                                                level4, level5, level6,
                                                            ],
                                                        );
                                                        break;
                                                    }
                                                    continue;
                                                }
                                                if 2 != round {
                                                    break;
                                                }
                                                Self::try_add_face(
                                                    &points,
                                                    &triangle_normals,
                                                    &mut corners,
                                                    &mut self.half_edges,
                                                    &mut self.remeshed_polygons,
                                                    &[
                                                        level0, level1, level2, level3, level4,
                                                        level5,
                                                    ],
                                                );
                                                break;
                                            }
                                            continue;
                                        }
                                        if 1 != round {
                                            break;
                                        }
                                        Self::try_add_face(
                                            &points,
                                            &triangle_normals,
                                            &mut corners,
                                            &mut self.half_edges,
                                            &mut self.remeshed_polygons,
                                            &[level0, level1, level2, level3, level4],
                                        );
                                        break;
                                    }
                                    continue;
                                }
                                if 0 != round {
                                    break;
                                }
                                Self::try_add_face(
                                    &points,
                                    &triangle_normals,
                                    &mut corners,
                                    &mut self.half_edges,
                                    &mut self.remeshed_polygons,
                                    &[level0, level1, level2, level3],
                                );
                                break;
                            }
                        }
                    }
                }
            }
        }

        self.remeshed_vertices = points;
    }

    fn add_cross_point(
        cross_point_map: &mut BTreeMap<PositionKey, usize>,
        cross_points: &mut Vec<Vector3>,
        source_triangles: &mut Vec<usize>,
        position: Vector3,
        triangle_index: usize,
    ) -> usize {
        if let Some(existing) = cross_point_map.get(&PositionKey::from_vector(&position)) {
            return *existing;
        }
        let index = cross_points.len();
        cross_point_map.insert(PositionKey::from_vector(&position), index);
        cross_points.push(position);
        source_triangles.push(triangle_index);
        index
    }

    fn add_connection(
        &mut self,
        connections: &mut BTreeSet<(usize, usize)>,
        from_point_index: usize,
        to_point_index: usize,
        triangle_index: usize,
        coord_index: i32,
        integer: i32,
    ) {
        if from_point_index == to_point_index {
            return;
        }
        connections.insert((from_point_index, to_point_index));
        // `std::map::insert` keeps the FIRST entry on duplicate keys, like
        // `BTreeMap` entry-or-default below (no overwrite).
        self.connection_infos
            .entry((
                from_point_index.min(to_point_index),
                from_point_index.max(to_point_index),
            ))
            .or_insert(ConnectionInfo {
                triangle_index,
                coord_index,
                integer,
            });
    }

    pub(crate) fn extract_connections(
        &mut self,
        cross_points: &mut Vec<Vector3>,
        source_triangles: &mut Vec<usize>,
        connections: &mut BTreeSet<(usize, usize)>,
    ) {
        #[derive(Clone, Copy)]
        struct CrossPoint {
            position3: Vector3,
            position2: Vector2,
            integer: i32,
        }

        let mut cross_point_map = BTreeMap::new();

        self.connection_infos.clear();
        self.added_connections.clear();

        for triangle_index in 0..self.triangles.len() {
            // Copied out (all `Copy`) so the `&mut self` calls below
            // compile; same values the C++ references read.
            let corner_uvs = [
                self.triangle_uvs[triangle_index][0],
                self.triangle_uvs[triangle_index][1],
                self.triangle_uvs[triangle_index][2],
            ];
            let corner_indices = [
                self.triangles[triangle_index][0],
                self.triangles[triangle_index][1],
                self.triangles[triangle_index][2],
            ];

            // Extract intersections of isolines with edges
            let mut lines: [BTreeMap<i32, Vec<Vec<CrossPoint>>>; 2] =
                [BTreeMap::new(), BTreeMap::new()];
            let mut edge_collapsed = [[false; 3]; 2];
            for i in 0..2 {
                for j in 0..3 {
                    let k = (j + 1) % 3;
                    let current = corner_uvs[j];
                    let next = corner_uvs[k];
                    if is_zero((current[i] as i32) as f64 - current[i])
                        && is_zero(current[i] - next[i])
                    {
                        let integer = current[i] as i32;
                        edge_collapsed[i][j] = true;
                        let from_point = CrossPoint {
                            position3: self.vertices[corner_indices[j]],
                            position2: corner_uvs[j],
                            integer,
                        };
                        let to_point = CrossPoint {
                            position3: self.vertices[corner_indices[k]],
                            position2: corner_uvs[k],
                            integer,
                        };
                        lines[i]
                            .entry(integer)
                            .or_default()
                            .push(vec![from_point, to_point]);
                    }
                }
                let mut points: BTreeMap<i32, Vec<CrossPoint>> = BTreeMap::new();
                for j in 0..3 {
                    let k = (j + 1) % 3;
                    let current = corner_uvs[j];
                    let next = corner_uvs[k];
                    let distance = (current[i] - next[i]).abs();
                    if current[i] as i32 != next[i] as i32 || (current[i] > 0.0) != (next[i] > 0.0)
                    {
                        let (
                            low_integer,
                            high_integer,
                            from_position,
                            _to_position,
                            from_index,
                            to_index,
                        );
                        if current[i] < next[i] {
                            low_integer = current[i] as i32;
                            high_integer = next[i] as i32;
                            from_position = current[i];
                            _to_position = next[i];
                            from_index = j;
                            to_index = k;
                        } else {
                            low_integer = next[i] as i32;
                            high_integer = current[i] as i32;
                            from_position = next[i];
                            _to_position = current[i];
                            from_index = k;
                            to_index = j;
                        }
                        for integer in low_integer..=high_integer {
                            let ratio = (integer as f64 - from_position) / distance;
                            if ratio < 0.0 || ratio > 1.0 {
                                continue;
                            }
                            if (is_zero(ratio) || is_zero(ratio - 1.0)) && edge_collapsed[i][j] {
                                continue;
                            }
                            let point = CrossPoint {
                                // Unfused operator lerp (see module FMA note).
                                position3: lerp_vec3(
                                    self.vertices[corner_indices[from_index]],
                                    self.vertices[corner_indices[to_index]],
                                    ratio,
                                ),
                                position2: lerp_vec2(
                                    corner_uvs[from_index],
                                    corner_uvs[to_index],
                                    ratio,
                                ),
                                integer,
                            };
                            points.entry(integer).or_default().push(point);
                        }
                    }
                }
                for (integer, point_list) in &points {
                    for point_index in 0..point_list.len() {
                        let next_point_index = (point_index + 1) % point_list.len();
                        let point = point_list[point_index];
                        let next_point = point_list[next_point_index];
                        lines[i]
                            .entry(*integer)
                            .or_default()
                            .push(vec![point, next_point]);
                    }
                }
            }

            // Segment lines by isolines
            for i in 0..2 {
                let j = (i + 1) % 2;
                for (target_integer, target_list) in &lines[i] {
                    for target in target_list {
                        let mut segments = vec![target.clone()];
                        for split_list in lines[j].values() {
                            let split = &split_list[0];
                            let coord_index = j;
                            let segment_position = split[0].position2[coord_index];
                            for segment_index in (0..segments.len()).rev() {
                                // Read phase: decide the split from a copy
                                // (the C++ mutates the segment and pushes a
                                // new one while holding the reference).
                                let outcome = {
                                    let segment = &segments[segment_index];
                                    let uv0 = segment[0].position2;
                                    let uv1 = segment[1].position2;
                                    let distance = (uv0[coord_index] - uv1[coord_index]).abs();
                                    if is_zero(distance) {
                                        None
                                    } else {
                                        let (from_position, to_position, from_index, to_index);
                                        if uv0[coord_index] < uv1[coord_index] {
                                            from_position = uv0[coord_index];
                                            to_position = uv1[coord_index];
                                            from_index = 0;
                                            to_index = 1;
                                        } else {
                                            from_position = uv1[coord_index];
                                            to_position = uv0[coord_index];
                                            from_index = 1;
                                            to_index = 0;
                                        }
                                        if segment_position < from_position
                                            || segment_position > to_position
                                        {
                                            None
                                        } else {
                                            let ratio =
                                                (segment_position - from_position) / distance;
                                            // Unfused operator lerps (see module FMA note).
                                            let position3 = lerp_vec3(
                                                segment[from_index].position3,
                                                segment[to_index].position3,
                                                ratio,
                                            );
                                            let position2 = lerp_vec2(
                                                segment[from_index].position2,
                                                segment[to_index].position2,
                                                ratio,
                                            );
                                            let integer = segment[to_index].integer;
                                            let new_from_point = CrossPoint {
                                                position3,
                                                position2,
                                                integer,
                                            };
                                            let new_to_point = segment[to_index];
                                            Some((to_index, new_from_point, new_to_point))
                                        }
                                    }
                                };
                                if let Some((to_index, new_from_point, new_to_point)) = outcome {
                                    segments[segment_index][to_index] = new_from_point;
                                    segments.push(vec![new_from_point, new_to_point]);
                                }
                            }
                        }
                        for segment in &segments {
                            // Argument order audit: Clang evaluates the two
                            // addCrossPoint calls left-to-right (verified in
                            // -O3 IR), same as these statements.
                            let from = Self::add_cross_point(
                                &mut cross_point_map,
                                cross_points,
                                source_triangles,
                                segment[0].position3,
                                triangle_index,
                            );
                            let to = Self::add_cross_point(
                                &mut cross_point_map,
                                cross_points,
                                source_triangles,
                                segment[1].position3,
                                triangle_index,
                            );
                            self.add_connection(
                                connections,
                                from,
                                to,
                                triangle_index,
                                i as i32,
                                *target_integer,
                            );
                        }
                    }
                }
            }
        }
    }
}
