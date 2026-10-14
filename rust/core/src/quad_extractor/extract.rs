use super::QuadExtractor;
use crate::progress::ProgressHandler;
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

impl<'a> QuadExtractor<'a> {
    pub fn extract(&mut self) -> bool {
        // The fractions are the measured share of extraction each step
        // costs. The topology cleanup passes at the end are over half of
        // it, so they report individually instead of as one long silent
        // block.
        // Research probe (RETOPO_IGM_DEBUG): integer-grid-map validity
        // census of the uv triangles at extractor entry.
        if std::env::var_os("RETOPO_IGM_DEBUG").is_some() {
            let (mut pos, mut neg, mut degen, mut uv_area) = (0usize, 0usize, 0usize, 0.0f64);
            for t in self.triangle_uvs {
                let (a, b, c) = (&t[0], &t[1], &t[2]);
                let s =
                    0.5 * ((b.x() - a.x()) * (c.y() - a.y()) - (c.x() - a.x()) * (b.y() - a.y()));
                if s.abs() < 1e-6 {
                    degen += 1;
                } else if s > 0.0 {
                    pos += 1;
                } else {
                    neg += 1;
                }
                uv_area += s;
            }
            eprintln!(
                "IGM tris={} pos={pos} neg={neg} degen(<1e-6 cell)={degen} signed_uv_area={uv_area:.1}",
                self.triangle_uvs.len()
            );
        }
        self.report(0.0, "Extracting connections");
        self.diagnose(|| "Extract connections...\n".to_string());
        let mut cross_points = Vec::new();
        let mut cross_point_source_triangles = Vec::new();
        let mut connections = BTreeSet::new();
        self.extract_connections(
            &mut cross_points,
            &mut cross_point_source_triangles,
            &mut connections,
        );
        self.report(0.07, "Holding singular lines");
        self.hold_singular_lines(
            &mut cross_points,
            &mut cross_point_source_triangles,
            &mut connections,
        );
        self.extracted_connections.clear();
        self.extracted_connection_moved.clear();
        self.extracted_connections.reserve(connections.len());
        let mut triangle_moved = Vec::new();
        if let Some(original) = self.original_triangle_uvs
            && original.len() == self.triangle_uvs.len()
        {
            triangle_moved = vec![0u8; self.triangle_uvs.len()];
            for (i, moved) in triangle_moved.iter_mut().enumerate() {
                let before = &original[i];
                let after = &self.triangle_uvs[i];
                for k in 0..3 {
                    if k >= before.len() || k >= after.len() {
                        break;
                    }
                    if before[k].x() != after[k].x() || before[k].y() != after[k].y() {
                        *moved = 1;
                        break;
                    }
                }
            }
        }
        self.extracted_connection_moved.reserve(connections.len());
        for (first, second) in &connections {
            let first = *first;
            let second = *second;
            self.extracted_connections
                .push((cross_points[first], cross_points[second]));
            let edge = (first.min(second), first.max(second));
            if self.added_connections.contains(&edge) {
                self.extracted_connection_moved.push(2);
            } else if !triangle_moved.is_empty() {
                let first_triangle = cross_point_source_triangles[first];
                let second_triangle = cross_point_source_triangles[second];
                self.extracted_connection_moved.push(u8::from(
                    triangle_moved[first_triangle] != 0 || triangle_moved[second_triangle] != 0,
                ));
            } else {
                self.extracted_connection_moved.push(0);
            }
        }
        self.diagnose(|| "Extract connections done\n".to_string());

        self.report(0.21, "Extracting edges");
        self.diagnose(|| "Extract edges...\n".to_string());
        let mut edge_connect_map: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
        Self::extract_edges(&connections, &mut edge_connect_map);
        if Self::collapse_short_edges(&mut cross_points, &mut edge_connect_map) {
            Self::simplify_graph(&mut edge_connect_map);
        }
        Self::collapse_triangles(&mut cross_points, &mut edge_connect_map);
        if Self::remove_single_endpoints(&mut cross_points, &mut edge_connect_map) {
            Self::simplify_graph(&mut edge_connect_map);
        }
        self.diagnose(|| "Extract edges done\n".to_string());

        self.report(0.25, "Extracting mesh");
        self.diagnose(|| "Extract mesh...\n".to_string());
        self.extract_mesh(cross_points, cross_point_source_triangles, edge_connect_map);
        self.diagnose(|| "Extract mesh done\n".to_string());

        self.report(0.29, "Fixing holes");
        self.fix_holes();

        self.report(0.30, "Removing non-manifold faces");
        let mut changed = false;
        if self.remove_isolated_faces() {
            changed = true;
        }
        while self.remove_non_manifold_faces() {
            changed = true;
            self.remove_isolated_faces();
        }

        if changed {
            self.rebuild_half_edges();
            self.fix_holes();
        }

        {
            let mut used_vertices = BTreeSet::new();
            for face in &self.remeshed_polygons {
                for v in face {
                    used_vertices.insert(*v);
                }
            }
            if used_vertices.len() < self.remeshed_vertices.len() {
                let mut compacted_vertices = Vec::with_capacity(used_vertices.len());
                let mut old_to_new = BTreeMap::new();
                for old_index in &used_vertices {
                    old_to_new.insert(*old_index, compacted_vertices.len());
                    compacted_vertices.push(self.remeshed_vertices[*old_index]);
                }
                for face in &mut self.remeshed_polygons {
                    for v in face {
                        *v = old_to_new[v];
                    }
                }
                self.remeshed_vertices = compacted_vertices;
            }
        }

        self.report(0.31, "Smoothing and projecting");
        self.diagnose(|| "Smooth and project...\n".to_string());
        self.smooth_and_project(5, None);
        self.diagnose(|| "Smooth and project done\n".to_string());

        self.report(0.44, "Splitting seven edge faces");
        self.split_seven_edge_faces();
        self.report(0.45, "Splitting six edge faces");
        self.split_six_edge_faces();
        // A pentagon is the best place for a triangle to end up, it comes
        // out of the collapse as a quad, so the triangles run first and the
        // merge takes care of whatever pentagons are left over
        self.report(0.46, "Cleaning up triangles");
        self.cleanup_triangles();
        self.report(0.53, "Merging shared five edge faces");
        // Restructure: the C++ builds a remapping closure borrowing the
        // outer handler while calling a `&mut self` method; the mirror
        // parks the outer handler in an `Arc` (still `Send + Sync`, still
        // the same handler object), calls through a shared clone, then
        // restores it. Same fractions reach the same handler in the same
        // order.
        let outer_progress = self.progress_handler.take();
        if let Some(outer) = outer_progress {
            let shared: Arc<ProgressHandler> = Arc::new(outer);
            let inner = Arc::clone(&shared);
            // FMA: `0.53 + (0.85 - 0.53) * fraction` is `fmaf` in f32.
            let remapped: ProgressHandler = Box::new(move |fraction, name| {
                inner((0.85f32 - 0.53f32).mul_add(fraction, 0.53f32), name);
            });
            self.merge_shared_five_edge_faces(Some(&remapped));
            drop(remapped);
            // `remapped` held the only other clone, so the `Arc` is uniquely
            // owned again; restore the exact same handler object.
            if let Ok(outer) = Arc::try_unwrap(shared) {
                self.progress_handler = Some(outer);
            }
        } else {
            self.merge_shared_five_edge_faces(None);
        }
        // Runs last, it only reconnects quad pairs, so it wants the
        // triangles and pentagons to have become quads already
        self.report(0.85, "Switching high valence edges");
        self.switch_high_valence_edges();
        self.report(0.89, "Converting triangle and five edge fans");
        self.convert_triangle_and_five_edge_fans();
        self.report(0.93, "Collapsing three valence diagonals");
        self.collapse_three_valence_diagonals();
        self.report(0.95, "Merging double shared edge quads");
        self.merge_double_shared_edge_quads();
        self.report(0.96, "Merging three and five valence triangles");
        self.merge_three_and_five_valence_triangles();
        self.report(0.97, "Collapsing three valence corners");
        self.collapse_three_valence_corners();
        self.report(0.98, "Splitting high valence triangle fans");
        self.split_high_valence_triangle_fans();
        self.report(0.99, "Collapsing three valence edge pairs");
        self.collapse_three_valence_edge_pairs();
        // No progress event (the engine oracle pins the sequence): a fast
        // tail sweep, silent unless defects were actually removed.
        self.cleanup_residual_routes();
        self.report(1.0, "");

        // Pure post-pass over the final positions: reads m_remeshedVertices,
        // never writes it, so geometry is identical with the flag on or off.
        if self.compute_vertex_uvs {
            self.compute_remeshed_vertex_uvs();
        }

        true
    }

    fn extract_edges(
        connections: &BTreeSet<(usize, usize)>,
        edge_connect_map: &mut BTreeMap<usize, BTreeSet<usize>>,
    ) {
        for (first, second) in connections {
            edge_connect_map.entry(*first).or_default().insert(*second);
            edge_connect_map.entry(*second).or_default().insert(*first);
        }
        Self::simplify_graph(edge_connect_map);
    }
}
