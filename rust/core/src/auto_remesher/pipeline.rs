use super::AutoRemesher;
use super::atlas::pack_island_uvs_into_atlas;
use super::island::{AttemptOutputs, IslandContext, ParameterizationThread};
use super::{CoverageReport, DecimationStats};
use super::{DECIMATE_TRIGGER_RATIO, PARALLEL_PHASE_BEGIN, PARALLEL_PHASE_END};
use crate::density::Density;
use crate::mesh_separator::MeshSeparator;
use crate::par::{parallel_each, worker_chunk_len};
use crate::parameterizer::Parameterizer;
use crate::quad_extractor::QuadExtractor;
use crate::symmetry::{Symmetry, SymmetryPlane};
use crate::vector2::Vector2;
use crate::vector3::Vector3;
use std::collections::BTreeMap;
use std::sync::atomic::{AtomicI64, Ordering};
use std::thread;
use std::time::Instant;

// How an island's own 0..1 progress splits across its three stages, from
// the measured cost of each on a typical model. The phase report prints
// the real accumulated times, so these can be re-checked against a run.
const ISLAND_RESAMPLE_END: f32 = 0.17;
const ISLAND_PARAMETERIZE_END: f32 = 0.50;
// Quad extraction runs from islandParameterizeEnd to 1.0.

// Minimum input support for the symmetry plane: below this fraction of
// mirrored vertices the run falls back to unconstrained output rather
// than snapping an asymmetric mesh onto a plane it does not have.
const MIN_SYMMETRY_SCORE: f64 = 0.75;

impl AutoRemesher {
    /// Merges per-island vertex/triangle vectors into one mesh, offsetting
    /// triangle corners by each island's vertex base (the C++ `mergeIslands`
    /// lambda, which captures nothing).
    fn merge_islands(
        island_vertices: &[Vec<Vector3>],
        island_triangles: &[Vec<Vec<usize>>],
        merged_vertices: &mut Vec<Vector3>,
        merged_triangles: &mut Vec<Vec<usize>>,
    ) {
        for i in 0..island_vertices.len() {
            let vertex_offset = merged_vertices.len();
            merged_vertices.extend(island_vertices[i].iter().cloned());
            for triangle in &island_triangles[i] {
                let mut offset_triangle = Vec::with_capacity(triangle.len());
                for &index in triangle {
                    offset_triangle.push(index + vertex_offset);
                }
                merged_triangles.push(offset_triangle);
            }
        }
    }

    /// Formats a microsecond count like the C++ phase-report `milliseconds`
    /// closure (one decimal place: whole milliseconds hide the per-island
    /// steps on a mesh split into many small islands).
    fn format_ms(microseconds: i64) -> String {
        format!("{:.1} ms", microseconds as f64 / 1000.0)
    }

    /// Pushes one `"<name>: <ms>"` line onto the phase report (the C++
    /// `phase` lambda).
    fn push_phase_line(&mut self, name: &str, microseconds: i64) {
        self.phase_report
            .push(format!("{name}: {}", Self::format_ms(microseconds)));
    }

    /// Mirrors `remesh`: validates the input, resolves the symmetry plane,
    /// sizes voxels, splits into islands, resamples / parameterizes /
    /// extracts each island on worker threads, merges the outputs (plus
    /// the optional UV atlas and symmetry snap), and builds the phase
    /// report. Returns `false` (with no outputs populated) when the input
    /// is rejected.
    pub fn remesh(&mut self) -> bool {
        self.island_output_quad_counts.clear();
        self.coverage_reports.clear();
        self.island_dipole_counts.clear();
        // Validate inputs before any sizing math. In particular a zero
        // target triangle count would divide by zero in
        // initializeVoxelSize().
        let mut invalid_input_reason: Option<&str> = None;
        if self.vertices.is_empty() {
            invalid_input_reason = Some("input mesh has no vertices");
        } else if self.triangles.is_empty() {
            invalid_input_reason = Some("input mesh has no triangles");
        } else if self.target_triangle_count == 0 {
            invalid_input_reason = Some("target triangle count must be greater than zero");
        } else {
            // Last line of defense behind the loaders: the island build
            // below indexes m_vertices[face[i]] for i < 3, so a
            // non-triangle face or an out-of-range corner is an
            // out-of-bounds access (observed segfault via a corrupt face
            // line). Valid meshes never trip this.
            for face in &self.triangles {
                let mut face_valid = face.len() == 3;
                for &corner in face {
                    if !face_valid {
                        break;
                    }
                    face_valid = corner < self.vertices.len();
                }
                if !face_valid {
                    invalid_input_reason = Some("input mesh has invalid face indices");
                    break;
                }
            }
        }
        if let Some(reason) = invalid_input_reason {
            // (C++ `Invalid remesh input: ...` stderr omitted per the
            // stderr-gap memo; the handler call below carries it.)
            self.progress.report_direct(1.0, reason);
            return false;
        }

        // Power-of-two input normalization: absolute thresholds in the
        // pipeline (notably PositionKey's 1e-5 truncation) collapse
        // quality on tiny inputs (measured: diag 4e-4 yields 4% of the
        // target quads, diag 4e-5 yields nothing). Inputs with bbox
        // diagonal below 1 are scaled by 2^k into [1, 2) and every
        // position output is scaled back at the end; powers of two are
        // exact in binary floating point, so the round trip is lossless.
        // Inputs at diag >= 1 take the identical unscaled path (the
        // proven range: the bench corpus sits at 1.3-658).
        let normalize_scale = Self::normalization_scale(&self.vertices);
        if normalize_scale != 1.0 {
            Self::scale_positions(&mut self.vertices, normalize_scale);
            for line in self
                .guide_polylines
                .iter_mut()
                .chain(self.sharp_polylines.iter_mut())
            {
                Self::scale_positions(line, normalize_scale);
            }
        }

        // Resolve the symmetry plane once for the whole run. Islands share
        // the input's global coordinates, so the plane applies to every
        // island as-is. From here on, m_symmetryPlane.valid() alone gates
        // all symmetry work.
        self.symmetry_plane = SymmetryPlane::default();
        if self.symmetry_enabled {
            if self.symmetry_axis >= 0 && self.symmetry_axis < 3 {
                self.symmetry_plane = Symmetry::fixed_plane(&self.vertices, self.symmetry_axis);
            } else {
                self.symmetry_plane = Symmetry::detect_plane(&self.vertices);
            }
            if !self.symmetry_plane.valid() || self.symmetry_plane.score < MIN_SYMMETRY_SCORE {
                // (C++ `Symmetry skipped: ...` stderr omitted per the
                // stderr-gap memo; the getters report the fallback.)
                self.symmetry_plane = SymmetryPlane::default();
            }
        }

        let t_start = Instant::now();

        // Each label names the step that is about to run, not the one that
        // just finished, so the status line matches what the process is
        // actually doing.
        self.progress.report_direct(0.0, "Computing voxel size");
        let t_voxel_start = Instant::now();
        self.initialize_voxel_size();
        let t_voxel_end = Instant::now();

        self.progress
            .report_direct(0.01, "Splitting mesh into islands");
        let mut triangles_islands: Vec<Vec<Vec<usize>>> = Vec::new();
        let t_split_start = Instant::now();
        MeshSeparator::split_to_islands(&self.triangles, &mut triangles_islands);
        let t_after_split = Instant::now();

        if triangles_islands.is_empty() {
            // (C++ `Input mesh is empty` stderr omitted per the
            // stderr-gap memo; the handler call below carries it.)
            self.restore_normalized_inputs(normalize_scale);
            self.progress.report_direct(1.0, "Input mesh is empty");
            return false;
        }

        self.progress
            .report_direct(0.02, "Building island contexts");
        // Islands are compacted independently of each other, and writing
        // into a pre-sized vector by index keeps them in the original
        // order.
        let mut island_ctxs: Vec<IslandContext> = (0..triangles_islands.len())
            .map(|_| IslandContext::default())
            .collect();
        {
            let this = &*self;
            parallel_each(&mut island_ctxs, |island_index, context| {
                let island = &triangles_islands[island_index];
                context.triangles.reserve(island.len());
                // `BTreeMap` for the C++ `unordered_map` (insert/lookup
                // only, never iterated — deterministic either way).
                let mut old_to_new_vertex_map = BTreeMap::new();
                let use_density = !this.density_multipliers.is_empty();
                for face in island {
                    let mut triangle = Vec::with_capacity(3);
                    // (Island faces are validated triangles upstream, so
                    // the C++ `i < 3` loop reads the whole face; `take(3)`
                    // keeps the bound literal.)
                    for &corner in face.iter().take(3) {
                        let next = context.vertices.len();
                        let new_index = *old_to_new_vertex_map.entry(corner).or_insert(next);
                        if new_index == next {
                            context.vertices.push(this.vertices[corner]);
                            if use_density {
                                context.density.push(this.density_multipliers[corner]);
                            }
                        }
                        triangle.push(new_index);
                    }
                    context.triangles.push(triangle);
                }
                if use_density {
                    // Islands fully outside the mask normalize back to
                    // empty and run the unmodified pipeline.
                    context.density = Density::normalize_field(&context.density);
                }
                // Snapshot the input island before `resample` mutates
                // `vertices`/`triangles` in place (input-side coverage).
                context.input_vertices = context.vertices.clone();
                context.input_triangles = context.triangles.clone();

                context.scaling = this.scaling;
                context.voxel_size = this.voxel_size;
                context.adaptivity = this.adaptivity;
                context.anisotropy = this.anisotropy;
                context.sharp_edge_degrees = this.sharp_edge_degrees;
                context.smooth_normal_degrees = this.smooth_normal_degrees;
                context.symmetry_plane = this.symmetry_plane;
            });
        }
        let t_build_end = Instant::now();
        self.progress
            .report_direct(PARALLEL_PHASE_BEGIN, "Remeshing uniformly");

        let resample_time_us = AtomicI64::new(0);
        let adaptive_field_time_us = AtomicI64::new(0);
        let decimation_stats = DecimationStats::default();

        {
            {
                let mut data = self.progress.lock_progress();
                data.thread_progress_weights = vec![1.0; island_ctxs.len()];
                for (i, ctx) in island_ctxs.iter().enumerate() {
                    if !self.triangles.is_empty() {
                        data.thread_progress_weights[i] =
                            (ctx.triangles.len() as f64 / self.triangles.len() as f64) as f32;
                    }
                }
                data.thread_progress = vec![0.0; island_ctxs.len()];
                data.thread_status = vec![None; island_ctxs.len()];
                data.progress_sum = 0.0;
            }

            self.isotropic_vertices.clear();
            self.isotropic_triangles.clear();
            self.decimated_vertices.clear();
            self.decimated_triangles.clear();
            let mut isotropic_island_vertices: Vec<Vec<Vector3>> =
                (0..island_ctxs.len()).map(|_| Vec::new()).collect();
            let mut isotropic_island_triangles: Vec<Vec<Vec<usize>>> =
                (0..island_ctxs.len()).map(|_| Vec::new()).collect();
            let mut decimated_island_vertices: Vec<Vec<Vector3>> =
                (0..island_ctxs.len()).map(|_| Vec::new()).collect();
            let mut decimated_island_triangles: Vec<Vec<Vec<usize>>> =
                (0..island_ctxs.len()).map(|_| Vec::new()).collect();
            // The C++ `IsotropicPhase` functor, as an inline five-way
            // zipped chunk loop (one worker per chunk, disjoint slices):
            // `parallel_each` covers one `&mut` slice, but the worker
            // needs the context plus four output slices at once.
            if !island_ctxs.is_empty() {
                let this = &*self;
                // Share by reference: the `move` worker closures would
                // otherwise try to move these into the first thread.
                let resample_time = &resample_time_us;
                let adaptive_field_time = &adaptive_field_time_us;
                let stats = &decimation_stats;
                let chunk_len = worker_chunk_len(island_ctxs.len());
                thread::scope(|s| {
                    let zipped = island_ctxs
                        .chunks_mut(chunk_len)
                        .zip(isotropic_island_vertices.chunks_mut(chunk_len))
                        .zip(isotropic_island_triangles.chunks_mut(chunk_len))
                        .zip(decimated_island_vertices.chunks_mut(chunk_len))
                        .zip(decimated_island_triangles.chunks_mut(chunk_len));
                    for (
                        chunk_index,
                        ((((ctx_chunk, iso_v_chunk), iso_t_chunk), dec_v_chunk), dec_t_chunk),
                    ) in zipped.enumerate()
                    {
                        let base = chunk_index * chunk_len;
                        s.spawn(move || {
                            let rows = ctx_chunk
                                .iter_mut()
                                .zip(iso_v_chunk.iter_mut())
                                .zip(iso_t_chunk.iter_mut())
                                .zip(dec_v_chunk.iter_mut())
                                .zip(dec_t_chunk.iter_mut());
                            for (k, ((((ctx, iso_v), iso_t), dec_v), dec_t)) in rows.enumerate() {
                                let i = base + k;
                                this.update_progress(i, 0.0, Some("Remeshing uniformly"));
                                // Quiet runs skip downstream progress
                                // installation: no per-stage timings, and
                                // downstream progress echoes (the
                                // extractor's stderr chatter) stay off with
                                // no handler.
                                let quiet = this.quiet();
                                let isotropic_progress = if quiet {
                                    None
                                } else {
                                    Some(this.make_stage_progress(
                                        i,
                                        0.0,
                                        ISLAND_RESAMPLE_END,
                                        -1.0,
                                    ))
                                };

                                let t0 = Instant::now();
                                Self::resample(
                                    &mut ctx.vertices,
                                    &mut ctx.triangles,
                                    ctx.voxel_size,
                                    ctx.adaptivity,
                                    ctx.sharp_edge_degrees,
                                    ctx.smooth_normal_degrees,
                                    i,
                                    stats,
                                    adaptive_field_time,
                                    isotropic_progress,
                                    dec_v,
                                    dec_t,
                                    &ctx.density,
                                    &mut ctx.resampled_density,
                                    this.verbose,
                                );
                                let t1 = Instant::now();
                                resample_time.fetch_add(
                                    t1.duration_since(t0).as_micros() as i64,
                                    Ordering::SeqCst,
                                );

                                *iso_v = ctx.vertices.clone();
                                *iso_t = ctx.triangles.clone();

                                this.update_progress(i, ISLAND_RESAMPLE_END, None);
                            }
                        });
                    }
                });
            }
            Self::merge_islands(
                &isotropic_island_vertices,
                &isotropic_island_triangles,
                &mut self.isotropic_vertices,
                &mut self.isotropic_triangles,
            );
            self.decimated = decimation_stats.islands_decimated.load(Ordering::SeqCst) > 0;
            if self.decimated {
                Self::merge_islands(
                    &decimated_island_vertices,
                    &decimated_island_triangles,
                    &mut self.decimated_vertices,
                    &mut self.decimated_triangles,
                );
            }
        }
        let t_isotropic_end = Instant::now();

        let mut parameterization_threads: Vec<ParameterizationThread<'_>> = island_ctxs
            .iter()
            .enumerate()
            .map(|(i, context)| ParameterizationThread {
                island_index: i,
                island: context,
                coverage: None,
                captured_uvs: Vec::new(),
                captured_original_uvs: Vec::new(),
                captured_extracted_connection_moved: Vec::new(),
                captured_singular_vertices: Vec::new(),
                captured_singular_vertex_indices: Vec::new(),
                captured_extracted_connections: Vec::new(),
                captured_vertex_uvs: Vec::new(),
                remeshed_vertices: Vec::new(),
                remeshed_quads: Vec::new(),
                dipoles_placed: 0,
                compute_vertex_uvs: self.compute_remeshed_uvs,
            })
            .collect();

        let parameterize_time_us = AtomicI64::new(0);
        let extract_time_us = AtomicI64::new(0);
        // The C++ `SurfaceParameterizer` functor.
        {
            let this = &*self;
            parallel_each(&mut parameterization_threads, |_, thread| {
                let island = thread.island;
                let triangles = &island.triangles;

                if island.vertices.is_empty() || triangles.is_empty() {
                    // Still retire the island, otherwise its share of the
                    // bar is never filled in and the total stalls short of
                    // the end.
                    this.update_progress(thread.island_index, 1.0, None);
                    return;
                }

                // Coverage retry: attempt 0 runs the island as-is; when it
                // fails coverage (a region-scale uv fold dropped part of
                // the working mesh), retries re-run parameterize+extract
                // over deterministically jittered vertices and the first
                // full-coverage result wins (else attempt 0 is kept).
                let island_diag = Self::bbox_diag(&island.vertices);
                let mut jittered: Vec<Vector3> = Vec::new();
                let mut saved_attempt: Option<AttemptOutputs> = None;
                let mut coverage_fired = false;
                let mut initial_uncovered = 0usize;
                let mut winning_attempt = 0usize;
                let mut retries_run = 0usize;
                let mut final_uncovered = 0usize;
                // Report side, fixed by attempt 0: input-side counts
                // only when working-side passed and input-side fired.
                let mut report_input_side = false;
                // Fallback chain when no retry covers fully: the
                // earliest working-quiet attempt, so the input side
                // can never downgrade working-side coverage.
                let mut attempt0_working_quiet = false;
                let mut quiet_fallback: Option<(usize, AttemptOutputs, usize)> = None;
                for attempt in 0..=Self::COVERAGE_RETRY_SEEDS.len() {
                    if attempt > 0 {
                        if !coverage_fired {
                            break;
                        }
                        retries_run += 1;
                        thread.clear_attempt_outputs();
                        jittered = island.vertices.clone();
                        Self::jitter_working_vertices(
                            &mut jittered,
                            island_diag,
                            Self::COVERAGE_RETRY_SEEDS[attempt - 1],
                        );
                    }
                    let t0 = Instant::now();
                    let vertices: &[Vector3] = if attempt == 0 {
                        &island.vertices
                    } else {
                        &jittered
                    };

                    this.update_progress(thread.island_index, ISLAND_RESAMPLE_END, None);
                    let mut parameterizer = Parameterizer::new(vertices, triangles, None);
                    if !this.quiet() {
                        parameterizer.set_progress_handler(this.make_stage_progress(
                            thread.island_index,
                            ISLAND_RESAMPLE_END,
                            ISLAND_PARAMETERIZE_END,
                            0.0,
                        ));
                    }
                    if island.scaling > 0.0 {
                        parameterizer.set_scaling(island.scaling);
                    }
                    parameterizer.set_gradient_adaptivity(island.adaptivity);
                    parameterizer.set_anisotropy(island.anisotropy);
                    parameterizer.set_sharp_edge_degrees(island.sharp_edge_degrees);
                    parameterizer.set_symmetry_plane(island.symmetry_plane);
                    // Always `Some` (possibly empty, never null — the C++
                    // stores `&m_guidePolylines` unconditionally).
                    parameterizer.set_guide_polylines(Some(&this.guide_polylines));
                    parameterizer.set_sharp_polylines(Some(&this.sharp_polylines));
                    if !island.resampled_density.is_empty() {
                        parameterizer.set_density_field(island.resampled_density.clone());
                    }
                    // Always set (the product default flows down even when it
                    // is off: the leaf default must never shadow it).
                    parameterizer.set_dipoles(this.dipoles);
                    // (No try/catch counterpart: the port signals failure
                    // through the `bool` return, so there is nothing to catch —
                    // and the C++ `Island N: parameterization failed` stderr
                    // falls under the stderr-gap memo besides.)
                    let parameterize_succeeded = parameterizer.parameterize();

                    let t1 = Instant::now();
                    parameterize_time_us
                        .fetch_add(t1.duration_since(t0).as_micros() as i64, Ordering::SeqCst);

                    if parameterize_succeeded {
                        thread.dipoles_placed = parameterizer.dipole_flips();
                        this.update_progress(thread.island_index, ISLAND_PARAMETERIZE_END, None);
                        // `take_triangle_uvs` is `Some` on every success path
                        // (the C++ `if (uvs)` null branch is unreachable after
                        // success: `parameterize` populates the UVs before its
                        // single `return true`).
                        if let Some(uvs) = parameterizer.take_triangle_uvs() {
                            // Save a copy of UVs for the [param] preview overlay
                            thread.captured_uvs = uvs.clone();
                            thread.captured_original_uvs =
                                parameterizer.original_triangle_uvs().to_vec();
                            // Capture singular vertex positions for the [param]
                            // preview
                            thread.captured_singular_vertices =
                                parameterizer.singular_vertex_positions().to_vec();
                            thread.captured_singular_vertex_indices =
                                parameterizer.singular_vertex_indices().to_vec();
                            // Research probe (RETOPO_DUMP_STAGES=dir):
                            // stage-2 singularities + stage-3 (original + rounded)
                            // triangle uvs, parallel to the working triangles.
                            // Research probe (kept for item-7 stage analysis); no state touched.
                            // Attempt 0 only: stage dumps always show the
                            // first attempt (use coverage.log for retries).
                            if attempt == 0
                                && let Some(dir) = std::env::var_os("RETOPO_DUMP_STAGES")
                            {
                                let idx = thread.island_index;
                                let sing_path = std::path::Path::new(&dir)
                                    .join(format!("stage2_singular_island{idx}.txt"));
                                let mut sing = String::new();
                                for (k, p) in thread
                                    .captured_singular_vertex_indices
                                    .iter()
                                    .zip(thread.captured_singular_vertices.iter())
                                {
                                    sing.push_str(&format!("{k} {} {} {}\n", p.x(), p.y(), p.z()));
                                }
                                let _ = std::fs::write(sing_path, sing);
                                let uv_path = std::path::Path::new(&dir)
                                    .join(format!("stage3_uv_island{idx}.txt"));
                                let mut uv = String::new();
                                for (i, t) in uvs.iter().enumerate() {
                                    let o = &thread.captured_original_uvs[i];
                                    uv.push_str(&format!(
                                        "{i} {} {} {} {} {} {} {} {} {} {} {} {}\n",
                                        t[0].x(),
                                        t[0].y(),
                                        t[1].x(),
                                        t[1].y(),
                                        t[2].x(),
                                        t[2].y(),
                                        o[0].x(),
                                        o[0].y(),
                                        o[1].x(),
                                        o[1].y(),
                                        o[2].x(),
                                        o[2].y(),
                                    ));
                                }
                                let _ = std::fs::write(uv_path, uv);
                            }
                            // The extractor always embeds in the UNJITTERED
                            // working mesh: retry jitter steers only the
                            // parameterization (field/rounding/layout),
                            // never the output positions (identical to
                            // `vertices` on attempt 0).
                            let mut remesher =
                                QuadExtractor::new(&island.vertices, triangles, &uvs);
                            remesher.set_original_triangle_uvs(&thread.captured_original_uvs);
                            remesher
                                .set_singular_vertices(&thread.captured_singular_vertex_indices);
                            remesher.set_verbose_dump(this.verbose());
                            // No handler in quiet mode: the extractor's stderr
                            // progress echoes key off handler presence.
                            if !this.quiet() {
                                remesher.set_progress_handler(this.make_stage_progress(
                                    thread.island_index,
                                    ISLAND_PARAMETERIZE_END,
                                    1.0,
                                    1.0,
                                ));
                            }
                            // Research probe: the stage-4 uv dump needs
                            // per-vertex uvs, a pure post-pass (geometry
                            // identical on or off).
                            let dump_stages = std::env::var_os("RETOPO_DUMP_STAGES").is_some();
                            remesher
                                .set_compute_vertex_uvs(thread.compute_vertex_uvs || dump_stages);
                            if remesher.extract() {
                                thread.captured_extracted_connections =
                                    remesher.extracted_connections().to_vec();
                                thread.captured_extracted_connection_moved =
                                    remesher.extracted_connection_moved().to_vec();
                                thread.captured_vertex_uvs =
                                    remesher.remeshed_vertex_uvs().to_vec();
                                thread.remeshed_vertices = remesher.remeshed_vertices().to_vec();
                                thread.remeshed_quads = remesher.remeshed_quads().to_vec();
                                // Research probe (RETOPO_DUMP_STAGES=dir):
                                // stage-4 per-island extraction output.
                                // Research probe (kept for item-7 stage analysis); no state touched.
                                // Attempt 0 only (see the stage-2/3 probe).
                                if attempt == 0
                                    && let Some(dir) = std::env::var_os("RETOPO_DUMP_STAGES")
                                {
                                    let idx = thread.island_index;
                                    let uv_path = std::path::Path::new(&dir)
                                        .join(format!("stage4_uv_island{idx}.txt"));
                                    let mut uv = String::new();
                                    for w in remesher.remeshed_vertex_uvs().iter() {
                                        uv.push_str(&format!("{} {}\n", w.x(), w.y()));
                                    }
                                    let _ = std::fs::write(uv_path, uv);
                                    let path = std::path::Path::new(&dir)
                                        .join(format!("stage4_extract_island{idx}.obj"));
                                    let mut obj = String::new();
                                    for v in thread.remeshed_vertices.iter() {
                                        obj.push_str(&format!("v {} {} {}\n", v.x(), v.y(), v.z()));
                                    }
                                    for q in thread.remeshed_quads.iter() {
                                        obj.push('f');
                                        for c in q.iter() {
                                            obj.push_str(&format!(" {}", c + 1));
                                        }
                                        obj.push('\n');
                                    }
                                    let _ = std::fs::write(path, obj);
                                }
                            }
                        }
                    }
                    // Coverage verdict for this attempt (unconditional:
                    // the retry gate runs on every island). Measured
                    // against the unjittered working mesh (what the
                    // extractor embeds in; identical to `vertices` on
                    // attempt 0).
                    let attempt_gaps = Self::coverage_gaps(
                        &island.vertices,
                        &thread.remeshed_vertices,
                        &thread.remeshed_quads,
                    );
                    let attempt_diag = Self::bbox_diag(&island.vertices);
                    let (working_failed, working_uncovered) = Self::coverage_failed(
                        &attempt_gaps,
                        attempt_diag,
                        thread.remeshed_quads.len(),
                        triangles,
                        island.vertices.len(),
                    );
                    // Input side: original input verts vs the same
                    // output (catches extremities the working mesh
                    // keeps only as stretched-triangle surface). Same
                    // island bar; quiet on empty quads like above.
                    // Skipped when working-side already failed (the
                    // attempt retries regardless, and the report side
                    // is working) unless the research probe is set, so
                    // its log lines stay complete.
                    let probe_log = std::env::var_os("RETOPO_COVERAGE_LOG").is_some();
                    let (input_failed, input_uncovered, input_patch) =
                        if !working_failed || probe_log {
                            Self::input_coverage_failed(
                                &island.input_vertices,
                                &island.input_triangles,
                                &thread.remeshed_vertices,
                                &thread.remeshed_quads,
                                attempt_diag,
                            )
                        } else {
                            (false, 0, 0)
                        };
                    let attempt_failed = working_failed || input_failed;
                    let attempt_input_side = !working_failed && input_failed;
                    let attempt_empty = thread.remeshed_quads.is_empty();
                    // Research probe (RETOPO_COVERAGE_LOG=dir): one
                    // appended line per island attempt with the
                    // working-vertex coverage distribution (gap/diag
                    // fractions). O_APPEND keeps parallel island workers
                    // race-free. No state touched.
                    if let Some(dir) = std::env::var_os("RETOPO_COVERAGE_LOG") {
                        use std::io::Write;
                        let path = std::path::Path::new(&dir).join("coverage.log");
                        let diag = attempt_diag;
                        let nquads = thread.remeshed_quads.len();
                        let mut gaps = attempt_gaps.clone();
                        gaps.sort_by(|a, b| a.total_cmp(b));
                        let n = gaps.len().max(1);
                        let at = |q: f64| gaps[((n - 1) as f64 * q).round() as usize];
                        let over = |f: f64| {
                            gaps.iter().filter(|g| **g > f * diag).count() as f64 / n as f64
                        };
                        // Resolution-relative: verts beyond 3x the nominal
                        // quad width (diag/sqrt(nquads)).
                        let unit = diag / (nquads.max(1) as f64).sqrt();
                        let c3 = gaps.iter().filter(|g| **g > 3.0 * unit).count();
                        // Largest connected uncovered patch (the per-region
                        // bar's input); shares the verdict helper. Uses the
                        // UNSORTED gaps (vertex order): the sorted copy
                        // above would scramble adjacency into garbage.
                        let p3n = Self::largest_uncovered_patch(
                            &attempt_gaps,
                            triangles,
                            island.vertices.len(),
                            3.0 * unit,
                        );
                        if let Ok(mut f) = std::fs::OpenOptions::new()
                            .create(true)
                            .append(true)
                            .open(&path)
                        {
                            // Single write_all: one syscall stays atomic under
                            // O_APPEND when island workers log concurrently.
                            let line = format!(
                                "island={} attempt={attempt} nverts={} quads={} worst={} diag={diag} frac={} p50={} p90={} p99={} f02={} f05={} f10={} c3={c3} p3n={p3n} ic3={input_uncovered} ip3n={input_patch}\n",
                                thread.island_index,
                                vertices.len(),
                                nquads,
                                gaps[n - 1],
                                gaps[n - 1] / diag,
                                at(0.50) / diag,
                                at(0.90) / diag,
                                at(0.99) / diag,
                                over(0.02),
                                over(0.05),
                                over(0.10),
                            );
                            let _ = f.write_all(line.as_bytes());
                        }
                    }
                    let t2 = Instant::now();
                    extract_time_us
                        .fetch_add(t2.duration_since(t1).as_micros() as i64, Ordering::SeqCst);
                    if attempt == 0 {
                        if attempt_empty {
                            // The failed-island path owns empty outputs:
                            // no retry.
                            break;
                        }
                        if !attempt_failed {
                            break;
                        }
                        coverage_fired = true;
                        report_input_side = attempt_input_side;
                        attempt0_working_quiet = !working_failed;
                        initial_uncovered = if attempt_input_side {
                            input_uncovered
                        } else {
                            working_uncovered
                        };
                        saved_attempt = Some(thread.take_attempt_outputs());
                    } else if !attempt_empty && !attempt_failed {
                        winning_attempt = attempt;
                        final_uncovered = if report_input_side {
                            input_uncovered
                        } else {
                            working_uncovered
                        };
                        break;
                    } else if !attempt_empty && !working_failed && quiet_fallback.is_none() {
                        // First working-quiet retry: fallback if no
                        // later attempt covers fully (the loop keeps
                        // seeking a both-quiet winner past this).
                        quiet_fallback =
                            Some((attempt, thread.clone_attempt_outputs(), working_uncovered));
                    }
                }
                if coverage_fired {
                    let recovered = winning_attempt > 0;
                    let mut kept_attempt = winning_attempt;
                    if !recovered {
                        // Earliest working-quiet attempt: attempt 0
                        // when it passed working-side, else the first
                        // quiet retry, else attempt 0 (pre-input-side
                        // behavior exactly when nothing is quiet).
                        if attempt0_working_quiet {
                            if let Some(saved) = saved_attempt {
                                thread.restore_attempt_outputs(saved);
                            }
                            final_uncovered = initial_uncovered;
                        } else if let Some((idx, outputs, w_unc)) = quiet_fallback {
                            thread.restore_attempt_outputs(outputs);
                            kept_attempt = idx;
                            // Report side is working here (attempt 0
                            // failed it), counted on the kept retry.
                            final_uncovered = w_unc;
                        } else {
                            if let Some(saved) = saved_attempt {
                                thread.restore_attempt_outputs(saved);
                            }
                            final_uncovered = initial_uncovered;
                        }
                    }
                    thread.coverage = Some(CoverageReport {
                        island_index: thread.island_index,
                        retries_made: retries_run,
                        recovered,
                        initial_uncovered,
                        final_uncovered,
                        input_side: report_input_side,
                        kept_attempt,
                    });
                }
                this.update_progress(thread.island_index, 1.0, None);
            });
        }
        let t_parallel_end = Instant::now();

        self.progress
            .report_direct(PARALLEL_PHASE_END, "Merging mesh islands");

        // Merge isotropic UVs from all islands (for [param] preview)
        self.isotropic_triangle_uvs.clear();
        self.isotropic_original_triangle_uvs.clear();
        for thread in &parameterization_threads {
            if thread.captured_uvs.is_empty() {
                continue;
            }
            self.isotropic_triangle_uvs
                .extend(thread.captured_uvs.iter().cloned());
            self.isotropic_original_triangle_uvs
                .extend(thread.captured_original_uvs.iter().cloned());
        }

        // Merge singular vertex positions from all islands (for [param]
        // preview)
        self.isotropic_singular_vertices.clear();
        for thread in &parameterization_threads {
            if thread.captured_singular_vertices.is_empty() {
                continue;
            }
            self.isotropic_singular_vertices
                .extend(thread.captured_singular_vertices.iter().cloned());
        }

        // Merge the raw quad-extraction connections for the [param] preview.
        self.isotropic_extracted_connections.clear();
        self.isotropic_extracted_connection_moved.clear();
        for thread in &parameterization_threads {
            self.isotropic_extracted_connections
                .extend(thread.captured_extracted_connections.iter().cloned());
            let keep = self.isotropic_extracted_connections.len()
                - thread.captured_extracted_connections.len();
            self.isotropic_extracted_connection_moved.resize(keep, 0);
            self.isotropic_extracted_connection_moved
                .extend(thread.captured_extracted_connection_moved.iter().cloned());
            let total = self.isotropic_extracted_connections.len();
            self.isotropic_extracted_connection_moved.resize(total, 0);
        }
        self.remeshed_vertices.clear();
        self.remeshed_quads.clear();
        self.remeshed_vertex_uvs.clear();
        self.island_output_quad_counts = vec![0; parameterization_threads.len()];
        self.island_dipole_counts = vec![0; parameterization_threads.len()];
        self.coverage_reports.clear();
        let mut island_uv_spans: Vec<(usize, usize)> = Vec::new();
        for thread in &parameterization_threads {
            // Dipole counts merge for every island (placement happens in
            // parameterize(), even when extraction later yields nothing).
            self.island_dipole_counts[thread.island_index] = thread.dipoles_placed;
            if let Some(report) = thread.coverage {
                self.coverage_reports.push(report);
            }
            // (The C++ null-remesher skip: `remeshed_quads` stays empty when
            // the island produced nothing, subsuming both C++ skip cases.)
            if thread.remeshed_quads.is_empty() {
                continue;
            }
            self.island_output_quad_counts[thread.island_index] = thread.remeshed_quads.len();
            let vertex_start_index = self.remeshed_vertices.len();
            self.remeshed_vertices
                .reserve(thread.remeshed_vertices.len());
            for it in &thread.remeshed_vertices {
                self.remeshed_vertices.push(*it);
            }
            for it in &thread.remeshed_quads {
                let mut quad = Vec::with_capacity(it.len());
                for &v in it {
                    quad.push(vertex_start_index + v);
                }
                self.remeshed_quads.push(quad);
            }
            if self.compute_remeshed_uvs {
                // The extractor guarantees one UV per vertex; pad
                // defensively so the merged accessor can never disagree
                // with the vertex count.
                island_uv_spans.push((vertex_start_index, thread.remeshed_vertices.len()));
                self.remeshed_vertex_uvs
                    .reserve(self.remeshed_vertices.len());
                for i in 0..thread.remeshed_vertices.len() {
                    self.remeshed_vertex_uvs.push(
                        thread
                            .captured_vertex_uvs
                            .get(i)
                            .cloned()
                            .unwrap_or(Vector2::new(0.5, 0.5)),
                    );
                }
            }
        }
        if self.compute_remeshed_uvs {
            self.remeshed_vertex_uvs
                .resize(self.remeshed_vertices.len(), Vector2::new(0.5, 0.5));
            pack_island_uvs_into_atlas(&mut self.remeshed_vertex_uvs, &island_uv_spans);
        } else {
            self.remeshed_vertex_uvs.clear();
        }

        // Mirror partners can live on different islands (two disconnected
        // halves), so the vertex constraint runs once on the merged output,
        // not per island.
        if self.symmetry_plane.valid() && !self.remeshed_vertices.is_empty() {
            Symmetry::symmetrize_vertices(&mut self.remeshed_vertices, &self.symmetry_plane);
        }

        let t_merge_end = Instant::now();

        let elapsed_us = |from: Instant, to: Instant| to.duration_since(from).as_micros() as i64;
        let t_voxel_us = elapsed_us(t_voxel_start, t_voxel_end);
        let t_split_us = elapsed_us(t_split_start, t_after_split);
        let t_build_us = elapsed_us(t_after_split, t_build_end);
        let t_isotropic_wall_us = elapsed_us(t_build_end, t_isotropic_end);
        let t_parameterize_wall_us = elapsed_us(t_isotropic_end, t_parallel_end);
        let t_parallel_wall_us = elapsed_us(t_build_end, t_parallel_end);
        let t_merge_us = elapsed_us(t_parallel_end, t_merge_end);
        let t_total_us = elapsed_us(t_start, t_merge_end);

        let t_decimate_us = decimation_stats.time_us.load(Ordering::SeqCst);
        let t_adaptive_field_us = adaptive_field_time_us.load(Ordering::SeqCst);
        let decimated_islands = decimation_stats.islands_decimated.load(Ordering::SeqCst);

        self.phase_report.clear();
        // (The C++ builds these lines through a `phase` lambda capturing
        // an ostringstream; direct pushes keep the borrow checker quiet
        // across the stage-timing lock below. Same lines, same order.)
        self.phase_report.push(format!(
            "Islands: {}, input triangles: {}",
            island_ctxs.len(),
            self.triangles.len()
        ));
        self.push_phase_line("Compute voxel size", t_voxel_us);
        self.push_phase_line("Split into islands", t_split_us);
        self.push_phase_line("Build island contexts", t_build_us);

        if decimated_islands > 0 {
            self.phase_report.push(format!(
                "Mesh simplifier: RAN on {decimated_islands} of {} islands, {} -> {} triangles, {}",
                decimation_stats.islands_considered.load(Ordering::SeqCst),
                decimation_stats.triangles_before.load(Ordering::SeqCst),
                decimation_stats.triangles_after.load(Ordering::SeqCst),
                Self::format_ms(t_decimate_us)
            ));
        } else {
            self.phase_report.push(format!(
                "Mesh simplifier: SKIPPED (no island above {}x target triangle count), {}",
                DECIMATE_TRIGGER_RATIO as i64,
                Self::format_ms(t_decimate_us)
            ));
        }

        // The accumulated figures sum the islands, so on a multi-island
        // mesh they add up to more than the wall clock next to them. That
        // gap is the point: accumulated / wall is how many cores the phase
        // actually kept busy.
        self.push_phase_line(
            "Adaptive target length field (accumulated)",
            t_adaptive_field_us,
        );
        self.push_phase_line(
            "Isotropic remesh (accumulated)",
            resample_time_us.load(Ordering::SeqCst) - t_decimate_us - t_adaptive_field_us,
        );
        self.push_phase_line(
            "Parameterize (accumulated)",
            parameterize_time_us.load(Ordering::SeqCst),
        );
        self.push_phase_line(
            "Quad extract (accumulated)",
            extract_time_us.load(Ordering::SeqCst),
        );

        {
            let mut stages = self.progress.lock_stages();
            // `std::sort` transcribes as `sort_unstable_by` (both unstable;
            // equal orders keep no defined relative order on either side).
            stages.sort_unstable_by(|a, b| {
                a.order
                    .partial_cmp(&b.order)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            for it in stages.iter() {
                self.phase_report.push(format!(
                    "    {}: {}",
                    it.name,
                    Self::format_ms(it.microseconds)
                ));
            }
        }

        self.push_phase_line("Isotropic phase wall clock", t_isotropic_wall_us);
        self.push_phase_line("Parameterize phase wall clock", t_parameterize_wall_us);
        self.push_phase_line("Parallel phase wall clock", t_parallel_wall_us);

        {
            let accumulated = resample_time_us.load(Ordering::SeqCst)
                + parameterize_time_us.load(Ordering::SeqCst)
                + extract_time_us.load(Ordering::SeqCst);
            self.phase_report.push(format!(
                "Cores kept busy across the parallel phase: {:.2} (islands are the unit of parallelism)",
                if t_parallel_wall_us > 0 {
                    accumulated as f64 / t_parallel_wall_us as f64
                } else {
                    0.0
                }
            ));
        }

        self.push_phase_line("Merge islands", t_merge_us);
        self.push_phase_line("Total", t_total_us);

        // The report itself is always populated (phaseReport()); only the
        // stderr dump is quiet-gated. Warnings and errors above still
        // print. (The dump itself falls under the stderr-gap memo: omitted
        // on both paths; the main lane owns the stderr audit.)

        if normalize_scale != 1.0 {
            // Exact for powers of two (and 2^-1022 at worst — the scale
            // computation refuses denormal diagonals, so this never
            // underflows to zero).
            let inv = 1.0 / normalize_scale;
            Self::scale_positions(&mut self.remeshed_vertices, inv);
            Self::scale_positions(&mut self.decimated_vertices, inv);
            Self::scale_positions(&mut self.isotropic_vertices, inv);
            Self::scale_positions(&mut self.isotropic_singular_vertices, inv);
            for (a, b) in self.isotropic_extracted_connections.iter_mut() {
                *a *= inv;
                *b *= inv;
            }
            self.restore_normalized_inputs(normalize_scale);
        }

        self.progress.report_direct(1.0, "Done");

        true
    }
}
