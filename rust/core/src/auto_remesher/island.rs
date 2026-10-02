use super::CoverageReport;
use crate::symmetry::SymmetryPlane;
use crate::vector2::Vector2;
use crate::vector3::Vector3;

pub(crate) struct IslandContext {
    pub(crate) vertices: Vec<Vector3>,
    pub(crate) triangles: Vec<Vec<usize>>,
    // Pre-resample input island (snapshot before `resample` decimates
    // in place): the input side of the coverage verdict compares the
    // extracted quads against these, catching extremities the working
    // mesh keeps only as stretched-triangle surface (sparse working
    // verts the working-side check cannot see).
    pub(crate) input_vertices: Vec<Vector3>,
    pub(crate) input_triangles: Vec<Vec<usize>>,
    pub(crate) voxel_size: f64,
    pub(crate) scaling: f64,
    pub(crate) adaptivity: f64,
    pub(crate) anisotropy: f64,
    pub(crate) sharp_edge_degrees: f64,
    pub(crate) smooth_normal_degrees: f64,
    pub(crate) symmetry_plane: SymmetryPlane,
    // NOTE: no stored guide/sharp polyline pointers (the C++
    // `&m_guidePolylines` / `&m_sharpPolylines`): their only consumer is
    // the parameterization worker, which reads `&self.guide_polylines` /
    // `&self.sharp_polylines` (always `Some`, possibly empty — never null
    // like the C++) from the scope capture instead. Storing the borrows
    // here would hold `&self` across the serial `&mut self` merge phase.
    // Density mask slice on the island's input vertices (empty = off),
    // plus the same mask carried onto the resampled island vertices.
    pub(crate) density: Vec<f64>,
    pub(crate) resampled_density: Vec<f64>,
}

impl Default for IslandContext {
    fn default() -> Self {
        Self {
            vertices: Vec::new(),
            triangles: Vec::new(),
            input_vertices: Vec::new(),
            input_triangles: Vec::new(),
            voxel_size: 0.0,
            scaling: 0.0,
            adaptivity: 0.0,
            anisotropy: 0.0,
            sharp_edge_degrees: 0.0,
            smooth_normal_degrees: 0.0,
            symmetry_plane: SymmetryPlane::default(),
            density: Vec::new(),
            resampled_density: Vec::new(),
        }
    }
}

/// Everything one island attempt writes (coverage-retry save/restore
/// bundle): extractor outputs plus the preview captures and dipole
/// count, so a retried island commits exactly one attempt's state.
pub(crate) struct AttemptOutputs {
    pub(crate) captured_uvs: Vec<Vec<Vector2>>,
    pub(crate) captured_original_uvs: Vec<Vec<Vector2>>,
    pub(crate) captured_extracted_connection_moved: Vec<u8>,
    pub(crate) captured_singular_vertices: Vec<Vector3>,
    pub(crate) captured_singular_vertex_indices: Vec<usize>,
    pub(crate) captured_extracted_connections: Vec<(Vector3, Vector3)>,
    pub(crate) captured_vertex_uvs: Vec<Vector2>,
    pub(crate) remeshed_vertices: Vec<Vector3>,
    pub(crate) remeshed_quads: Vec<Vec<usize>>,
    pub(crate) dipoles_placed: usize,
}

impl<'a> ParameterizationThread<'a> {
    /// Clears all attempt outputs (fresh retry start: attempts only
    /// write their success paths, so stale data must not linger).
    pub(crate) fn clear_attempt_outputs(&mut self) {
        self.captured_uvs.clear();
        self.captured_original_uvs.clear();
        self.captured_extracted_connection_moved.clear();
        self.captured_singular_vertices.clear();
        self.captured_singular_vertex_indices.clear();
        self.captured_extracted_connections.clear();
        self.captured_vertex_uvs.clear();
        self.remeshed_vertices.clear();
        self.remeshed_quads.clear();
        self.dipoles_placed = 0;
    }

    /// Moves all attempt outputs out (saving attempt 0 before retries).
    pub(crate) fn take_attempt_outputs(&mut self) -> AttemptOutputs {
        AttemptOutputs {
            captured_uvs: std::mem::take(&mut self.captured_uvs),
            captured_original_uvs: std::mem::take(&mut self.captured_original_uvs),
            captured_extracted_connection_moved: std::mem::take(
                &mut self.captured_extracted_connection_moved,
            ),
            captured_singular_vertices: std::mem::take(&mut self.captured_singular_vertices),
            captured_singular_vertex_indices: std::mem::take(
                &mut self.captured_singular_vertex_indices,
            ),
            captured_extracted_connections: std::mem::take(
                &mut self.captured_extracted_connections,
            ),
            captured_vertex_uvs: std::mem::take(&mut self.captured_vertex_uvs),
            remeshed_vertices: std::mem::take(&mut self.remeshed_vertices),
            remeshed_quads: std::mem::take(&mut self.remeshed_quads),
            dipoles_placed: std::mem::take(&mut self.dipoles_placed),
        }
    }

    /// Clones the current outputs into a bundle (working-quiet
    /// fallback: the loop continues past it seeking full coverage).
    pub(crate) fn clone_attempt_outputs(&self) -> AttemptOutputs {
        AttemptOutputs {
            captured_uvs: self.captured_uvs.clone(),
            captured_original_uvs: self.captured_original_uvs.clone(),
            captured_extracted_connection_moved: self.captured_extracted_connection_moved.clone(),
            captured_singular_vertices: self.captured_singular_vertices.clone(),
            captured_singular_vertex_indices: self.captured_singular_vertex_indices.clone(),
            captured_extracted_connections: self.captured_extracted_connections.clone(),
            captured_vertex_uvs: self.captured_vertex_uvs.clone(),
            remeshed_vertices: self.remeshed_vertices.clone(),
            remeshed_quads: self.remeshed_quads.clone(),
            dipoles_placed: self.dipoles_placed,
        }
    }

    /// Restores a bundle taken by [`Self::take_attempt_outputs`] (no
    /// full-coverage winner: the fallback attempt is kept).
    pub(crate) fn restore_attempt_outputs(&mut self, outputs: AttemptOutputs) {
        self.captured_uvs = outputs.captured_uvs;
        self.captured_original_uvs = outputs.captured_original_uvs;
        self.captured_extracted_connection_moved = outputs.captured_extracted_connection_moved;
        self.captured_singular_vertices = outputs.captured_singular_vertices;
        self.captured_singular_vertex_indices = outputs.captured_singular_vertex_indices;
        self.captured_extracted_connections = outputs.captured_extracted_connections;
        self.captured_vertex_uvs = outputs.captured_vertex_uvs;
        self.remeshed_vertices = outputs.remeshed_vertices;
        self.remeshed_quads = outputs.remeshed_quads;
        self.dipoles_placed = outputs.dipoles_placed;
    }
}

pub(crate) struct ParameterizationThread<'a> {
    pub(crate) island_index: usize,
    pub(crate) island: &'a IslandContext,
    pub(crate) coverage: Option<CoverageReport>,
    pub(crate) captured_uvs: Vec<Vec<Vector2>>,
    pub(crate) captured_original_uvs: Vec<Vec<Vector2>>,
    pub(crate) captured_extracted_connection_moved: Vec<u8>,
    pub(crate) captured_singular_vertices: Vec<Vector3>,
    pub(crate) captured_singular_vertex_indices: Vec<usize>,
    pub(crate) captured_extracted_connections: Vec<(Vector3, Vector3)>,
    pub(crate) captured_vertex_uvs: Vec<Vector2>,
    // Owned copies of the extractor outputs: the C++ merge phase reads
    // them off the live `thread.remesher`, which cannot be expressed here
    // (see the module docs on the self-referential thread struct).
    // `remeshed_vertices`/`remeshed_quads` stay empty when the island
    // produced nothing, which subsumes both C++ skip cases (null
    // remesher after a failed extract, and empty quads).
    pub(crate) remeshed_vertices: Vec<Vector3>,
    pub(crate) remeshed_quads: Vec<Vec<usize>>,
    // Dipole flips applied on this island (captured off the parameterizer
    // even when extraction later yields nothing).
    pub(crate) dipoles_placed: usize,
    // Copied from AutoRemesher::m_computeRemeshedUvs before the parallel
    // loop (the worker below is not a member, so it cannot read it).
    pub(crate) compute_vertex_uvs: bool,
    // NOTE: no `auto_remesher` back-pointer (the C++ `AutoRemesher*`): the
    // worker closures capture `&AutoRemesher` from the scope directly.
}
