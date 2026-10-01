//! Rust port of the retopoforge core modules (strangler-fig).
//!
//! One submodule per C++ module, named after the C++ file:
//! `core/objreader.*` -> [`obj_reader`], etc. Each module's contract is its
//! differential oracle: see `docs/rust-port-conventions.md`.

// Wave 1 leaves land here:
pub mod double_utils;
pub mod iso_remesh_kernel;
pub mod isotropic_remesher;
pub mod mesh_separator;
pub mod obj_reader;
pub mod progress;
// Wave 2 foundation (ported by the coordinator, unblocks the fan-out):
pub mod position_key;
pub mod surface_mesh;
pub mod vector2;
pub mod vector3;
// Wave 2 lane: rs-density.
pub mod density;
// Wave 2 rs-symmetry lane:
pub mod symmetry;
// Wave 2 lane: quad parameterizer (+ private SurfaceMesh/Guides mirrors).
pub mod quad_parameterizer;
// Wave 3 lane: rs-guides.
pub mod guides;
// Wave 3 lane: rs-framefield (+ private Guides mirrors, pending dedup).
pub mod frame_field;
// Wave 3 lane: rs-glb (port of cli/glb.*).
pub mod glb;
