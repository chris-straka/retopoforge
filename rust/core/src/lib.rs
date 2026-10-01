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
pub mod vector2;
pub mod vector3;
