# Third-party notices (retopoforge)

retopoforge itself is MIT licensed (see `LICENSE`, copyright Jeremy HU and
contributors): the Rust engine is a port of Jeremy HU's AutoRemesher. The
`retopo` binary additionally includes the following third-party software.
Full license texts live in `ACKNOWLEDGEMENTS.html` (and next to the
vendored source under `thirdparty/`).

## `retopo` CLI and `retopo_core` library (cargo build)

| Dependency | License | How it is used | Full text |
|---|---|---|---|
| meshoptimizer | MIT (Arseny Kapoulkine) | `simplifier.cpp`, `indexgenerator.cpp` compiled in via `rust/core/build.rs` | `thirdparty/meshoptimizer/LICENSE.md` |
| isotropicremesher | MIT (Jeremy HU) | Ported to Rust (`iso_remesh_kernel.rs`, `isotropic_remesher.rs`) | `ACKNOWLEDGEMENTS.html` |
| faer (MIT) and its dependency crates | Permissive per crate: mostly MIT and/or Apache-2.0, plus BSD-2-Clause, Unicode-3.0, Zlib, Unlicense options | Sparse/dense linear algebra for the solvers | crate sources; full set pinned in `rust/Cargo.lock` |
| cc (build-time only) | MIT / Apache-2.0 | Compiles meshoptimizer; not shipped | crate source |

None of the above is GPL. The Blender extension (`blender/`) is
GPL-3.0-or-later as Blender requires; it talks to the MIT engine only as a
subprocess, never links it.

## Test and benchmark data (not shipped in binaries)

- `common-3d-test-models` (https://github.com/alecjacobson/common-3d-test-models),
  downloaded at bench time by `bench/fetch_models.sh`, never distributed.

## Historical note

Upstream AutoRemesher used GPL-licensed libraries (CoMISo, libQEx, CGAL)
before version 1.0.0 and reimplemented them for its MIT relicense
("Relicense from GPLv3 to MIT (reimplemented MIT-incompatible dependencies)").
Those dependencies are not present in this tree. The frozen C++ port of
the engine (with Eigen, oneTBB, cgltf, zlib and Apple Accelerate) was
removed after the Rust switch.
