# Third-party notices (retopoforge)

retopoforge itself is MIT licensed (see `LICENSE`, copyright Jeremy HU and
contributors): the Rust engine is a port of Jeremy HU's AutoRemesher. The
`retopo` binary additionally includes the following third-party software.
Full license texts are at the end of this file. Nothing is vendored:
the tree holds zero third-party code since the item-6 flip.

## `retopo` CLI and `retopo_core` library (cargo build)

| Dependency | License | How it is used | Full text |
|---|---|---|---|
| meshoptimizer algorithm (Arseny Kapoulkine) | MIT | Transcribed to Rust (`retopo_core::decimator`, derived work); no vendored source since the item-6 flip | below |
| isotropicremesher | MIT (Jeremy HU) | Ported to Rust (`iso_remesh_kernel.rs`, `isotropic_remesher.rs`) | below |
| faer (MIT) and its dependency crates | Permissive per crate: mostly MIT and/or Apache-2.0, plus BSD-2-Clause, Unicode-3.0, Zlib, Unlicense options | Sparse/dense linear algebra for the solvers | crate sources; full set pinned in `rust/Cargo.lock` |

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

## Full license texts

### meshoptimizer (algorithm transcribed into `retopo_core::decimator`; https://github.com/zeux/meshoptimizer)

```
MIT License

Copyright (c) 2016-2026 Arseny Kapoulkine

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

### isotropicremesher (ported to `rust/core/src/iso_remesh_kernel.rs` and `rust/core/src/isotropic_remesher.rs`; derived work)

```
MIT License

Copyright (c) 2020-2021 Jeremy HU . All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
