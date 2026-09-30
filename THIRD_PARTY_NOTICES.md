# Third-party notices (retopoforge)

retopoforge itself is MIT licensed (see `LICENSE`, copyright Jeremy HU and
contributors). The binaries below additionally include or link the following
third-party software. Full license texts live next to each dependency under
`thirdparty/` and in digest form in `ACKNOWLEDGEMENTS.html`.

## Headless engine (`retopo` CLI and `retopo_core` library, CMake build)

| Dependency | License | How it is used | Full text |
|---|---|---|---|
| Eigen | MPL 2.0 (some files BSD/MPL2-compatible) | Header-only linear algebra | `thirdparty/eigen/COPYING.*` |
| oneTBB | Apache-2.0 | Linked from the system install (Homebrew `tbb` / apt `libtbb-dev`) | https://github.com/oneapi-src/oneTBB |
| meshoptimizer | MIT | `simplifier.cpp`, `indexgenerator.cpp` compiled in | `thirdparty/meshoptimizer/LICENSE.md` |
| isotropicremesher | MIT (Jeremy HU) | Compiled in | `thirdparty/isotropicremesher/LICENSE` |
| zlib | zlib license | Linked on Unix | system / `ACKNOWLEDGEMENTS.html` |
| Apple Accelerate | System framework (macOS only) | BLAS/LAPACK via `Eigen/AccelerateSupport` | system |

None of the above is GPL. Eigen's MPL 2.0 is file-level copyleft only and
explicitly permits distribution in larger MIT-licensed works; keep its
`COPYING.*` notices with any binary distribution, plus TBB's Apache-2.0
`NOTICE` attribution.

## Qt desktop app (`retopoforge` target only)

Everything above, plus:

| Dependency | License | How it is used |
|---|---|---|
| Qt 5 / Qt 6 | LGPLv3 (dynamically linked) | GUI, OpenGL widgets |
| QtAwesome | MIT | Icon font helper (`thirdparty/QtAwesome`) |
| QuantumCD dark Fusion palette | Credit (design reference) | Color values inspired by https://gist.github.com/QuantumCD/6245215 |

Qt is used under the LGPL via dynamic linking; no GPL obligations arise from it.

## Test and benchmark data (not shipped in binaries)

- `common-3d-test-models` (https://github.com/alecjacobson/common-3d-test-models),
  downloaded at bench time by `bench/fetch_models.sh`, never distributed.

## Historical note

Upstream AutoRemesher used GPL-licensed libraries (CoMISo, libQEx, CGAL)
before version 1.0.0 and reimplemented them for its MIT relicense
("Relicense from GPLv3 to MIT (reimplemented MIT-incompatible dependencies)").
Those dependencies are not present in this tree: `ACKNOWLEDGEMENTS.html` was
audited at fork time and now lists only dependencies verified to be in the
source tree or the build.
