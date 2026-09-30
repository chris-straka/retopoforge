# cgltf (vendored)

Single-file glTF 2.0 parser (C99), used by the `retopo` CLI for `.glb` input
only. Output is written by a minimal hand-rolled GLB writer in `cli/glb.cpp`
(positions + indices; no writer dependency needed).

- Upstream: https://github.com/jkuhlmann/cgltf
- Version: v1.15 (upstream commit `360db1a95480`)
- License: MIT, see `LICENSE` in this directory
- Files vendored: `cgltf.h`, `LICENSE` (nothing else needed; header-only)
- Integration: `CGLTF_IMPLEMENTATION` is defined once, in `cli/glb.cpp`;
  `cli/CMakeLists.txt` adds this directory to the `retopo` include path.
