/*
 *  retopoforge - GLB input/output for the Qt-free CLI.
 *  Forked from AutoRemesher by Jeremy HU <jeremy-at-dust3d dot org>
 *  (https://github.com/huxingyi/autoremesher), MIT licensed.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

#pragma once

// GLB support for the retopo CLI: cgltf-based input plus a minimal
// hand-written GLB writer (positions + indices only).
//
// Library decision (researched Sep 2026): cgltf v1.15 for input, hand-written
// writer for output. cgltf is a single C99 header (~200KB), MIT licensed,
// zero transitive dependencies, and read-only parsing is exactly what input
// needs. tinygltf was rejected: same MIT license but ~50k LOC of transitive
// deps (nlohmann/json + stb_image) and heavy compile times for a
// positions-and-faces use case. fastgltf was rejected: C++17 with SIMD-tuned
// parsing whose speed is irrelevant next to remesh time, and a bigger API
// surface to vendor. The writer needs no library at all: the GLB container
// is a 12-byte header plus a JSON chunk and a BIN chunk, and our output is
// one mesh with POSITION + indices, so ~100 lines of spec'd code beats
// vendoring a writer-capable dependency.

import retopo.core.vector2;
import retopo.core.vector3;

#include <string>
#include <string_view>
#include <vector>

namespace GlbIo {

// Case-insensitive ".glb" extension check (".GLB" from AI exporters counts).
bool hasGlbExtension(std::string_view path);

// Case-insensitive ".obj"/".glb" check for batch-mode input scanning.
bool isSupportedInputExtension(std::string_view path);

// Parse every mesh primitive in a .glb file into flat positions + triangles,
// mirroring AutoRemesher::loadObjPositionsAndTriangles (same shape: positions
// as xyz triples, one index vector per triangle; warn/err strings, false on
// fatal errors). Node world transforms are applied; non-triangle primitives
// are skipped with a warning; out-of-range indices fail the load.
bool loadGlbPositionsAndTriangles(const char* filename,
    std::vector<float>* positions,
    std::vector<std::vector<size_t>>* triangles,
    std::string* warn,
    std::string* err);

// Write vertices + faces (quads, tris, any n-gon) as a single-mesh .glb file.
// Faces are fan-triangulated; degenerate (<3 verts) faces are skipped.
// Returns false when there is nothing to write or the write fails.
bool saveGlb(const char* filename,
    const char* generator,
    const std::vector<AutoRemesher::Vector3>& vertices,
    const std::vector<std::vector<size_t>>& faces);

// --uvs on twin: same mesh plus a TEXCOORD_0 accessor (one VEC2 per
// vertex; uvs.size() must equal vertices.size()). The 4-arg overload is
// untouched so default output stays byte-identical.
bool saveGlb(const char* filename,
    const char* generator,
    const std::vector<AutoRemesher::Vector3>& vertices,
    const std::vector<std::vector<size_t>>& faces,
    const std::vector<AutoRemesher::Vector2>& uvs);

}
