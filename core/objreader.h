/*
 *  Copyright (c) 2026 retopoforge contributors. All rights reserved.
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
#ifndef AUTO_REMESHER_OBJ_READER_H
#define AUTO_REMESHER_OBJ_READER_H
#include <cstddef>
#include <string>
#include <vector>

namespace AutoRemesher {

// Minimal Wavefront OBJ reader: vertex positions plus triangulated faces
// only (texture coordinates, normals, materials and groups are ignored).
// Mirrors the tinyobjloader behavior the callers relied on: positions are
// parsed as doubles rounded to float, face indices are 1-based with
// negative relative indices supported, and polygons are ear-clip
// triangulated.
bool loadObjPositionsAndTriangles(const char* filename,
    std::vector<float>* positions,
    std::vector<std::vector<size_t>>* triangles,
    std::string* warn,
    std::string* err);

}

#endif
