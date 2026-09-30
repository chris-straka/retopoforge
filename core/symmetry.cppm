/*
 *  Copyright (c) 2026 Jeremy HU <jeremy-at-dust3d dot org>. All rights reserved.
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
module;
#include <cstddef>
#include <vector>

export module retopo.core.symmetry;

import retopo.core.vector3;

namespace AutoRemesher {

export struct SymmetryPlane {
    // Mirror axis: 0 = X, 1 = Y, 2 = Z; -1 = no plane.
    int axis = -1;
    // Plane position: points with p[axis] == offset lie on the plane.
    double offset = 0.0;
    // Detection support: fraction of the input vertices that have a mirrored
    // partner within tolerance (1.0 for a perfectly mirrored mesh).
    double score = 0.0;

    bool valid() const
    {
        return axis >= 0 && axis < 3;
    }
};

export class Symmetry {
public:
    // Vote-based dominant-plane detection over the X, Y and Z axis-aligned
    // planes through the bounding-box center. Ties prefer Y, then Z, then X:
    // organic sculpts are Y-symmetric in practice. Always returns a valid
    // plane for a non-empty input; the caller gates on `score`.
    static SymmetryPlane detectPlane(const std::vector<Vector3>& vertices);

    // Fixed-axis plane at the bounding-box center, with its support score.
    static SymmetryPlane fixedPlane(const std::vector<Vector3>& vertices, int axis);

    // Fraction of vertices whose mirror across the plane lands within
    // `tolerance` of another vertex.
    static double scorePlane(const std::vector<Vector3>& vertices, int axis, double offset, double tolerance);

    static Vector3 mirrorPoint(const Vector3& point, const SymmetryPlane& plane);
    static Vector3 mirrorDirection(const Vector3& direction, const SymmetryPlane& plane);

    // Mirror-average a per-face cross field: each face is averaged (under the
    // cross's 4-way symmetry) with the mirrored field of the face nearest to
    // its mirrored centroid. Faces without a partner are left unchanged.
    static void symmetrizeFrameField(const std::vector<Vector3>& vertices,
        const std::vector<std::vector<size_t>>& triangles,
        std::vector<Vector3>& field,
        const SymmetryPlane& plane);

    // Snap vertices to exact mirror symmetry: every vertex ends up either
    // exactly mirrored by a partner or exactly on the plane.
    static void symmetrizeVertices(std::vector<Vector3>& vertices, const SymmetryPlane& plane);
};

}
