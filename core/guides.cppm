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

export module retopo.core.guides;

import retopo.core.surface_mesh;
import retopo.core.vector3;

namespace AutoRemesher {

export class Guides {
public:
    // Influence radius for guide constraints on this mesh: faces and edges
    // within this distance of a guide polyline follow the guide tangent.
    static double influenceRadius(const SurfaceMesh& mesh);

    // Nearest guide segment tangent at `point`, projected onto the tangent
    // plane of `normal` and normalized. Returns the zero vector when no
    // guide segment passes within `radius` of `point`, when every nearby
    // segment is degenerate, or when the nearest tangent runs into the
    // surface (more than 60 degrees out of the tangent plane) and so
    // carries no flow direction for it.
    static Vector3 tangentNear(const std::vector<std::vector<Vector3>>& guides,
        const Vector3& point, const Vector3& normal, double radius);
};

}
