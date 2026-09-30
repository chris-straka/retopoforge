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
#include <algorithm>
#include <cstddef>
#include <vector>
module retopo.core.guides;

import retopo.core.surface_mesh;
import retopo.core.vector3;

namespace AutoRemesher {

double Guides::influenceRadius(const SurfaceMesh& mesh)
{
    // Six edge lengths: wide enough that the quad cover realizes the guided
    // flow inside the region instead of compromising it away, narrow enough
    // to stay local to the drawn curve.
    return 6.0 * mesh.averageEdgeLength();
}

Vector3 Guides::tangentNear(const std::vector<std::vector<Vector3>>& guides,
    const Vector3& point, const Vector3& normal, double radius)
{
    if (!(radius > 0.0))
        return Vector3();
    const double radiusSquared = radius * radius;
    Vector3 bestDirection;
    double bestDistanceSquared = radiusSquared;
    bool found = false;
    for (const auto& polyline : guides) {
        for (size_t i = 0; i + 1 < polyline.size(); ++i) {
            const Vector3 delta = polyline[i + 1] - polyline[i];
            const double length = delta.length();
            if (length <= 1e-12)
                continue;
            const Vector3 direction = delta / length;
            double along = Vector3::dotProduct(point - polyline[i], direction);
            along = std::max(0.0, std::min(length, along));
            const double distanceSquared = (point - (polyline[i] + direction * along)).lengthSquared();
            if (distanceSquared < bestDistanceSquared)
                bestDistanceSquared = distanceSquared, bestDirection = direction, found = true;
        }
    }
    if (!found)
        return Vector3();
    const Vector3 tangent = bestDirection - Vector3::dotProduct(bestDirection, normal) * normal;
    if (tangent.length() <= 0.5)
        return Vector3();
    return tangent.normalized();
}

}
