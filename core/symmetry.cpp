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
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <unordered_map>
#include <vector>
module retopo.core.symmetry;

import retopo.core.vector3;

namespace AutoRemesher {

namespace {
    constexpr size_t npos = std::numeric_limits<size_t>::max();

    struct CellKey {
        long long x = 0;
        long long y = 0;
        long long z = 0;

        bool operator==(const CellKey& other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct CellKeyHash {
        size_t operator()(const CellKey& key) const noexcept
        {
            // FNV-1a over the three coordinates.
            size_t hash = 1469598103934665603u;
            const auto mix = [&hash](long long value) {
                const auto bits = static_cast<uint64_t>(value);
                for (size_t i = 0; i < 8; ++i) {
                    hash ^= static_cast<size_t>((bits >> (i * 8)) & 0xffu);
                    hash *= 1099511628211u;
                }
            };
            mix(key.x);
            mix(key.y);
            mix(key.z);
            return hash;
        }
    };

    // Uniform-grid nearest lookup over the ORIGINAL point positions. Queries
    // expand ring by ring and stop once the closest unvisited ring is farther
    // than the best hit, so near-symmetric partners resolve in ring 0-2 while
    // a `maxRadius <= 0` query still degenerates to a global nearest search.
    class PointGrid {
    public:
        PointGrid(const std::vector<Vector3>& points, double cellSize)
            : m_points(&points)
            , m_cellSize(cellSize > 0.0 ? cellSize : 1e-9)
        {
            m_cells.reserve(points.size() * 2 + 1);
            for (size_t i = 0; i < points.size(); ++i)
                m_cells[cellOf(points[i])].push_back(i);
        }

        size_t nearest(const Vector3& query, double maxRadius) const
        {
            if (m_points->empty())
                return npos;
            const CellKey center = cellOf(query);
            size_t best = npos;
            double bestDistanceSquared = maxRadius > 0.0 ? maxRadius * maxRadius
                                                         : std::numeric_limits<double>::infinity();
            // The grid spans at most points.size() occupied cells along any
            // axis, so this many rings always cover the whole grid.
            const long long maxRing = static_cast<long long>(m_points->size()) + 1;
            for (long long ring = 0; ring <= maxRing; ++ring) {
                // Closest possible distance to any cell strictly outside the
                // rings visited so far is (ring - 1) * cellSize; stop once the
                // best hit beats it.
                if (ring > 0 && (static_cast<double>(ring - 1) * m_cellSize) * (static_cast<double>(ring - 1) * m_cellSize) >= bestDistanceSquared)
                    break;
                if (maxRadius > 0.0 && static_cast<double>(ring - 1) * m_cellSize > maxRadius)
                    break;
                // Visit only the shell of the cube: at least one coordinate
                // offset has magnitude `ring`.
                for (long long dx = -ring; dx <= ring; ++dx) {
                    for (long long dy = -ring; dy <= ring; ++dy) {
                        for (long long dz = -ring; dz <= ring; ++dz) {
                            if (std::max({ std::llabs(dx), std::llabs(dy), std::llabs(dz) }) != ring)
                                continue;
                            const auto it = m_cells.find({ center.x + dx, center.y + dy, center.z + dz });
                            if (it == m_cells.end())
                                continue;
                            for (const size_t index : it->second) {
                                const double distanceSquared = ((*m_points)[index] - query).lengthSquared();
                                if (distanceSquared < bestDistanceSquared)
                                    bestDistanceSquared = distanceSquared, best = index;
                            }
                        }
                    }
                }
                if (bestDistanceSquared <= 0.0)
                    break;
            }
            return best;
        }

        bool hasNear(const Vector3& query, double radius) const
        {
            return npos != nearest(query, radius);
        }

    private:
        CellKey cellOf(const Vector3& point) const
        {
            return {
                static_cast<long long>(std::floor(point.x() / m_cellSize)),
                static_cast<long long>(std::floor(point.y() / m_cellSize)),
                static_cast<long long>(std::floor(point.z() / m_cellSize)),
            };
        }

        const std::vector<Vector3>* m_points;
        double m_cellSize;
        std::unordered_map<CellKey, std::vector<size_t>, CellKeyHash> m_cells;
    };

    void boundingBox(const std::vector<Vector3>& points, Vector3& lower, Vector3& upper)
    {
        lower = points.front();
        upper = points.front();
        for (const auto& point : points) {
            for (size_t i = 0; i < 3; ++i) {
                lower[i] = std::min(lower[i], point[i]);
                upper[i] = std::max(upper[i], point[i]);
            }
        }
    }

    double boundingDiagonal(const std::vector<Vector3>& points)
    {
        if (points.empty())
            return 0.0;
        Vector3 lower, upper;
        boundingBox(points, lower, upper);
        return (upper - lower).length();
    }

    // Cell sized for surface-distributed points: the mean neighbor spacing of
    // N points over an area ~ D^2 scales as D/sqrt(N).
    double partnerCellSize(const std::vector<Vector3>& points, double diagonal)
    {
        if (points.empty() || diagonal <= 0.0)
            return 1e-9;
        return std::max(4.0 * diagonal / std::sqrt(static_cast<double>(points.size())), 1e-9);
    }
}

Vector3 Symmetry::mirrorPoint(const Vector3& point, const SymmetryPlane& plane)
{
    Vector3 mirrored = point;
    if (plane.valid())
        mirrored[static_cast<size_t>(plane.axis)] = 2.0 * plane.offset - point[static_cast<size_t>(plane.axis)];
    return mirrored;
}

Vector3 Symmetry::mirrorDirection(const Vector3& direction, const SymmetryPlane& plane)
{
    Vector3 mirrored = direction;
    if (plane.valid())
        mirrored[static_cast<size_t>(plane.axis)] = -direction[static_cast<size_t>(plane.axis)];
    return mirrored;
}

double Symmetry::scorePlane(const std::vector<Vector3>& vertices, int axis, double offset, double tolerance)
{
    if (vertices.empty() || axis < 0 || axis > 2 || tolerance <= 0.0)
        return 0.0;
    const SymmetryPlane plane { axis, offset, 0.0 };
    const PointGrid grid(vertices, tolerance);
    size_t hits = 0;
    for (const auto& vertex : vertices) {
        if (grid.hasNear(mirrorPoint(vertex, plane), tolerance))
            ++hits;
    }
    return static_cast<double>(hits) / static_cast<double>(vertices.size());
}

SymmetryPlane Symmetry::detectPlane(const std::vector<Vector3>& vertices)
{
    SymmetryPlane best;
    if (vertices.empty())
        return best;
    Vector3 lower, upper;
    boundingBox(vertices, lower, upper);
    const double diagonal = (upper - lower).length();
    const double tolerance = std::max(0.01 * diagonal, 1e-9);
    // Ties prefer Y, then Z, then X (strictly-greater keeps the first best).
    for (const int axis : { 1, 2, 0 }) {
        const double offset = 0.5 * (lower[static_cast<size_t>(axis)] + upper[static_cast<size_t>(axis)]);
        const double score = scorePlane(vertices, axis, offset, tolerance);
        if (score > best.score)
            best = SymmetryPlane { axis, offset, score };
    }
    return best;
}

SymmetryPlane Symmetry::fixedPlane(const std::vector<Vector3>& vertices, int axis)
{
    SymmetryPlane plane;
    if (vertices.empty() || axis < 0 || axis > 2)
        return plane;
    Vector3 lower, upper;
    boundingBox(vertices, lower, upper);
    const double diagonal = (upper - lower).length();
    const double tolerance = std::max(0.01 * diagonal, 1e-9);
    plane.axis = axis;
    plane.offset = 0.5 * (lower[static_cast<size_t>(axis)] + upper[static_cast<size_t>(axis)]);
    plane.score = scorePlane(vertices, axis, plane.offset, tolerance);
    return plane;
}

void Symmetry::symmetrizeFrameField(const std::vector<Vector3>& vertices,
    const std::vector<std::vector<size_t>>& triangles,
    std::vector<Vector3>& field,
    const SymmetryPlane& plane)
{
    if (!plane.valid() || triangles.empty() || field.size() != triangles.size())
        return;

    std::vector<Vector3> centroids(triangles.size());
    std::vector<Vector3> normals(triangles.size());
    for (size_t f = 0; f < triangles.size(); ++f) {
        const auto& triangle = triangles[f];
        if (triangle.size() < 3)
            continue;
        const Vector3& a = vertices[triangle[0]];
        const Vector3& b = vertices[triangle[1]];
        const Vector3& c = vertices[triangle[2]];
        centroids[f] = (a + b + c) / 3.0;
        normals[f] = Vector3::normal(a, b, c);
    }
    const double diagonal = boundingDiagonal(centroids);
    const PointGrid grid(centroids, partnerCellSize(centroids, diagonal));

    // Match a mirrored tangent vector against the cross it is averaged into:
    // project onto the face tangent plane, then pick the k*90-degree rotation
    // about the normal closest to the face's own field direction.
    const auto matchCross = [](const Vector3& own, const Vector3& mirrored, const Vector3& normal) {
        const Vector3 axis = normal.normalized();
        Vector3 tangent = mirrored - axis * Vector3::dotProduct(mirrored, axis);
        if (tangent.length() <= 1e-12)
            return Vector3();
        tangent.normalize();
        const Vector3 ownDirection = own.normalized();
        const Vector3 perpendicular = Vector3::crossProduct(axis, tangent);
        Vector3 best = tangent;
        double bestDot = Vector3::dotProduct(tangent, ownDirection);
        const Vector3 candidates[3] = { perpendicular, -tangent, -perpendicular };
        for (const auto& candidate : candidates) {
            const double dot = Vector3::dotProduct(candidate, ownDirection);
            if (dot > bestDot)
                bestDot = dot, best = candidate;
        }
        return best;
    };

    std::vector<char> done(triangles.size(), 0);
    for (size_t f = 0; f < triangles.size(); ++f) {
        if (done[f] || field[f].length() <= 1e-12 || normals[f].length() <= 1e-12)
            continue;
        const size_t partner = grid.nearest(mirrorPoint(centroids[f], plane), 0.0);
        if (npos == partner || normals[partner].length() <= 1e-12)
            continue;
        if (partner == f) {
            // A face straddling the plane mirrors onto itself: average its
            // field with its own mirror so the cross is plane-symmetric.
            const Vector3 matched = matchCross(field[f], mirrorDirection(field[f], plane), normals[f]);
            if (matched.length() <= 1e-12) {
                done[f] = 1;
                continue;
            }
            field[f] = (field[f].normalized() + matched).normalized();
            done[f] = 1;
            continue;
        }
        if (done[partner] || field[partner].length() <= 1e-12)
            continue;
        const Vector3 matched = matchCross(field[f], mirrorDirection(field[partner], plane), normals[f]);
        if (matched.length() <= 1e-12)
            continue;
        const Vector3 averaged = (field[f].normalized() + matched).normalized();
        if (averaged.length() <= 1e-12)
            continue;
        field[f] = averaged;
        // The partner gets the exact mirror, re-projected onto its own
        // tangent plane (its plane is the mirror of this face's plane).
        const Vector3 axis = normals[partner].normalized();
        Vector3 mirrored = mirrorDirection(averaged, plane);
        mirrored = mirrored - axis * Vector3::dotProduct(mirrored, axis);
        if (mirrored.length() > 1e-12)
            field[partner] = mirrored.normalized();
        done[f] = 1;
        done[partner] = 1;
    }
}

void Symmetry::symmetrizeVertices(std::vector<Vector3>& vertices, const SymmetryPlane& plane)
{
    if (!plane.valid() || vertices.empty())
        return;
    const size_t axis = static_cast<size_t>(plane.axis);
    const double diagonal = boundingDiagonal(vertices);
    // Pairing runs on the original positions: the grid is built once and the
    // averaged positions are written into a separate buffer, so every vertex
    // pairs against the unmoved mesh.
    const PointGrid grid(vertices, partnerCellSize(vertices, diagonal));
    const std::vector<Vector3> original = vertices;
    std::vector<Vector3> snapped = vertices;
    std::vector<char> done(vertices.size(), 0);
    for (size_t i = 0; i < original.size(); ++i) {
        if (done[i])
            continue;
        const size_t partner = grid.nearest(mirrorPoint(original[i], plane), 0.0);
        if (npos == partner) {
            done[i] = 1;
            continue;
        }
        if (partner == i) {
            snapped[i][axis] = plane.offset;
            done[i] = 1;
            continue;
        }
        if (done[partner]) {
            // The partner already belongs to an exact pair; coincide with its
            // mirror rather than breaking that pair's exactness.
            snapped[i] = mirrorPoint(snapped[partner], plane);
            done[i] = 1;
            continue;
        }
        // Exact mirror pair: shared coordinates are averaged, the axis
        // coordinates are mirrored about the plane through their midpoint.
        Vector3 first = original[i];
        Vector3 second = original[partner];
        for (size_t c = 0; c < 3; ++c) {
            if (c == axis)
                continue;
            const double mean = 0.5 * (first[c] + second[c]);
            first[c] = mean;
            second[c] = mean;
        }
        first[axis] = 0.5 * (first[axis] + (2.0 * plane.offset - second[axis]));
        second[axis] = 2.0 * plane.offset - first[axis];
        snapped[i] = first;
        snapped[partner] = second;
        done[i] = 1;
        done[partner] = 1;
    }
    vertices = std::move(snapped);
}

}
