/*
 *  Copyright (c) 2026 Jeremy HU <jeremy-at-dust3d dot org>. All rights reserved.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software.
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
module retopo.core.density;

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

    // Uniform-grid exact nearest lookup. Queries expand ring by ring and stop
    // once the closest unvisited ring is farther than the best hit, so
    // near-coincident points (decimated/remeshed verts on the same surface)
    // resolve in the first rings while far queries still terminate.
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

        size_t nearest(const Vector3& query) const
        {
            if (m_points->empty())
                return npos;
            const CellKey center = cellOf(query);
            size_t best = npos;
            double bestDistanceSquared = std::numeric_limits<double>::infinity();
            const long long maxRing = static_cast<long long>(m_points->size()) + 1;
            for (long long ring = 0; ring <= maxRing; ++ring) {
                if (ring > 0 && (static_cast<double>(ring - 1) * m_cellSize) * (static_cast<double>(ring - 1) * m_cellSize) >= bestDistanceSquared)
                    break;
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

    double boundingDiagonal(const std::vector<Vector3>& points)
    {
        Vector3 lower = points.front(), upper = points.front();
        for (const auto& point : points) {
            for (size_t i = 0; i < 3; ++i) {
                lower[i] = std::min(lower[i], point[i]);
                upper[i] = std::max(upper[i], point[i]);
            }
        }
        return (upper - lower).length();
    }
}

std::vector<double> Density::normalizeField(const std::vector<double>& field)
{
    if (field.empty())
        return {};
    std::vector<double> normalized;
    normalized.reserve(field.size());
    bool uniform = true;
    for (double value : field) {
        if (!std::isfinite(value))
            value = 1.0;
        else if (value < minMultiplier)
            value = minMultiplier;
        else if (value > maxMultiplier)
            value = maxMultiplier;
        normalized.push_back(value);
        if (value != 1.0)
            uniform = false;
    }
    if (uniform)
        return {};
    return normalized;
}

double Density::edgeScaleFor(double density)
{
    if (!std::isfinite(density) || density <= 0.0)
        return 1.0;
    return 1.0 / std::sqrt(density);
}

std::vector<double> Density::resampleNearest(const std::vector<Vector3>& srcPositions,
    const std::vector<double>& srcField,
    const std::vector<Vector3>& dstPositions)
{
    std::vector<double> resampled(dstPositions.size(), 1.0);
    if (srcPositions.empty() || srcField.size() != srcPositions.size() || dstPositions.empty())
        return resampled;
    // Cell sized for surface-distributed points: the mean neighbor spacing of
    // N points over an area ~ D^2 scales as D/sqrt(N).
    const double diagonal = boundingDiagonal(srcPositions);
    const double cellSize = diagonal > 0.0
        ? std::max(4.0 * diagonal / std::sqrt(static_cast<double>(srcPositions.size())), 1e-9)
        : 1e-9;
    const PointGrid grid(srcPositions, cellSize);
    for (size_t i = 0; i < dstPositions.size(); ++i) {
        const size_t hit = grid.nearest(dstPositions[i]);
        if (npos != hit)
            resampled[i] = srcField[hit];
    }
    return resampled;
}

void Density::applyToScalingField(const std::vector<Vector3>& vertices,
    const std::vector<std::vector<size_t>>& triangles,
    const std::vector<double>& densityPerVertex,
    std::vector<double>& faceScaling)
{
    if (densityPerVertex.size() != vertices.size() || faceScaling.size() != triangles.size()
        || vertices.empty() || triangles.empty()) {
        return;
    }
    std::vector<double> faceAreas(triangles.size(), 0.0);
    for (size_t i = 0; i < triangles.size(); ++i) {
        const auto& triangle = triangles[i];
        if (triangle.size() >= 3) {
            const Vector3 e0 = vertices[triangle[1]] - vertices[triangle[0]];
            const Vector3 e1 = vertices[triangle[2]] - vertices[triangle[0]];
            faceAreas[i] = 0.5 * Vector3::crossProduct(e0, e1).length();
        }
    }
    double budgetBefore = 0.0;
    for (size_t i = 0; i < triangles.size(); ++i) {
        const double m = faceScaling[i];
        if (m > 0.0)
            budgetBefore += faceAreas[i] / (m * m);
    }
    if (!(budgetBefore > 0.0) || !std::isfinite(budgetBefore))
        return;
    for (size_t i = 0; i < triangles.size(); ++i) {
        const auto& triangle = triangles[i];
        double faceDensity = 0.0;
        for (const size_t v : triangle)
            faceDensity += v < densityPerVertex.size() ? densityPerVertex[v] : 1.0;
        faceDensity /= static_cast<double>(triangle.size());
        const double scale = edgeScaleFor(faceDensity);
        if (scale > 0.0 && std::isfinite(scale))
            faceScaling[i] *= scale;
    }
    double budgetAfter = 0.0;
    for (size_t i = 0; i < triangles.size(); ++i) {
        const double m = faceScaling[i];
        if (m > 0.0 && std::isfinite(m))
            budgetAfter += faceAreas[i] / (m * m);
    }
    if (!(budgetAfter > 0.0) || !std::isfinite(budgetAfter))
        return;
    const double rescale = std::sqrt(budgetAfter / budgetBefore);
    if (rescale > 0.0 && std::isfinite(rescale)) {
        for (double& m : faceScaling)
            m *= rescale;
    }
}

}
