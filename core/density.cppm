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

export module retopo.core.density;

import retopo.core.vector3;

namespace AutoRemesher {

export class Density {
public:
    // Supported per-vertex density multiplier range. 1.0 leaves a region
    // unchanged; values outside the range clamp to it.
    static constexpr double minMultiplier = 0.25;
    static constexpr double maxMultiplier = 4.0;

    // Clamp every entry to [minMultiplier, maxMultiplier] (non-finite values
    // become 1.0). An empty or all-1.0 field normalizes to empty, which is the
    // OFF state: callers skip all density work when the result is empty, so
    // the pipeline stays bit-identical to a run without any field.
    static std::vector<double> normalizeField(const std::vector<double>& field);

    // Target edge-length scale for a density multiplier: quads-per-area scale
    // as 1/h^2, so d times the quads need edges 1/sqrt(d) as long.
    static double edgeScaleFor(double density);

    // Nearest-neighbor resample of a per-vertex scalar field onto a new point
    // set (used to carry the density mask across decimation and isotropic
    // remeshing, which both retopologize the island). Returns uniform 1.0 on
    // any size mismatch so the caller normalizes back to OFF.
    static std::vector<double> resampleNearest(const std::vector<Vector3>& srcPositions,
        const std::vector<double>& srcField,
        const std::vector<Vector3>& dstPositions);

    // Multiply each face scaling entry by the edge scale of its averaged
    // vertex density, then renormalize all entries so SUM A_f/m_f^2 is
    // preserved: quads move into dense regions without changing the total
    // budget. No-op on any size mismatch.
    static void applyToScalingField(const std::vector<Vector3>& vertices,
        const std::vector<std::vector<size_t>>& triangles,
        const std::vector<double>& densityPerVertex,
        std::vector<double>& faceScaling);
};

}
