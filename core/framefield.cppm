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

export module retopo.core.frame_field;

import retopo.core.surface_mesh;
import retopo.core.vector3;

namespace AutoRemesher {

export class FrameField {
public:
    // `guides` holds user guide polylines as ordered point chains in mesh
    // coordinates; faces near a segment lock to the segment tangent (sharp
    // edges win ties). Empty by default: no guide pass runs at all.
    // `sharps` holds explicit sharp/feature polylines under the same
    // contract (point chains in mesh coordinates, snapped post-resample by
    // the caller). Sharp locks run before guide locks and win ties: faces
    // a sharp claims are skipped by the guide pass. Empty by default.
    static bool create(const SurfaceMesh& mesh, double sharpEdgeDegrees,
        std::vector<Vector3>* field,
        const std::vector<std::vector<Vector3>>& guides = {},
        const std::vector<std::vector<Vector3>>& sharps = {});
};
}
