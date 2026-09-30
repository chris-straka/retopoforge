/*
 *  Copyright (c) 2020 Jeremy HU <jeremy-at-dust3d dot org>. All rights reserved. 
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
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
#include <map>
#include <memory>
#include <utility>
#include <vector>

export module retopo.core.parameterizer;

import retopo.core.progress;
import retopo.core.symmetry;
import retopo.core.vector2;
import retopo.core.vector3;

namespace AutoRemesher {

export class Parameterizer {
public:
    Parameterizer(const std::vector<Vector3>* vertices,
        const std::vector<std::vector<size_t>>* triangles,
        const std::vector<Vector3>* triangleFieldVectors)
        : m_vertices(vertices)
        , m_triangles(triangles)
        , m_triangleFieldVectors(triangleFieldVectors)
    {
    }

    std::unique_ptr<std::vector<std::vector<Vector2>>> takeTriangleUvs()
    {
        return std::move(m_triangleUvs);
    }

    const std::vector<std::vector<Vector2>>& originalTriangleUvs() const
    {
        return m_originalTriangleUvs;
    }

    const std::vector<Vector3>& singularVertexPositions() const
    {
        return m_singularVertexPositions;
    }

    const std::vector<size_t>& singularVertexIndices() const
    {
        return m_singularVertexIndices;
    }

    void setScaling(double scaling)
    {
        m_scaling = scaling;
    }

    void setGradientAdaptivity(double adaptivity)
    {
        m_adaptivity = adaptivity;
    }

    void setSharpEdgeDegrees(double degrees)
    {
        m_sharpEdgeDegrees = degrees;
    }

    void setAnisotropy(double anisotropy)
    {
        m_anisotropy = anisotropy;
    }

    void setSingularitySimplification(bool simplify)
    {
        m_singularitySimplification = simplify;
    }

    void setMaximumSingularityPairDistance(size_t faceHops)
    {
        m_maximumSingularityPairDistance = faceHops;
    }

    // Mirror-symmetry constraint for the frame field. Default is no plane
    // (axis -1), which leaves the field untouched.
    void setSymmetryPlane(SymmetryPlane plane)
    {
        m_symmetryPlane = plane;
    }

    // User guide polylines for the frame field: ordered point chains in mesh
    // coordinates (see FrameField::create). Null (the default) disables the
    // guide pass; the caller keeps the pointee alive through parameterize().
    void setGuidePolylines(const std::vector<std::vector<Vector3>>* guides)
    {
        m_guidePolylines = guides;
    }

    // Local density control: per-vertex multipliers on this parameterizer's
    // input mesh (the resampled island mesh), 1.0 = unchanged. Values clamp
    // to [0.25, 4.0]; an empty, uniform, or wrong-sized field disables the
    // modulation and leaves the scaling field untouched.
    void setDensityField(std::vector<double> field)
    {
        m_densityField = std::move(field);
    }

    void setProgressHandler(ProgressHandler progressHandler)
    {
        m_progressHandler = std::move(progressHandler);
    }

    bool parameterize();

private:
    const std::vector<Vector3>* m_vertices = nullptr;
    const std::vector<std::vector<size_t>>* m_triangles = nullptr;
    const std::vector<Vector3>* m_triangleFieldVectors = nullptr;
    std::unique_ptr<std::vector<std::vector<Vector2>>> m_triangleUvs;
    std::vector<Vector3> m_singularVertexPositions;
    std::vector<size_t> m_singularVertexIndices;
    std::vector<std::vector<Vector2>> m_originalTriangleUvs;
    double m_scaling = 1.0;
    double m_adaptivity = 0.5;
    double m_sharpEdgeDegrees = 90.0;
    double m_anisotropy = 1.0;
    double m_maxAspectRatio = 2.3;
    bool m_singularitySimplification = true;
    size_t m_maximumSingularityPairDistance = 6;
    SymmetryPlane m_symmetryPlane;
    const std::vector<std::vector<Vector3>>* m_guidePolylines = nullptr;
    std::vector<double> m_densityField;
    ProgressHandler m_progressHandler;

    std::vector<double> computeFaceScalingField(const std::vector<Vector3>& vertices,
        const std::vector<std::vector<size_t>>& triangles,
        const std::vector<Vector3>& vertexNormals,
        const std::vector<std::vector<size_t>>& faceAroundVertexMap) const;
};

}
