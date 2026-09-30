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
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
module retopo.core.frame_field;

import retopo.core.constrained_least_squares;
import retopo.core.guides;
import retopo.core.surface_mesh;
import retopo.core.vector3;

namespace AutoRemesher {
namespace {
    constexpr double kSymmetry = 4.0;

    struct FacetTangentBasis {
        Vector3 tangent;
        Vector3 perpendicularTangent;
        Vector3 normal;
    };

    Vector3 normalizedOrFallback(const Vector3& vector, const Vector3& fallback)
    {
        return vector.length() <= 1e-12 ? fallback.normalized() : vector.normalized();
    }

    FacetTangentBasis createFacetTangentBasis(const SurfaceMesh& mesh, size_t face)
    {
        const Vector3 normal = normalizedOrFallback(mesh.faceNormal(face), Vector3(0, 0, 1));
        Vector3 tangent = mesh.edgeVector(3 * face);
        tangent = tangent - Vector3::dotProduct(tangent, normal) * normal;
        if (tangent.length() <= 1e-12) {
            tangent = std::fabs(normal.x()) < .9 ? Vector3(1, 0, 0) : Vector3(0, 1, 0);
            tangent = tangent - Vector3::dotProduct(tangent, normal) * normal;
        }
        tangent = normalizedOrFallback(tangent, Vector3(1, 0, 0));
        return { tangent, Vector3::crossProduct(normal, tangent), normal };
    }

    double tangentAngle(const Vector3& vector, const FacetTangentBasis& basis)
    {
        return std::atan2(Vector3::dotProduct(vector, basis.perpendicularTangent), Vector3::dotProduct(vector, basis.tangent));
    }

    void accumulateCurvatureTensor(std::array<double, 6>* tensor, const Vector3& edge, double dihedral)
    {
        const Vector3 unitEdge = normalizedOrFallback(edge, Vector3(1, 0, 0));
        const double weightedDihedral = edge.length() * dihedral;
        (*tensor)[0] += weightedDihedral * unitEdge.x() * unitEdge.x();
        (*tensor)[1] += weightedDihedral * unitEdge.x() * unitEdge.y();
        (*tensor)[2] += weightedDihedral * unitEdge.y() * unitEdge.y();
        (*tensor)[3] += weightedDihedral * unitEdge.x() * unitEdge.z();
        (*tensor)[4] += weightedDihedral * unitEdge.y() * unitEdge.z();
        (*tensor)[5] += weightedDihedral * unitEdge.z() * unitEdge.z();
    }

    Eigen::Matrix3d curvatureTensorMatrix(const std::array<double, 6>& coefficients)
    {
        Eigen::Matrix3d matrix;
        matrix << coefficients[0], coefficients[1], coefficients[3], coefficients[1], coefficients[2], coefficients[4], coefficients[3], coefficients[4], coefficients[5];
        return matrix;
    }

    // Lock faces near a guide polyline to the nearest segment tangent, so
    // quad edge flow follows the drawn curve. Only faces the sharp-edge pass
    // left unlocked are touched; the locked values then enter the same
    // hard-constraint + smoothing solve as sharp edges above.
    void lockGuideFaces(const SurfaceMesh& mesh,
        const std::vector<FacetTangentBasis>& facetBases,
        const std::vector<std::vector<Vector3>>& guides,
        std::vector<double>* periodic, std::vector<char>* locked)
    {
        const double radius = Guides::influenceRadius(mesh);
        for (size_t faceIndex = 0; faceIndex < mesh.faceCount(); ++faceIndex) {
            if ((*locked)[faceIndex])
                continue;
            const auto& triangle = mesh.triangle(faceIndex);
            const Vector3 centroid = (mesh.position(triangle[0]) + mesh.position(triangle[1]) + mesh.position(triangle[2])) / 3.0;
            const Vector3 tangent = Guides::tangentNear(guides, centroid,
                facetBases[faceIndex].normal, radius);
            if (tangent.length() <= 1e-12)
                continue;
            const double fieldAngle = kSymmetry * tangentAngle(tangent, facetBases[faceIndex]);
            (*periodic)[2 * faceIndex] = std::cos(fieldAngle);
            (*periodic)[2 * faceIndex + 1] = std::sin(fieldAngle);
            (*locked)[faceIndex] = 1;
        }
    }

    // Sharp influence radius: tighter than the guide radius (6 edge
    // lengths) because sharp polylines arrive snapped to mesh vertices —
    // adjacent faces sit within ~1 edge length, and a crisp feature line
    // must not wash out into a broad flow region.
    double sharpInfluenceRadius(const SurfaceMesh& mesh)
    {
        return 2.0 * mesh.averageEdgeLength();
    }

    // Lock faces near an explicit sharp/feature polyline. Same
    // hard-constraint pattern as lockGuideFaces (skip faces an earlier pass
    // claimed); runs BEFORE the guide pass so sharps win ties over guides.
    void lockSharpFaces(const SurfaceMesh& mesh,
        const std::vector<FacetTangentBasis>& facetBases,
        const std::vector<std::vector<Vector3>>& sharps,
        std::vector<double>* periodic, std::vector<char>* locked)
    {
        const double radius = sharpInfluenceRadius(mesh);
        for (size_t faceIndex = 0; faceIndex < mesh.faceCount(); ++faceIndex) {
            if ((*locked)[faceIndex])
                continue;
            const auto& triangle = mesh.triangle(faceIndex);
            const Vector3 centroid = (mesh.position(triangle[0]) + mesh.position(triangle[1]) + mesh.position(triangle[2])) / 3.0;
            const Vector3 tangent = Guides::tangentNear(sharps, centroid,
                facetBases[faceIndex].normal, radius);
            if (tangent.length() <= 1e-12)
                continue;
            const double fieldAngle = kSymmetry * tangentAngle(tangent, facetBases[faceIndex]);
            (*periodic)[2 * faceIndex] = std::cos(fieldAngle);
            (*periodic)[2 * faceIndex + 1] = std::sin(fieldAngle);
            (*locked)[faceIndex] = 1;
        }
    }
}

bool FrameField::create(const SurfaceMesh& mesh, double sharpEdgeDegrees,
    std::vector<Vector3>* field,
    const std::vector<std::vector<Vector3>>& guides,
    const std::vector<std::vector<Vector3>>& sharps)
{
    if (nullptr == field || mesh.faceCount() == 0)
        return false;
    const size_t faces = mesh.faceCount();
    std::vector<FacetTangentBasis> facetBases(faces);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, faces), [&](const tbb::blocked_range<size_t>& range) {
        for (size_t faceIndex = range.begin(); faceIndex != range.end(); ++faceIndex)
            facetBases[faceIndex] = createFacetTangentBasis(mesh, faceIndex);
    });

    std::vector<double> periodic(2 * faces, 0.0), certainty(faces, 0.0);
    std::vector<char> locked(faces, 0);
    const double sharpRadians = sharpEdgeDegrees * M_PI / 180.0;
    for (size_t faceIndex = 0; faceIndex < faces; ++faceIndex)
        for (size_t cornerIndex = 3 * faceIndex; cornerIndex < 3 * faceIndex + 3; ++cornerIndex) {
            const size_t oppositeCornerIndex = mesh.oppositeCorner(cornerIndex);
            if (oppositeCornerIndex != SurfaceMesh::npos && std::fabs(mesh.normalAngle(cornerIndex)) <= sharpRadians)
                continue;
            const double fieldAngle = kSymmetry * tangentAngle(mesh.edgeVector(cornerIndex), facetBases[faceIndex]);
            periodic[2 * faceIndex] = std::cos(fieldAngle);
            periodic[2 * faceIndex + 1] = std::sin(fieldAngle);
            locked[faceIndex] = 1;
        }

    if (!sharps.empty())
        lockSharpFaces(mesh, facetBases, sharps, &periodic, &locked);

    if (!guides.empty())
        lockGuideFaces(mesh, facetBases, guides, &periodic, &locked);

    std::vector<std::array<double, 6>> vertexTensor(mesh.vertexCount());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, vertexTensor.size()), [&](const tbb::blocked_range<size_t>& range) {
        for (size_t vertexIndex = range.begin(); vertexIndex != range.end(); ++vertexIndex)
            vertexTensor[vertexIndex].fill(0.0);
    });
    for (size_t cornerIndex = 0; cornerIndex < mesh.cornerCount(); ++cornerIndex) {
        const size_t oppositeCornerIndex = mesh.oppositeCorner(cornerIndex);
        if (oppositeCornerIndex == SurfaceMesh::npos || oppositeCornerIndex < cornerIndex)
            continue;
        accumulateCurvatureTensor(&vertexTensor[mesh.cornerVertex(cornerIndex)], mesh.edgeVector(cornerIndex), mesh.normalAngle(cornerIndex));
        accumulateCurvatureTensor(&vertexTensor[mesh.cornerVertex(mesh.nextCorner(cornerIndex))], mesh.edgeVector(cornerIndex), mesh.normalAngle(cornerIndex));
    }
    double maximumCertainty = 0.0;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, faces), [&](const tbb::blocked_range<size_t>& range) {
        for (size_t faceIndex = range.begin(); faceIndex != range.end(); ++faceIndex)
            if (!locked[faceIndex]) {
                std::array<double, 6> total {};
                for (size_t cornerIndex = 3 * faceIndex; cornerIndex < 3 * faceIndex + 3; ++cornerIndex)
                    for (size_t coefficientIndex = 0; coefficientIndex < 6; ++coefficientIndex)
                        total[coefficientIndex] += vertexTensor[mesh.cornerVertex(cornerIndex)][coefficientIndex];
                Eigen::Matrix3d tensor = curvatureTensorMatrix(total);
                double trace = tensor(0, 0) + tensor(1, 1) + tensor(2, 2);
                const double regularizer = trace == 0.0 ? 1e-6 : 1e-6 * trace;
                tensor(0, 0) += regularizer;
                tensor(1, 1) += regularizer;
                tensor(2, 2) += regularizer;
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(tensor);
                if (eig.info() != Eigen::Success)
                    continue;
                std::array<int, 3> ordered = { 0, 1, 2 };
                std::sort(ordered.begin(), ordered.end(), [&](int a, int b) {
                    return std::fabs(eig.eigenvalues()[a]) > std::fabs(eig.eigenvalues()[b]);
                });
                const int primaryEigenvectorIndex = ordered[0], secondaryEigenvectorIndex = ordered[1];
                const Eigen::Vector3d direction = eig.eigenvectors().col(primaryEigenvectorIndex);
                const Vector3 principalDirection(direction.x(), direction.y(), direction.z());
                const double fieldAngle = kSymmetry * tangentAngle(principalDirection, facetBases[faceIndex]);
                periodic[2 * faceIndex] = std::cos(fieldAngle);
                periodic[2 * faceIndex + 1] = std::sin(fieldAngle);
                certainty[faceIndex] = std::fabs(eig.eigenvalues()[primaryEigenvectorIndex] - eig.eigenvalues()[secondaryEigenvectorIndex]);
            }
    });
    for (double certaintyValue : certainty)
        maximumCertainty = std::max(maximumCertainty, certaintyValue);
    if (maximumCertainty > 0.0)
        for (double& certaintyValue : certainty)
            certaintyValue /= maximumCertainty;

    const auto normalizePeriodic = [&]() {
        tbb::parallel_for(tbb::blocked_range<size_t>(0, faces), [&](const tbb::blocked_range<size_t>& range) {
            for (size_t faceIndex = range.begin(); faceIndex != range.end(); ++faceIndex) {
                const double periodicLength = std::hypot(periodic[2 * faceIndex], periodic[2 * faceIndex + 1]);
                if (periodicLength > 1e-30) {
                    periodic[2 * faceIndex] /= periodicLength;
                    periodic[2 * faceIndex + 1] /= periodicLength;
                }
            }
        });
    };

    normalizePeriodic();
    ConstrainedLeastSquares system(2 * faces);
    for (size_t f = 0; f < faces; ++f)
        if (locked[f]) {
            system.addConstraint({ { 2 * f, 1.0 } }, periodic[2 * f]);
            system.addConstraint({ { 2 * f + 1, 1.0 } }, periodic[2 * f + 1]);
        }
    for (size_t c = 0; c < mesh.cornerCount(); ++c) {
        const size_t other = mesh.oppositeCorner(c);
        if (other == SurfaceMesh::npos)
            continue;
        const size_t f = mesh.cornerFace(c), g = mesh.cornerFace(other);
        if (f < g)
            continue;
        const double transport = -kSymmetry * (tangentAngle(mesh.edgeVector(c), facetBases[g]) - tangentAngle(mesh.edgeVector(c), facetBases[f]));
        const double co = std::cos(transport), si = std::sin(transport);
        system.addEnergy({ { 2 * f, co }, { 2 * f + 1, si }, { 2 * g, -1.0 } }, 0.0);
        system.addEnergy({ { 2 * f, -si }, { 2 * f + 1, co }, { 2 * g + 1, -1.0 } }, 0.0);
    }
    std::vector<std::pair<size_t, size_t>> certaintyRows;
    certaintyRows.reserve(faces);
    for (size_t f = 0; f < faces; ++f)
        if (certainty[f] > 0.0) {
            const double weight = certainty[f] * certainty[f];
            const size_t rowU = system.addEnergy({ { 2 * f, 1.0 } }, periodic[2 * f], weight);
            const size_t rowV = system.addEnergy({ { 2 * f + 1, 1.0 } }, periodic[2 * f + 1], weight);
            certaintyRows.push_back({ rowU, rowV });
        }

    std::vector<double> solved;
    for (size_t iteration = 0; iteration < 5; ++iteration) {
        size_t rowIndex = 0;
        for (size_t f = 0; f < faces; ++f)
            if (certainty[f] > 0.0) {
                const auto& [rowU, rowV] = certaintyRows[rowIndex];
                system.setEnergyRightHandSide(rowU, periodic[2 * f]);
                system.setEnergyRightHandSide(rowV, periodic[2 * f + 1]);
                ++rowIndex;
            }
        if (!system.solve(&solved))
            return false;
        periodic.swap(solved);
        normalizePeriodic();
    }
    field->resize(faces);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, faces), [&](const tbb::blocked_range<size_t>& range) {
        for (size_t faceIndex = range.begin(); faceIndex != range.end(); ++faceIndex) {
            const double fieldAngle = std::atan2(periodic[2 * faceIndex + 1], periodic[2 * faceIndex]) / kSymmetry;
            (*field)[faceIndex] = std::cos(fieldAngle) * facetBases[faceIndex].tangent + std::sin(fieldAngle) * facetBases[faceIndex].perpendicularTangent;
        }
    });
    return true;
}

}
