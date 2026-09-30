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
#include "rendermeshgenerator.h"
#include <AutoRemesher/AutoRemesher>
#include <QDebug>
#include <algorithm>
#include <cstring>
#include <limits>

import retopo.app.model_shader_vertex;

void RenderMeshGenerator::process()
{
    generate();

    emit finished();
}

void RenderMeshGenerator::calculateNormalizedFactors(const std::vector<AutoRemesher::Vector3>& vertices, AutoRemesher::Vector3* origin, double* maxLength)
{
    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();
    double minZ = std::numeric_limits<double>::max();
    double maxZ = std::numeric_limits<double>::lowest();
    for (const auto& v : vertices) {
        if (v.x() < minX)
            minX = v.x();
        if (v.x() > maxX)
            maxX = v.x();
        if (v.y() < minY)
            minY = v.y();
        if (v.y() > maxY)
            maxY = v.y();
        if (v.z() < minZ)
            minZ = v.z();
        if (v.z() > maxZ)
            maxZ = v.z();
    }
    AutoRemesher::Vector3 length = {
        (maxX - minX) * 0.5,
        (maxY - minY) * 0.5,
        (maxZ - minZ) * 0.5,
    };
    *maxLength = length[0];
    if (length[1] > *maxLength)
        *maxLength = length[1];
    if (length[2] > *maxLength)
        *maxLength = length[2];
    *origin = {
        (maxX + minX) * 0.5,
        (maxY + minY) * 0.5,
        (maxZ + minZ) * 0.5,
    };
}

void RenderMeshGenerator::normalizeVertices()
{
    AutoRemesher::Vector3 origin;
    double maxLength = 1.0;
    calculateNormalizedFactors(*m_vertices, &origin, &maxLength);
    for (auto& v : *m_vertices) {
        v = (v - origin) / maxLength;
    }
}

void RenderMeshGenerator::generate()
{
    normalizeVertices();

    // The source model can carry tens of millions of faces. Expanding every face
    // corner into its own vertex would need gigabytes of vertex data, more than
    // QOpenGLBuffer::allocate() can even describe with its int size argument, so
    // emit one shared vertex per source vertex and address them through index
    // buffers instead.
    const size_t sourceVertexCount = m_vertices->size();
    if (sourceVertexCount > std::numeric_limits<uint32_t>::max()) {
        qWarning() << "Model has too many vertices to index:" << sourceVertexCount;
        m_renderMesh = std::make_unique<ModelShaderMesh>();
        return;
    }

    std::vector<AutoRemesher::Vector3> vertexNormals(sourceVertexCount);
    size_t triangleIndexCount = 0;
    size_t edgeIndexCount = 0;
    for (const auto& sourceFace : *m_faces) {
        if (sourceFace.size() < 3)
            continue;

        AutoRemesher::Vector3 center;
        for (const auto& it : sourceFace) {
            center += (*m_vertices)[it];
        }
        center /= sourceFace.size();

        AutoRemesher::Vector3 normal;
        for (size_t corner = 0; corner < sourceFace.size(); ++corner) {
            size_t nextCorner = (corner + 1) % sourceFace.size();
            normal += AutoRemesher::Vector3::normal(center,
                (*m_vertices)[sourceFace[corner]],
                (*m_vertices)[sourceFace[nextCorner]]);
        }
        normal.normalize();

        for (size_t j = 0; j < sourceFace.size(); ++j)
            vertexNormals[sourceFace[j]] += normal;

        // Fan the face from its first corner, so a quad becomes two triangles
        triangleIndexCount += (sourceFace.size() - 2) * 3;
        edgeIndexCount += sourceFace.size() * 2;
    }
    for (auto& it : vertexNormals)
        it.normalize();

    const size_t maxIndexCount = std::numeric_limits<int>::max() / sizeof(uint32_t);
    if (triangleIndexCount > maxIndexCount || edgeIndexCount > maxIndexCount) {
        qWarning() << "Model has too many faces to render:" << m_faces->size();
        m_renderMesh = std::make_unique<ModelShaderMesh>();
        return;
    }

    auto triangleVertices = std::make_unique<ModelShaderVertex[]>(sourceVertexCount);
    memset(triangleVertices.get(), 0, sizeof(ModelShaderVertex) * sourceVertexCount);
    for (size_t i = 0; i < sourceVertexCount; ++i) {
        auto& v = triangleVertices[i];
        const auto& src = (*m_vertices)[i];
        const auto& normal = vertexNormals[i];
        v.posX = static_cast<float>(src.x());
        v.posY = static_cast<float>(src.y());
        v.posZ = static_cast<float>(src.z());
        v.normX = static_cast<float>(normal.x());
        v.normY = static_cast<float>(normal.y());
        v.normZ = static_cast<float>(normal.z());
        v.colorR = 1.0f;
        v.colorG = 0.996f;
        v.colorB = 0.890f;
        v.roughness = 1.0f;
        v.alpha = 1.0f;
    }

    auto triangleIndices = std::make_unique<uint32_t[]>(triangleIndexCount);
    size_t triangleIndexOffset = 0;
    // Undirected edges packed as (low << 32) | high, so shared edges collapse to
    // one key and the wireframe draws each edge once instead of once per face
    std::vector<uint64_t> edgeKeys;
    edgeKeys.reserve(edgeIndexCount / 2);
    for (const auto& sourceFace : *m_faces) {
        if (sourceFace.size() < 3)
            continue;
        const uint32_t first = static_cast<uint32_t>(sourceFace[0]);
        for (size_t j = 1; j + 1 < sourceFace.size(); ++j) {
            triangleIndices[triangleIndexOffset++] = first;
            triangleIndices[triangleIndexOffset++] = static_cast<uint32_t>(sourceFace[j]);
            triangleIndices[triangleIndexOffset++] = static_cast<uint32_t>(sourceFace[j + 1]);
        }
        for (size_t j = 0; j < sourceFace.size(); ++j) {
            const uint32_t from = static_cast<uint32_t>(sourceFace[j]);
            const uint32_t to = static_cast<uint32_t>(sourceFace[(j + 1) % sourceFace.size()]);
            if (from == to)
                continue;
            edgeKeys.push_back(from < to
                    ? (static_cast<uint64_t>(from) << 32) | to
                    : (static_cast<uint64_t>(to) << 32) | from);
        }
    }
    std::sort(edgeKeys.begin(), edgeKeys.end());
    edgeKeys.erase(std::unique(edgeKeys.begin(), edgeKeys.end()), edgeKeys.end());

    auto edgeIndices = std::make_unique<uint32_t[]>(edgeKeys.size() * 2);
    size_t edgeIndexOffset = 0;
    for (const auto& key : edgeKeys) {
        edgeIndices[edgeIndexOffset++] = static_cast<uint32_t>(key >> 32);
        edgeIndices[edgeIndexOffset++] = static_cast<uint32_t>(key & 0xffffffff);
    }

    m_renderMesh = std::make_unique<ModelShaderMesh>(triangleVertices.release(), static_cast<int>(sourceVertexCount), nullptr, 0,
        m_vertices.get(), m_faces.get());
    m_renderMesh->updateTriangleIndices(triangleIndices.release(), static_cast<int>(triangleIndexOffset));
    m_renderMesh->updateEdgeIndices(edgeIndices.release(), static_cast<int>(edgeIndexOffset));

    // ModelShaderMesh copies these, so the working copies are ours to release
    m_vertices.reset();
    m_faces.reset();
}
