/*
 *  retopoforge - GLB input/output for the Qt-free CLI.
 *  Forked from AutoRemesher by Jeremy HU <jeremy-at-dust3d dot org>
 *  (https://github.com/huxingyi/autoremesher), MIT licensed.
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

#include "glb.h"

#include <AutoRemesher/AutoRemesher>

#define CGLTF_IMPLEMENTATION
#include "cgltf.h"

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sstream>

namespace GlbIo {

static std::string lowerExtension(const std::string& path)
{
    const size_t dot = path.find_last_of('.');
    if (dot == std::string::npos)
        return std::string();
    // A trailing dot or a dot in a directory component is not an extension.
    const size_t slash = path.find_last_of("/\\");
    if (slash != std::string::npos && dot < slash)
        return std::string();
    std::string ext = path.substr(dot);
    for (char& c : ext)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return ext;
}

bool hasGlbExtension(const std::string& path)
{
    return lowerExtension(path) == ".glb";
}

bool isSupportedInputExtension(const std::string& path)
{
    const std::string ext = lowerExtension(path);
    return ext == ".obj" || ext == ".glb";
}

static void applyMatrix(const float* m, float* v)
{
    // cgltf matrices are column-major: out = M * [x y z 1].
    const float x = v[0];
    const float y = v[1];
    const float z = v[2];
    v[0] = m[0] * x + m[4] * y + m[8] * z + m[12];
    v[1] = m[1] * x + m[5] * y + m[9] * z + m[13];
    v[2] = m[2] * x + m[6] * y + m[10] * z + m[14];
}

struct PrimitiveStats {
    size_t skippedNonTriangles = 0;
    size_t skippedNoPosition = 0;
};

// Append one mesh's triangle primitives (world-transformed) to the outputs.
static bool appendMesh(const cgltf_mesh* mesh,
    const float* worldMatrix,
    std::vector<float>* positions,
    std::vector<std::vector<size_t>>* triangles,
    PrimitiveStats* stats,
    std::string* err)
{
    for (cgltf_size p = 0; p < mesh->primitives_count; ++p) {
        const cgltf_primitive* prim = &mesh->primitives[p];
        if (prim->type != cgltf_primitive_type_triangles) {
            ++stats->skippedNonTriangles;
            continue;
        }
        const cgltf_accessor* posAccessor = nullptr;
        for (cgltf_size a = 0; a < prim->attributes_count; ++a) {
            if (prim->attributes[a].type == cgltf_attribute_type_position) {
                posAccessor = prim->attributes[a].data;
                break;
            }
        }
        if (nullptr == posAccessor || posAccessor->count == 0) {
            ++stats->skippedNoPosition;
            continue;
        }
        const size_t base = positions->size() / 3;
        const size_t vertCount = static_cast<size_t>(posAccessor->count);
        positions->reserve(positions->size() + vertCount * 3);
        for (size_t i = 0; i < vertCount; ++i) {
            float v[3] = { 0.0f, 0.0f, 0.0f };
            if (!cgltf_accessor_read_float(posAccessor, i, v, 3)) {
                *err = "failed to read POSITION accessor";
                return false;
            }
            applyMatrix(worldMatrix, v);
            positions->push_back(v[0]);
            positions->push_back(v[1]);
            positions->push_back(v[2]);
        }
        if (nullptr == prim->indices) {
            // Non-indexed soup: every three vertices are one triangle.
            if (vertCount % 3 != 0) {
                *err = "non-indexed primitive vertex count is not a multiple of 3";
                return false;
            }
            for (size_t i = 0; i < vertCount; i += 3)
                triangles->push_back({ base + i, base + i + 1, base + i + 2 });
            continue;
        }
        const size_t indexCount = static_cast<size_t>(prim->indices->count);
        if (indexCount % 3 != 0) {
            *err = "index count is not a multiple of 3";
            return false;
        }
        for (size_t i = 0; i < indexCount; i += 3) {
            cgltf_uint idx[3] = { 0, 0, 0 };
            for (int k = 0; k < 3; ++k) {
                if (!cgltf_accessor_read_uint(prim->indices, i + k, &idx[k], 1)) {
                    *err = "failed to read index accessor";
                    return false;
                }
                if (static_cast<size_t>(idx[k]) >= vertCount) {
                    *err = "index out of range in " + std::to_string(i + k) + "-th index";
                    return false;
                }
            }
            triangles->push_back({ base + idx[0], base + idx[1], base + idx[2] });
        }
    }
    return true;
}

bool loadGlbPositionsAndTriangles(const char* filename,
    std::vector<float>* positions,
    std::vector<std::vector<size_t>>* triangles,
    std::string* warn,
    std::string* err)
{
    positions->clear();
    triangles->clear();

    cgltf_options options = {};
    cgltf_data* data = nullptr;
    if (cgltf_parse_file(&options, filename, &data) != cgltf_result_success) {
        *err = std::string("failed to parse GLB file ") + filename;
        return false;
    }
    if (cgltf_load_buffers(&options, data, filename) != cgltf_result_success) {
        *err = std::string("failed to load GLB buffers from ") + filename;
        cgltf_free(data);
        return false;
    }
    if (data->meshes_count == 0) {
        *err = std::string("GLB file has no meshes: ") + filename;
        cgltf_free(data);
        return false;
    }

    PrimitiveStats stats;
    bool ok = true;
    // Scene flattening: every node carrying a mesh contributes its mesh once,
    // transformed to world space (instanced meshes are duplicated, which is
    // what a remesher wants). Meshes no node references are still loaded once,
    // untransformed, so node-less exports are not silently empty.
    std::vector<char> meshReferenced(data->meshes_count, 0);
    for (cgltf_size n = 0; n < data->nodes_count; ++n) {
        const cgltf_node* node = &data->nodes[n];
        if (nullptr == node->mesh)
            continue;
        meshReferenced[cgltf_mesh_index(data, node->mesh)] = 1;
        float world[16];
        cgltf_node_transform_world(node, world);
        if (!appendMesh(node->mesh, world, positions, triangles, &stats, err)) {
            ok = false;
            break;
        }
    }
    if (ok) {
        const float identity[16] = {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
        };
        for (cgltf_size m = 0; m < data->meshes_count; ++m) {
            if (meshReferenced[m])
                continue;
            if (!appendMesh(&data->meshes[m], identity, positions, triangles, &stats, err)) {
                ok = false;
                break;
            }
        }
    }
    cgltf_free(data);
    if (!ok) {
        positions->clear();
        triangles->clear();
        return false;
    }
    if (triangles->empty()) {
        *err = std::string("GLB file has no triangle geometry: ") + filename;
        positions->clear();
        return false;
    }
    if (stats.skippedNonTriangles > 0 || stats.skippedNoPosition > 0) {
        std::ostringstream text;
        text << "skipped " << stats.skippedNonTriangles << " non-triangle and "
             << stats.skippedNoPosition << " position-less primitives in " << filename;
        *warn = text.str();
    }
    return true;
}

static void writeU32(std::vector<char>* out, uint32_t value)
{
    out->push_back(static_cast<char>(value & 0xff));
    out->push_back(static_cast<char>((value >> 8) & 0xff));
    out->push_back(static_cast<char>((value >> 16) & 0xff));
    out->push_back(static_cast<char>((value >> 24) & 0xff));
}

static bool writeGlb(const char* filename,
    const char* generator,
    const std::vector<AutoRemesher::Vector3>& vertices,
    const std::vector<std::vector<size_t>>& faces,
    const std::vector<AutoRemesher::Vector2>* uvs)
{
    // Fan-triangulate: quads become (0,1,2)+(0,2,3), matching the winding the
    // OBJ writer emits. Out-of-range indices fail loudly instead of writing
    // a corrupt file.
    std::vector<uint32_t> indices;
    indices.reserve(faces.size() * 6);
    for (const auto& face : faces) {
        if (face.size() < 3)
            continue;
        for (size_t index : face) {
            if (index >= vertices.size())
                return false;
        }
        for (size_t i = 1; i + 1 < face.size(); ++i) {
            indices.push_back(static_cast<uint32_t>(face[0]));
            indices.push_back(static_cast<uint32_t>(face[i]));
            indices.push_back(static_cast<uint32_t>(face[i + 1]));
        }
    }
    if (vertices.empty() || indices.empty())
        return false;
    const bool haveUvs = nullptr != uvs && uvs->size() == vertices.size();

    float minPos[3] = {
        static_cast<float>(vertices[0].x()),
        static_cast<float>(vertices[0].y()),
        static_cast<float>(vertices[0].z())
    };
    float maxPos[3] = { minPos[0], minPos[1], minPos[2] };
    for (const auto& v : vertices) {
        if (v.x() < minPos[0])
            minPos[0] = v.x();
        if (v.y() < minPos[1])
            minPos[1] = v.y();
        if (v.z() < minPos[2])
            minPos[2] = v.z();
        if (v.x() > maxPos[0])
            maxPos[0] = v.x();
        if (v.y() > maxPos[1])
            maxPos[1] = v.y();
        if (v.z() > maxPos[2])
            maxPos[2] = v.z();
    }

    const size_t vertBytes = vertices.size() * 3 * sizeof(float);
    const size_t indexBytes = indices.size() * sizeof(uint32_t);
    const size_t uvBytes = haveUvs ? vertices.size() * 2 * sizeof(float) : 0;
    // All sections are multiples of 4 by construction (3 floats, uint32,
    // 2 floats), so no inter-section padding is needed.
    const size_t binLength = vertBytes + indexBytes + uvBytes;

    std::ostringstream json;
    json << "{\"asset\":{\"version\":\"2.0\",\"generator\":\"" << generator << "\"}"
         << ",\"scene\":0"
         << ",\"scenes\":[{\"nodes\":[0]}]"
         << ",\"nodes\":[{\"mesh\":0,\"name\":\"retopoforge\"}]"
         << ",\"meshes\":[{\"name\":\"retopoforge\",\"primitives\":["
         << "{\"attributes\":{\"POSITION\":0";
    if (haveUvs)
        json << ",\"TEXCOORD_0\":2";
    json << "},\"indices\":1,\"mode\":4}]}]"
         << ",\"accessors\":["
         << "{\"bufferView\":0,\"componentType\":5126,\"count\":" << vertices.size()
         << ",\"type\":\"VEC3\",\"min\":[" << minPos[0] << "," << minPos[1] << "," << minPos[2]
         << "],\"max\":[" << maxPos[0] << "," << maxPos[1] << "," << maxPos[2] << "]}"
         << ",{\"bufferView\":1,\"componentType\":5125,\"count\":" << indices.size()
         << ",\"type\":\"SCALAR\"}";
    if (haveUvs)
        json << ",{\"bufferView\":2,\"componentType\":5126,\"count\":" << vertices.size()
             << ",\"type\":\"VEC2\"}";
    json << "]"
         << ",\"bufferViews\":["
         << "{\"buffer\":0,\"byteOffset\":0,\"byteLength\":" << vertBytes << "}"
         << ",{\"buffer\":0,\"byteOffset\":" << vertBytes << ",\"byteLength\":" << indexBytes << "}";
    if (haveUvs)
        json << ",{\"buffer\":0,\"byteOffset\":" << (vertBytes + indexBytes)
             << ",\"byteLength\":" << uvBytes << "}";
    json << "]"
         << ",\"buffers\":[{\"byteLength\":" << binLength << "}]}";
    std::string jsonText = json.str();
    while (jsonText.size() % 4 != 0)
        jsonText.push_back(' ');

    std::vector<char> blob;
    blob.reserve(12 + 8 + jsonText.size() + 8 + binLength);
    // GLB header: magic 'glTF', version 2, total length.
    blob.push_back('g');
    blob.push_back('l');
    blob.push_back('T');
    blob.push_back('F');
    writeU32(&blob, 2);
    writeU32(&blob, static_cast<uint32_t>(12 + 8 + jsonText.size() + 8 + binLength));
    // JSON chunk (type 0x4E4F534A "JSON").
    writeU32(&blob, static_cast<uint32_t>(jsonText.size()));
    writeU32(&blob, 0x4E4F534Au);
    blob.insert(blob.end(), jsonText.begin(), jsonText.end());
    // BIN chunk (type 0x004E4942 "BIN\0"). UVs append after indices, so the
    // UV-less prefix (positions + indices) keeps identical bytes and offsets.
    writeU32(&blob, static_cast<uint32_t>(binLength));
    writeU32(&blob, 0x004E4942u);
    const size_t vertFloats = vertices.size() * 3;
    const size_t uvFloats = haveUvs ? vertices.size() * 2 : 0;
    blob.resize(blob.size() + vertFloats * sizeof(float) + indexBytes + uvFloats * sizeof(float));
    char* bin = blob.data() + blob.size() - vertFloats * sizeof(float) - indexBytes - uvFloats * sizeof(float);
    float* posOut = reinterpret_cast<float*>(bin);
    for (const auto& v : vertices) {
        *posOut++ = v.x();
        *posOut++ = v.y();
        *posOut++ = v.z();
    }
    std::memcpy(posOut, indices.data(), indexBytes);
    if (haveUvs) {
        float* uvOut = reinterpret_cast<float*>(reinterpret_cast<char*>(posOut) + indexBytes);
        for (const auto& uv : *uvs) {
            *uvOut++ = static_cast<float>(uv.x());
            *uvOut++ = static_cast<float>(uv.y());
        }
    }

    FILE* file = std::fopen(filename, "wb");
    if (nullptr == file)
        return false;
    const size_t written = std::fwrite(blob.data(), 1, blob.size(), file);
    const int closeResult = std::fclose(file);
    return written == blob.size() && closeResult == 0;
}

bool saveGlb(const char* filename,
    const char* generator,
    const std::vector<AutoRemesher::Vector3>& vertices,
    const std::vector<std::vector<size_t>>& faces)
{
    return writeGlb(filename, generator, vertices, faces, nullptr);
}

bool saveGlb(const char* filename,
    const char* generator,
    const std::vector<AutoRemesher::Vector3>& vertices,
    const std::vector<std::vector<size_t>>& faces,
    const std::vector<AutoRemesher::Vector2>& uvs)
{
    if (uvs.size() != vertices.size())
        return false;
    return writeGlb(filename, generator, vertices, faces, &uvs);
}

}
