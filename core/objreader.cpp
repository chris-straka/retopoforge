/*
 *  Copyright (c) 2026 retopoforge contributors. All rights reserved.
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
#include <AutoRemesher/ObjReader>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>

namespace AutoRemesher {
namespace {

bool isSpace(char c)
{
    return c == ' ' || c == '\t';
}

bool isNewLine(char c)
{
    return c == '\r' || c == '\n' || c == '\0';
}

// Zero-based index with relative-index support, mirroring tinyobj's
// fixIndex: positive values are 1-based, negatives are relative to the
// end, zero is invalid.
bool fixIndex(int idx, size_t count, size_t* ret)
{
    if (idx > 0) {
        *ret = static_cast<size_t>(idx - 1);
        return true;
    }
    if (idx == 0)
        return false;
    const long long resolved = static_cast<long long>(count) + idx;
    if (resolved < 0)
        return false;
    *ret = static_cast<size_t>(resolved);
    return true;
}

// Parse the leading vertex index of one face corner (`v`, `v/vt`,
// `v//vn` or `v/vt/vn`); texture and normal parts are ignored.
bool parseFaceCorner(const char** cursor, size_t vertexCount, size_t* ret)
{
    char* end = nullptr;
    const long idx = std::strtol(*cursor, &end, 10);
    if (end == *cursor)
        return false;
    *cursor = end;
    *cursor += std::strspn(*cursor, "/");
    while (**cursor != '\0' && !isSpace(**cursor) && !isNewLine(**cursor)) {
        if (**cursor == '/') {
            ++(*cursor);
            continue;
        }
        char* partEnd = nullptr;
        std::strtol(*cursor, &partEnd, 10);
        *cursor = (partEnd == *cursor) ? *cursor + 1 : partEnd;
    }
    return fixIndex(static_cast<int>(idx), vertexCount, ret);
}

// Point-in-polygon test after W. Randolph Franklin's pnpoly
// (https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html).
bool pointInPolygon(int nvert, const float* vertx, const float* verty, float testx, float testy)
{
    int c = 0;
    for (int i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((verty[i] > testy) != (verty[j] > testy)) && (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
            c = !c;
    }
    return c != 0;
}

bool positionValid(size_t vertexIndex, size_t component, const std::vector<float>& positions)
{
    return (vertexIndex * 3 + component) < positions.size();
}

// Ear-clipping triangulation matching the triangulation tinyobjloader
// applied to polygon faces: project to the dominant plane, then clip
// ears starting from the first corner.
void triangulateFace(const std::vector<float>& positions,
    const std::vector<size_t>& corners,
    std::vector<std::vector<size_t>>* out)
{
    const size_t cornerCount = corners.size();
    if (cornerCount < 3)
        return;

    // Find the two axes to work in.
    size_t axes[2] = { 1, 2 };
    for (size_t k = 0; k < cornerCount; ++k) {
        const size_t vi0 = corners[(k + 0) % cornerCount];
        const size_t vi1 = corners[(k + 1) % cornerCount];
        const size_t vi2 = corners[(k + 2) % cornerCount];
        if (((3 * vi0 + 2) >= positions.size()) || ((3 * vi1 + 2) >= positions.size()) || ((3 * vi2 + 2) >= positions.size()))
            continue;
        const float v0x = positions[vi0 * 3 + 0];
        const float v0y = positions[vi0 * 3 + 1];
        const float v0z = positions[vi0 * 3 + 2];
        const float v1x = positions[vi1 * 3 + 0];
        const float v1y = positions[vi1 * 3 + 1];
        const float v1z = positions[vi1 * 3 + 2];
        const float v2x = positions[vi2 * 3 + 0];
        const float v2y = positions[vi2 * 3 + 1];
        const float v2z = positions[vi2 * 3 + 2];
        const float e0x = v1x - v0x;
        const float e0y = v1y - v0y;
        const float e0z = v1z - v0z;
        const float e1x = v2x - v1x;
        const float e1y = v2y - v1y;
        const float e1z = v2z - v1z;
        const float cx = std::fabs(e0y * e1z - e0z * e1y);
        const float cy = std::fabs(e0z * e1x - e0x * e1z);
        const float cz = std::fabs(e0x * e1y - e0y * e1x);
        const float epsilon = std::numeric_limits<float>::epsilon();
        if (cx > epsilon || cy > epsilon || cz > epsilon) {
            // Found a corner.
            if (cx > cy && cx > cz) {
            } else {
                axes[0] = 0;
                if (cz > cx && cz > cy)
                    axes[1] = 1;
            }
            break;
        }
    }

    float area = 0;
    for (size_t k = 0; k < cornerCount; ++k) {
        const size_t vi0 = corners[(k + 0) % cornerCount];
        const size_t vi1 = corners[(k + 1) % cornerCount];
        if (!positionValid(vi0, axes[0], positions) || !positionValid(vi0, axes[1], positions)
            || !positionValid(vi1, axes[0], positions) || !positionValid(vi1, axes[1], positions))
            continue;
        const float v0x = positions[vi0 * 3 + axes[0]];
        const float v0y = positions[vi0 * 3 + axes[1]];
        const float v1x = positions[vi1 * 3 + axes[0]];
        const float v1y = positions[vi1 * 3 + axes[1]];
        area += (v0x * v1y - v0y * v1x) * static_cast<float>(0.5);
    }

    std::vector<size_t> remaining = corners;
    size_t guessVert = 0;
    size_t remainingIterations = cornerCount;
    size_t previousRemainingVertices = remaining.size();

    while (remaining.size() > 3 && remainingIterations > 0) {
        const size_t npolys = remaining.size();
        if (guessVert >= npolys)
            guessVert -= npolys;

        if (previousRemainingVertices != npolys) {
            // The number of remaining vertices decreased. Reset counters.
            previousRemainingVertices = npolys;
            remainingIterations = npolys;
        } else {
            // We didn't consume a vertex on previous iteration, reduce the
            // available iterations.
            remainingIterations--;
        }

        size_t ind[3];
        float vx[3];
        float vy[3];
        for (size_t k = 0; k < 3; k++) {
            ind[k] = remaining[(guessVert + k) % npolys];
            if (!positionValid(ind[k], axes[0], positions) || !positionValid(ind[k], axes[1], positions)) {
                vx[k] = static_cast<float>(0.0);
                vy[k] = static_cast<float>(0.0);
            } else {
                vx[k] = positions[ind[k] * 3 + axes[0]];
                vy[k] = positions[ind[k] * 3 + axes[1]];
            }
        }
        const float e0x = vx[1] - vx[0];
        const float e0y = vy[1] - vy[0];
        const float e1x = vx[2] - vx[1];
        const float e1y = vy[2] - vy[1];
        const float cross = e0x * e1y - e0y * e1x;
        // If an internal angle.
        if (cross * area < static_cast<float>(0.0)) {
            guessVert += 1;
            continue;
        }

        // Check all other verts in case they are inside this triangle.
        bool overlap = false;
        for (size_t otherVert = 3; otherVert < npolys; ++otherVert) {
            const size_t idx = (guessVert + otherVert) % npolys;
            if (idx >= remaining.size())
                continue;
            const size_t ovi = remaining[idx];
            if (!positionValid(ovi, axes[0], positions) || !positionValid(ovi, axes[1], positions))
                continue;
            const float tx = positions[ovi * 3 + axes[0]];
            const float ty = positions[ovi * 3 + axes[1]];
            if (pointInPolygon(3, vx, vy, tx, ty)) {
                overlap = true;
                break;
            }
        }

        if (overlap) {
            guessVert += 1;
            continue;
        }

        // This triangle is an ear.
        out->push_back(std::vector<size_t> { ind[0], ind[1], ind[2] });

        // Remove v1 from the list.
        remaining.erase(remaining.begin() + (guessVert + 1) % npolys);
    }

    if (remaining.size() == 3)
        out->push_back(std::vector<size_t> { remaining[0], remaining[1], remaining[2] });
}

}

bool loadObjPositionsAndTriangles(const char* filename,
    std::vector<float>* positions,
    std::vector<std::vector<size_t>>* triangles,
    std::string* warn,
    std::string* err)
{
    std::string warnString;
    std::string errString;

    std::ifstream file(filename, std::ios::in | std::ios::binary);
    if (!file.is_open()) {
        std::ostringstream oss;
        oss << "Cannot open file [" << (filename ? filename : "(null)") << "]" << '\n';
        errString = oss.str();
        if (warn)
            *warn = warnString;
        if (err)
            *err = errString;
        return false;
    }

    std::vector<float> localPositions;
    std::vector<std::vector<size_t>> localTriangles;
    std::string line;
    size_t lineNumber = 0;
    while (std::getline(file, line)) {
        ++lineNumber;
        const char* cursor = line.c_str();
        cursor += std::strspn(cursor, " \t");
        if (isNewLine(*cursor) || *cursor == '#')
            continue;

        // Vertex position (`v` only; `vn`, `vt`, `vp` are ignored).
        if (cursor[0] == 'v' && isSpace(cursor[1])) {
            cursor += 2;
            float xyz[3] = { 0.0f, 0.0f, 0.0f };
            for (int i = 0; i < 3; ++i) {
                cursor += std::strspn(cursor, " \t");
                if (isNewLine(*cursor) || *cursor == '#')
                    break;
                char* end = nullptr;
                const double value = std::strtod(cursor, &end);
                if (end == cursor)
                    break;
                xyz[i] = static_cast<float>(value);
                cursor = end;
            }
            localPositions.push_back(xyz[0]);
            localPositions.push_back(xyz[1]);
            localPositions.push_back(xyz[2]);
            continue;
        }

        // Face.
        if (cursor[0] == 'f' && isSpace(cursor[1])) {
            cursor += 2;
            std::vector<size_t> corners;
            bool failed = false;
            while (true) {
                cursor += std::strspn(cursor, " \t");
                if (isNewLine(*cursor) || *cursor == '#')
                    break;
                size_t index = 0;
                if (!parseFaceCorner(&cursor, localPositions.size() / 3, &index)) {
                    failed = true;
                    break;
                }
                corners.push_back(index);
            }
            if (failed) {
                std::ostringstream oss;
                oss << "Failed parse `f' line(e.g. zero value for face index. line "
                    << lineNumber << ".)\n";
                errString += oss.str();
                if (warn)
                    *warn = warnString;
                if (err)
                    *err = errString;
                return false;
            }
            triangulateFace(localPositions, corners, &localTriangles);
            continue;
        }
    }

    if (warn)
        *warn = warnString;
    if (err)
        *err = errString;
    positions->assign(localPositions.begin(), localPositions.end());
    triangles->assign(localTriangles.begin(), localTriangles.end());
    return true;
}

}
