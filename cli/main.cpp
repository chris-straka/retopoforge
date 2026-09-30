/*
 *  retopoforge - Qt-free command line frontend.
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

// Headless remeshing CLI. Same remeshing semantics as the Qt app's --input mode
// (MainWindow::runHeadless + QuadMeshGenerator::generate), without any Qt
// dependency: no QApplication, no event loop, works over ssh and in CI.

#include <AutoRemesher/AutoRemesher>
#include <AutoRemesher/Vector3>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#define RETOPO_VERSION "0.1.0"

struct Params {
    std::string inputPath;
    std::string outputPath;
    std::string reportPath;
    int targetQuads = 50000;
    double edgeScaling = 1.0;
    double sharpEdgeDegrees = 90.0;
    double smoothNormalDegrees = 0.0;
    double adaptivity = 1.0;
    double anisotropy = 1.0;
    AutoRemesher::ModelType modelType = AutoRemesher::ModelType::Organic;
};

static void printUsage(const char* argv0)
{
    std::cout << "Usage: " << argv0 << " --input file.obj --output output.obj [options]\n"
              << "\n"
              << "Options:\n"
              << "  -i, --input <file.obj>      Input .obj file to remesh (required)\n"
              << "  -o, --output <output.obj>   Output .obj file path (required)\n"
              << "  --report <report.txt>       Write a stats report file (optional)\n"
              << "  --target-quads <count>      Target quad count (default: 50000)\n"
              << "  --edge-scaling <factor>     Edge scaling factor (default: 1.0, range: 1.0-4.0)\n"
              << "  --sharp-edge <degrees>      Sharp edge dihedral angle threshold\n"
              << "                              (default: 90.0, range: 30.0-180.0)\n"
              << "  --smooth-normal <degrees>   Smooth normal angle threshold\n"
              << "                              (default: 0.0, range: 0.0-180.0)\n"
              << "  --adaptivity <value>        Curvature-adaptive quad density\n"
              << "                              (default: 1.0, range: 0.0-1.0)\n"
              << "  --anisotropy <value>        Curvature-adaptive quad elongation\n"
              << "                              (default: 1.0, range: 0.0-1.0)\n"
              << "  --model-type <organic|hardsurface>\n"
              << "                              Model type hint (default: organic)\n"
              << "  -h, --help                  Show this help\n"
              << "  -v, --version               Show version\n";
}

static bool takeValue(int argc, char** argv, int& i, const char* flag, std::string* out)
{
    const char* arg = argv[i];
    const char* eq = strchr(arg, '=');
    if (nullptr != eq) {
        *out = eq + 1;
        return true;
    }
    if (i + 1 >= argc) {
        std::cerr << "Error: " << flag << " requires a value" << std::endl;
        return false;
    }
    *out = argv[++i];
    return true;
}

static bool parseDouble(const std::string& text, const char* flag, double* out)
{
    char* end = nullptr;
    double value = strtod(text.c_str(), &end);
    if (nullptr == end || '\0' != *end) {
        std::cerr << "Error: " << flag << " expects a number, got '" << text << "'" << std::endl;
        return false;
    }
    *out = value;
    return true;
}

static bool parseInt(const std::string& text, const char* flag, int* out)
{
    char* end = nullptr;
    long value = strtol(text.c_str(), &end, 10);
    if (nullptr == end || '\0' != *end || value < 0) {
        std::cerr << "Error: " << flag << " expects a non-negative integer, got '" << text << "'" << std::endl;
        return false;
    }
    *out = (int)value;
    return true;
}

static bool matches(const char* arg, const char* longFlag, char shortFlag)
{
    if (0 == shortFlag)
        return 0 == strncmp(arg, longFlag, strlen(longFlag))
            && (arg[strlen(longFlag)] == '\0' || arg[strlen(longFlag)] == '=');
    char shortOpt[3] = { '-', shortFlag, '\0' };
    if (0 == strcmp(arg, shortOpt))
        return true;
    return 0 == strncmp(arg, longFlag, strlen(longFlag))
        && (arg[strlen(longFlag)] == '\0' || arg[strlen(longFlag)] == '=');
}

static bool parseArgs(int argc, char** argv, Params* params)
{
    for (int i = 1; i < argc; ++i) {
        const char* arg = argv[i];
        std::string value;
        if (matches(arg, "--help", 'h')) {
            printUsage(argv[0]);
            exit(0);
        } else if (matches(arg, "--version", 'v')) {
            std::cout << "retopoforge " << RETOPO_VERSION << std::endl;
            exit(0);
        } else if (matches(arg, "--input", 'i')) {
            if (!takeValue(argc, argv, i, "--input", &value))
                return false;
            params->inputPath = value;
        } else if (matches(arg, "--output", 'o')) {
            if (!takeValue(argc, argv, i, "--output", &value))
                return false;
            params->outputPath = value;
        } else if (matches(arg, "--report", '\0')) {
            if (!takeValue(argc, argv, i, "--report", &value))
                return false;
            params->reportPath = value;
        } else if (matches(arg, "--target-quads", '\0')) {
            if (!takeValue(argc, argv, i, "--target-quads", &value))
                return false;
            if (!parseInt(value, "--target-quads", &params->targetQuads))
                return false;
        } else if (matches(arg, "--edge-scaling", '\0')) {
            if (!takeValue(argc, argv, i, "--edge-scaling", &value))
                return false;
            if (!parseDouble(value, "--edge-scaling", &params->edgeScaling))
                return false;
        } else if (matches(arg, "--sharp-edge", '\0')) {
            if (!takeValue(argc, argv, i, "--sharp-edge", &value))
                return false;
            if (!parseDouble(value, "--sharp-edge", &params->sharpEdgeDegrees))
                return false;
        } else if (matches(arg, "--smooth-normal", '\0')) {
            if (!takeValue(argc, argv, i, "--smooth-normal", &value))
                return false;
            if (!parseDouble(value, "--smooth-normal", &params->smoothNormalDegrees))
                return false;
        } else if (matches(arg, "--adaptivity", '\0')) {
            if (!takeValue(argc, argv, i, "--adaptivity", &value))
                return false;
            if (!parseDouble(value, "--adaptivity", &params->adaptivity))
                return false;
        } else if (matches(arg, "--anisotropy", '\0')) {
            if (!takeValue(argc, argv, i, "--anisotropy", &value))
                return false;
            if (!parseDouble(value, "--anisotropy", &params->anisotropy))
                return false;
        } else if (matches(arg, "--model-type", '\0')) {
            if (!takeValue(argc, argv, i, "--model-type", &value))
                return false;
            if (value == "organic")
                params->modelType = AutoRemesher::ModelType::Organic;
            else if (value == "hardsurface" || value == "hard-surface" || value == "hard_surface")
                params->modelType = AutoRemesher::ModelType::HardSurface;
            else {
                std::cerr << "Error: --model-type expects 'organic' or 'hardsurface', got '" << value << "'" << std::endl;
                return false;
            }
        } else {
            std::cerr << "Error: unknown option '" << arg << "'" << std::endl;
            printUsage(argv[0]);
            return false;
        }
    }
    if (params->inputPath.empty() || params->outputPath.empty()) {
        std::cerr << "Error: --input and --output are required" << std::endl;
        printUsage(argv[0]);
        return false;
    }
    return true;
}

struct ProgressState {
    int lastPercent = -1;
    std::string lastStatus;
};

static void reportProgress(void* tag, float progress, const char* status)
{
    ProgressState* state = (ProgressState*)tag;
    int percent = (int)(progress * 100);
    std::string statusText = (nullptr != status ? status : "");
    // Reprint on a new step as well as a new percent: several steps are shorter
    // than one percent of the run and would otherwise never be named.
    if (percent == state->lastPercent && statusText == state->lastStatus)
        return;
    state->lastPercent = percent;
    state->lastStatus = statusText;
    if (statusText.empty())
        fprintf(stdout, "%d%% done.\n", percent);
    else
        fprintf(stdout, "%d%% done. %s\n", percent, statusText.c_str());
    fflush(stdout);
}

static bool loadObj(const std::string& filename,
    std::vector<AutoRemesher::Vector3>* vertices,
    std::vector<std::vector<size_t>>* triangles)
{
    tinyobj::attrib_t attributes;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    // Note: tinyobj triangulates by default, so indices stride by 3 below.
    bool loadSuccess = tinyobj::LoadObj(&attributes, &shapes, &materials, &warn, &err, filename.c_str());
    if (!warn.empty())
        std::cerr << "WARN: " << warn << std::endl;
    if (!err.empty())
        std::cerr << err << std::endl;
    if (!loadSuccess)
        return false;

    vertices->resize(attributes.vertices.size() / 3);
    for (size_t i = 0, j = 0; i < vertices->size(); ++i) {
        auto& dest = (*vertices)[i];
        dest.setX(attributes.vertices[j++]);
        dest.setY(attributes.vertices[j++]);
        dest.setZ(attributes.vertices[j++]);
    }

    triangles->clear();
    for (const auto& shape : shapes) {
        for (size_t i = 0; i + 2 < shape.mesh.indices.size(); i += 3) {
            triangles->push_back(std::vector<size_t> {
                (size_t)shape.mesh.indices[i + 0].vertex_index,
                (size_t)shape.mesh.indices[i + 1].vertex_index,
                (size_t)shape.mesh.indices[i + 2].vertex_index });
        }
    }
    return true;
}

static bool saveObj(const std::string& filename,
    const std::vector<AutoRemesher::Vector3>& vertices,
    const std::vector<std::vector<size_t>>& quads)
{
    std::ofstream file(filename.c_str(), std::ios::out | std::ios::trunc);
    if (!file.is_open())
        return false;
    file << "# retopoforge " << RETOPO_VERSION << "\n";
    file << "# https://github.com/chris-straka/retopoforge\n";
    for (const auto& v : vertices)
        file << "v " << v.x() << " " << v.y() << " " << v.z() << "\n";
    for (const auto& face : quads) {
        file << "f";
        for (size_t index : face)
            file << " " << (1 + index);
        file << "\n";
    }
    file.close();
    return !file.fail();
}

int main(int argc, char** argv)
{
    Params params;
    if (!parseArgs(argc, argv, &params))
        return 1;

    auto startTime = std::chrono::steady_clock::now();

    std::vector<AutoRemesher::Vector3> vertices;
    std::vector<std::vector<size_t>> triangles;
    if (!loadObj(params.inputPath, &vertices, &triangles)) {
        std::cerr << "Error: failed to load " << params.inputPath << std::endl;
        return 1;
    }
    std::cerr << "Loaded " << vertices.size() << " vertices, "
              << triangles.size() << " triangles" << std::endl;

    AutoRemesher::AutoRemesher remesher(vertices, triangles);
    // Same derivation as the Qt app: one quad ~= two triangles.
    remesher.setTargetTriangleCount((size_t)params.targetQuads * 2);
    if (params.edgeScaling > 0)
        remesher.setScaling(params.edgeScaling);
    remesher.setModelType(params.modelType);
    remesher.setGradientAdaptivity(params.adaptivity);
    remesher.setAnisotropy(params.anisotropy);
    remesher.setSharpEdgeDegrees(params.sharpEdgeDegrees);
    remesher.setSmoothNormalDegrees(params.smoothNormalDegrees);
    ProgressState progressState;
    remesher.setTag(&progressState);
    remesher.setProgressHandler(reportProgress);

    if (!remesher.remesh()) {
        std::cerr << "Error: remeshing produced no result" << std::endl;
        return 1;
    }

    for (const auto& line : remesher.phaseReport())
        std::cerr << "  " << line << std::endl;

    const auto& remeshedVertices = remesher.remeshedVertices();
    const auto& remeshedQuads = remesher.remeshedQuads();

    size_t quadCount = 0;
    size_t nonQuadCount = 0;
    for (const auto& face : remeshedQuads) {
        if (face.size() == 4)
            ++quadCount;
        else
            ++nonQuadCount;
    }

    if (!saveObj(params.outputPath, remeshedVertices, remeshedQuads)) {
        std::cerr << "Error: failed to write " << params.outputPath << std::endl;
        return 1;
    }

    auto endTime = std::chrono::steady_clock::now();
    double elapsedSeconds = std::chrono::duration<double>(endTime - startTime).count();

    std::cout << "=== retopoforge Report ===" << std::endl;
    std::cout << "Input: " << params.inputPath << std::endl;
    std::cout << "Output: " << params.outputPath << std::endl;
    std::cout << "Quads: " << quadCount << std::endl;
    std::cout << "Non-quads: " << nonQuadCount << std::endl;
    std::cout << "Vertices: " << remeshedVertices.size() << std::endl;
    std::cout << "Time: " << elapsedSeconds << " seconds" << std::endl;
    std::cout << "==========================" << std::endl;

    if (!params.reportPath.empty()) {
        std::ofstream report(params.reportPath.c_str(), std::ios::out | std::ios::trunc);
        if (!report.is_open()) {
            std::cerr << "Error: failed to write " << params.reportPath << std::endl;
            return 1;
        }
        report << "retopoforge Report\n";
        report << "==================\n\n";
        report << "Input file: " << params.inputPath << "\n";
        report << "Output file: " << params.outputPath << "\n";
        report << "Target quads: " << params.targetQuads << "\n";
        report << "Edge scaling: " << params.edgeScaling << "\n";
        report << "Sharp edge degrees: " << params.sharpEdgeDegrees << "\n";
        report << "Smooth normal degrees: " << params.smoothNormalDegrees << "\n";
        report << "Adaptivity: " << params.adaptivity << "\n";
        report << "Anisotropy: " << params.anisotropy << "\n";
        report << "Model type: " << (params.modelType == AutoRemesher::ModelType::Organic ? "organic" : "hardsurface") << "\n\n";
        report << "Results:\n";
        report << "  Quads: " << quadCount << "\n";
        report << "  Non-quads: " << nonQuadCount << "\n";
        report << "  Vertices: " << remeshedVertices.size() << "\n";
        report << "  Total time: " << elapsedSeconds << " seconds\n";
        report.close();
        if (report.fail()) {
            std::cerr << "Error: failed to write " << params.reportPath << std::endl;
            return 1;
        }
    }

    return 0;
}
