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
#include <AutoRemesher/ObjReader>
import retopo.core.mesh_separator;
import retopo.core.vector3;

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>
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
    bool quiet = false;
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
              << "  --quiet                     Silence progress and info output; only\n"
              << "                              warnings, errors and the report print\n"
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
        std::cerr << "Error: " << flag << " requires a value" << '\n';
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
        std::cerr << "Error: " << flag << " expects a number, got '" << text << "'" << '\n';
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
        std::cerr << "Error: " << flag << " expects a non-negative integer, got '" << text << "'" << '\n';
        return false;
    }
    *out = static_cast<int>(value);
    return true;
}

static bool matches(const char* arg, const char* longFlag, char shortFlag)
{
    if (0 != shortFlag) {
        char shortOpt[3] = { '-', shortFlag, '\0' };
        if (0 == strcmp(arg, shortOpt))
            return true;
    }
    const std::string_view view(arg);
    const std::string_view flag(longFlag);
    return view.starts_with(flag)
        && (view.size() == flag.size() || view[flag.size()] == '=');
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
            std::cout << "retopoforge " << RETOPO_VERSION << '\n';
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
        } else if (matches(arg, "--quiet", '\0')) {
            params->quiet = true;
        } else if (matches(arg, "--model-type", '\0')) {
            if (!takeValue(argc, argv, i, "--model-type", &value))
                return false;
            if (value == "organic")
                params->modelType = AutoRemesher::ModelType::Organic;
            else if (value == "hardsurface" || value == "hard-surface" || value == "hard_surface")
                params->modelType = AutoRemesher::ModelType::HardSurface;
            else {
                std::cerr << "Error: --model-type expects 'organic' or 'hardsurface', got '" << value << "'" << '\n';
                return false;
            }
        } else {
            std::cerr << "Error: unknown option '" << arg << "'" << '\n';
            printUsage(argv[0]);
            return false;
        }
    }
    if (params->inputPath.empty() || params->outputPath.empty()) {
        std::cerr << "Error: --input and --output are required" << '\n';
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
    ProgressState* state = static_cast<ProgressState*>(tag);
    int percent = static_cast<int>(progress * 100);
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
    std::vector<std::vector<size_t>>* triangles,
    size_t* preWeldVertices = nullptr,
    size_t* preWeldTriangles = nullptr)
{
    std::vector<float> positions;
    std::vector<std::vector<size_t>> loadedTriangles;
    std::string warn, err;

    // Note: the reader ear-clip triangulates polygons, so every face below
    // is a triangle.
    bool loadSuccess = AutoRemesher::loadObjPositionsAndTriangles(filename.c_str(), &positions, &loadedTriangles, &warn, &err);
    if (!warn.empty())
        std::cerr << "WARN: " << warn << '\n';
    if (!err.empty())
        std::cerr << err << '\n';
    if (!loadSuccess)
        return false;

    if (nullptr != preWeldVertices)
        *preWeldVertices = positions.size() / 3;
    if (nullptr != preWeldTriangles)
        *preWeldTriangles = loadedTriangles.size();
    // Weld-on-load: AI exporters emit non-indexed soup, which would split
    // into one island per triangle. Already-welded input is untouched.
    AutoRemesher::weldPositionsAndTriangles(&positions, &loadedTriangles);

    vertices->resize(positions.size() / 3);
    for (size_t i = 0, j = 0; i < vertices->size(); ++i) {
        auto& dest = (*vertices)[i];
        dest.setX(positions[j++]);
        dest.setY(positions[j++]);
        dest.setZ(positions[j++]);
    }

    triangles->assign(loadedTriangles.begin(), loadedTriangles.end());
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

// Count input islands with no output vertex near them. The engine merges
// per-island outputs without attribution, so a failed or skipped island is
// visible only as missing output geometry. A remeshed island stays in
// place, so any output vertex inside the island's slightly expanded
// bounding box proves the island produced output. Conservative by design:
// a failed island nested inside another island's box can hide, but a
// successful island always leaves vertices behind on its own surface.
static size_t countIslandsWithoutOutput(
    const std::vector<std::vector<std::vector<size_t>>>& islands,
    const std::vector<AutoRemesher::Vector3>& inputVertices,
    const std::vector<AutoRemesher::Vector3>& outputVertices)
{
    size_t failed = 0;
    for (const auto& island : islands) {
        bool first = true;
        double minX = 0.0, minY = 0.0, minZ = 0.0;
        double maxX = 0.0, maxY = 0.0, maxZ = 0.0;
        for (const auto& face : island) {
            for (const size_t index : face) {
                const auto& v = inputVertices[index];
                if (first) {
                    minX = maxX = v.x();
                    minY = maxY = v.y();
                    minZ = maxZ = v.z();
                    first = false;
                } else {
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
            }
        }
        if (first) {
            ++failed;
            continue;
        }
        const double dx = maxX - minX;
        const double dy = maxY - minY;
        const double dz = maxZ - minZ;
        const double pad = std::sqrt(dx * dx + dy * dy + dz * dz) * 0.01 + 1e-6;
        bool found = false;
        for (const auto& v : outputVertices) {
            if (v.x() >= minX - pad && v.x() <= maxX + pad
                && v.y() >= minY - pad && v.y() <= maxY + pad
                && v.z() >= minZ - pad && v.z() <= maxZ + pad) {
                found = true;
                break;
            }
        }
        if (!found)
            ++failed;
    }
    return failed;
}

int main(int argc, char** argv)
{
    Params params;
    if (!parseArgs(argc, argv, &params))
        return 1;

    auto startTime = std::chrono::steady_clock::now();

    std::vector<AutoRemesher::Vector3> vertices;
    std::vector<std::vector<size_t>> triangles;
    size_t preWeldVertices = 0;
    size_t preWeldTriangles = 0;
    if (!loadObj(params.inputPath, &vertices, &triangles, &preWeldVertices, &preWeldTriangles)) {
        std::cerr << "Error: failed to load " << params.inputPath << '\n';
        return 1;
    }
    if (!params.quiet) {
        std::cerr << "Loaded " << vertices.size() << " vertices, "
                  << triangles.size() << " triangles" << '\n';
        if (preWeldVertices != vertices.size() || preWeldTriangles != triangles.size()) {
            std::cerr << "Welded input: " << preWeldVertices << " -> " << vertices.size()
                      << " vertices, " << preWeldTriangles << " -> " << triangles.size()
                      << " triangles" << '\n';
        }
    }

    AutoRemesher::AutoRemesher remesher(vertices, triangles);
    // Same derivation as the Qt app: one quad ~= two triangles.
    remesher.setTargetTriangleCount(static_cast<size_t>(params.targetQuads) * 2);
    if (params.edgeScaling > 0)
        remesher.setScaling(params.edgeScaling);
    remesher.setModelType(params.modelType);
    remesher.setGradientAdaptivity(params.adaptivity);
    remesher.setAnisotropy(params.anisotropy);
    remesher.setSharpEdgeDegrees(params.sharpEdgeDegrees);
    remesher.setSmoothNormalDegrees(params.smoothNormalDegrees);
    ProgressState progressState;
    if (!params.quiet) {
        remesher.setTag(&progressState);
        remesher.setProgressHandler(reportProgress);
    }

    if (!remesher.remesh()) {
        std::cerr << "Error: remeshing produced no result" << '\n';
        return 1;
    }

    if (!params.quiet) {
        for (const auto& line : remesher.phaseReport())
            std::cerr << "  " << line << '\n';
    }

    const auto& remeshedVertices = remesher.remeshedVertices();
    const auto& remeshedQuads = remesher.remeshedQuads();

    // Island accounting on the welded input: same splitter the engine ran,
    // so the total matches its phase report. A failed island leaves no
    // output geometry behind; warn loudly but keep exit 0 on partial
    // success so pipelines still get the surviving output.
    std::vector<std::vector<std::vector<size_t>>> inputIslands;
    AutoRemesher::MeshSeparator::splitToIslands(triangles, inputIslands);
    const size_t failedIslands = countIslandsWithoutOutput(inputIslands, vertices, remeshedVertices);
    if (failedIslands > 0) {
        std::cerr << "Warning: " << failedIslands << " of " << inputIslands.size()
                  << " islands produced no output and were dropped from the mesh" << '\n';
    }

    size_t quadCount = 0;
    size_t nonQuadCount = 0;
    for (const auto& face : remeshedQuads) {
        if (face.size() == 4)
            ++quadCount;
        else
            ++nonQuadCount;
    }

    if (!saveObj(params.outputPath, remeshedVertices, remeshedQuads)) {
        std::cerr << "Error: failed to write " << params.outputPath << '\n';
        return 1;
    }

    auto endTime = std::chrono::steady_clock::now();
    double elapsedSeconds = std::chrono::duration<double>(endTime - startTime).count();

    std::cout << "=== retopoforge Report ===" << '\n';
    std::cout << "Input: " << params.inputPath << '\n';
    std::cout << "Output: " << params.outputPath << '\n';
    std::cout << "Islands: " << inputIslands.size() << '\n';
    std::cout << "Failed islands: " << failedIslands << '\n';
    std::cout << "Quads: " << quadCount << '\n';
    std::cout << "Non-quads: " << nonQuadCount << '\n';
    std::cout << "Vertices: " << remeshedVertices.size() << '\n';
    std::cout << "Time: " << elapsedSeconds << " seconds" << '\n';
    std::cout << "==========================" << '\n';

    if (!params.reportPath.empty()) {
        std::ofstream report(params.reportPath.c_str(), std::ios::out | std::ios::trunc);
        if (!report.is_open()) {
            std::cerr << "Error: failed to write " << params.reportPath << '\n';
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
        report << "  Islands: " << inputIslands.size() << "\n";
        report << "  Failed islands: " << failedIslands << "\n";
        report << "  Quads: " << quadCount << "\n";
        report << "  Non-quads: " << nonQuadCount << "\n";
        report << "  Vertices: " << remeshedVertices.size() << "\n";
        report << "  Total time: " << elapsedSeconds << " seconds\n";
        report.close();
        if (report.fail()) {
            std::cerr << "Error: failed to write " << params.reportPath << '\n';
            return 1;
        }
    }

    return 0;
}
