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
#include "glb.h"
import retopo.core.mesh_separator;
import retopo.core.vector3;

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
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
    std::vector<int> lodTargets;
    int targetQuads = 50000;
    double edgeScaling = 1.0;
    double sharpEdgeDegrees = 90.0;
    double smoothNormalDegrees = 0.0;
    double adaptivity = 1.0;
    double anisotropy = 1.0;
    AutoRemesher::ModelType modelType = AutoRemesher::ModelType::Organic;
    bool symmetryEnabled = false;
    int symmetryAxis = -1; // -1 = auto-detect, 0/1/2 = X/Y/Z
    std::string guidesPath;
    bool quiet = false;
};

static void printUsage(const char* argv0)
{
    std::cout << "Usage: " << argv0 << " --input <file.obj|file.glb|dir> --output <output.obj|output.glb|dir> [options]\n"
              << "\n"
              << "Options:\n"
              << "  -i, --input <file|dir>      Input .obj or .glb file to remesh (required).\n"
              << "                              A directory remeshes every .obj and .glb\n"
              << "                              in it (non-recursive); --output is then a\n"
              << "                              directory (created if missing) and each\n"
              << "                              input foo.ext is written as <dir>/foo.ext.\n"
              << "  -o, --output <path>         Output file path (required): .obj writes\n"
              << "                              OBJ, .glb writes GLB (quads triangulated).\n"
              << "                              A directory in batch mode (see --input).\n"
              << "  --report <report.txt>       Write a stats report file (optional)\n"
              << "  --target-quads <count>      Target quad count (default: 50000,\n"
              << "                              ignored when --lods is given)\n"
              << "  --lods <q0,q1,...>          Emit a full LOD chain in one run, e.g.\n"
              << "                              --lods 10000,5000,2000 writes\n"
              << "                              <stem>_lod0.<ext>, <stem>_lod1.<ext>, ...\n"
              << "                              next to --output, keeping its extension\n"
              << "                              (overrides --target-quads)\n"
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
              << "  --symmetry <off|auto|x|y|z>  Mirror-symmetry constraints\n"
              << "                              (default: off). auto detects the dominant\n"
              << "                              plane; x/y/z pin it. Falls back to\n"
              << "                              unconstrained output when the input scores\n"
              << "                              below threshold on the chosen plane\n"
              << "  --guides <file>             Guide-curve constraints: quad edge flow\n"
              << "                              follows the polylines (eye/mouth loops).\n"
              << "                              File format: one 'x y z' point per line,\n"
              << "                              blank lines separate polylines, '#' starts\n"
              << "                              a comment. Points live in input-mesh\n"
              << "                              coordinates. Single-file and --lods runs\n"
              << "                              only (rejected in batch mode)\n"
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

static bool parseLods(const std::string& text, std::vector<int>* out)
{
    out->clear();
    size_t start = 0;
    while (start <= text.size()) {
        size_t end = text.find(',', start);
        if (end == std::string::npos)
            end = text.size();
        std::string token = text.substr(start, end - start);
        // Trim surrounding whitespace so "10000, 5000" works.
        const size_t first = token.find_first_not_of(" \t");
        const size_t last = token.find_last_not_of(" \t");
        token = (first == std::string::npos) ? "" : token.substr(first, last - first + 1);
        int value = 0;
        if (!parseInt(token, "--lods", &value))
            return false;
        if (value <= 0) {
            std::cerr << "Error: --lods expects positive integers, got '" << token << "'" << '\n';
            return false;
        }
        out->push_back(value);
        start = end + 1;
    }
    if (out->empty()) {
        std::cerr << "Error: --lods expects a comma-separated list, got '" << text << "'" << '\n';
        return false;
    }
    return true;
}

// Guide file: one "x y z" point per line, blank lines separate polylines,
// "#" starts a comment. Single-point chains are dropped (the engine
// ignores them); a file with no usable polyline is an error, since an
// explicitly passed --guides that silently does nothing hides mistakes.
static bool parseGuidesFile(const std::string& path,
    std::vector<std::vector<AutoRemesher::Vector3>>* guides)
{
    guides->clear();
    std::ifstream in(path);
    if (!in.is_open()) {
        std::cerr << "Error: cannot open --guides file '" << path << "'" << '\n';
        return false;
    }
    std::vector<AutoRemesher::Vector3> current;
    auto flushCurrent = [&]() {
        if (current.size() >= 2)
            guides->push_back(current);
        current.clear();
    };
    std::string line;
    long lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;
        const size_t hash = line.find('#');
        if (hash != std::string::npos)
            line.erase(hash);
        const size_t first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos) {
            flushCurrent();
            continue;
        }
        double x = 0.0, y = 0.0, z = 0.0;
        int endPos = 0;
        if (3 != std::sscanf(line.c_str() + first, "%lf %lf %lf %n", &x, &y, &z, &endPos)
            || line.find_first_not_of(" \t\r", first + static_cast<size_t>(endPos))
                != std::string::npos) {
            std::cerr << "Error: --guides file '" << path << "' line " << lineNo
                      << " expects 'x y z', got '" << line << "'" << '\n';
            return false;
        }
        AutoRemesher::Vector3 point;
        point.setX(x);
        point.setY(y);
        point.setZ(z);
        current.push_back(point);
    }
    flushCurrent();
    if (guides->empty()) {
        std::cerr << "Error: --guides file '" << path << "' holds no usable polyline"
                  << " (need 2+ points per polyline)" << '\n';
        return false;
    }
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
        } else if (matches(arg, "--lods", '\0')) {
            if (!takeValue(argc, argv, i, "--lods", &value))
                return false;
            if (!parseLods(value, &params->lodTargets))
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
        } else if (matches(arg, "--symmetry", '\0')) {
            if (!takeValue(argc, argv, i, "--symmetry", &value))
                return false;
            if (value == "off") {
                params->symmetryEnabled = false;
                params->symmetryAxis = -1;
            } else if (value == "auto") {
                params->symmetryEnabled = true;
                params->symmetryAxis = -1;
            } else if (value == "x" || value == "X") {
                params->symmetryEnabled = true;
                params->symmetryAxis = 0;
            } else if (value == "y" || value == "Y") {
                params->symmetryEnabled = true;
                params->symmetryAxis = 1;
            } else if (value == "z" || value == "Z") {
                params->symmetryEnabled = true;
                params->symmetryAxis = 2;
            } else {
                std::cerr << "Error: --symmetry expects 'off', 'auto', 'x', 'y' or 'z', got '"
                          << value << "'" << '\n';
                return false;
            }
        } else if (matches(arg, "--guides", '\0')) {
            if (!takeValue(argc, argv, i, "--guides", &value))
                return false;
            params->guidesPath = value;
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

// GLB twin of loadObj: identical weld + Vector3 conversion, only the parser
// differs (cgltf via GlbIo). loadObj above is intentionally untouched so the
// OBJ path keeps byte-identical stdout/stderr and counts.
static bool loadGlb(const std::string& filename,
    std::vector<AutoRemesher::Vector3>* vertices,
    std::vector<std::vector<size_t>>* triangles,
    size_t* preWeldVertices = nullptr,
    size_t* preWeldTriangles = nullptr)
{
    std::vector<float> positions;
    std::vector<std::vector<size_t>> loadedTriangles;
    std::string warn, err;

    bool loadSuccess = GlbIo::loadGlbPositionsAndTriangles(filename.c_str(), &positions, &loadedTriangles, &warn, &err);
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
    // Weld-on-load: same as OBJ; GLB exporters also emit non-indexed soup.
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

static bool loadMesh(const std::string& filename,
    std::vector<AutoRemesher::Vector3>* vertices,
    std::vector<std::vector<size_t>>* triangles,
    size_t* preWeldVertices = nullptr,
    size_t* preWeldTriangles = nullptr)
{
    if (GlbIo::hasGlbExtension(filename))
        return loadGlb(filename, vertices, triangles, preWeldVertices, preWeldTriangles);
    return loadObj(filename, vertices, triangles, preWeldVertices, preWeldTriangles);
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

static bool saveMesh(const std::string& filename,
    const std::vector<AutoRemesher::Vector3>& vertices,
    const std::vector<std::vector<size_t>>& quads)
{
    // An empty result is a failure in every format: writing a 0-vertex file
    // that claims success is exactly the silent-failure class the island
    // accounting exists to kill. Fail here so OBJ and GLB agree.
    if (vertices.empty())
        return false;
    if (GlbIo::hasGlbExtension(filename)) {
        std::string generator = std::string("retopoforge ") + RETOPO_VERSION;
        return GlbIo::saveGlb(filename.c_str(), generator.c_str(), vertices, quads);
    }
    return saveObj(filename, vertices, quads);
}

static std::string lodOutputPath(const std::string& baseOutput, size_t lodIndex)
{
    const std::filesystem::path base(baseOutput);
    std::string ext = base.extension().string();
    if (ext.empty())
        ext = ".obj";
    const std::string name = base.stem().string() + "_lod" + std::to_string(lodIndex) + ext;
    return (base.parent_path() / name).string();
}

struct RungResult {
    bool ok = false;
    size_t quadCount = 0;
    size_t nonQuadCount = 0;
    size_t vertexCount = 0;
    double elapsedSeconds = 0.0;
    std::string error;
};

// Same remesher setup as single-file mode, parameterized by target count.
// Used by --lods and batch runs; single-file mode keeps its inline copy so
// its stdout/stderr bytes stay exactly as before.
static RungResult remeshLoadedMesh(const Params& params,
    const std::vector<AutoRemesher::Vector3>& vertices,
    const std::vector<std::vector<size_t>>& triangles,
    const std::vector<std::vector<AutoRemesher::Vector3>>& guides,
    int targetQuads,
    const std::string& outputPath)
{
    RungResult result;
    auto startTime = std::chrono::steady_clock::now();

    AutoRemesher::AutoRemesher remesher(vertices, triangles);
    // Same derivation as the Qt app: one quad ~= two triangles.
    remesher.setTargetTriangleCount(static_cast<size_t>(targetQuads) * 2);
    remesher.setSymmetryEnabled(params.symmetryEnabled);
    remesher.setSymmetryPlane(params.symmetryAxis);
    remesher.setGuidePolylines(guides);
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
        result.error = "remeshing produced no result";
        return result;
    }

    if (!params.quiet) {
        for (const auto& line : remesher.phaseReport())
            std::cerr << "  " << line << '\n';
    }

    const auto& remeshedVertices = remesher.remeshedVertices();
    const auto& remeshedQuads = remesher.remeshedQuads();

    for (const auto& face : remeshedQuads) {
        if (face.size() == 4)
            ++result.quadCount;
        else
            ++result.nonQuadCount;
    }
    result.vertexCount = remeshedVertices.size();

    if (!saveMesh(outputPath, remeshedVertices, remeshedQuads)) {
        result.error = "failed to write " + outputPath;
        return result;
    }

    auto endTime = std::chrono::steady_clock::now();
    result.elapsedSeconds = std::chrono::duration<double>(endTime - startTime).count();
    result.ok = true;
    return result;
}

static void printRungLine(const std::string& label, const std::string& outputPath,
    int targetQuads, const RungResult& result)
{
    std::cout << label << "target-quads=" << targetQuads
              << " output=" << outputPath
              << " quads=" << result.quadCount
              << " non-quads=" << result.nonQuadCount
              << " vertices=" << result.vertexCount
              << " time=" << result.elapsedSeconds << " seconds" << '\n';
}

static int runMultiMode(const Params& params, bool batch)
{
    // Guides live in input-mesh coordinates, so one file cannot span a
    // batch of different meshes; --lods over a single mesh is fine.
    if (batch && !params.guidesPath.empty()) {
        std::cerr << "Error: --guides needs a single input mesh, not a batch directory" << '\n';
        return 1;
    }
    std::vector<std::vector<AutoRemesher::Vector3>> guides;
    if (!params.guidesPath.empty() && !parseGuidesFile(params.guidesPath, &guides))
        return 1;
    std::vector<std::string> inputs;
    if (batch) {
        std::error_code ec;
        std::filesystem::directory_iterator it(params.inputPath, ec);
        if (ec) {
            std::cerr << "Error: cannot read directory " << params.inputPath << '\n';
            return 1;
        }
        for (; it != std::filesystem::directory_iterator(); it.increment(ec)) {
            if (ec) {
                std::cerr << "Error: cannot read directory " << params.inputPath << '\n';
                return 1;
            }
            std::error_code fileEc;
            if (!it->is_regular_file(fileEc) || fileEc)
                continue;
            if (GlbIo::isSupportedInputExtension(it->path().string()))
                inputs.push_back(it->path().string());
        }
        std::sort(inputs.begin(), inputs.end());
        if (inputs.empty()) {
            std::cerr << "Error: no .obj/.glb files in " << params.inputPath << '\n';
            return 1;
        }
        if (std::filesystem::exists(params.outputPath, ec) && !std::filesystem::is_directory(params.outputPath, ec)) {
            std::cerr << "Error: --output must be a directory when --input is a directory" << '\n';
            return 1;
        }
        std::filesystem::create_directories(params.outputPath, ec);
        if (ec) {
            std::cerr << "Error: cannot create output directory " << params.outputPath << '\n';
            return 1;
        }
    } else {
        inputs.push_back(params.inputPath);
    }

    const std::vector<int> targets = params.lodTargets.empty()
        ? std::vector<int>{ params.targetQuads }
        : params.lodTargets;
    const bool lodMode = !params.lodTargets.empty();

    std::ofstream report;
    if (!params.reportPath.empty()) {
        report.open(params.reportPath.c_str(), std::ios::out | std::ios::trunc);
        if (!report.is_open()) {
            std::cerr << "Error: failed to write " << params.reportPath << '\n';
            return 1;
        }
        report << "retopoforge Report\n";
        report << "==================\n\n";
        report << "Edge scaling: " << params.edgeScaling << "\n";
        report << "Sharp edge degrees: " << params.sharpEdgeDegrees << "\n";
        report << "Smooth normal degrees: " << params.smoothNormalDegrees << "\n";
        report << "Adaptivity: " << params.adaptivity << "\n";
        report << "Anisotropy: " << params.anisotropy << "\n";
        report << "Model type: " << (params.modelType == AutoRemesher::ModelType::Organic ? "organic" : "hardsurface") << "\n\n";
    }

    std::vector<std::string> failedFiles;
    auto noteFailed = [&](const std::string& name) {
        if (std::find(failedFiles.begin(), failedFiles.end(), name) == failedFiles.end())
            failedFiles.push_back(name);
    };

    for (const std::string& inputPath : inputs) {
        const std::string fileLabel = batch
            ? "FILE " + std::filesystem::path(inputPath).filename().string() + (lodMode ? " " : ": ")
            : "";
        const std::string failName = batch
            ? std::filesystem::path(inputPath).filename().string()
            : inputPath;

        std::vector<AutoRemesher::Vector3> vertices;
        std::vector<std::vector<size_t>> triangles;
        if (!loadMesh(inputPath, &vertices, &triangles)) {
            std::cerr << "Error: failed to load " << inputPath << '\n';
            std::cout << fileLabel << "FAILED to load " << inputPath << '\n';
            noteFailed(failName);
            continue;
        }
        if (!params.quiet) {
            std::cerr << "Loaded " << vertices.size() << " vertices, "
                      << triangles.size() << " triangles" << '\n';
        }

        for (size_t rung = 0; rung < targets.size(); ++rung) {
            std::string outputPath;
            if (batch) {
                const std::filesystem::path inFile(inputPath);
                // LOD outputs keep the input's extension (.obj stays .obj,
                // .glb stays .glb); plain batch already copies the filename.
                std::string lodExt = inFile.extension().string();
                if (lodExt.empty())
                    lodExt = ".obj";
                if (lodMode)
                    outputPath = (std::filesystem::path(params.outputPath) / (inFile.stem().string() + "_lod" + std::to_string(rung) + lodExt)).string();
                else
                    outputPath = (std::filesystem::path(params.outputPath) / inFile.filename()).string();
            } else {
                outputPath = lodOutputPath(params.outputPath, rung);
            }
            std::string label = fileLabel;
            if (lodMode)
                label += "LOD " + std::to_string(rung) + ": ";

            RungResult result = remeshLoadedMesh(params, vertices, triangles, guides, targets[rung], outputPath);
            if (!result.ok) {
                std::cerr << "Error: " << result.error << " (" << outputPath << ")" << '\n';
                std::cout << label << "FAILED " << result.error << '\n';
                noteFailed(failName);
                continue;
            }
            printRungLine(label, outputPath, targets[rung], result);
            if (report.is_open()) {
                report << "Input file: " << inputPath << "\n";
                report << "Output file: " << outputPath << "\n";
                report << "Target quads: " << targets[rung] << "\n";
                report << "Results:\n";
                report << "  Quads: " << result.quadCount << "\n";
                report << "  Non-quads: " << result.nonQuadCount << "\n";
                report << "  Vertices: " << result.vertexCount << "\n";
                report << "  Total time: " << result.elapsedSeconds << " seconds\n\n";
            }
        }
    }

    if (report.is_open()) {
        report.close();
        if (report.fail()) {
            std::cerr << "Error: failed to write " << params.reportPath << '\n';
            return 1;
        }
    }

    if (batch) {
        if (failedFiles.empty()) {
            std::cout << "Failed files: none" << '\n';
        } else {
            std::cout << "Failed files (" << failedFiles.size() << "):";
            for (const auto& name : failedFiles)
                std::cout << " " << name;
            std::cout << '\n';
        }
    }
    return failedFiles.empty() ? 0 : 1;
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

    std::error_code dirEc;
    const bool batch = std::filesystem::is_directory(params.inputPath, dirEc);
    if (!params.lodTargets.empty() || batch)
        return runMultiMode(params, batch);

    auto startTime = std::chrono::steady_clock::now();

    std::vector<AutoRemesher::Vector3> vertices;
    std::vector<std::vector<size_t>> triangles;
    size_t preWeldVertices = 0;
    size_t preWeldTriangles = 0;
    if (!loadMesh(params.inputPath, &vertices, &triangles, &preWeldVertices, &preWeldTriangles)) {
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

    std::vector<std::vector<AutoRemesher::Vector3>> guides;
    if (!params.guidesPath.empty()) {
        if (!parseGuidesFile(params.guidesPath, &guides))
            return 1;
        if (!params.quiet)
            std::cerr << "Guide polylines: " << guides.size() << '\n';
    }

    AutoRemesher::AutoRemesher remesher(vertices, triangles);
    // Same derivation as the Qt app: one quad ~= two triangles.
    remesher.setTargetTriangleCount(static_cast<size_t>(params.targetQuads) * 2);
    remesher.setSymmetryEnabled(params.symmetryEnabled);
    remesher.setSymmetryPlane(params.symmetryAxis);
    remesher.setGuidePolylines(guides);
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

    if (!saveMesh(params.outputPath, remeshedVertices, remeshedQuads)) {
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
