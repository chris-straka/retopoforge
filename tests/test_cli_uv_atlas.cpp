// CLI UV atlas test: multi-island --uvs on output packs island UV ranges
// into one shared 0..1 atlas instead of stacking every island over the full
// unit square. Asserts, on a deterministic two-component input (two fandisks
// side by side, built at runtime from the bench model):
//   - both output halves carry finite in-0..1 UVs whose bounding boxes are
//     disjoint with a gutter-sized margin (the pre-atlas engine stacked both
//     halves over the full [0,1]x[0,1]);
//   - geometry (v lines + face indices) is byte-identical to the --uvs off run.
// Plus a two-inline-cube run (UV validity + disjointness only: tiny meshes
// are run-to-run nondeterministic in this engine, so no cross-run geometry
// comparison there) and a single-island no-op check (fandisk --uvs on still
// spans exactly 0..1, proving the atlas is a no-op for one island).
// Plain assert-style main, no framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
//
// Paths arrive as compile definitions from tests/CMakeLists.txt:
//   RETOPO_BINARY       absolute path to the built retopo binary
//   RETOPO_MODEL_FANDISK absolute path to bench/models/fandisk.obj
#ifndef RETOPO_BINARY
#error "RETOPO_BINARY must be defined (absolute path to the built retopo binary)"
#endif
#ifndef RETOPO_MODEL_FANDISK
#error "RETOPO_MODEL_FANDISK must be defined (absolute path to bench/models/fandisk.obj)"
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <vector>
#include <unistd.h>

static int g_failures = 0;

#define CHECK(cond)                                                                                  \
    do {                                                                                             \
        if (!(cond)) {                                                                               \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                               \
            ++g_failures;                                                                            \
        }                                                                                            \
    } while (0)

namespace {

// Two unit cubes, side 2, the second offset by +5 in x (same triangulation
// as the symmetry test's cube, doubled). Tiny-mesh runs are nondeterministic
// run to run, so this input only checks UV validity + disjointness.
constexpr const char* kTwoCubesObj = R"(v -1 -1 -1
v 1 -1 -1
v 1 1 -1
v -1 1 -1
v -1 -1 1
v 1 -1 1
v 1 1 1
v -1 1 1
v 4 -1 -1
v 6 -1 -1
v 6 1 -1
v 4 1 -1
v 4 -1 1
v 6 -1 1
v 6 1 1
v 4 1 1
f 1 4 3
f 1 3 2
f 5 6 7
f 5 7 8
f 1 2 6
f 1 6 5
f 4 8 7
f 4 7 3
f 1 5 8
f 1 8 4
f 2 3 7
f 2 7 6
f 9 12 11
f 9 11 10
f 13 14 15
f 13 15 16
f 9 10 14
f 9 14 13
f 12 16 15
f 12 15 11
f 9 13 16
f 9 16 12
f 10 11 15
f 10 15 14
)";

struct RunResult {
    int exitCode = -1;
};

int runQuiet(const std::string& binary, const std::string& args)
{
    const int status = std::system(("\"" + binary + "\" " + args + " > /dev/null 2>&1").c_str());
    if (WIFEXITED(status))
        return WEXITSTATUS(status);
    return -1;
}

struct ObjMesh {
    bool loaded = false;
    std::vector<std::string> vLines;
    std::vector<std::string> vtLines;
    std::vector<std::string> fLines;
    std::vector<double> vx; // x of each v line, for the spatial island split
    std::vector<double> uvU;
    std::vector<double> uvV;
};

ObjMesh readObj(const std::string& path)
{
    ObjMesh mesh;
    std::ifstream in(path);
    if (!in.is_open())
        return mesh;
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("vt ", 0) == 0) {
            mesh.vtLines.push_back(line);
            double u = NAN, v = NAN;
            if (2 == std::sscanf(line.c_str() + 3, "%lf %lf", &u, &v)) {
                mesh.uvU.push_back(u);
                mesh.uvV.push_back(v);
            }
        } else if (line.rfind("v ", 0) == 0) {
            mesh.vLines.push_back(line);
            double x = NAN, y = NAN, z = NAN;
            if (3 == std::sscanf(line.c_str() + 2, "%lf %lf %lf", &x, &y, &z))
                mesh.vx.push_back(x);
        } else if (line.rfind("f ", 0) == 0) {
            mesh.fLines.push_back(line);
        }
    }
    mesh.loaded = true;
    return mesh;
}

// Strip the "/vt" suffix from every corner: "f 1/1 2/2" -> "f 1 2".
bool stripVtSuffixes(const std::string& faceLine, std::string* stripped)
{
    if (faceLine.rfind("f ", 0) != 0)
        return false;
    std::istringstream corners(faceLine.substr(2));
    std::string corner;
    std::ostringstream out;
    out << "f";
    while (corners >> corner) {
        const size_t slash = corner.find('/');
        if (slash == std::string::npos)
            out << " " << corner;
        else
            out << " " << corner.substr(0, slash);
    }
    *stripped = out.str();
    return true;
}

struct UvBox {
    double minU = 1.0, minV = 1.0, maxU = 0.0, maxV = 0.0;
};

// Split vertices into the two spatial halves at the largest x gap (the two
// input components sit far apart, so output halves separate cleanly even if
// one island's output fragments). Returns false without a clear gap.
bool splitHalves(const ObjMesh& mesh, std::vector<char>* inRight)
{
    if (mesh.vx.empty())
        return false;
    std::vector<double> sorted = mesh.vx;
    std::sort(sorted.begin(), sorted.end());
    size_t gapAt = 0;
    double gap = 0.0;
    for (size_t i = 1; i < sorted.size(); ++i) {
        const double d = sorted[i] - sorted[i - 1];
        if (d > gap) {
            gap = d;
            gapAt = i;
        }
    }
    if (!(gap > 1.0))
        return false;
    const double cut = 0.5 * (sorted[gapAt - 1] + sorted[gapAt]);
    inRight->assign(mesh.vx.size(), 0);
    size_t left = 0, right = 0;
    for (size_t i = 0; i < mesh.vx.size(); ++i) {
        if (mesh.vx[i] > cut) {
            (*inRight)[i] = 1;
            ++right;
        } else {
            ++left;
        }
    }
    return left >= 10 && right >= 10;
}

UvBox boxOf(const ObjMesh& mesh, const std::vector<char>& inRight, bool wantRight)
{
    UvBox box;
    bool first = true;
    for (size_t i = 0; i < mesh.uvU.size() && i < inRight.size(); ++i) {
        if ((inRight[i] != 0) != wantRight)
            continue;
        const double u = mesh.uvU[i], v = mesh.uvV[i];
        if (!std::isfinite(u) || !std::isfinite(v))
            continue;
        if (first) {
            box.minU = box.maxU = u;
            box.minV = box.maxV = v;
            first = false;
        } else {
            box.minU = std::min(box.minU, u);
            box.maxU = std::max(box.maxU, u);
            box.minV = std::min(box.minV, v);
            box.maxV = std::max(box.maxV, v);
        }
    }
    return box;
}

// Widest separation between two boxes: positive when disjoint. The atlas
// gutter is 2 texels at 1k (~1.95e-3); the 1e-3 bar leaves room for the
// CLI's 6-significant-digit vt printing.
double boxGap(const UvBox& a, const UvBox& b)
{
    const double gapU = std::max(a.minU - b.maxU, b.minU - a.maxU);
    const double gapV = std::max(a.minV - b.maxV, b.minV - a.maxV);
    return std::max(gapU, gapV);
}

} // namespace

int main()
{
    const std::string binary = RETOPO_BINARY;
    const std::string fandisk = RETOPO_MODEL_FANDISK;

    if (!std::filesystem::exists(binary)) {
        std::printf("FAIL %s:%d: retopo binary not found: %s\n", __FILE__, __LINE__, binary.c_str());
        return 1;
    }
    if (!std::filesystem::exists(fandisk)) {
        std::printf("FAIL %s:%d: model not found: %s\n", __FILE__, __LINE__, fandisk.c_str());
        return 1;
    }

    std::error_code ec;
    // PID-suffixed: parallel lanes run the same suite from sibling checkouts,
    // and a fixed tmpdir name lets them clobber each other's files (observed
    // test_cli_glb flake: two writers racing one fixed path).
    const std::filesystem::path tmpdir = std::filesystem::temp_directory_path()
        / ("retopo_cli_uv_atlas_" + std::to_string(::getpid()));
    std::filesystem::remove_all(tmpdir, ec);
    std::filesystem::create_directories(tmpdir, ec);

    // (a) Two fandisks side by side, built at runtime: the deterministic
    // multi-component input (two tiny cubes are NOT run-to-run
    // deterministic in this engine, so they cannot carry the geometry
    // comparison below).
    const std::string twoPath = (tmpdir / "two_fandisk.obj").string();
    {
        std::vector<std::string> vLines, fLines;
        double minX = 0.0, maxX = 0.0;
        bool first = true;
        std::ifstream in(fandisk);
        CHECK(in.is_open());
        std::string line;
        while (std::getline(in, line)) {
            if (line.rfind("v ", 0) == 0) {
                vLines.push_back(line);
                double x = 0.0, y = 0.0, z = 0.0;
                if (3 == std::sscanf(line.c_str() + 2, "%lf %lf %lf", &x, &y, &z)) {
                    if (first) {
                        minX = maxX = x;
                        first = false;
                    } else {
                        minX = std::min(minX, x);
                        maxX = std::max(maxX, x);
                    }
                }
            } else if (line.rfind("f ", 0) == 0) {
                fLines.push_back(line);
            }
        }
        CHECK(!vLines.empty());
        CHECK(!fLines.empty());
        const double dx = (maxX - minX) + 2.0;
        std::ofstream out(twoPath);
        CHECK(out.is_open());
        for (int copy = 0; copy < 2; ++copy) {
            for (const auto& v : vLines) {
                double x = 0.0, y = 0.0, z = 0.0;
                if (3 == std::sscanf(v.c_str() + 2, "%lf %lf %lf", &x, &y, &z))
                    out << "v " << x + copy * dx << " " << y << " " << z << "\n";
            }
        }
        const long n = static_cast<long>(vLines.size());
        for (int copy = 0; copy < 2; ++copy) {
            for (const auto& f : fLines) {
                std::istringstream corners(f.substr(2));
                std::string corner;
                out << "f";
                while (corners >> corner) {
                    const long v = std::strtol(corner.c_str(), nullptr, 10);
                    out << " " << (v + copy * n);
                }
                out << "\n";
            }
        }
    }

    // (b) Atlas layout: both halves carry finite in-0..1 UVs and their
    // boxes no longer overlap.
    const std::string atlasObj = (tmpdir / "atlas.obj").string();
    {
        const int code = runQuiet(binary,
            "--input \"" + twoPath + "\" --output \"" + atlasObj
                + "\" --target-quads 400 --uvs on");
        CHECK(code == 0);
        const ObjMesh mesh = readObj(atlasObj);
        CHECK(mesh.loaded);
        CHECK(!mesh.vLines.empty());
        CHECK(mesh.vtLines.size() == mesh.vLines.size());
        CHECK(mesh.uvU.size() == mesh.vLines.size());
        for (size_t i = 0; i < mesh.uvU.size(); ++i) {
            CHECK(std::isfinite(mesh.uvU[i]));
            CHECK(std::isfinite(mesh.uvV[i]));
            CHECK(mesh.uvU[i] >= -1e-9 && mesh.uvU[i] <= 1.0 + 1e-9);
            CHECK(mesh.uvV[i] >= -1e-9 && mesh.uvV[i] <= 1.0 + 1e-9);
        }
        std::vector<char> inRight;
        CHECK(splitHalves(mesh, &inRight));
        if (!inRight.empty()) {
            const UvBox left = boxOf(mesh, inRight, false);
            const UvBox right = boxOf(mesh, inRight, true);
            // Neither half still spans the whole axis both ways (pre-atlas
            // behavior: both boxes were exactly [0,1]x[0,1]).
            CHECK((left.maxU - left.minU) < 0.999 || (left.maxV - left.minV) < 0.999);
            CHECK((right.maxU - right.minU) < 0.999 || (right.maxV - right.minV) < 0.999);
            CHECK(boxGap(left, right) >= 1e-3);
        }
    }

    // (c) Geometry unchanged vs --uvs off on the same deterministic input.
    {
        const std::string offObj = (tmpdir / "atlas_off.obj").string();
        const int code = runQuiet(binary,
            "--input \"" + twoPath + "\" --output \"" + offObj
                + "\" --target-quads 400 --uvs off");
        CHECK(code == 0);
        const ObjMesh on = readObj(atlasObj);
        const ObjMesh off = readObj(offObj);
        CHECK(on.loaded && off.loaded);
        CHECK(on.vLines == off.vLines);
        CHECK(on.fLines.size() == off.fLines.size());
        for (size_t i = 0; i < on.fLines.size() && i < off.fLines.size(); ++i) {
            std::string stripped;
            CHECK(stripVtSuffixes(on.fLines[i], &stripped));
            CHECK(stripped == off.fLines[i]);
        }
    }

    // (d) Two inline cubes: UV validity + disjointness only (tiny-mesh
    // outputs vary run to run, so no cross-run geometry comparison).
    {
        const std::string cubesIn = (tmpdir / "two_cubes.obj").string();
        const std::string cubesOut = (tmpdir / "cubes_uv.obj").string();
        std::ofstream out(cubesIn);
        CHECK(out.is_open());
        out << kTwoCubesObj;
        out.close();
        const int code = runQuiet(binary,
            "--input \"" + cubesIn + "\" --output \"" + cubesOut
                + "\" --target-quads 200 --uvs on");
        CHECK(code == 0);
        const ObjMesh mesh = readObj(cubesOut);
        CHECK(mesh.loaded);
        CHECK(mesh.vtLines.size() == mesh.vLines.size());
        for (size_t i = 0; i < mesh.uvU.size(); ++i) {
            CHECK(std::isfinite(mesh.uvU[i]));
            CHECK(std::isfinite(mesh.uvV[i]));
            CHECK(mesh.uvU[i] >= -1e-9 && mesh.uvU[i] <= 1.0 + 1e-9);
            CHECK(mesh.uvV[i] >= -1e-9 && mesh.uvV[i] <= 1.0 + 1e-9);
        }
        std::vector<char> inRight;
        CHECK(splitHalves(mesh, &inRight));
        if (!inRight.empty())
            CHECK(boxGap(boxOf(mesh, inRight, false), boxOf(mesh, inRight, true)) >= 1e-3);
    }

    // (e) Single-island no-op: fandisk --uvs on still spans exactly 0..1.
    {
        const std::string singleOut = (tmpdir / "single_uv.obj").string();
        const int code = runQuiet(binary,
            "--input \"" + fandisk + "\" --output \"" + singleOut
                + "\" --target-quads 200 --uvs on");
        CHECK(code == 0);
        const ObjMesh mesh = readObj(singleOut);
        CHECK(mesh.loaded);
        CHECK(mesh.vtLines.size() == mesh.vLines.size());
        double minU = 1.0, minV = 1.0, maxU = 0.0, maxV = 0.0;
        for (size_t i = 0; i < mesh.uvU.size(); ++i) {
            minU = std::min(minU, mesh.uvU[i]);
            minV = std::min(minV, mesh.uvV[i]);
            maxU = std::max(maxU, mesh.uvU[i]);
            maxV = std::max(maxV, mesh.uvV[i]);
        }
        CHECK(std::fabs(minU - 0.0) < 1e-9);
        CHECK(std::fabs(minV - 0.0) < 1e-9);
        CHECK(std::fabs(maxU - 1.0) < 1e-9);
        CHECK(std::fabs(maxV - 1.0) < 1e-9);
    }

    std::filesystem::remove_all(tmpdir, ec);

    if (g_failures == 0)
        std::printf("PASS test_cli_uv_atlas\n");
    return g_failures == 0 ? 0 : 1;
}
