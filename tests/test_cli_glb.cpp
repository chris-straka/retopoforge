// CLI GLB test: .glb input/output round-trips plus batch/LOD/error paths.
// Deliberately asserts SHAPE, not exact engine counts (counts move with
// engine work; exact pins live in test_cli_roundtrip/test_cli_multimode):
// writer output must re-parse with accessor counts matching the source
// mesh, batch/LOD must emit .glb files, truncated input must exit 1.
// Plain assert-style main, no framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
//
// Paths arrive as compile definitions from tests/CMakeLists.txt:
//   RETOPO_BINARY      absolute path to the built retopo binary
//   RETOPO_MODEL_TETRA absolute path to tests/fixtures/tetra.glb
#ifndef RETOPO_BINARY
#error "RETOPO_BINARY must be defined (absolute path to the built retopo binary)"
#endif
#ifndef RETOPO_MODEL_TETRA
#error "RETOPO_MODEL_TETRA must be defined (absolute path to tests/fixtures/tetra.glb)"
#endif
#ifndef RETOPO_MODEL_FANDISK
#error "RETOPO_MODEL_FANDISK must be defined (absolute path to bench/models/fandisk.obj)"
#endif

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/wait.h>
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

struct RunResult {
    int exitCode = -1;
    std::string stdoutText;
};

RunResult runCapture(const std::string& binary, const std::string& args,
    const std::filesystem::path& stdoutPath)
{
    RunResult result;
    const std::string command = "\"" + binary + "\" " + args + " > \"" + stdoutPath.string() + "\"";
    const int status = std::system(command.c_str());
    if (WIFEXITED(status))
        result.exitCode = WEXITSTATUS(status);
    std::ifstream in(stdoutPath);
    if (in.is_open()) {
        std::ostringstream text;
        text << in.rdbuf();
        result.stdoutText = text.str();
    }
    return result;
}

// Highest integer after the first "Quads: " on report lines, or -1.
long reportQuads(const std::string& text)
{
    std::istringstream lines(text);
    std::string line;
    while (std::getline(lines, line)) {
        if (line.rfind("Quads: ", 0) != 0)
            continue;
        long quads = -1;
        if (std::sscanf(line.c_str(), "Quads: %ld", &quads) == 1)
            return quads;
    }
    return -1;
}

struct ObjShape {
    long verts = 0;
    long fanTris = 0; // fan-triangulated triangle count (n-2 per n-gon)
};

ObjShape readObjShape(const std::string& path)
{
    ObjShape shape;
    std::ifstream in(path);
    if (!in.is_open())
        return shape;
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("v ", 0) == 0) {
            ++shape.verts;
            continue;
        }
        if (line.rfind("f ", 0) != 0)
            continue;
        std::istringstream corners(line.substr(2));
        std::string corner;
        long n = 0;
        while (corners >> corner)
            ++n;
        if (n >= 3)
            shape.fanTris += n - 2;
    }
    return shape;
}

struct GlbShape {
    bool valid = false;
    long vertCount = -1; // POSITION accessor count
    long indexCount = -1; // indices accessor count
};

// Minimal GLB validation: magic, version 2, total length, JSON chunk type,
// then the first two accessor "count" values (writer emits POSITION first).
GlbShape readGlbShape(const std::string& path)
{
    GlbShape shape;
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open())
        return shape;
    std::string blob((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (blob.size() < 20)
        return shape;
    if (blob[0] != 'g' || blob[1] != 'l' || blob[2] != 'T' || blob[3] != 'F')
        return shape;
    uint32_t version = 0, total = 0, jsonLen = 0, jsonType = 0;
    std::memcpy(&version, blob.data() + 4, 4);
    std::memcpy(&total, blob.data() + 8, 4);
    std::memcpy(&jsonLen, blob.data() + 12, 4);
    std::memcpy(&jsonType, blob.data() + 16, 4);
    if (version != 2 || total != blob.size() || jsonType != 0x4E4F534A)
        return shape;
    const std::string json = blob.substr(20, jsonLen);
    size_t first = json.find("\"count\":");
    if (first == std::string::npos)
        return shape;
    size_t second = json.find("\"count\":", first + 1);
    if (second == std::string::npos)
        return shape;
    if (std::sscanf(json.c_str() + first, "\"count\":%ld", &shape.vertCount) != 1)
        return shape;
    if (std::sscanf(json.c_str() + second, "\"count\":%ld", &shape.indexCount) != 1)
        return shape;
    shape.valid = true;
    return shape;
}

} // namespace

int main()
{
    const std::string binary = RETOPO_BINARY;
    const std::string tetra = RETOPO_MODEL_TETRA;

    if (!std::filesystem::exists(binary)) {
        std::printf("FAIL %s:%d: retopo binary not found: %s\n", __FILE__, __LINE__, binary.c_str());
        return 1;
    }
    if (!std::filesystem::exists(tetra)) {
        std::printf("FAIL %s:%d: fixture not found: %s\n", __FILE__, __LINE__, tetra.c_str());
        return 1;
    }
    if (!std::filesystem::exists(RETOPO_MODEL_FANDISK)) {
        std::printf("FAIL %s:%d: model not found: %s (run bench/fetch_models.sh)\n",
            __FILE__, __LINE__, RETOPO_MODEL_FANDISK);
        return 1;
    }

    std::error_code ec;
    // PID-suffixed: parallel lanes run the same suite from sibling checkouts,
    // and a fixed tmpdir name lets them clobber each other's files (this
    // test flaked exactly that way: two writers racing one fixed path).
    const std::filesystem::path tmpdir = std::filesystem::temp_directory_path()
        / ("retopo_cli_glb_" + std::to_string(::getpid()));
    std::filesystem::remove_all(tmpdir, ec);
    std::filesystem::create_directories(tmpdir, ec);

    // (a) glb -> obj: exit 0, sane output mesh.
    const std::string tetraObj = (tmpdir / "tetra.obj").string();
    {
        const RunResult run = runCapture(binary,
            "--input \"" + tetra + "\" --output \"" + tetraObj + "\" --target-quads 4",
            tmpdir / "a_stdout.txt");
        CHECK(run.exitCode == 0);
        CHECK(reportQuads(run.stdoutText) > 0);
        const ObjShape shape = readObjShape(tetraObj);
        CHECK(shape.verts > 0);
        CHECK(shape.fanTris > 0);
    }

    // (b) obj -> glb: output re-parses with accessor counts matching the
    // source mesh (writer validity, independent of engine counts).
    // Uses fandisk, not the tetra output above: re-remeshing the tiny
    // tetra mesh collapses to empty on some platforms (Linux), which
    // correctly fails loudly but tests nothing about the writer.
    {
        const std::string fandisk = RETOPO_MODEL_FANDISK;
        const std::string outGlb = (tmpdir / "fandisk.glb").string();
        const RunResult run = runCapture(binary,
            "--input \"" + fandisk + "\" --output \"" + outGlb + "\" --target-quads 200",
            tmpdir / "b_stdout.txt");
        CHECK(run.exitCode == 0);
        const GlbShape glb = readGlbShape(outGlb);
        CHECK(glb.valid);
        // The CLI remeshes instead of converting, so compare against the
        // remeshed output mesh, re-read through the obj twin below.
        CHECK(glb.vertCount > 0);
        CHECK(glb.indexCount > 0);
        CHECK(glb.indexCount % 3 == 0);
        // Same input to .obj: writer accessor counts must match that mesh.
        const std::string twinObj = (tmpdir / "twin.obj").string();
        const RunResult twin = runCapture(binary,
            "--input \"" + fandisk + "\" --output \"" + twinObj + "\" --target-quads 200",
            tmpdir / "b2_stdout.txt");
        CHECK(twin.exitCode == 0);
        const ObjShape twinShape = readObjShape(twinObj);
        CHECK(glb.vertCount == twinShape.verts);
        CHECK(glb.indexCount == twinShape.fanTris * 3);
    }

    // (c) batch dir with one .obj + one .glb yields both outputs.
    {
        const std::filesystem::path batchIn = tmpdir / "batch_in";
        const std::filesystem::path batchOut = tmpdir / "batch_out";
        std::filesystem::create_directories(batchIn, ec);
        std::filesystem::copy_file(tetraObj, batchIn / "m.obj", ec);
        CHECK(!ec);
        std::filesystem::copy_file(tetra, batchIn / "m.glb", ec);
        CHECK(!ec);
        const RunResult run = runCapture(binary,
            "--input \"" + batchIn.string() + "\" --output \"" + batchOut.string() + "\"",
            tmpdir / "c_stdout.txt");
        CHECK(run.exitCode == 0);
        CHECK(run.stdoutText.find("Failed files: none") != std::string::npos);
        CHECK(std::filesystem::exists(batchOut / "m.obj"));
        CHECK(std::filesystem::exists(batchOut / "m.glb"));
    }

    // (d) --lods with a .glb output writes *_lodN.glb rungs. Uses fandisk:
    // the 4-tri tetra supports exactly one viable rung (target 4), so a
    // real decreasing chain needs a bigger mesh.
    {
        const std::string lodBase = (tmpdir / "chain.glb").string();
        const RunResult run = runCapture(binary,
            "--input \"" + std::string(RETOPO_MODEL_FANDISK)
                + "\" --output \"" + lodBase + "\" --lods 2000,1000",
            tmpdir / "d_stdout.txt");
        CHECK(run.exitCode == 0);
        CHECK(run.stdoutText.find("LOD 0:") != std::string::npos);
        CHECK(run.stdoutText.find("LOD 1:") != std::string::npos);
        CHECK(std::filesystem::exists(tmpdir / "chain_lod0.glb"));
        CHECK(std::filesystem::exists(tmpdir / "chain_lod1.glb"));
    }

    // (f) Empty engine output fails loudly and identically in both
    // formats (target 8 oversubscribes the 4-tri tetra): no silent
    // 0-vertex success file in either extension.
    {
        const RunResult objRun = runCapture(binary,
            "--input \"" + tetra + "\" --output \"" + (tmpdir / "e.obj").string()
                + "\" --lods 8,4",
            tmpdir / "f_obj_stdout.txt");
        const RunResult glbRun = runCapture(binary,
            "--input \"" + tetra + "\" --output \"" + (tmpdir / "e.glb").string()
                + "\" --lods 8,4",
            tmpdir / "f_glb_stdout.txt");
        CHECK(objRun.exitCode == 1);
        CHECK(glbRun.exitCode == 1);
        CHECK(objRun.stdoutText.find("LOD 0: FAILED") != std::string::npos);
        CHECK(glbRun.stdoutText.find("LOD 0: FAILED") != std::string::npos);
    }

    // (e) truncated .glb exits 1.
    {
        std::ifstream src(tetra, std::ios::binary);
        std::string head(100, '\0');
        src.read(head.data(), 100);
        head.resize(static_cast<size_t>(src.gcount()));
        const std::string bad = (tmpdir / "truncated.glb").string();
        std::ofstream out(bad, std::ios::binary);
        out.write(head.data(), static_cast<std::streamsize>(head.size()));
        out.close();
        const RunResult run = runCapture(binary,
            "--input \"" + bad + "\" --output \"" + (tmpdir / "x.obj").string() + "\"",
            tmpdir / "e_stdout.txt");
        CHECK(run.exitCode == 1);
    }

    std::filesystem::remove_all(tmpdir, ec);

    if (g_failures == 0)
        std::printf("PASS test_cli_glb\n");
    return g_failures == 0 ? 0 : 1;
}
