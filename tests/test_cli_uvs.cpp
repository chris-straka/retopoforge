// CLI --uvs test: remeshed UV emission from the internal parameterization.
// Asserts the on/off contract, not exact engine counts (counts move with
// engine work; exact pins live in test_cli_roundtrip/test_cli_multimode):
// default output has no vt lines and UV-less faces, --uvs on emits one vt
// per vertex with finite 0..1 values and v/vt corners, GLB gains
// TEXCOORD_0, and geometry (v lines + face indices) is identical with the
// flag on or off. Plain assert-style main, no framework.
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

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <vector>

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

struct ObjMesh {
    bool loaded = false;
    std::vector<std::string> vLines;
    std::vector<std::string> vtLines;
    std::vector<std::string> fLines;
};

ObjMesh readObj(const std::string& path)
{
    ObjMesh mesh;
    std::ifstream in(path);
    if (!in.is_open())
        return mesh;
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("vt ", 0) == 0)
            mesh.vtLines.push_back(line);
        else if (line.rfind("v ", 0) == 0)
            mesh.vLines.push_back(line);
        else if (line.rfind("f ", 0) == 0)
            mesh.fLines.push_back(line);
    }
    mesh.loaded = true;
    return mesh;
}

// Parse "vt u v" into finite doubles; false on any malformed line.
bool parseVt(const std::string& line, double* u, double* v)
{
    if (line.rfind("vt ", 0) != 0)
        return false;
    std::istringstream corners(line.substr(3));
    std::string token;
    if (!(corners >> token))
        return false;
    char* end = nullptr;
    *u = std::strtod(token.c_str(), &end);
    if (nullptr == end || '\0' != *end || !std::isfinite(*u))
        return false;
    if (!(corners >> token))
        return false;
    *v = std::strtod(token.c_str(), &end);
    if (nullptr == end || '\0' != *end || !std::isfinite(*v))
        return false;
    return true;
}

// Strip the "/vt" suffix from every corner: "f 1/1 2/2" -> "f 1 2".
// Returns false when a corner is not exactly "i/i" with 1 <= i <= vertCount.
bool stripVtSuffixes(const std::string& faceLine, size_t vertCount, std::string* stripped)
{
    if (faceLine.rfind("f ", 0) != 0)
        return false;
    std::istringstream corners(faceLine.substr(2));
    std::string corner;
    std::ostringstream out;
    out << "f";
    while (corners >> corner) {
        const size_t slash = corner.find('/');
        if (slash == std::string::npos || corner.find('/', slash + 1) != std::string::npos)
            return false;
        char* end = nullptr;
        const long v = std::strtol(corner.c_str(), &end, 10);
        if (nullptr == end || *end != '/' || v < 1 || static_cast<size_t>(v) > vertCount)
            return false;
        const long vt = std::strtol(corner.c_str() + slash + 1, &end, 10);
        if (nullptr == end || '\0' != *end || vt != v)
            return false;
        out << " " << v;
    }
    *stripped = out.str();
    return true;
}

struct GlbJson {
    bool valid = false;
    std::string json;
};

GlbJson readGlbJson(const std::string& path)
{
    GlbJson result;
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open())
        return result;
    std::string blob((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (blob.size() < 20)
        return result;
    if (blob[0] != 'g' || blob[1] != 'l' || blob[2] != 'T' || blob[3] != 'F')
        return result;
    uint32_t version = 0, total = 0, jsonLen = 0, jsonType = 0;
    std::memcpy(&version, blob.data() + 4, 4);
    std::memcpy(&total, blob.data() + 8, 4);
    std::memcpy(&jsonLen, blob.data() + 12, 4);
    std::memcpy(&jsonType, blob.data() + 16, 4);
    if (version != 2 || total != blob.size() || jsonType != 0x4E4F534A)
        return result;
    result.json = blob.substr(20, jsonLen);
    result.valid = true;
    return result;
}

// Every accessor "count" value in JSON order (writer emits POSITION first).
std::vector<long> accessorCounts(const std::string& json)
{
    std::vector<long> counts;
    size_t at = 0;
    while ((at = json.find("\"count\":", at)) != std::string::npos) {
        long count = -1;
        if (std::sscanf(json.c_str() + at, "\"count\":%ld", &count) == 1)
            counts.push_back(count);
        at += 8;
    }
    return counts;
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
    const std::filesystem::path tmpdir = std::filesystem::temp_directory_path() / "retopo_cli_uvs";
    std::filesystem::remove_all(tmpdir, ec);
    std::filesystem::create_directories(tmpdir, ec);

    // (a) --help documents --uvs.
    {
        const RunResult run = runCapture(binary, "--help", tmpdir / "a_stdout.txt");
        CHECK(run.exitCode == 0);
        CHECK(run.stdoutText.find("--uvs") != std::string::npos);
    }

    // (b) Bogus --uvs value exits 1.
    {
        const RunResult run = runCapture(binary,
            "--input \"" + fandisk + "\" --output \"" + (tmpdir / "b.obj").string()
                + "\" --uvs maybe",
            tmpdir / "b_stdout.txt");
        CHECK(run.exitCode == 1);
    }

    // (c) Default output: no vt lines, UV-less faces.
    const std::string defaultObj = (tmpdir / "default.obj").string();
    ObjMesh defaultMesh;
    {
        const RunResult run = runCapture(binary,
            "--input \"" + fandisk + "\" --output \"" + defaultObj + "\" --target-quads 200",
            tmpdir / "c_stdout.txt");
        CHECK(run.exitCode == 0);
        defaultMesh = readObj(defaultObj);
        CHECK(defaultMesh.loaded);
        CHECK(!defaultMesh.vLines.empty());
        CHECK(!defaultMesh.fLines.empty());
        CHECK(defaultMesh.vtLines.empty());
        for (const auto& face : defaultMesh.fLines)
            CHECK(face.find('/') == std::string::npos);
    }

    // (d) --uvs on: one vt per vertex, finite 0..1 values, v/vt corners,
    // and geometry identical to the default run.
    {
        const std::string uvsObj = (tmpdir / "uvs.obj").string();
        const RunResult run = runCapture(binary,
            "--input \"" + fandisk + "\" --output \"" + uvsObj
                + "\" --target-quads 200 --uvs on",
            tmpdir / "d_stdout.txt");
        CHECK(run.exitCode == 0);
        const ObjMesh mesh = readObj(uvsObj);
        CHECK(mesh.loaded);
        CHECK(mesh.vtLines.size() == mesh.vLines.size());
        CHECK(!mesh.vLines.empty());
        bool allSame = true;
        for (size_t i = 0; i < mesh.vtLines.size(); ++i) {
            double u = 0.0, v = 0.0;
            CHECK(parseVt(mesh.vtLines[i], &u, &v));
            CHECK(u >= -1e-9 && u <= 1.0 + 1e-9);
            CHECK(v >= -1e-9 && v <= 1.0 + 1e-9);
            if (i > 0 && mesh.vtLines[i] != mesh.vtLines[0])
                allSame = false;
        }
        CHECK(!allSame);
        // Faces carry v/vt corners whose indices match 1:1, and strip back
        // to exactly the default run's faces; v lines match byte for byte.
        CHECK(mesh.fLines.size() == defaultMesh.fLines.size());
        CHECK(mesh.vLines == defaultMesh.vLines);
        for (size_t i = 0; i < mesh.fLines.size(); ++i) {
            std::string stripped;
            CHECK(stripVtSuffixes(mesh.fLines[i], mesh.vLines.size(), &stripped));
            if (i < defaultMesh.fLines.size())
                CHECK(stripped == defaultMesh.fLines[i]);
        }
    }

    // (e) Explicit --uvs off matches the default (no vt lines).
    {
        const std::string offObj = (tmpdir / "off.obj").string();
        const RunResult run = runCapture(binary,
            "--input \"" + fandisk + "\" --output \"" + offObj
                + "\" --target-quads 200 --uvs off",
            tmpdir / "e_stdout.txt");
        CHECK(run.exitCode == 0);
        const ObjMesh mesh = readObj(offObj);
        CHECK(mesh.loaded);
        CHECK(mesh.vtLines.empty());
        CHECK(mesh.vLines == defaultMesh.vLines);
        CHECK(mesh.fLines == defaultMesh.fLines);
    }

    // (f) GLB: --uvs on adds TEXCOORD_0 with one VEC2 per vertex; the
    // default GLB has no TEXCOORD_0 and both agree on POSITION counts.
    {
        const std::string defaultGlb = (tmpdir / "default.glb").string();
        const std::string uvsGlb = (tmpdir / "uvs.glb").string();
        const RunResult runDefault = runCapture(binary,
            "--input \"" + fandisk + "\" --output \"" + defaultGlb + "\" --target-quads 200",
            tmpdir / "f_default_stdout.txt");
        const RunResult runUvs = runCapture(binary,
            "--input \"" + fandisk + "\" --output \"" + uvsGlb
                + "\" --target-quads 200 --uvs on",
            tmpdir / "f_uvs_stdout.txt");
        CHECK(runDefault.exitCode == 0);
        CHECK(runUvs.exitCode == 0);
        const GlbJson plain = readGlbJson(defaultGlb);
        const GlbJson withUvs = readGlbJson(uvsGlb);
        CHECK(plain.valid);
        CHECK(withUvs.valid);
        CHECK(plain.json.find("TEXCOORD_0") == std::string::npos);
        CHECK(withUvs.json.find("\"TEXCOORD_0\":2") != std::string::npos);
        CHECK(withUvs.json.find("\"type\":\"VEC2\"") != std::string::npos);
        const std::vector<long> plainCounts = accessorCounts(plain.json);
        const std::vector<long> uvCounts = accessorCounts(withUvs.json);
        CHECK(plainCounts.size() == 2);
        CHECK(uvCounts.size() == 3);
        if (plainCounts.size() == 2 && uvCounts.size() == 3) {
            CHECK(plainCounts[0] > 0);
            CHECK(uvCounts[0] == plainCounts[0]);
            CHECK(uvCounts[1] == plainCounts[1]);
            CHECK(uvCounts[2] == plainCounts[0]);
        }
        CHECK(withUvs.json.find("\"bufferView\":2") != std::string::npos);
    }

    std::filesystem::remove_all(tmpdir, ec);

    if (g_failures == 0)
        std::printf("PASS test_cli_uvs\n");
    return g_failures == 0 ? 0 : 1;
}
