// Pole-cap manifold regression test: runs the BUILT retopo binary (from
// the build tree, never from PATH) on tests/fixtures/sphere-pole.obj, a
// 32x16 UV sphere with single-vertex poles, and asserts the remeshed
// output is edge-manifold: zero boundary edges (used by 1 face) and zero
// non-manifold edges (used by 3+ faces), counting edges used by != 2
// faces. The extractor used to emit hole-cap quads twice (fixHoles ran
// its second pass over already-capped 4-holes), leaving doubled faces
// whose 4 edges each show use-count 3 -- at --target-quads 10000 this
// fixture reproduced exactly 4 such edges near the north pole.
// Plain assert-style main, no framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
//
// Paths arrive as compile definitions from tests/CMakeLists.txt:
//   RETOPO_BINARY        absolute path to the built retopo binary
//   RETOPO_FIXTURE_SPHERE  absolute path to tests/fixtures/sphere-pole.obj
#ifndef RETOPO_BINARY
#error "RETOPO_BINARY must be defined (absolute path to the built retopo binary)"
#endif
#ifndef RETOPO_FIXTURE_SPHERE
#error "RETOPO_FIXTURE_SPHERE must be defined (absolute path to tests/fixtures/sphere-pole.obj)"
#endif

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
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

struct ManifoldCounts {
    long verts = 0;
    long faces = 0;
    long boundaryEdges = 0; // edge used by exactly 1 face
    long nonManifoldEdges = 0; // edge used by 3+ faces
    long duplicateFaces = 0; // face whose sorted corner set repeats
    long errors = 0; // unparsable/out-of-range/degenerate corners
};

// Parse an output OBJ and count per-edge face incidence plus duplicate
// faces (same corner set emitted twice). Corners parse like
// bench/run.py validate_obj: integer before any '/', 1-based.
bool analyzeOutputObj(const std::string& path, ManifoldCounts* counts)
{
    std::ifstream in(path);
    if (!in.is_open()) {
        std::printf("FAIL %s:%d: cannot open output OBJ %s\n", __FILE__, __LINE__, path.c_str());
        ++g_failures;
        return false;
    }
    std::map<std::pair<long, long>, long> edgeUses;
    std::map<std::vector<long>, long> faceSets;
    std::string line;
    long lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;
        if (line.rfind("v ", 0) == 0) {
            ++counts->verts;
        } else if (line.rfind("f ", 0) == 0) {
            std::istringstream words(line.substr(2));
            std::vector<long> indices;
            std::string corner;
            bool cornerOk = true;
            while (words >> corner) {
                const size_t slash = corner.find('/');
                const std::string first = corner.substr(0, slash);
                char* end = nullptr;
                const long index = std::strtol(first.c_str(), &end, 10);
                if (end == first.c_str() || *end != '\0') {
                    std::printf("FAIL %s:%d: line %ld: unparsable face corner\n",
                        __FILE__, __LINE__, lineNo);
                    ++g_failures;
                    ++counts->errors;
                    cornerOk = false;
                    break;
                }
                indices.push_back(index);
            }
            if (!cornerOk)
                continue;
            ++counts->faces;
            for (const long index : indices) {
                if (index < 1 || index > counts->verts) {
                    std::printf("FAIL %s:%d: line %ld: index out of range\n",
                        __FILE__, __LINE__, lineNo);
                    ++g_failures;
                    ++counts->errors;
                }
            }
            for (size_t i = 0; i < indices.size(); ++i) {
                const size_t j = (i + 1) % indices.size();
                const long first = std::min(indices[i], indices[j]);
                const long second = std::max(indices[i], indices[j]);
                ++edgeUses[{ first, second }];
            }
            std::vector<long> key = indices;
            std::sort(key.begin(), key.end());
            ++faceSets[key];
        }
    }
    for (const auto& [edge, uses] : edgeUses) {
        if (uses == 1)
            ++counts->boundaryEdges;
        else if (uses > 2)
            ++counts->nonManifoldEdges;
    }
    for (const auto& [key, repeats] : faceSets) {
        if (repeats > 1)
            ++counts->duplicateFaces;
    }
    return true;
}

} // namespace

int main()
{
    const std::string binary = RETOPO_BINARY;
    const std::string fixture = RETOPO_FIXTURE_SPHERE;
    CHECK(std::filesystem::exists(binary));
    CHECK(std::filesystem::exists(fixture));

    // PID-suffixed tmpdir: parallel ctest lanes and sibling checkouts
    // share /tmp, and a fixed name lets them clobber each other's outputs.
    std::error_code ec;
    const std::filesystem::path tmpdir = std::filesystem::temp_directory_path()
        / ("retopo_pole_manifold_" + std::to_string(::getpid()));
    std::filesystem::remove_all(tmpdir, ec);
    std::filesystem::create_directories(tmpdir, ec);
    CHECK(std::filesystem::exists(tmpdir));

    const std::filesystem::path output = tmpdir / "sphere-pole.out.obj";
    const std::filesystem::path stdoutPath = tmpdir / "pole_manifold.out";
    const std::filesystem::path stderrPath = tmpdir / "pole_manifold.err";
    // --target-quads 10000 is the reproducing density for this fixture:
    // pre-fix it emitted one doubled cap quad (4 use-count-3 edges).
    const std::string command = "\"" + binary + "\" --input \"" + fixture + "\" --output \""
        + output.string() + "\" --target-quads 10000 > \"" + stdoutPath.string() + "\" 2> \""
        + stderrPath.string() + "\"";
    const int status = std::system(command.c_str());
    CHECK(0 != WIFEXITED(status));
    if (0 != WIFEXITED(status))
        CHECK(0 == WEXITSTATUS(status));

    ManifoldCounts counts;
    if (analyzeOutputObj(output.string(), &counts)) {
        std::printf("pole manifold: verts=%ld faces=%ld boundary=%ld nonmanifold=%ld dups=%ld\n",
            counts.verts, counts.faces, counts.boundaryEdges, counts.nonManifoldEdges,
            counts.duplicateFaces);
        CHECK(counts.verts > 0);
        CHECK(counts.faces > 0);
        CHECK(counts.errors == 0);
        CHECK(counts.boundaryEdges == 0);
        CHECK(counts.nonManifoldEdges == 0);
        CHECK(counts.duplicateFaces == 0);
    }

    std::filesystem::remove_all(tmpdir, ec);

    if (0 == g_failures)
        std::printf("PASS test_pole_manifold\n");
    return 0 == g_failures ? 0 : 1;
}
