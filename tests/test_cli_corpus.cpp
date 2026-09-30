// CLI nasty-corpus test: runs every tests/fixtures/nasty-*.obj class plus
// two procedurally generated large inputs through the BUILT retopo binary
// (from the build tree, never from PATH) and asserts the robustness
// contracts: no crash/hang/signal, loud per-part accounting for dropped
// geometry, and honest exit codes (0 = usable output written, 1 = nothing
// usable). Loader-level lines (Loaded/Welded counts, island totals) are
// pinned exactly because the loader, weld, and splitter are
// deterministic; engine-behavior numbers (quad counts, per-island success)
// are deliberately NOT pinned so engine lanes can move them.
// Plain assert-style main, no framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
//
// Paths arrive as compile definitions from tests/CMakeLists.txt:
//   RETOPO_BINARY    absolute path to the built retopo binary
//   RETOPO_FIXTURES  absolute path to tests/fixtures
#ifndef RETOPO_BINARY
#error "RETOPO_BINARY must be defined (absolute path to the built retopo binary)"
#endif
#ifndef RETOPO_FIXTURES
#error "RETOPO_FIXTURES must be defined (absolute path to tests/fixtures)"
#endif

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
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

struct RunResult {
    bool exitedNormally = false;
    int exitCode = -1;
    std::string stdoutText;
    std::string stderrText;
};

std::string readFile(const std::filesystem::path& path)
{
    std::ifstream in(path, std::ios::in | std::ios::binary);
    std::ostringstream out;
    out << in.rdbuf();
    return out.str();
}

int g_runCounter = 0;

RunResult runCapture(const std::string& binary, const std::string& args,
    const std::filesystem::path& tmpdir)
{
    RunResult result;
    const std::filesystem::path stdoutPath = tmpdir
        / ("corpus_run_" + std::to_string(g_runCounter) + ".out");
    const std::filesystem::path stderrPath = tmpdir
        / ("corpus_run_" + std::to_string(g_runCounter) + ".err");
    ++g_runCounter;
    const std::string command = "\"" + binary + "\" " + args + " > \"" + stdoutPath.string()
        + "\" 2> \"" + stderrPath.string() + "\"";
    // std::system returns the raw wait status; a crash (signal) leaves
    // exitedNormally false, which every case below asserts against.
    const int status = std::system(command.c_str());
    result.exitedNormally = (0 != WIFEXITED(status));
    if (result.exitedNormally)
        result.exitCode = WEXITSTATUS(status);
    result.stdoutText = readFile(stdoutPath);
    result.stderrText = readFile(stderrPath);
    return result;
}

struct ObjCounts {
    long verts = 0;
    long faces = 0;
    long errors = 0;
};

// Parse + validate an output OBJ the same way bench/run.py validate_obj
// does: every face corner must parse as an integer, lie in 1..vertCount,
// and differ from the other corners of its face (no degenerate faces).
// Valid means openable, non-empty, and error-free.
bool parseOutputObj(const std::string& path, ObjCounts* counts)
{
    std::ifstream in(path);
    if (!in.is_open())
        return false;
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("v ", 0) == 0) {
            ++counts->verts;
        } else if (line.rfind("f ", 0) == 0) {
            ++counts->faces;
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
                    ++counts->errors;
                    cornerOk = false;
                    break;
                }
                indices.push_back(index);
            }
            if (!cornerOk)
                continue;
            for (const long index : indices) {
                if (index < 1 || index > counts->verts)
                    ++counts->errors;
            }
            for (size_t i = 0; i < indices.size(); ++i) {
                for (size_t j = i + 1; j < indices.size(); ++j) {
                    if (indices[i] == indices[j])
                        ++counts->errors;
                }
            }
        }
    }
    return counts->verts > 0 && counts->faces > 0 && counts->errors == 0;
}

bool contains(const std::string& text, const std::string& needle)
{
    return text.find(needle) != std::string::npos;
}

// A single-file run over one committed fixture. expectedExit is 0 (usable
// output: the file must exist and validate) or 1 (nothing usable: no
// output file may exist). Returns the run so callers can assert on its
// stdout/stderr lines without re-running the binary.
RunResult checkFixture(const std::string& binary, const std::filesystem::path& tmpdir,
    const std::string& fixture, const std::string& extraArgs, int expectedExit)
{
    const int failuresBefore = g_failures;
    const std::filesystem::path input = std::filesystem::path(RETOPO_FIXTURES) / fixture;
    const std::filesystem::path output = tmpdir / (fixture + ".out.obj");
    if (!std::filesystem::exists(input)) {
        std::printf("FAIL %s:%d: fixture not found: %s\n", __FILE__, __LINE__, input.c_str());
        ++g_failures;
        return RunResult();
    }
    const RunResult run = runCapture(binary,
        "--input \"" + input.string() + "\" --output \"" + output.string() + "\" " + extraArgs,
        tmpdir);
    CHECK(run.exitedNormally);
    CHECK(run.exitCode == expectedExit);
    ObjCounts counts;
    const bool outputValid = parseOutputObj(output.string(), &counts);
    if (expectedExit == 0) {
        CHECK(outputValid);
    } else {
        CHECK(!std::filesystem::exists(output));
    }
    if (g_failures != failuresBefore) {
        std::printf("  while checking fixture %s (exit=%d stdout=[%s] stderr=[%s])\n",
            fixture.c_str(), run.exitCode, run.stdoutText.c_str(), run.stderrText.c_str());
    }
    return run;
}

} // namespace

int main()
{
    const std::string binary = RETOPO_BINARY;

    if (!std::filesystem::exists(binary)) {
        std::printf("FAIL %s:%d: retopo binary not found: %s\n", __FILE__, __LINE__, binary.c_str());
        return 1;
    }
    if (!std::filesystem::exists(std::filesystem::path(RETOPO_FIXTURES))) {
        std::printf("FAIL %s:%d: fixtures dir not found: %s\n", __FILE__, __LINE__, RETOPO_FIXTURES);
        return 1;
    }

    std::error_code ec;
    // PID-suffixed: parallel lanes run the same suite from sibling
    // checkouts, and a fixed tmpdir name lets them clobber each other's
    // batch dirs and capture files (observed test_cli_glb flake).
    const std::filesystem::path tmpdir = std::filesystem::temp_directory_path()
        / ("retopo_cli_corpus_" + std::to_string(::getpid()));
    std::filesystem::remove_all(tmpdir, ec);
    std::filesystem::create_directories(tmpdir, ec);

    // Control: one closed tetrahedron is one island, fully remeshed.
    {
        const RunResult run = checkFixture(binary, tmpdir, "nasty-single-tetra.obj", "--target-quads 10", 0);
        CHECK(contains(run.stdoutText, "Islands: 1"));
        CHECK(contains(run.stdoutText, "Failed islands: 0"));
    }

    // Non-indexed soup welds back to the control tetrahedron: exact Welded
    // line, one island, no failures.
    {
        const RunResult run = checkFixture(binary, tmpdir, "nasty-soup.obj", "--target-quads 10", 0);
        CHECK(contains(run.stderrText, "Welded input: 12 -> 4 vertices, 4 -> 4 triangles"));
        CHECK(contains(run.stdoutText, "Islands: 1"));
        CHECK(contains(run.stdoutText, "Failed islands: 0"));
    }

    // Non-manifold edge (three faces on edge A-B): exit 0 with valid
    // output. The failed-island count is engine behavior and is not
    // pinned; the accounting line must be present.
    {
        const RunResult run = checkFixture(binary, tmpdir, "nasty-nonmanifold.obj", "--target-quads 10", 0);
        CHECK(contains(run.stdoutText, "Islands: 1"));
        CHECK(contains(run.stdoutText, "Failed islands: "));
    }

    // Multi-component: two closed tetrahedra. Target 50 (not 10): at
    // target 10 the shared voxel size dwarfs each island and the engine
    // extracts nothing, which is honest exit 1 but not the success path
    // this case exists to pin.
    {
        const RunResult run = checkFixture(binary, tmpdir, "nasty-multicomp.obj", "--target-quads 50", 0);
        CHECK(contains(run.stdoutText, "Islands: 2"));
        CHECK(contains(run.stdoutText, "Failed islands: 0"));
    }

    // Degenerate mix: the two index-degenerate faces drop in the weld
    // (exact counts), the zero-area island is accounted for, the
    // tetrahedron remeshes.
    {
        const RunResult run = checkFixture(binary, tmpdir, "nasty-degenerate.obj", "--target-quads 10", 0);
        CHECK(contains(run.stderrText, "Welded input: 7 -> 7 vertices, 7 -> 5 triangles"));
        CHECK(contains(run.stdoutText, "Islands: 2"));
        CHECK(contains(run.stdoutText, "Failed islands: "));
    }

    // All-degenerate: every face drops in the weld, so exit 1 with no
    // output file and the exact Welded line.
    {
        const RunResult run = checkFixture(binary, tmpdir, "nasty-all-degenerate.obj", "--target-quads 10", 1);
        CHECK(contains(run.stderrText, "Welded input: 3 -> 3 vertices, 3 -> 0 triangles"));
    }

    // All zero-area: the loader keeps area-less triangles, so whether the
    // engine extracts output is engine behavior. Assert only the contract:
    // exit 0 iff a valid, non-empty mesh was written. (Today this exits
    // 1; if a future engine remeshes area-less islands, exit 0 + a valid
    // mesh is equally acceptable.)
    {
        const std::filesystem::path input = std::filesystem::path(RETOPO_FIXTURES) / "nasty-all-zeroarea.obj";
        const std::filesystem::path output = tmpdir / "allzero.out.obj";
        const RunResult run = runCapture(binary,
            "--input \"" + input.string() + "\" --output \"" + output.string() + "\" --target-quads 10",
            tmpdir);
        CHECK(run.exitedNormally);
        CHECK(run.exitCode == 0 || run.exitCode == 1);
        ObjCounts counts;
        const bool outputValid = parseOutputObj(output.string(), &counts);
        CHECK((run.exitCode == 0) == outputValid);
        if (run.exitCode == 1)
            CHECK(!std::filesystem::exists(output));
        // Either way the splitter saw the two input islands.
        CHECK(contains(run.stderrText, "Islands: 2, input triangles: 2"));
    }

    // NaN corner: the weld drops exactly that triangle (exact counts plus
    // the dedicated warning) and the tetrahedron remeshes. Before the fix
    // the NaN poisoned the mesh area and the whole run exited 1.
    {
        const RunResult run = checkFixture(binary, tmpdir, "nasty-nan.obj", "--target-quads 10", 0);
        CHECK(contains(run.stderrText, "Welded input: 7 -> 4 vertices, 5 -> 4 triangles"));
        CHECK(contains(run.stderrText, "dropped 1 input triangles with non-finite corners"));
        CHECK(contains(run.stdoutText, "Islands: 1"));
        CHECK(contains(run.stdoutText, "Failed islands: 0"));
    }

    // Out-of-range face index: loud load rejection, exit 1, no output.
    // Before the fix this segfaulted; exitedNormally is the no-crash pin.
    {
        const RunResult run = checkFixture(binary, tmpdir, "nasty-oor.obj", "--target-quads 10", 1);
        CHECK(run.exitedNormally);
        CHECK(contains(run.stderrText, "Failed parse"));
        CHECK(contains(run.stderrText, "failed to load"));
    }

    // Empty file: exit 1, no output, no crash.
    checkFixture(binary, tmpdir, "nasty-empty.obj", "--target-quads 10", 1);

    // Floater: the tetrahedron remeshes; the distant sliver is accounted
    // for (its success is engine behavior and is not pinned).
    {
        const RunResult run = checkFixture(binary, tmpdir, "nasty-floaters.obj", "--target-quads 10", 0);
        CHECK(contains(run.stdoutText, "Islands: 2"));
        CHECK(contains(run.stdoutText, "Failed islands: "));
    }

    // Multi-mode exercises the same accounting in --lods runs: both rungs
    // succeed with valid outputs, and the weld report prints on stderr.
    {
        const std::filesystem::path input = std::filesystem::path(RETOPO_FIXTURES) / "nasty-multicomp.obj";
        const std::filesystem::path outBase = tmpdir / "lod_mc.obj";
        const RunResult run = runCapture(binary,
            "--input \"" + input.string() + "\" --output \"" + outBase.string() + "\" --lods 100,50",
            tmpdir);
        CHECK(run.exitedNormally);
        CHECK(run.exitCode == 0);
        CHECK(contains(run.stdoutText, "LOD 0:"));
        CHECK(contains(run.stdoutText, "LOD 1:"));
        ObjCounts counts0, counts1;
        CHECK(parseOutputObj((tmpdir / "lod_mc_lod0.obj").string(), &counts0));
        CHECK(parseOutputObj((tmpdir / "lod_mc_lod1.obj").string(), &counts1));
    }
    {
        const std::filesystem::path input = std::filesystem::path(RETOPO_FIXTURES) / "nasty-degenerate.obj";
        const std::filesystem::path outBase = tmpdir / "lod_dg.obj";
        const RunResult run = runCapture(binary,
            "--input \"" + input.string() + "\" --output \"" + outBase.string() + "\" --lods 10,10",
            tmpdir);
        CHECK(run.exitedNormally);
        CHECK(run.exitCode == 0);
        CHECK(contains(run.stderrText, "Welded input: 7 -> 7 vertices, 7 -> 5 triangles"));
        ObjCounts counts0, counts1;
        CHECK(parseOutputObj((tmpdir / "lod_dg_lod0.obj").string(), &counts0));
        CHECK(parseOutputObj((tmpdir / "lod_dg_lod1.obj").string(), &counts1));
    }

    // Batch mode: the NaN file remeshes (multi-mode weld warning on
    // stderr), the empty file fails, the exit is 1 with the failed-files
    // list naming exactly the empty file.
    {
        const std::filesystem::path batchIn = tmpdir / "batch_in";
        const std::filesystem::path batchOut = tmpdir / "batch_out";
        std::filesystem::create_directories(batchIn, ec);
        std::filesystem::copy_file(std::filesystem::path(RETOPO_FIXTURES) / "nasty-nan.obj",
            batchIn / "nasty-nan.obj", ec);
        CHECK(!ec);
        std::filesystem::copy_file(std::filesystem::path(RETOPO_FIXTURES) / "nasty-empty.obj",
            batchIn / "nasty-empty.obj", ec);
        CHECK(!ec);
        const RunResult run = runCapture(binary,
            "--input \"" + batchIn.string() + "\" --output \"" + batchOut.string()
                + "\" --target-quads 50",
            tmpdir);
        CHECK(run.exitedNormally);
        CHECK(run.exitCode == 1);
        CHECK(contains(run.stdoutText, "FILE nasty-nan.obj:"));
        CHECK(!contains(run.stdoutText, "FILE nasty-nan.obj: FAILED"));
        CHECK(contains(run.stdoutText, "FILE nasty-empty.obj: FAILED"));
        CHECK(contains(run.stdoutText, "Failed files (1): nasty-empty.obj"));
        CHECK(contains(run.stderrText, "dropped 1 input triangles with non-finite corners"));
        ObjCounts counts;
        CHECK(parseOutputObj((batchOut / "nasty-nan.obj").string(), &counts));
    }

    // Procedural large-scale input, built at test time (never committed):
    // a 120x120 grid (28,800 triangles, bigger than fandisk) with junk
    // appended: every 500th face duplicated (non-manifold), 20
    // index-degenerate faces, 5 zero-area colinear triangles on new
    // far-away vertices, and 1 NaN-cornered triangle. The weld drops
    // exactly the 20 degenerates + the NaN face (and the NaN face's 3
    // orphaned vertices); the grid remeshes.
    {
        const int n = 120;
        const std::filesystem::path input = tmpdir / "proc_grid.obj";
        long long preVerts = 0;
        long long preTris = 0;
        {
            std::ofstream out(input, std::ios::out | std::ios::binary);
            for (int j = 0; j <= n; ++j) {
                for (int i = 0; i <= n; ++i)
                    out << "v " << i * 0.1 << ' ' << j * 0.1 << " 0\n";
            }
            preVerts = (n + 1) * (n + 1);
            std::vector<std::string> faces;
            for (int j = 0; j < n; ++j) {
                for (int i = 0; i < n; ++i) {
                    const long long a = j * (n + 1) + i + 1;
                    const long long b = a + 1;
                    const long long c = a + (n + 1);
                    const long long d = c + 1;
                    std::ostringstream first, second;
                    first << "f " << a << ' ' << b << ' ' << d;
                    second << "f " << a << ' ' << d << ' ' << c;
                    faces.push_back(first.str());
                    faces.push_back(second.str());
                }
            }
            for (size_t k = 0; k < faces.size(); k += 500)
                faces.push_back(faces[k]); // duplicate: non-manifold double cover
            for (int k = 0; k < 20; ++k)
                faces.push_back("f 1 1 2"); // index-degenerate
            for (int k = 0; k < 5; ++k) {
                // Zero-area colinear triangle on fresh far-away vertices.
                const double x = 1000.0 + k * 10.0;
                out << "v " << x << " 0 0\n";
                out << "v " << x + 1.0 << " 0 0\n";
                out << "v " << x + 2.0 << " 0 0\n";
                const long long base = preVerts + k * 3 + 1;
                std::ostringstream face;
                face << "f " << base << ' ' << base + 1 << ' ' << base + 2;
                faces.push_back(face.str());
            }
            preVerts += 15;
            out << "v nan 0 0\nv 5000 0 0\nv 5000 1 0\n";
            preVerts += 3;
            {
                std::ostringstream face;
                face << "f " << preVerts - 2 << ' ' << preVerts - 1 << ' ' << preVerts;
                faces.push_back(face.str());
            }
            preTris = static_cast<long long>(faces.size());
            for (const auto& face : faces)
                out << face << '\n';
        }
        const std::filesystem::path output = tmpdir / "proc_grid.out.obj";
        const RunResult run = runCapture(binary,
            "--input \"" + input.string() + "\" --output \"" + output.string()
                + "\" --target-quads 1000",
            tmpdir);
        CHECK(run.exitedNormally);
        CHECK(run.exitCode == 0);
        ObjCounts counts;
        CHECK(parseOutputObj(output.string(), &counts));
        // Exact loader accounting: 20 degenerates + 1 NaN face dropped,
        // plus the NaN face's 3 orphaned vertices.
        std::ostringstream welded;
        welded << "Welded input: " << preVerts << " -> " << (preVerts - 3) << " vertices, "
               << preTris << " -> " << (preTris - 21) << " triangles";
        CHECK(contains(run.stderrText, welded.str()));
        CHECK(contains(run.stderrText, "dropped 1 input triangles with non-finite corners"));
        // Grid island + 5 zero-area islands.
        CHECK(contains(run.stdoutText, "Islands: 6"));
        CHECK(contains(run.stdoutText, "Failed islands: "));
    }

    // Procedural many-component input: 100 closed tetrahedra in a row.
    // Each is one island; every island remeshes or is accounted for.
    {
        const int tetraCount = 100;
        const std::filesystem::path input = tmpdir / "proc_tetras.obj";
        {
            std::ofstream out(input, std::ios::out | std::ios::binary);
            for (int t = 0; t < tetraCount; ++t) {
                const double x = t * 10.0;
                out << "v " << x << " 0 0\n";
                out << "v " << x + 1.0 << " 0 0\n";
                out << "v " << x << " 1 0\n";
                out << "v " << x << " 0 1\n";
            }
            for (int t = 0; t < tetraCount; ++t) {
                const long long b = t * 4 + 1;
                out << "f " << b << ' ' << b + 2 << ' ' << b + 1 << '\n';
                out << "f " << b << ' ' << b + 1 << ' ' << b + 3 << '\n';
                out << "f " << b << ' ' << b + 3 << ' ' << b + 2 << '\n';
                out << "f " << b + 1 << ' ' << b + 2 << ' ' << b + 3 << '\n';
            }
        }
        const std::filesystem::path output = tmpdir / "proc_tetras.out.obj";
        const RunResult run = runCapture(binary,
            "--input \"" + input.string() + "\" --output \"" + output.string()
                + "\" --target-quads 1000",
            tmpdir);
        CHECK(run.exitedNormally);
        CHECK(run.exitCode == 0);
        ObjCounts counts;
        CHECK(parseOutputObj(output.string(), &counts));
        CHECK(contains(run.stdoutText, "Islands: 100"));
        CHECK(contains(run.stdoutText, "Failed islands: "));
    }

    std::filesystem::remove_all(tmpdir, ec);

    if (g_failures == 0)
        std::printf("PASS test_cli_corpus\n");
    return g_failures == 0 ? 0 : 1;
}
