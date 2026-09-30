// CLI --quiet engine-silence test: runs the BUILT retopo binary (from the
// build tree, never from PATH) with --quiet and asserts the full silence
// contract on BOTH streams: stdout keeps the === retopoforge Report ===
// block but carries no progress lines, and stderr carries no engine-owned
// progress or phase-report lines either (the phase dump and the extractor
// progress echoes are quiet-gated in the engine). Warnings and errors
// still print: a failing --quiet run must exit nonzero with an "Error:"
// line on stderr. Complements test_cli_quiet (stdout-only checks).
// Plain assert-style main, no framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
//
// Paths arrive as compile definitions from tests/CMakeLists.txt:
//   RETOPO_BINARY            absolute path to the built retopo binary
//   RETOPO_MODEL_ARMADDILLO  absolute path to bench/models/armadillo.obj
//   RETOPO_MODEL_FANDISK     absolute path to bench/models/fandisk.obj
#ifndef RETOPO_BINARY
#error "RETOPO_BINARY must be defined (absolute path to the built retopo binary)"
#endif
#ifndef RETOPO_MODEL_ARMADDILLO
#error "RETOPO_MODEL_ARMADDILLO must be defined (absolute path to bench/models/armadillo.obj)"
#endif
#ifndef RETOPO_MODEL_FANDISK
#error "RETOPO_MODEL_FANDISK must be defined (absolute path to bench/models/fandisk.obj)"
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

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                             \
            ++g_failures;                                                                           \
        }                                                                                          \
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

bool contains(const std::string& text, const std::string& needle)
{
    return text.find(needle) != std::string::npos;
}

int g_runCounter = 0;

RunResult runCapture(const std::string& binary, const std::string& args,
    const std::filesystem::path& tmpdir)
{
    RunResult result;
    const std::filesystem::path stdoutPath = tmpdir
        / ("quiet_run_" + std::to_string(g_runCounter) + ".out");
    const std::filesystem::path stderrPath = tmpdir
        / ("quiet_run_" + std::to_string(g_runCounter) + ".err");
    ++g_runCounter;
    const std::string command = "\"" + binary + "\" " + args + " > \"" + stdoutPath.string()
        + "\" 2> \"" + stderrPath.string() + "\"";
    const int status = std::system(command.c_str());
    result.exitedNormally = (0 != WIFEXITED(status));
    if (result.exitedNormally)
        result.exitCode = WEXITSTATUS(status);
    result.stdoutText = readFile(stdoutPath);
    result.stderrText = readFile(stderrPath);
    return result;
}

// Engine phase-report markers: every phaseReport() line contains one of
// these (summary lines and indented leaf-stage lines alike).
const std::vector<std::string> kPhaseMarkers = {
    "input triangles:",
    "Compute voxel size",
    "Split into islands",
    "Build island contexts",
    "Mesh simplifier:",
    "(accumulated)",
    "wall clock",
    "Cores kept busy",
    "Merge islands",
    "Total:",
};

// Extractor progress echoes: each mirrors a progress-handler step name.
const std::vector<std::string> kExtractorMarkers = {
    "Extract connections",
    "Extract edges",
    "Extract mesh",
    "Smooth and project",
    "Searching boundaries",
    "Searching loop from:",
    "Found valid loop",
    "Fixing hole",
    "fixHoleWithQuads",
    "Hold singular lines",
    "Split six edge",
    "Six edge face kept",
    "Split seven edge",
    "Seven edge face kept",
    "Convert triangle",
    "Collapse three",
    "Merge double",
    "Merge three",
    "Merge shared five",
    "Split high valence",
    "Switch high valence",
    "Cleanup triangle",
};

void checkSilent(const RunResult& run, const char* label)
{
    const int failuresBefore = g_failures;
    CHECK(!contains(run.stdoutText, "done."));
    CHECK(!contains(run.stderrText, "done."));
    for (const auto& marker : kPhaseMarkers) {
        CHECK(!contains(run.stdoutText, marker));
        CHECK(!contains(run.stderrText, marker));
    }
    for (const auto& marker : kExtractorMarkers) {
        CHECK(!contains(run.stdoutText, marker));
        CHECK(!contains(run.stderrText, marker));
    }
    // CLI-side loud lines stay off too (pre-existing --quiet behavior).
    CHECK(!contains(run.stderrText, "Loaded "));
    CHECK(!contains(run.stderrText, "Welded input:"));
    if (g_failures != failuresBefore) {
        std::printf("  while checking %s (exit=%d stdout=[%s] stderr=[%s])\n",
            label, run.exitCode, run.stdoutText.c_str(), run.stderrText.c_str());
    }
}

} // namespace

int main()
{
    const std::string binary = RETOPO_BINARY;
    const std::string armadillo = RETOPO_MODEL_ARMADDILLO;
    const std::string fandisk = RETOPO_MODEL_FANDISK;

    if (!std::filesystem::exists(binary)) {
        std::printf("FAIL %s:%d: retopo binary not found: %s\n", __FILE__, __LINE__, binary.c_str());
        return 1;
    }
    if (!std::filesystem::exists(armadillo) || !std::filesystem::exists(fandisk)) {
        std::printf("FAIL %s:%d: model not found (run bench/fetch_models.sh)\n", __FILE__, __LINE__);
        return 1;
    }

    std::error_code ec;
    const std::filesystem::path tmpdir = std::filesystem::temp_directory_path()
        / ("retopo_cli_engine_quiet_" + std::to_string(::getpid()));
    std::filesystem::remove_all(tmpdir, ec);
    std::filesystem::create_directories(tmpdir, ec);

    // Single-file --quiet: report block survives, progress/phase gone.
    {
        const std::filesystem::path output = tmpdir / "quiet.out.obj";
        const RunResult run = runCapture(binary,
            "--input \"" + armadillo + "\" --output \"" + output.string()
                + "\" --target-quads 1000 --quiet",
            tmpdir);
        CHECK(run.exitedNormally);
        CHECK(run.exitCode == 0);
        CHECK(contains(run.stdoutText, "=== retopoforge Report ==="));
        CHECK(contains(run.stdoutText, "Failed islands: 0"));
        checkSilent(run, "single-file --quiet");
        std::printf("single-file --quiet stderr (%zu bytes):\n%s\n",
            run.stderrText.size(), run.stderrText.c_str());
    }

    // --lods --quiet covers the multi-mode plumbing path too.
    {
        const std::filesystem::path output = tmpdir / "lods.obj";
        const RunResult run = runCapture(binary,
            "--input \"" + fandisk + "\" --output \"" + output.string()
                + "\" --lods 200,100 --quiet",
            tmpdir);
        CHECK(run.exitedNormally);
        CHECK(run.exitCode == 0);
        CHECK(contains(run.stdoutText, "LOD 0:"));
        CHECK(contains(run.stdoutText, "LOD 1:"));
        CHECK(std::filesystem::exists(tmpdir / "lods_lod0.obj"));
        CHECK(std::filesystem::exists(tmpdir / "lods_lod1.obj"));
        checkSilent(run, "--lods --quiet");
    }

    // Errors still print under --quiet.
    {
        const RunResult run = runCapture(binary,
            "--input \"" + (tmpdir / "does-not-exist.obj").string()
                + "\" --output \"" + (tmpdir / "never.obj").string() + "\" --quiet",
            tmpdir);
        CHECK(run.exitedNormally);
        CHECK(run.exitCode != 0);
        CHECK(contains(run.stderrText, "Error:"));
        std::printf("missing-input --quiet stderr:\n%s\n", run.stderrText.c_str());
    }

    std::filesystem::remove_all(tmpdir, ec);
    if (g_failures == 0)
        std::printf("PASS test_cli_engine_quiet\n");
    return g_failures == 0 ? 0 : 1;
}
