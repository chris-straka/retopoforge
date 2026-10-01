// Differential oracle dump for the auto_remesher Rust port.
// Generates seeded random + adversarial engine cases (mesh x settings
// cross product, incl. multi-island, holes, degenerate and floating
// shapes), runs them through the C++ AutoRemesher, and prints inputs +
// outputs in a token format that rust/core/tests/auto_remesher_diff.rs
// replays. Doubles print with %.17g (exact round-trip); f32 progress
// fractions with %.9g (exact round-trip). Statuses print with spaces
// mapped to '_' (empty prints as '.'). Phase-report lines print with
// their timings replaced by T (times are nondeterministic; names, order,
// and counts are the oracle facts). Also times a cover-sized case for the
// runtime ratio (informational T lines; the Rust timing harness
// regenerates the identical mesh analytically and prints its own ms).
//
// Build-only helper: not registered with ctest. Build Release, run it,
// and redirect stdout to tests/fixtures/autoremesher_diff.txt, then commit.
//
// Determinism note: the engine's own stages are run-to-run identical
// (isotropic/decimated/symmetry outputs are bitwise-stable), but the C++
// cover solve carries ~1e-13 TBB noise that the extractor's integer
// rounding amplifies into topological flips on marginal cases (~1/3 of
// this corpus flips somewhere across three runs; case 14, a flat quad,
// lands on 32/30 or 36/34 verts/quads). Those cases are listed in
// isEpxId/isEcxId and replayed robustness-only. The middle
// (parallel-phase) progress events on multi-island cases also vary with
// thread interleaving. The replay pins single-island progress exactly and
// checks robust facts (serial prefix/suffix, monotonicity, range, known
// names) on multi-island cases.
//
// Regeneration rule: adding, removing, or reordering cases shifts ids and
// invalidates the EPX/ECX lists. After any corpus change, run the tool
// three times, diff the runs under the replay's strict rules (see
// rust/core/tests/auto_remesher_diff.rs), and refresh the lists with the
// ids that disagree with themselves.
import retopo.core.auto_remesher;
import retopo.core.vector2;
import retopo.core.vector3;

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using AutoRemesher::ModelType;
using AutoRemesher::Vector2;
using AutoRemesher::Vector3;
using Engine = AutoRemesher::AutoRemesher;

// splitmix64 (fixed seed: the fixture is committed, not regenerated per run).
static std::uint64_t g_state = 0xE461EC09A17EBu;

static std::uint64_t nextU64()
{
    std::uint64_t z = (g_state += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

static std::uint64_t below(std::uint64_t n)
{
    return nextU64() % n;
}

static void printDouble(double v)
{
    std::printf("%.17g", v);
}

static void printFrac(float v)
{
    std::printf("%.9g", static_cast<double>(v));
}

static void printVec3(const Vector3& v)
{
    printDouble(v.x());
    std::printf(" ");
    printDouble(v.y());
    std::printf(" ");
    printDouble(v.z());
}

static void printVec2(const Vector2& v)
{
    printDouble(v.x());
    std::printf(" ");
    printDouble(v.y());
}

// Statuses contain spaces; map them to '_' so the token format survives.
// Empty prints as '.' (the engine passes "" for the final "no status"
// slots... in practice only non-empty names and the invalid-input reasons
// reach the handler; '.' disambiguates anyway).
static std::string statusToken(const char* status)
{
    if (nullptr == status || '\0' == status[0])
        return ".";
    std::string out(status);
    for (char& c : out) {
        if (' ' == c)
            c = '_';
    }
    return out;
}

// Strip the nondeterministic timing suffix from one phase-report line,
// keeping names, order, and counts. Lines ending in "<num> ms" lose the
// number ("...: T"); the cores-busy line loses its ratio; timeless lines
// (Islands: ...) pass through. Spaces map to '_' like statuses.
static std::string phaseToken(const std::string& line)
{
    std::string work = line;
    const std::string msSuffix = " ms";
    if (work.size() > msSuffix.size()
        && work.compare(work.size() - msSuffix.size(), msSuffix.size(), msSuffix) == 0) {
        work.erase(work.size() - msSuffix.size());
        const size_t numEnd = work.size();
        size_t numBegin = numEnd;
        while (numBegin > 0
            && (('0' <= work[numBegin - 1] && work[numBegin - 1] <= '9')
                || '.' == work[numBegin - 1]))
            --numBegin;
        work.erase(numBegin, numEnd - numBegin);
        work += "T";
    }
    const std::string coresPrefix = "Cores kept busy across the parallel phase: ";
    if (work.compare(0, coresPrefix.size(), coresPrefix) == 0) {
        size_t numBegin = coresPrefix.size();
        size_t numEnd = numBegin;
        while (numEnd < work.size()
            && (('0' <= work[numEnd] && work[numEnd] <= '9') || '.' == work[numEnd]))
            ++numEnd;
        work.replace(numBegin, numEnd - numBegin, "T");
    }
    for (char& c : work) {
        if (' ' == c)
            c = '_';
    }
    return work;
}

struct ProgEvent {
    float fraction;
    std::string status;
};

struct Mesh {
    std::vector<Vector3> vertices;
    std::vector<std::vector<size_t>> triangles;
};

static double randSym(double amplitude)
{
    return (static_cast<int>(below(2001)) - 1000) / 1000.0 * amplitude;
}

static Mesh makeGrid(size_t w, size_t h, double jitter)
{
    Mesh mesh;
    for (size_t j = 0; j <= h; ++j) {
        for (size_t i = 0; i <= w; ++i) {
            mesh.vertices.push_back(Vector3(
                static_cast<double>(i) + (jitter > 0.0 ? randSym(jitter) : 0.0),
                static_cast<double>(j) + (jitter > 0.0 ? randSym(jitter) : 0.0),
                jitter > 0.0 ? randSym(jitter) : 0.0));
        }
    }
    for (size_t j = 0; j < h; ++j) {
        for (size_t i = 0; i < w; ++i) {
            const size_t a = j * (w + 1) + i, b = a + 1, c = a + w + 1, d = c + 1;
            mesh.triangles.push_back({ a, b, d });
            mesh.triangles.push_back({ a, d, c });
        }
    }
    return mesh;
}

static Mesh makeBox(double jitter)
{
    Mesh mesh;
    const double c[8][3] = { { -1, -1, -1 }, { 1, -1, -1 }, { 1, 1, -1 }, { -1, 1, -1 },
        { -1, -1, 1 }, { 1, -1, 1 }, { 1, 1, 1 }, { -1, -1, 1 } };
    for (const auto& p : c)
        mesh.vertices.push_back(Vector3(p[0] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[1] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[2] + (jitter > 0.0 ? randSym(jitter) : 0.0)));
    // Outward winding (verified face by face).
    const size_t f[12][3] = { { 0, 2, 1 }, { 0, 3, 2 }, { 4, 5, 6 }, { 4, 6, 7 },
        { 0, 1, 5 }, { 0, 5, 4 }, { 2, 3, 7 }, { 2, 7, 6 }, { 0, 4, 7 }, { 0, 7, 3 },
        { 1, 2, 6 }, { 1, 6, 5 } };
    for (const auto& t : f)
        mesh.triangles.push_back({ t[0], t[1], t[2] });
    return mesh;
}

static Mesh makeTetra(double jitter)
{
    Mesh mesh;
    const double c[4][3] = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
    for (const auto& p : c)
        mesh.vertices.push_back(Vector3(p[0] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[1] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[2] + (jitter > 0.0 ? randSym(jitter) : 0.0)));
    const size_t f[4][3] = { { 0, 2, 1 }, { 0, 1, 3 }, { 0, 3, 2 }, { 1, 2, 3 } };
    for (const auto& t : f)
        mesh.triangles.push_back({ t[0], t[1], t[2] });
    return mesh;
}

static Mesh makeOcta(double jitter)
{
    Mesh mesh;
    const double c[6][3]
        = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
    for (const auto& p : c)
        mesh.vertices.push_back(Vector3(p[0] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[1] + (jitter > 0.0 ? randSym(jitter) : 0.0),
            p[2] + (jitter > 0.0 ? randSym(jitter) : 0.0)));
    const size_t f[8][3] = { { 4, 0, 2 }, { 4, 2, 1 }, { 4, 1, 3 }, { 4, 3, 0 },
        { 5, 2, 0 }, { 5, 1, 2 }, { 5, 3, 1 }, { 5, 0, 3 } };
    for (const auto& t : f)
        mesh.triangles.push_back({ t[0], t[1], t[2] });
    return mesh;
}

static Mesh makeSphere(size_t nlat, size_t nlon, double jitter)
{
    Mesh mesh;
    mesh.vertices.push_back(Vector3(0, 0, 1));
    for (size_t j = 1; j < nlat; ++j) {
        const double theta = M_PI * static_cast<double>(j) / static_cast<double>(nlat);
        for (size_t i = 0; i < nlon; ++i) {
            const double phi = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(nlon);
            mesh.vertices.push_back(Vector3(std::sin(theta) * std::cos(phi)
                        + (jitter > 0.0 ? randSym(jitter) : 0.0),
                std::sin(theta) * std::sin(phi) + (jitter > 0.0 ? randSym(jitter) : 0.0),
                std::cos(theta) + (jitter > 0.0 ? randSym(jitter) : 0.0)));
        }
    }
    mesh.vertices.push_back(Vector3(0, 0, -1));
    const size_t south = mesh.vertices.size() - 1;
    for (size_t i = 0; i < nlon; ++i) {
        const size_t a = 1 + i, b = 1 + (i + 1) % nlon;
        mesh.triangles.push_back({ 0, b, a });
        const size_t ring = 1 + (nlat - 2) * nlon;
        mesh.triangles.push_back({ south, ring + i, ring + (i + 1) % nlon });
    }
    for (size_t j = 0; j + 2 < nlat; ++j) {
        for (size_t i = 0; i < nlon; ++i) {
            const size_t a = 1 + j * nlon + i, b = 1 + j * nlon + (i + 1) % nlon;
            const size_t c = a + nlon, d = b + nlon;
            mesh.triangles.push_back({ a, b, d });
            mesh.triangles.push_back({ a, d, c });
        }
    }
    return mesh;
}

// Analytic saddle z = (x^2 - y^2)/2: indefinite curvature tensors.
static Mesh makeSaddle(size_t w, size_t h)
{
    Mesh mesh;
    for (size_t j = 0; j <= h; ++j) {
        for (size_t i = 0; i <= w; ++i) {
            const double x = static_cast<double>(i) - static_cast<double>(w) / 2.0;
            const double y = static_cast<double>(j) - static_cast<double>(h) / 2.0;
            mesh.vertices.push_back(Vector3(x, y, (x * x - y * y) / 2.0));
        }
    }
    for (size_t j = 0; j < h; ++j) {
        for (size_t i = 0; i < w; ++i) {
            const size_t a = j * (w + 1) + i, b = a + 1, c = a + w + 1, d = c + 1;
            mesh.triangles.push_back({ a, b, d });
            mesh.triangles.push_back({ a, d, c });
        }
    }
    return mesh;
}

static Mesh makeStrip(size_t n, double jitter)
{
    Mesh mesh;
    for (size_t i = 0; i <= n; ++i) {
        mesh.vertices.push_back(Vector3(static_cast<double>(i)
                    + (jitter > 0.0 ? randSym(jitter) : 0.0),
            (jitter > 0.0 ? randSym(jitter) : 0.0), (jitter > 0.0 ? randSym(jitter) : 0.0)));
        mesh.vertices.push_back(Vector3(static_cast<double>(i)
                    + (jitter > 0.0 ? randSym(jitter) : 0.0),
            1.0 + (jitter > 0.0 ? randSym(jitter) : 0.0),
            (jitter > 0.0 ? randSym(jitter) : 0.0)));
    }
    for (size_t i = 0; i < n; ++i)
        mesh.triangles.push_back({ 2 * i, 2 * i + 1, 2 * i + 2 });
    return mesh;
}

static Mesh makeFan(size_t n, double jitter)
{
    Mesh mesh;
    mesh.vertices.push_back(Vector3(0, 0, (jitter > 0.0 ? randSym(jitter) : 0.0)));
    for (size_t i = 0; i < n; ++i) {
        const double a = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
        mesh.vertices.push_back(Vector3(std::cos(a) + (jitter > 0.0 ? randSym(jitter) : 0.0),
            std::sin(a) + (jitter > 0.0 ? randSym(jitter) : 0.0),
            (jitter > 0.0 ? randSym(jitter) : 0.0)));
    }
    for (size_t i = 0; i < n; ++i)
        mesh.triangles.push_back({ 0, 1 + i, 1 + (i + 1) % n });
    return mesh;
}

static Mesh makeSoup(size_t nv, size_t nt)
{
    Mesh mesh;
    for (size_t i = 0; i < nv; ++i)
        mesh.vertices.push_back(Vector3(randSym(2.0), randSym(2.0), randSym(2.0)));
    for (size_t t = 0; t < nt; ++t) {
        size_t a = below(nv), b = below(nv), c = below(nv);
        if (b == a)
            b = (b + 1) % nv;
        if (c == a || c == b)
            c = (c + 2) % nv;
        if (c == a || c == b)
            c = (c + 1) % nv;
        mesh.triangles.push_back({ a, b, c });
    }
    return mesh;
}

static Mesh makeFlatQuad()
{
    Mesh mesh;
    mesh.vertices.push_back(Vector3(0, 0, 0));
    mesh.vertices.push_back(Vector3(1, 0, 0));
    mesh.vertices.push_back(Vector3(1, 1, 0));
    mesh.vertices.push_back(Vector3(0, 1, 0));
    mesh.triangles.push_back({ 0, 1, 2 });
    mesh.triangles.push_back({ 0, 2, 3 });
    return mesh;
}

// Concatenate parts with an offset so each stays its own island.
static Mesh makeComposite(const std::vector<Mesh>& parts, double spacing)
{
    Mesh mesh;
    double dx = 0.0;
    for (const Mesh& part : parts) {
        const size_t base = mesh.vertices.size();
        double maxX = 0.0;
        for (const Vector3& v : part.vertices) {
            mesh.vertices.push_back(Vector3(v.x() + dx, v.y(), v.z()));
            maxX = std::max(maxX, v.x() + dx);
        }
        for (const auto& t : part.triangles)
            mesh.triangles.push_back({ base + t[0], base + t[1], base + t[2] });
        dx = maxX + spacing;
    }
    return mesh;
}

// An open box (one face removed): a mesh with a boundary hole.
static Mesh makeOpenBox()
{
    Mesh mesh = makeBox(0.0);
    mesh.triangles.erase(mesh.triangles.begin());
    mesh.triangles.erase(mesh.triangles.begin());
    return mesh;
}

// A grid with its middle quad punched out: an interior hole.
static Mesh makeGridHole(size_t w, size_t h)
{
    Mesh mesh = makeGrid(w, h, 0.0);
    std::vector<std::vector<size_t>> kept;
    for (size_t j = 0; j < h; ++j) {
        for (size_t i = 0; i < w; ++i) {
            if (i == w / 2 && j == h / 2)
                continue;
            const size_t a = j * (w + 1) + i, b = a + 1, c = a + w + 1, d = c + 1;
            kept.push_back({ a, b, d });
            kept.push_back({ a, d, c });
        }
    }
    mesh.triangles = kept;
    return mesh;
}

// Polylines snapped to actual mesh edges (distance 0: guaranteed hits)
// unless `far` (offset by +10: guaranteed misses).
static std::vector<std::vector<Vector3>> randomEdgeLines(const Mesh& mesh, size_t k, bool far)
{
    std::vector<std::vector<Vector3>> lines;
    const double off = far ? 10.0 : 0.0;
    for (size_t i = 0; i < k; ++i) {
        if (mesh.triangles.empty())
            break;
        const auto& t = mesh.triangles[below(mesh.triangles.size())];
        if (t.size() != 3 || t[0] >= mesh.vertices.size() || t[1] >= mesh.vertices.size())
            continue;
        std::vector<Vector3> line;
        line.push_back(mesh.vertices[t[0]] + Vector3(off, off, off));
        line.push_back(mesh.vertices[t[1]] + Vector3(off, off, off));
        lines.push_back(line);
    }
    return lines;
}

// A 3-point polyline through the mesh bounding box: usually hits something.
static std::vector<std::vector<Vector3>> randomBoxLines(const Mesh& mesh, size_t k)
{
    std::vector<std::vector<Vector3>> lines;
    if (mesh.vertices.empty())
        return lines;
    Vector3 lo = mesh.vertices[0], hi = mesh.vertices[0];
    for (const Vector3& v : mesh.vertices) {
        lo = Vector3(std::min(lo.x(), v.x()), std::min(lo.y(), v.y()), std::min(lo.z(), v.z()));
        hi = Vector3(std::max(hi.x(), v.x()), std::max(hi.y(), v.y()), std::max(hi.z(), v.z()));
    }
    for (size_t i = 0; i < k; ++i) {
        const double t = (below(101) + 25) / 150.0;
        std::vector<Vector3> line;
        line.push_back(Vector3(lo.x(), lo.y() + t * (hi.y() - lo.y()), lo.z() + t * (hi.z() - lo.z())));
        line.push_back(Vector3((lo.x() + hi.x()) / 2.0, lo.y() + (1.0 - t) * (hi.y() - lo.y()),
            hi.z() - t * (hi.z() - lo.z())));
        line.push_back(Vector3(hi.x(), hi.y() - t * (hi.y() - lo.y()), lo.z() + (1.0 - t) * (hi.z() - lo.z())));
        lines.push_back(line);
    }
    return lines;
}

static std::vector<std::vector<Vector3>> junkLines()
{
    return {
        {},
        { Vector3(1.0, 0.0, 0.0) },
        { Vector3(0.0, 0.0, 1.0), Vector3(0.0, 0.0, 1.0) },
        { Vector3(100.0, 0.0, 0.0), Vector3(101.0, 0.0, 0.0) },
    };
}

// Density modes: 0 = empty (default off), 1 = random per-vertex field,
// 2 = uniform 1.0 (normalizes to OFF), 3 = wrong-size (ignored),
// 4 = adversarial boundary values (clamps, non-finite).
static std::vector<double> randomDensity(size_t nv, int mode)
{
    std::vector<double> field;
    if (mode == 0 || nv == 0)
        return field;
    if (mode == 2) {
        field.assign(nv, 1.0);
        return field;
    }
    if (mode == 3) {
        field.assign(nv > 1 ? nv - 1 : 2, 2.0);
        return field;
    }
    field.reserve(nv);
    for (size_t i = 0; i < nv; ++i) {
        if (mode == 4) {
            static const double kVals[]
                = { 0.25, 4.0, 0.24, 4.01, 0.0, -1.0, 1.0, 2.0, 0.5, 100.0 };
            double v = kVals[below(10)];
            if (below(10) == 0)
                v = below(2) == 0 ? std::numeric_limits<double>::quiet_NaN()
                                  : std::numeric_limits<double>::infinity();
            field.push_back(v);
        } else {
            // -0.5..3.5: exercises the clamp on both sides plus interior.
            field.push_back(randSym(2.0) + 1.5);
        }
    }
    return field;
}

struct Settings {
    size_t target = 100;
    double scaling = 0.0;
    double adaptivity = 1.0;
    double hard = 90.0;
    double anisotropy = 1.0;
    double smooth = 0.0;
    int model = 0;
    bool symmetry = false;
    int symmetryAxis = -1;
    int densityMode = 0;
    bool uvs = false;
    bool quiet = false;
};

// Listed C++-self-nondeterministic cases (demonstrated by three dump runs
// during development: the listed ids disagreed with THEMSELVES across
// runs, so no port can pin them exactly).
//
// Mechanism (one for all): the C++ cover solve carries ~1e-13 run-to-run
// TBB noise (isotropic outputs are bitwise-stable; IUV wobbles), and the
// quad extractor's integer rounding amplifies it into topological flips
// (e.g. case 14, a flat quad: runs 1-2 agree at 32 verts/30 quads, run 3
// lands on 36/34). The repo's own bench already accepts this reality
// (bench/run.py compares counts with a 5% tolerance, not exactly).
// - EPX_IDS: real outputs (remeshed quads/verts/uvs, island counts)
//   flipped. Robustness-only.
// - ECX_IDS: real outputs stable; only the raw extracted-connection
//   capture (a [param]-preview intermediate the cleanup absorbs) flipped
//   in count or value. Strict except CONN/MOVED, which report.
static bool isEpxId(int id)
{
    static const int kIds[] = { 0, 14, 16, 17, 22, 24, 31, 35, 45, 46, 62, 69,
        73, 74, 80, 93, 94, 96, 99, 101, 102, 103, 114, 117, 118, 120, 123, 129,
        133, 134, 135, 141, 143, 145, 168, 173, 175, 176, 177, 183, 190, 194,
        195, 234, 235, 243, 244, 245, 248, 250, 251, 254, 255, 256, 261, 265,
        275, 276, 277, 280, 282, 286, 288 };
    for (int k : kIds) {
        if (k == id)
            return true;
    }
    return false;
}

static bool isEcxId(int id)
{
    static const int kIds[] = { 8, 9, 27, 32, 57, 58, 60, 61, 64, 67, 71, 79,
        100, 107, 111, 119, 122, 125, 128, 130, 131, 132, 137, 160, 163, 170,
        172, 174, 186, 187, 208, 232, 246, 260, 278, 283 };
    for (int k : kIds) {
        if (k == id)
            return true;
    }
    return false;
}

static void dumpCase(int id, const char* kind, const Mesh& mesh, const Settings& s,
    const std::vector<std::vector<Vector3>>& guides,
    const std::vector<std::vector<Vector3>>& sharps, bool robust = false)
{
    // EPX (like the parameterizer lane's PPX): robustness-only. The port
    // must match every deterministic structural fact (ok flag, island
    // length, decimated + isotropic + cover + singular + symmetry outputs,
    // phase head/tail, progress prefix/suffix + robust facts) and solve
    // everything C++ solves; cliff-amplified outputs (remeshed
    // verts/quads/uvs, connections) are reported, not asserted. Tagged by
    // input construction (non-manifold soup / degenerate meshes where
    // backend noise legitimately exceeds 1e-6) plus the listed
    // C++-self-nondeterministic ids above. ECX is the middle tier (listed
    // ids): strict except the connection capture, which reports.
    robust = robust || isEpxId(id);
    const bool connOnly = !robust && isEcxId(id);
    const char* tag = robust ? "EPX" : (connOnly ? "ECX" : "CASE");
    std::printf("%s %d KIND %s NV %zu NT %zu TARGET %zu SCALING ", tag, id, kind,
        mesh.vertices.size(), mesh.triangles.size(), s.target);
    printDouble(s.scaling);
    std::printf(" ADAPT ");
    printDouble(s.adaptivity);
    std::printf(" HARD ");
    printDouble(s.hard);
    std::printf(" ANISO ");
    printDouble(s.anisotropy);
    std::printf(" SMOOTH ");
    printDouble(s.smooth);
    std::printf(" MODEL %d SYM %d SYMAXIS %d GUIDES %zu SHARPS %zu DENSITY %d UVS %d QUIET %d\n",
        s.model, s.symmetry ? 1 : 0, s.symmetryAxis, guides.size(), sharps.size(),
        s.densityMode, s.uvs ? 1 : 0, s.quiet ? 1 : 0);
    for (const Vector3& v : mesh.vertices) {
        std::printf("V ");
        printVec3(v);
        std::printf("\n");
    }
    for (const auto& t : mesh.triangles) {
        std::printf("T %zu", t.size());
        for (size_t i : t)
            std::printf(" %zu", i);
        std::printf("\n");
    }
    for (const auto& line : guides) {
        std::printf("GUIDE %zu", line.size());
        for (const Vector3& p : line) {
            std::printf(" ");
            printVec3(p);
        }
        std::printf("\n");
    }
    for (const auto& line : sharps) {
        std::printf("SHARP %zu", line.size());
        for (const Vector3& p : line) {
            std::printf(" ");
            printVec3(p);
        }
        std::printf("\n");
    }
    const std::vector<double> density = randomDensity(mesh.vertices.size(), s.densityMode);
    // NOTE: randomDensity consumes the splitmix stream, so the replay must
    // NOT regenerate it: the values print below.
    std::printf("RHO %zu", density.size());
    for (double d : density) {
        std::printf(" ");
        printDouble(d);
    }
    std::printf("\n");

    std::vector<ProgEvent> events;
    Engine remesher(mesh.vertices, mesh.triangles);
    remesher.setTargetTriangleCount(s.target);
    // Unconditional (unlike the CLI's `> 0` gate): the oracle covers the
    // stored-but-skipped path too.
    remesher.setScaling(s.scaling);
    remesher.setModelType(s.model == 0 ? ModelType::Organic : ModelType::HardSurface);
    remesher.setGradientAdaptivity(s.adaptivity);
    remesher.setAnisotropy(s.anisotropy);
    remesher.setSharpEdgeDegrees(s.hard);
    remesher.setSmoothNormalDegrees(s.smooth);
    remesher.setSymmetryEnabled(s.symmetry);
    remesher.setSymmetryPlane(s.symmetryAxis);
    remesher.setGuidePolylines(guides);
    if (id % 2 == 0)
        remesher.setFeaturePolylines(sharps);
    else
        remesher.setSharpPolylines(sharps);
    remesher.setDensityMultipliers(density);
    remesher.setComputeRemeshedUvs(s.uvs);
    remesher.setQuiet(s.quiet);
    // The handler stays installed even on quiet cases: quiet only skips
    // DOWNSTREAM installation, and the engine-level events still fire.
    remesher.setProgressHandler([](void* tag, float progress, const char* status) {
        static_cast<std::vector<ProgEvent>*>(tag)->push_back(
            { progress, statusToken(status) });
    });
    remesher.setTag(&events);
    const bool ok = remesher.remesh();

    std::printf("PROG %zu", events.size());
    for (const ProgEvent& e : events) {
        std::printf(" ");
        printFrac(e.fraction);
        std::printf(" %s", e.status.c_str());
    }
    std::printf("\nRES %d\n", ok ? 1 : 0);
    if (!ok)
        return;
    const auto& rverts = remesher.remeshedVertices();
    std::printf("RV %zu", rverts.size());
    for (const Vector3& v : rverts) {
        std::printf(" ");
        printVec3(v);
    }
    std::printf("\n");
    const auto& rquads = remesher.remeshedQuads();
    std::printf("RQ %zu", rquads.size());
    for (const auto& q : rquads) {
        std::printf(" %zu", q.size());
        for (size_t i : q)
            std::printf(" %zu", i);
    }
    std::printf("\n");
    const auto& ruvs = remesher.remeshedVertexUvs();
    std::printf("RUV %zu", ruvs.size());
    for (const Vector2& uv : ruvs) {
        std::printf(" ");
        printVec2(uv);
    }
    std::printf("\n");
    const auto& isl = remesher.islandOutputQuadCounts();
    std::printf("ISL %zu", isl.size());
    for (size_t c : isl)
        std::printf(" %zu", c);
    std::printf("\n");
    std::printf("DEC %d", remesher.decimated() ? 1 : 0);
    const auto& dverts = remesher.decimatedVertices();
    const auto& dtris = remesher.decimatedTriangles();
    std::printf(" %zu %zu", dverts.size(), dtris.size());
    for (const Vector3& v : dverts) {
        std::printf(" ");
        printVec3(v);
    }
    for (const auto& t : dtris) {
        std::printf(" T%zu", t.size());
        for (size_t i : t)
            std::printf(" %zu", i);
    }
    std::printf("\n");
    const auto& iverts = remesher.isotropicVertices();
    const auto& itris = remesher.isotropicTriangles();
    std::printf("ISO %zu %zu", iverts.size(), itris.size());
    for (const Vector3& v : iverts) {
        std::printf(" ");
        printVec3(v);
    }
    for (const auto& t : itris) {
        std::printf(" T%zu", t.size());
        for (size_t i : t)
            std::printf(" %zu", i);
    }
    std::printf("\n");
    const auto& iuvs = remesher.isotropicTriangleUvs();
    std::printf("IUV %zu", iuvs.size());
    for (const auto& tri : iuvs)
        for (const Vector2& uv : tri) {
            std::printf(" ");
            printVec2(uv);
        }
    std::printf("\n");
    const auto& iouvs = remesher.isotropicOriginalTriangleUvs();
    std::printf("IOUV %zu", iouvs.size());
    for (const auto& tri : iouvs)
        for (const Vector2& uv : tri) {
            std::printf(" ");
            printVec2(uv);
        }
    std::printf("\n");
    const auto& sing = remesher.isotropicSingularVertices();
    std::printf("SING %zu", sing.size());
    for (const Vector3& v : sing) {
        std::printf(" ");
        printVec3(v);
    }
    std::printf("\n");
    const auto& conns = remesher.isotropicExtractedConnections();
    const auto& moved = remesher.isotropicExtractedConnectionMoved();
    std::printf("CONN %zu %zu", conns.size(), moved.size());
    for (const auto& c : conns) {
        std::printf(" ");
        printVec3(c.first);
        std::printf(" ");
        printVec3(c.second);
    }
    for (unsigned char m : moved)
        std::printf(" %u", static_cast<unsigned>(m));
    std::printf("\n");
    std::printf("SYM %d ", remesher.symmetryPlaneAxis());
    printDouble(remesher.symmetryPlaneOffset());
    std::printf(" ");
    printDouble(remesher.symmetryPlaneScore());
    std::printf("\n");
    const auto& report = remesher.phaseReport();
    std::printf("PHASE %zu", report.size());
    for (const std::string& line : report)
        std::printf(" [%s]", phaseToken(line).c_str());
    std::printf("\n");
}

static size_t pickTarget()
{
    switch (below(10)) {
    case 0:
    case 1:
    case 2:
        return 100;
    case 3:
        return 20;
    case 4:
        return 50;
    case 5:
        return 200;
    case 6:
        return 500;
    case 7:
        // Tiny target over a dense mesh: trips the decimator (RAN path).
        return 1 + below(8);
    default:
        return 100;
    }
}

static double pickScaling()
{
    switch (below(8)) {
    case 0:
    case 1:
    case 2:
        return 0.0;
    case 3:
        return 1.0;
    case 4:
        return 0.5;
    case 5:
        return 2.0;
    case 6:
        return -1.0;
    default:
        return 0.75;
    }
}

static double pickAdaptivity()
{
    switch (below(8)) {
    case 0:
    case 1:
    case 2:
        return 1.0;
    case 3:
        return 0.0;
    case 4:
        return 0.5;
    case 5:
        return 2.0;
    case 6:
        return -0.5;
    default:
        return 1.5;
    }
}

static double pickHard()
{
    switch (below(8)) {
    case 0:
    case 1:
    case 2:
        return 90.0;
    case 3:
        return 0.0;
    case 4:
        return 30.0;
    case 5:
        return 60.0;
    case 6:
        return 180.0;
    default:
        return 45.0;
    }
}

static double pickAnisotropy()
{
    switch (below(6)) {
    case 0:
    case 1:
        return 1.0;
    case 2:
        return 0.0;
    case 3:
        return 0.5;
    case 4:
        return 2.0;
    default:
        return 1.5;
    }
}

static double pickSmooth()
{
    switch (below(6)) {
    case 0:
    case 1:
    case 2:
        return 0.0;
    case 3:
        return 30.0;
    case 4:
        return 60.0;
    default:
        return 15.0;
    }
}

static std::pair<const char*, Mesh> randomMesh()
{
    const double jitterPick = below(2) == 0 ? 0.0 : (below(2) == 0 ? 0.05 : 0.3);
    switch (below(14)) {
    case 0:
    case 1:
        return { "grid", makeGrid(1 + below(5), 1 + below(5), jitterPick) };
    case 2:
        return { "box", makeBox(jitterPick) };
    case 3:
        return below(2) == 0 ? std::pair<const char*, Mesh> { "tetra", makeTetra(jitterPick) }
                             : std::pair<const char*, Mesh> { "octa", makeOcta(jitterPick) };
    case 4:
        return { "sphere", makeSphere(3 + below(4), 4 + below(6), jitterPick) };
    case 5:
        return { "strip", makeStrip(2 + below(8), jitterPick) };
    case 6:
        return { "fan", makeFan(3 + below(8), jitterPick) };
    case 7:
        return { "soup", makeSoup(4 + below(12), 2 + below(10)) };
    case 8:
        return { "quad", makeFlatQuad() };
    case 9:
        return { "saddle", makeSaddle(2 + below(4), 2 + below(4)) };
    case 10:
        return { "twobox",
            makeComposite({ makeBox(jitterPick), makeBox(jitterPick) }, 5.0) };
    case 11:
        return { "boxtetra",
            makeComposite({ makeBox(jitterPick), makeTetra(jitterPick) }, 5.0) };
    case 12:
        return { "openbox", makeOpenBox() };
    default:
        return { "gridhole", makeGridHole(3 + below(3), 3 + below(3)) };
    }
}

static std::vector<std::vector<Vector3>> randomLines(const Mesh& mesh)
{
    switch (below(10)) {
    case 0:
    case 1:
    case 2:
        return {};
    case 3:
    case 4:
        return randomEdgeLines(mesh, 1 + below(2), false);
    case 5:
        return randomEdgeLines(mesh, 1, true);
    case 6:
        return randomBoxLines(mesh, 1);
    case 7:
        return junkLines();
    case 8:
        return randomEdgeLines(mesh, 2 + below(3), false);
    default:
        return randomBoxLines(mesh, 2);
    }
}

static Settings randomSettings()
{
    Settings s;
    s.target = pickTarget();
    s.scaling = pickScaling();
    s.adaptivity = pickAdaptivity();
    s.hard = pickHard();
    s.anisotropy = pickAnisotropy();
    s.smooth = pickSmooth();
    s.model = below(4) == 0 ? 1 : 0;
    if (below(4) == 0) {
        s.symmetry = true;
        const uint64_t ax = below(6);
        s.symmetryAxis = ax < 3 ? static_cast<int>(ax) : -1;
    }
    const std::uint64_t dr = below(10);
    if (dr == 0)
        s.densityMode = 1;
    else if (dr == 1)
        s.densityMode = 2;
    else if (dr == 2)
        s.densityMode = 4;
    else if (dr == 3)
        s.densityMode = 3;
    s.uvs = below(2) == 0;
    s.quiet = below(10) == 0;
    return s;
}

static void dumpSeededCase(int id)
{
    auto [meshKind, mesh] = randomMesh();
    const Settings s = randomSettings();
    const bool isSoup = std::string(meshKind) == "soup";
    dumpCase(id, meshKind, mesh, s, randomLines(mesh), randomLines(mesh), isSoup);
}

// Adversarial battery: every setter cliff, the invalid-input rejections,
// decimation RAN/SKIPPED incl. the sharp-lock path, symmetry active vs
// fallback, density modes, the UV atlas single/multi island shapes, quiet
// with a handler installed, and degenerate geometry.
static void dumpAdversarial(int& id)
{
    const Settings def;
    const Mesh quad = makeFlatQuad();
    const Mesh box = makeBox(0.0);
    const Mesh twoBox = makeComposite({ makeBox(0.0), makeBox(0.0) }, 5.0);
    const Mesh dense = makeGrid(20, 20, 0.0);
    // Target cliffs on the box (default mesh, one knob). NOTE: no absurd
    // targets (1e6 over-tessellates the box into millions of triangles and
    // effectively hangs the isotropic remesher on both sides — out of
    // contract, since real target-quads stay near input scale).
    for (size_t target : { 1, 2, 7, 8, 9, 12, 13, 100, 2000 }) {
        Settings s = def;
        s.target = target;
        dumpCase(id++, "target", box, s, {}, {});
    }
    // Decimation RAN battery (dense grid, small target): sharp on/off
    // crosses the vertex-lock path; adaptivity 0/1 crosses the field.
    for (double hard : { 0.0, 90.0 }) {
        for (double adapt : { 0.0, 1.0 }) {
            Settings s = def;
            s.target = 20;
            s.hard = hard;
            s.adaptivity = adapt;
            dumpCase(id++, "decrun", dense, s, {}, {});
        }
    }
    // Mixed RAN/SKIPPED across islands (dense + coarse parts).
    {
        const Mesh mixed
            = makeComposite({ makeGrid(20, 20, 0.0), makeTetra(0.0) }, 30.0);
        Settings s = def;
        s.target = 40;
        dumpCase(id++, "decmix", mixed, s, {}, {});
    }
    // Scalar setting cliffs.
    // NOTE: no tiny-positive scalings (1e-12 asks the extractor for
    // ~1e24 quads and OOMs both sides — out of contract, since real
    // --edge-scaling stays near 1).
    for (double scaling : { -2.0, -1.0, 0.0, 0.25, 0.5, 1.0, 2.0, 4.0 }) {
        Settings s = def;
        s.scaling = scaling;
        dumpCase(id++, "scale", quad, s, {}, {});
    }
    for (double adapt : { -1.0, -0.5, 0.0, 1e-9, 0.5, 1.0, 2.0, 3.0 }) {
        Settings s = def;
        s.adaptivity = adapt;
        dumpCase(id++, "adapt", quad, s, {}, {});
    }
    for (double hard : { -10.0, 0.0, 30.0, 90.0, 180.0, 360.0 }) {
        Settings s = def;
        s.hard = hard;
        dumpCase(id++, "hard", box, s, {}, {});
    }
    for (double aniso : { -1.0, 0.0, 0.5, 1.0, 2.0, 3.0 }) {
        Settings s = def;
        s.anisotropy = aniso;
        dumpCase(id++, "aniso", quad, s, {}, {});
    }
    for (double smooth : { -5.0, 0.0, 15.0, 30.0, 90.0 }) {
        Settings s = def;
        s.smooth = smooth;
        dumpCase(id++, "smooth", box, s, {}, {});
    }
    for (int model : { 0, 1 }) {
        Settings s = def;
        s.model = model;
        dumpCase(id++, "model", box, s, {}, {});
    }
    // Symmetry: off, auto on symmetric (active) vs asymmetric (fallback),
    // fixed axes incl. out-of-range (falls back to detect).
    {
        Settings s = def;
        dumpCase(id++, "symoff", box, s, {}, {});
        s.symmetry = true;
        s.symmetryAxis = -1;
        dumpCase(id++, "symauto", box, s, {}, {});
        const Mesh asym = makeComposite({ makeBox(0.0), makeTetra(0.0) }, 5.0);
        dumpCase(id++, "symfallback", asym, s, {}, {});
        for (int axis : { 0, 1, 2, 3, -2 }) {
            Settings f = def;
            f.symmetry = true;
            f.symmetryAxis = axis;
            dumpCase(id++, "symfixed", box, f, {}, {});
        }
    }
    // Density modes on single + multi island.
    for (int dm : { 0, 1, 2, 3, 4 }) {
        Settings s = def;
        s.densityMode = dm;
        dumpCase(id++, "rho", box, s, {}, {});
        dumpCase(id++, "rhotwo", twoBox, s, {}, {});
    }
    // UV atlas shapes: off, single island (no-op pack), multi island
    // (real pack), with and without quiet.
    for (bool uvs : { false, true }) {
        for (bool quiet : { false, true }) {
            Settings s = def;
            s.uvs = uvs;
            s.quiet = quiet;
            dumpCase(id++, "uvbox", box, s, {}, {});
            dumpCase(id++, "uvtwo", twoBox, s, {}, {});
        }
    }
    // Guide/sharp junk + far + hits on the box.
    dumpCase(id++, "junkboth", box, def, junkLines(), junkLines());
    dumpCase(id++, "guidefar", box, def,
        { { Vector3(100.0, 0.0, 0.0), Vector3(101.0, 0.0, 0.0) } }, {});
    dumpCase(id++, "guidehit", box, def, randomEdgeLines(box, 2, false), {});
    dumpCase(id++, "sharphit", box, def, {}, randomEdgeLines(box, 2, false));
    // Degenerate geometry (all valid indices: the engine reads tri[0..2]
    // unconditionally past validation, so wrong-length triangles are out
    // of contract... except the validator itself rejects them — covered
    // below as RES 0).
    {
        Mesh zeroArea = makeFlatQuad();
        zeroArea.vertices.push_back(Vector3(0, 0, 0));
        zeroArea.vertices.push_back(Vector3(0, 0, 0));
        zeroArea.vertices.push_back(Vector3(0, 0, 0));
        const size_t n = zeroArea.vertices.size();
        zeroArea.triangles.push_back({ n - 3, n - 2, n - 1 });
        dumpCase(id++, "zeroarea", zeroArea, def, {}, {}, true);
        Mesh dupVert;
        dupVert.vertices.push_back(Vector3(0, 0, 0));
        dupVert.vertices.push_back(Vector3(0, 0, 0));
        dupVert.vertices.push_back(Vector3(1, 0, 0));
        dupVert.vertices.push_back(Vector3(0, 1, 0));
        dupVert.triangles.push_back({ 0, 1, 2 });
        dupVert.triangles.push_back({ 0, 2, 3 });
        dumpCase(id++, "dupvert", dupVert, def, {}, {}, true);
        Mesh single;
        single.vertices.push_back(Vector3(0, 0, 0));
        single.vertices.push_back(Vector3(1, 0, 0));
        single.vertices.push_back(Vector3(0, 1, 0));
        single.triangles.push_back({ 0, 1, 2 });
        dumpCase(id++, "single", single, def, {}, {});
        Mesh disjoint = single;
        disjoint.vertices.push_back(Vector3(5, 5, 5));
        disjoint.vertices.push_back(Vector3(6, 5, 5));
        disjoint.vertices.push_back(Vector3(5, 6, 5));
        disjoint.triangles.push_back({ 3, 4, 5 });
        dumpCase(id++, "disjoint", disjoint, def, {}, {});
        Mesh nonmanifold;
        nonmanifold.vertices.push_back(Vector3(0, 0, 0));
        nonmanifold.vertices.push_back(Vector3(1, 0, 0));
        nonmanifold.vertices.push_back(Vector3(0, 1, 0));
        nonmanifold.vertices.push_back(Vector3(0, 0, 1));
        nonmanifold.vertices.push_back(Vector3(0, -1, 0));
        nonmanifold.triangles.push_back({ 0, 1, 2 });
        nonmanifold.triangles.push_back({ 1, 0, 3 });
        nonmanifold.triangles.push_back({ 0, 1, 4 });
        dumpCase(id++, "nonmanifold", nonmanifold, def, {}, {}, true);
        Mesh coincident;
        coincident.vertices.push_back(Vector3(0, 0, 0));
        coincident.vertices.push_back(Vector3(1, 0, 0));
        coincident.vertices.push_back(Vector3(0, 1, 0));
        coincident.triangles.push_back({ 0, 1, 2 });
        coincident.triangles.push_back({ 0, 2, 1 });
        dumpCase(id++, "coincident", coincident, def, {}, {}, true);
        Mesh tiny = makeTetra(0.0);
        for (Vector3& v : tiny.vertices)
            v = Vector3(v.x() * 1e-9, v.y() * 1e-9, v.z() * 1e-9);
        dumpCase(id++, "tiny", tiny, def, {}, {});
        Mesh huge = makeTetra(0.0);
        for (Vector3& v : huge.vertices)
            v = Vector3(v.x() * 1e9, v.y() * 1e9, v.z() * 1e9);
        dumpCase(id++, "huge", huge, def, {}, {});
        Mesh floaters = makeComposite(
            { makeTetra(0.0), makeTetra(0.0), makeTetra(0.0), makeTetra(0.0) }, 5.0);
        dumpCase(id++, "floaters", floaters, def, {}, {});
        Mesh openbox = makeOpenBox();
        dumpCase(id++, "openbox", openbox, def, {}, {});
        Mesh gridhole = makeGridHole(4, 4);
        dumpCase(id++, "gridhole", gridhole, def, {}, {});
    }
    // Invalid inputs: all RES 0 (the handler still fires once at 1.0 with
    // the reason; stderr is CLI territory, not the oracle's).
    {
        Mesh empty;
        dumpCase(id++, "empty", empty, def, {}, {});
        Mesh noTris;
        noTris.vertices.push_back(Vector3(0, 0, 0));
        dumpCase(id++, "notris", noTris, def, {}, {});
        Settings zero = def;
        zero.target = 0;
        dumpCase(id++, "target0", box, zero, {}, {});
        Mesh nonTri = box;
        nonTri.triangles.push_back({ 0, 1 });
        dumpCase(id++, "nontri", nonTri, def, {}, {});
        Mesh oor = box;
        oor.triangles.push_back({ 0, 1, 999 });
        dumpCase(id++, "oor", oor, def, {}, {});
    }
}

// Timing mesh, analytic so the Rust harness regenerates it bit-identically:
// 64x64 grid, z = 0.1 * sin(i) * cos(j) (same mesh the frame/quad lanes
// time, so the ratios are comparable).
static Mesh makeTimingMesh()
{
    const size_t w = 64, h = 64;
    Mesh mesh;
    for (size_t j = 0; j <= h; ++j)
        for (size_t i = 0; i <= w; ++i)
            mesh.vertices.push_back(Vector3(static_cast<double>(i), static_cast<double>(j),
                0.1 * std::sin(static_cast<double>(i)) * std::cos(static_cast<double>(j))));
    for (size_t j = 0; j < h; ++j)
        for (size_t i = 0; i < w; ++i) {
            const size_t a = j * (w + 1) + i, b = a + 1, c = a + w + 1, d = c + 1;
            mesh.triangles.push_back({ a, b, d });
            mesh.triangles.push_back({ a, d, c });
        }
    return mesh;
}

static void timeRemesh()
{
    const Mesh mesh = makeTimingMesh();
    // Warmup (unprinted): page faults, TBB pool spin-up, allocator caches.
    {
        Engine warmup(mesh.vertices, mesh.triangles);
        warmup.setTargetTriangleCount(2000);
        warmup.remesh();
    }
    for (int sample = 0; sample < 3; ++sample) {
        Engine remesher(mesh.vertices, mesh.triangles);
        remesher.setTargetTriangleCount(2000);
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = remesher.remesh();
        const auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double checksum = 0.0;
        if (ok) {
            for (const Vector3& v : remesher.remeshedVertices())
                checksum += v.x() + v.y() + v.z();
        }
        std::printf("T ar ok=%d ms=%.3f quads=%zu verts=%zu checksum=", ok ? 1 : 0, ms,
            remesher.remeshedQuads().size(), remesher.remeshedVertices().size());
        printDouble(checksum);
        std::printf("\n");
    }
}

int main()
{
    std::printf("ARDIFF1\n");
    int id = 0;
    for (int i = 0; i < 200; ++i)
        dumpSeededCase(id++);
    dumpAdversarial(id);
    std::printf("CASES %d\n", id);
    timeRemesh();
    return 0;
}
