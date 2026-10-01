// Differential oracle dump for the quadextractor Rust port (wave 2,
// rs-quadextractor lane). Generates fixed edge/adversarial cases plus seeded
// random jittered-grid meshes, runs each through the C++ QuadExtractor, and
// prints inputs + outputs in a token format that
// rust/core/tests/quad_extractor_diff.rs replays. Also times a large grid
// extraction for the runtime ratio (3 samples).
//
// Build-only helper: not registered with ctest. Run it and redirect stdout
// to tests/fixtures/quadextractor_diff.txt, then commit the fixture.
import retopo.core.quad_extractor;
import retopo.core.vector2;
import retopo.core.vector3;

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <vector>

using AutoRemesher::QuadExtractor;
using AutoRemesher::Vector2;
using AutoRemesher::Vector3;

// splitmix64: fixed edge cases below are literal, the random cases draw
// from this stream with a fixed seed, so the fixture is reproducible.
static std::uint64_t g_state = 0x9E3779B97F4A7C15ull ^ 0x514EAD37ull;

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

static double nextFrac()
{
    return (nextU64() >> 11) * (1.0 / 9007199254740992.0);
}

static double range(double lo, double hi)
{
    return lo + (hi - lo) * nextFrac();
}

struct CaseInput {
    std::vector<Vector3> vertices;
    std::vector<std::vector<size_t>> triangles;
    std::vector<std::vector<Vector2>> uvs;
    std::vector<size_t> singular;
    bool computeUvs = false;
    // 0 = no original UVs, 1 = identical to uvs, 2 = perturbed copy.
    int origMode = 0;
    std::vector<std::vector<Vector2>> origUvs;
};

static void printVec3(const Vector3& v)
{
    std::printf("v %.17g %.17g %.17g\n", v.x(), v.y(), v.z());
}

static void printVec2(const Vector2& v)
{
    std::printf("u %.17g %.17g\n", v.x(), v.y());
}

static void dumpCase(size_t index, const CaseInput& in)
{
    if (nullptr != std::getenv("QX_VERBOSE"))
        std::fprintf(stderr, "CASE %zu\n", index);
    std::printf("CASE %zu\n", index);
    std::printf("V %zu\n", in.vertices.size());
    for (const auto& v : in.vertices)
        printVec3(v);
    std::printf("TRI %zu\n", in.triangles.size());
    for (const auto& t : in.triangles)
        std::printf("t %zu %zu %zu\n", t[0], t[1], t[2]);
    std::printf("UV %zu\n", in.uvs.size());
    for (const auto& row : in.uvs) {
        std::printf("u");
        for (const auto& uv : row)
            std::printf(" %.17g %.17g", uv.x(), uv.y());
        std::printf("\n");
    }
    std::printf("SING %zu\n", in.singular.size());
    if (!in.singular.empty()) {
        std::printf("s");
        for (size_t s : in.singular)
            std::printf(" %zu", s);
        std::printf("\n");
    }
    std::printf("FLAGS %d %d\n", in.computeUvs ? 1 : 0, in.origMode);
    if (in.origMode > 0) {
        std::printf("ORIG %zu\n", in.origUvs.size());
        for (const auto& row : in.origUvs) {
            std::printf("u");
            for (const auto& uv : row)
                std::printf(" %.17g %.17g", uv.x(), uv.y());
            std::printf("\n");
        }
    }

    QuadExtractor extractor(&in.vertices, &in.triangles, &in.uvs);
    extractor.setComputeVertexUvs(in.computeUvs);
    if (in.origMode > 0)
        extractor.setOriginalTriangleUvs(&in.origUvs);
    if (!in.singular.empty())
        extractor.setSingularVertices(&in.singular);
    // No progress handler: stdout stays a pure token stream (the C++
    // diagnostics are gated on the handler, so none fire here).
    if (nullptr != std::getenv("QX_VERBOSE")) {
        extractor.setProgressHandler(
            [](float, const char*) {});
    }
    bool ok = extractor.extract();
    std::printf("OK %d\n", ok ? 1 : 0);

    const auto& conns = extractor.extractedConnections();
    const auto& moved = extractor.extractedConnectionMoved();
    std::printf("CONN %zu\n", conns.size());
    for (size_t i = 0; i < conns.size(); ++i) {
        const auto& [a, b] = conns[i];
        std::printf("c %.17g %.17g %.17g %.17g %.17g %.17g %u\n",
            a.x(), a.y(), a.z(), b.x(), b.y(), b.z(),
            i < moved.size() ? moved[i] : 0);
    }
    const auto& verts = extractor.remeshedVertices();
    std::printf("REMESH %zu\n", verts.size());
    for (const auto& v : verts)
        printVec3(v);
    const auto& quads = extractor.remeshedQuads();
    std::printf("QUADS %zu\n", quads.size());
    for (const auto& q : quads) {
        std::printf("q %zu", q.size());
        for (size_t v : q)
            std::printf(" %zu", v);
        std::printf("\n");
    }
    const auto& ruv = extractor.remeshedVertexUvs();
    std::printf("RUV %zu\n", ruv.size());
    for (const auto& uv : ruv)
        printVec2(uv);
}

// Triangulated W(x)H grid, consistently wound: ids row-major over
// (W+1)x(H+1) vertices. Pure formula; the Rust timing test mirrors it.
static void makeGrid(size_t w, size_t h, CaseInput& in)
{
    in.vertices.clear();
    in.triangles.clear();
    in.uvs.clear();
    auto id = [w](size_t x, size_t y) { return y * (w + 1) + x; };
    for (size_t y = 0; y <= h; ++y) {
        for (size_t x = 0; x <= w; ++x)
            in.vertices.emplace_back((double)x, (double)y, 0.0);
    }
    for (size_t y = 0; y < h; ++y) {
        for (size_t x = 0; x < w; ++x) {
            size_t a = id(x, y);
            size_t b = id(x + 1, y);
            size_t c = id(x + 1, y + 1);
            size_t d = id(x, y + 1);
            in.triangles.push_back({ a, b, c });
            in.triangles.push_back({ a, c, d });
        }
    }
    in.uvs.resize(in.triangles.size());
    for (size_t i = 0; i < in.triangles.size(); ++i) {
        const auto& t = in.triangles[i];
        for (size_t k = 0; k < 3; ++k) {
            const auto& v = in.vertices[t[k]];
            in.uvs[i].emplace_back(v.x(), v.y());
        }
    }
}

static void finishOrig(CaseInput& in)
{
    if (1 == in.origMode) {
        in.origUvs = in.uvs;
    } else if (2 == in.origMode && !in.uvs.empty()) {
        in.origUvs = in.uvs;
        size_t t = (size_t)(nextU64() % in.origUvs.size());
        size_t k = (size_t)(nextU64() % 3);
        auto& uv = in.origUvs[t][k];
        uv = Vector2(uv.x() + 0.25, uv.y() - 0.125);
    }
}

// Fixed edge + adversarial cases: empty/single/degenerate inputs, collapsed
// and integer UVs, near-integer UVs (isoline-ratio cliff), positions on and
// near the PositionKey 1e-5 lattice, slivers, and a non-planar tetrahedron.
static void dumpEdgeCases(size_t& index)
{
    {
        CaseInput in; // empty mesh
        dumpCase(index++, in);
    }
    {
        // Single triangle, UVs spanning one integer in u.
        CaseInput in;
        in.vertices = { Vector3(0, 0, 0), Vector3(2, 0, 0), Vector3(0, 2, 0) };
        in.triangles = { { 0, 1, 2 } };
        in.uvs = { { Vector2(0, 0), Vector2(2, 0), Vector2(0, 2) } };
        in.computeUvs = true;
        dumpCase(index++, in);
    }
    {
        // Zero-area triangle (two corners identical).
        CaseInput in;
        in.vertices = { Vector3(0, 0, 0), Vector3(0, 0, 0), Vector3(0, 2, 0) };
        in.triangles = { { 0, 1, 2 } };
        in.uvs = { { Vector2(0, 0), Vector2(2, 0), Vector2(0, 2) } };
        dumpCase(index++, in);
    }
    {
        // Collapsed UVs: no isoline crosses anything.
        CaseInput in;
        in.vertices = { Vector3(0, 0, 0), Vector3(2, 0, 0), Vector3(0, 2, 0) };
        in.triangles = { { 0, 1, 2 } };
        in.uvs = { { Vector2(0.5, 0.5), Vector2(0.5, 0.5), Vector2(0.5, 0.5) } };
        in.computeUvs = true;
        dumpCase(index++, in);
    }
    {
        // Two disjoint triangles.
        CaseInput in;
        in.vertices = { Vector3(0, 0, 0), Vector3(2, 0, 0), Vector3(0, 2, 0),
            Vector3(5, 5, 0), Vector3(7, 5, 0), Vector3(5, 7, 0) };
        in.triangles = { { 0, 1, 2 }, { 3, 4, 5 } };
        in.uvs = { { Vector2(0, 0), Vector2(2, 0), Vector2(0, 2) },
            { Vector2(0, 0), Vector2(2, 0), Vector2(0, 2) } };
        dumpCase(index++, in);
    }
    {
        // Non-planar tetrahedron.
        CaseInput in;
        in.vertices = { Vector3(0, 0, 0), Vector3(2, 0, 0.5), Vector3(0, 2, -0.5),
            Vector3(0.5, 0.5, 2) };
        in.triangles = { { 0, 1, 2 }, { 0, 1, 3 }, { 1, 2, 3 }, { 2, 0, 3 } };
        in.uvs = { { Vector2(0, 0), Vector2(2, 0), Vector2(0, 2) },
            { Vector2(0, 0), Vector2(2, 0), Vector2(1, 1) },
            { Vector2(2, 0), Vector2(0, 2), Vector2(1, 1) },
            { Vector2(0, 2), Vector2(0, 0), Vector2(1, 1) } };
        in.computeUvs = true;
        dumpCase(index++, in);
    }
    {
        // 1x1 grid with exact integer UVs (edge-collapsed isolines).
        CaseInput in;
        makeGrid(1, 1, in);
        in.computeUvs = true;
        dumpCase(index++, in);
    }
    {
        // 2x2 integer-UV grid with a central singular vertex.
        CaseInput in;
        makeGrid(2, 2, in);
        in.singular = { 4 };
        in.computeUvs = true;
        in.origMode = 1;
        finishOrig(in);
        dumpCase(index++, in);
    }
    {
        // Near-integer UVs: isoline ratios hug 0 and 1 (FMA-sensitive).
        CaseInput in;
        in.vertices = { Vector3(0, 0, 0), Vector3(2, 0, 0), Vector3(0, 2, 0) };
        in.triangles = { { 0, 1, 2 } };
        in.uvs = { { Vector2(1.0 - 1e-13, 0.0), Vector2(1.0 + 1e-13, 0.0),
            Vector2(0.0, 2.0) } };
        dumpCase(index++, in);
    }
    {
        // Positions exactly on the PositionKey 1e-5 lattice.
        CaseInput in;
        in.vertices = { Vector3(0.00001, 0.00002, 0.0), Vector3(0.00003, 0.00001, 0.0),
            Vector3(0.00002, 0.00004, 0.0), Vector3(2, 0, 0), Vector3(0, 2, 0) };
        in.triangles = { { 0, 1, 2 }, { 0, 3, 4 } };
        in.uvs = { { Vector2(0, 0), Vector2(2, 0), Vector2(0, 2) },
            { Vector2(0, 0), Vector2(2, 0), Vector2(0, 2) } };
        dumpCase(index++, in);
    }
    {
        // Positions straddling lattice cells by 1e-12.
        CaseInput in;
        in.vertices = { Vector3(1e-5 + 1e-12, 0, 0), Vector3(1e-5 - 1e-12, 1, 0),
            Vector3(0, 1e-5 + 1e-12, 0) };
        in.triangles = { { 0, 1, 2 } };
        in.uvs = { { Vector2(0, 0), Vector2(2, 0), Vector2(0, 2) } };
        dumpCase(index++, in);
    }
    {
        // All three corners identical (point triangle).
        CaseInput in;
        in.vertices = { Vector3(1, 1, 1) };
        in.triangles = { { 0, 0, 0 } };
        in.uvs = { { Vector2(0, 0), Vector2(1, 0), Vector2(0, 1) } };
        dumpCase(index++, in);
    }
    {
        // UV span zero in v: all isolines run parallel in u.
        CaseInput in;
        makeGrid(2, 2, in);
        for (auto& row : in.uvs) {
            for (auto& uv : row)
                uv = Vector2(uv.x(), 0.5);
        }
        dumpCase(index++, in);
    }
    {
        // 3x3 grid, one singular vertex, perturbed original UVs.
        CaseInput in;
        makeGrid(3, 3, in);
        in.singular = { 5 };
        in.computeUvs = true;
        in.origMode = 2;
        finishOrig(in);
        dumpCase(index++, in);
    }
    {
        // Thin sliver triangle (area ~1e-12).
        CaseInput in;
        in.vertices = { Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0.5, 1e-12, 0) };
        in.triangles = { { 0, 1, 2 } };
        in.uvs = { { Vector2(0, 0), Vector2(3, 0), Vector2(0, 3) } };
        in.computeUvs = true;
        dumpCase(index++, in);
    }
    {
        // 2x2 grid, identical originals, vertex UVs on.
        CaseInput in;
        makeGrid(2, 2, in);
        in.computeUvs = true;
        in.origMode = 1;
        finishOrig(in);
        dumpCase(index++, in);
    }
    {
        // Dense isolines: 2x2 grid with UVs spanning 3 integers per axis.
        CaseInput in;
        makeGrid(2, 2, in);
        for (auto& row : in.uvs) {
            for (auto& uv : row)
                uv = Vector2(uv.x() * 1.5, uv.y() * 1.5);
        }
        in.computeUvs = true;
        dumpCase(index++, in);
    }
    {
        // Negative-quadrant UVs (truncation-toward-zero branch).
        CaseInput in;
        in.vertices = { Vector3(0, 0, 0), Vector3(2, 0, 0), Vector3(0, 2, 0) };
        in.triangles = { { 0, 1, 2 } };
        in.uvs = { { Vector2(-2, -1), Vector2(1, -1), Vector2(-2, 2) } };
        dumpCase(index++, in);
    }
    {
        // Integer-UV 4x4 grid, singulars on every boundary vertex:
        // crossPoints coincide with input verts, boundary stubs starve.
        CaseInput in;
        makeGrid(4, 4, in);
        for (size_t y = 0; y <= 4; ++y) {
            for (size_t x = 0; x <= 4; ++x) {
                if (0 == x || 4 == x || 0 == y || 4 == y)
                    in.singular.push_back(y * 5 + x);
            }
        }
        in.computeUvs = true;
        dumpCase(index++, in);
    }
    {
        // Integer-UV 5x5 grid, every vertex singular (walk search space).
        CaseInput in;
        makeGrid(5, 5, in);
        for (size_t i = 0; i < in.vertices.size(); ++i)
            in.singular.push_back(i);
        dumpCase(index++, in);
    }
    {
        // Fixed hole grid with rim singulars (dangling isoline ends).
        CaseInput in;
        makeGrid(6, 6, in);
        {
            std::vector<std::vector<size_t>> kept;
            std::vector<std::vector<Vector2>> keptUv;
            for (size_t c = 0; c < in.triangles.size(); ++c) {
                size_t cell = c / 2;
                size_t x = cell % 6;
                size_t y = cell / 6;
                if (x >= 2 && x < 4 && y >= 2 && y < 4)
                    continue;
                kept.push_back(in.triangles[c]);
                keptUv.push_back(in.uvs[c]);
            }
            in.triangles = std::move(kept);
            in.uvs = std::move(keptUv);
        }
        auto id = [](size_t x, size_t y) { return y * 7 + x; };
        in.singular = { id(2, 2), id(3, 2), id(4, 2), id(4, 3), id(4, 4),
            id(3, 4), id(2, 4), id(2, 3) };
        in.computeUvs = true;
        dumpCase(index++, in);
    }
    {
        // Fixed pole fan, ring 5 (pentagon pole).
        CaseInput in;
        in.vertices.emplace_back(0, 0, 0.2);
        for (size_t i = 0; i < 5; ++i) {
            double a = 2.0 * M_PI * (double)i / 5.0;
            in.vertices.emplace_back(2.0 * std::cos(a), 2.0 * std::sin(a), 0);
        }
        for (size_t i = 0; i < 5; ++i)
            in.triangles.push_back({ 0, 1 + i, 1 + (i + 1) % 5 });
        in.uvs.resize(5);
        for (size_t i = 0; i < 5; ++i) {
            in.uvs[i].push_back(Vector2((double)i + 0.5, 0.0));
            in.uvs[i].push_back(Vector2((double)i, 2.0));
            in.uvs[i].push_back(Vector2((double)(i + 1 > 4 ? 0 : i + 1), 2.0));
        }
        in.singular = { 0 };
        dumpCase(index++, in);
    }
    {
        // Fixed pole fan, ring 6 (hexagon pole).
        CaseInput in;
        in.vertices.emplace_back(0, 0, 0.2);
        for (size_t i = 0; i < 6; ++i) {
            double a = 2.0 * M_PI * (double)i / 6.0;
            in.vertices.emplace_back(2.0 * std::cos(a), 2.0 * std::sin(a), 0);
        }
        for (size_t i = 0; i < 6; ++i)
            in.triangles.push_back({ 0, 1 + i, 1 + (i + 1) % 6 });
        in.uvs.resize(6);
        for (size_t i = 0; i < 6; ++i) {
            in.uvs[i].push_back(Vector2((double)i + 0.5, 0.0));
            in.uvs[i].push_back(Vector2((double)i, 2.0));
            in.uvs[i].push_back(Vector2((double)((i + 1) % 6), 2.0));
        }
        dumpCase(index++, in);
    }
    {
        // Lifted from a seeded run (seed-10 case 112 (seven-split)): fires the pass named below.
        CaseInput in;
        in.vertices = {
            Vector3(-0.016960094381603421, -0.084339353036591375, 0.082451611261234528),
            Vector3(1.0280614216042634, -0.046632379454326772, 0.067587474905999742),
            Vector3(2.1039604222292478, -0.083215970588641899, -0.041217824808293037),
            Vector3(3.1270213914980478, -0.08839429452869868, 0.063486166587531687),
            Vector3(4.1017774388815935, -0.049997552920274482, 0.020182827927273619),
            Vector3(4.846279871586427, 0.13673223396235498, -0.048422325061824069),
            Vector3(5.9587795029840604, 0.059149064519037522, 0.039184412035181929),
            Vector3(0.036164109681634707, 1.1404026909559137, -0.038155556274659969),
            Vector3(0.84899590893385402, 0.97803456707304948, -0.05135722976149322),
            Vector3(2.1033658077181401, 1.1082096248798783, -0.095786439182759253),
            Vector3(2.8045728867250257, 0.80892842096780304, 0.078168611994307718),
            Vector3(4.064712743382092, 1.0460618687364112, -0.037480337332341018),
            Vector3(4.8261329355031393, 1.1783654294384822, -0.083125322382113287),
            Vector3(5.934245318305738, 0.84595943275834151, -0.069058527337027803),
            Vector3(0.17065096379894312, 2.0785111771584672, 0.078280575289518692),
            Vector3(0.83785082986487758, 2.0777201660800064, -0.031550028595372551),
            Vector3(2.1667883281086913, 2.1248348991952568, 0.062930669242328582),
            Vector3(2.930979080191022, 2.0362909448784414, -0.06197700632703125),
            Vector3(3.8980448825284282, 2.1304335484425825, 0.058669696076414861),
            Vector3(5.0833435380594878, 2.0991367306156494, 0.010945474624983699),
            Vector3(5.8015102950066444, 1.9139939235780372, 0.017025609354705252),
            Vector3(0.0039430727277476318, 2.9792141560703329, -0.064722469900455853),
            Vector3(1.1715914631397244, 2.8935675181751304, -0.078015999739208197),
            Vector3(2.1648266670414951, 2.9085480006918689, -0.0006264494050876924),
            Vector3(3.0827782573812059, 2.8292141756157312, -0.014904940829976222),
            Vector3(3.976656841224032, 3.1391033179749304, -0.0072048302810688199),
            Vector3(5.154094454109714, 3.0467301456250304, -0.046765529717508386),
            Vector3(6.1683465956001351, 3.1615340082812571, 0.079468126860692559),
            Vector3(0.08581433421557004, 4.165988561688506, 0.01965243331801967),
            Vector3(0.80019322380889035, 4.1753564733024611, 0.019349233264009592),
            Vector3(1.9097849415564949, 3.9844282588414957, -0.08017093410909143),
            Vector3(3.0751024631222288, 3.8468751446233282, 0.074332345504430061),
            Vector3(4.1783297667402506, 4.1115628912315056, -0.059859754355300249),
            Vector3(5.1430962170538299, 4.1701339775019726, -0.0043586414452260192),
            Vector3(6.0370830865026379, 4.1610162260600685, -0.013810253118141614),
            Vector3(0.17306101558447062, 5.1323790631556125, -0.061452405751445088),
            Vector3(0.95826094689411045, 5.1948552644956099, 0.040521536731660479),
            Vector3(1.9070871529220987, 4.9903574571580638, -0.052016977008993243),
            Vector3(3.1427256561934929, 4.9383331550484213, -0.049763414861513103),
            Vector3(4.0984033406920606, 5.1683297134912021, 0.092723722365408115),
            Vector3(4.985604198448498, 4.977974968076964, 0.036011414006939239),
            Vector3(5.9841729111655271, 4.8228061749237288, 0.065197354030964147),
            Vector3(-0.062356496825014943, 6.0281531725545001, -0.022409994934412272),
            Vector3(0.9294661532611852, 6.0480398126766346, 0.0098349079423102337),
            Vector3(1.8979416818272696, 6.0394249479270066, 0.075787280408516858),
            Vector3(2.8578887318649775, 6.175913828342769, 0.053968411336593095),
            Vector3(4.1559705560960181, 5.905790754472414, 0.046091967332520037),
            Vector3(4.9858978853673133, 6.1343012137361761, -0.062466108171416651),
            Vector3(5.9454966094282931, 6.1386810072287386, -0.093338681685988606),
        };
        in.triangles = {
            { 0, 1, 8 },
            { 0, 8, 7 },
            { 1, 2, 9 },
            { 1, 9, 8 },
            { 2, 3, 10 },
            { 2, 10, 9 },
            { 3, 4, 11 },
            { 3, 11, 10 },
            { 4, 5, 12 },
            { 4, 12, 11 },
            { 5, 6, 13 },
            { 5, 13, 12 },
            { 7, 8, 15 },
            { 7, 15, 14 },
            { 8, 9, 16 },
            { 8, 16, 15 },
            { 9, 10, 17 },
            { 9, 17, 16 },
            { 10, 11, 18 },
            { 10, 18, 17 },
            { 11, 12, 19 },
            { 11, 19, 18 },
            { 12, 13, 20 },
            { 12, 20, 19 },
            { 14, 15, 22 },
            { 14, 22, 21 },
            { 15, 16, 23 },
            { 15, 23, 22 },
            { 18, 19, 26 },
            { 18, 26, 25 },
            { 19, 20, 27 },
            { 19, 27, 26 },
            { 21, 22, 29 },
            { 21, 29, 28 },
            { 22, 23, 30 },
            { 22, 30, 29 },
            { 25, 26, 33 },
            { 25, 33, 32 },
            { 26, 27, 34 },
            { 26, 34, 33 },
            { 28, 29, 36 },
            { 28, 36, 35 },
            { 29, 30, 37 },
            { 29, 37, 36 },
            { 30, 31, 38 },
            { 30, 38, 37 },
            { 31, 32, 39 },
            { 31, 39, 38 },
            { 32, 33, 40 },
            { 32, 40, 39 },
            { 33, 34, 41 },
            { 33, 41, 40 },
            { 35, 36, 43 },
            { 35, 43, 42 },
            { 36, 37, 44 },
            { 36, 44, 43 },
            { 37, 38, 45 },
            { 37, 45, 44 },
            { 38, 39, 46 },
            { 38, 46, 45 },
            { 39, 40, 47 },
            { 39, 47, 46 },
            { 40, 41, 48 },
            { 40, 48, 47 },
        };
        in.uvs = {
            { Vector2(1e-13, -0.1595976954359119), Vector2(1.0190415849005634, -0.017955060539181014), Vector2(0.87556955037793094, 1.0319897706548649) },
            { Vector2(1e-13, -0.1595976954359119), Vector2(0.87556955037793094, 1.0319897706548649), Vector2(0.0077792872561632601, 1.0871771161685697) },
            { Vector2(1.0190415849005634, -0.017955060539181014), Vector2(2.107243038018058, -0.14807276831997629), Vector2(2.029780759724872, 1.1797679548496154) },
            { Vector2(1.0190415849005634, -0.017955060539181014), Vector2(2.029780759724872, 1.1797679548496154), Vector2(0.87556955037793094, 1.0319897706548649) },
            { Vector2(2.107243038018058, -0.14807276831997629), Vector2(3.0746740108817883, -0.050701081644243126), Vector2(2.7266139930284399, 0.88891593713809325) },
            { Vector2(2.107243038018058, -0.14807276831997629), Vector2(2.7266139930284399, 0.88891593713809325), Vector2(2.029780759724872, 1.1797679548496154) },
            { Vector2(3.0746740108817883, -0.050701081644243126), Vector2(4.146550474310521, -6.5289936922452441e-06), Vector2(4.0072619826577922, 1.1252434365172084) },
            { Vector2(3.0746740108817883, -0.050701081644243126), Vector2(4.0072619826577922, 1.1252434365172084), Vector2(2.7266139930284399, 0.88891593713809325) },
            { Vector2(4.146550474310521, -6.5289936922452441e-06), Vector2(4.8647801390277046, 0.13542350534551012), Vector2(4.8466548023844558, 1.1032461637019346) },
            { Vector2(4.146550474310521, -6.5289936922452441e-06), Vector2(4.8466548023844558, 1.1032461637019346), Vector2(4.0072619826577922, 1.1252434365172084) },
            { Vector2(4.8647801390277046, 0.13542350534551012), Vector2(5.9807699887463253, 0.0087962016049723882), Vector2(5.9317739764501853, 0.74599696900903867) },
            { Vector2(4.8647801390277046, 0.13542350534551012), Vector2(5.9317739764501853, 0.74599696900903867), Vector2(4.8466548023844558, 1.1032461637019346) },
            { Vector2(0.0077792872561632601, 1.0871771161685697), Vector2(0.87556955037793094, 1.0319897706548649), Vector2(0.85294695769379836, 2.0177316199749087) },
            { Vector2(0.0077792872561632601, 1.0871771161685697), Vector2(0.85294695769379836, 2.0177316199749087), Vector2(0.08798724629605946, 2.0877214040637742) },
            { Vector2(0.87556955037793094, 1.0319897706548649), Vector2(2.029780759724872, 1.1797679548496154), Vector2(2.2586018624542574, 2.2186831061723065) },
            { Vector2(0.87556955037793094, 1.0319897706548649), Vector2(2.2586018624542574, 2.2186831061723065), Vector2(0.85294695769379836, 2.0177316199749087) },
            { Vector2(2.029780759724872, 1.1797679548496154), Vector2(2.7266139930284399, 0.88891593713809325), Vector2(2.8823566356220747, 2.0292702265903091) },
            { Vector2(2.029780759724872, 1.1797679548496154), Vector2(2.8823566356220747, 2.0292702265903091), Vector2(2.2586018624542574, 2.2186831061723065) },
            { Vector2(2.7266139930284399, 0.88891593713809325), Vector2(4.0072619826577922, 1.1252434365172084), Vector2(3.8948944725869126, 2.1534177408799007) },
            { Vector2(2.7266139930284399, 0.88891593713809325), Vector2(3.8948944725869126, 2.1534177408799007), Vector2(2.8823566356220747, 2.0292702265903091) },
            { Vector2(4.0072619826577922, 1.1252434365172084), Vector2(4.8466548023844558, 1.1032461637019346), Vector2(4.9883569458610673, 2.193900372392346) },
            { Vector2(4.0072619826577922, 1.1252434365172084), Vector2(4.9883569458610673, 2.193900372392346), Vector2(3.8948944725869126, 2.1534177408799007) },
            { Vector2(4.8466548023844558, 1.1032461637019346), Vector2(5.9317739764501853, 0.74599696900903867), Vector2(5.8384700162640355, 1.9278256453808165) },
            { Vector2(4.8466548023844558, 1.1032461637019346), Vector2(5.8384700162640355, 1.9278256453808165), Vector2(4.9883569458610673, 2.193900372392346) },
            { Vector2(0.08798724629605946, 2.0877214040637742), Vector2(0.85294695769379836, 2.0177316199749087), Vector2(1.0862196354297247, 2.8061295197199492) },
            { Vector2(0.08798724629605946, 2.0877214040637742), Vector2(1.0862196354297247, 2.8061295197199492), Vector2(-0.094307917608920128, 3.0513771986844644) },
            { Vector2(0.85294695769379836, 2.0177316199749087), Vector2(2.2586018624542574, 2.2186831061723065), Vector2(2.1057437802792269, 2.8987920457946181) },
            { Vector2(0.85294695769379836, 2.0177316199749087), Vector2(2.1057437802792269, 2.8987920457946181), Vector2(1.0862196354297247, 2.8061295197199492) },
            { Vector2(3.8948944725869126, 2.1534177408799007), Vector2(4.9883569458610673, 2.193900372392346), Vector2(5.1748632938210877, 2.9790752490897847) },
            { Vector2(3.8948944725869126, 2.1534177408799007), Vector2(5.1748632938210877, 2.9790752490897847), Vector2(3.9569413204160147, 3.0722482802044149) },
            { Vector2(4.9883569458610673, 2.193900372392346), Vector2(5.8384700162640355, 1.9278256453808165), Vector2(6.2642115239424072, 3.2059599266950078) },
            { Vector2(4.9883569458610673, 2.193900372392346), Vector2(6.2642115239424072, 3.2059599266950078), Vector2(5.1748632938210877, 2.9790752490897847) },
            { Vector2(-0.094307917608920128, 3.0513771986844644), Vector2(1.0862196354297247, 2.8061295197199492), Vector2(0.73890027237726374, 4.2521146078134828) },
            { Vector2(-0.094307917608920128, 3.0513771986844644), Vector2(0.73890027237726374, 4.2521146078134828), Vector2(6.9052789408038939e-05, 4.2086194562619337) },
            { Vector2(1.0862196354297247, 2.8061295197199492), Vector2(2.1057437802792269, 2.8987920457946181), Vector2(1.9999999999999001, 3.9505641991024323) },
            { Vector2(1.0862196354297247, 2.8061295197199492), Vector2(1.9999999999999001, 3.9505641991024323), Vector2(0.73890027237726374, 4.2521146078134828) },
            { Vector2(3.9569413204160147, 3.0722482802044149), Vector2(5.1748632938210877, 2.9790752490897847), Vector2(5.1161764382928849, 4.2022047927951807) },
            { Vector2(3.9569413204160147, 3.0722482802044149), Vector2(5.1161764382928849, 4.2022047927951807), Vector2(4.2219029003123012, 4.1795540696456346) },
            { Vector2(5.1748632938210877, 2.9790752490897847), Vector2(6.2642115239424072, 3.2059599266950078), Vector2(6.1014368679066573, 4.1236348604578152) },
            { Vector2(5.1748632938210877, 2.9790752490897847), Vector2(6.1014368679066573, 4.1236348604578152), Vector2(5.1161764382928849, 4.2022047927951807) },
            { Vector2(6.9052789408038939e-05, 4.2086194562619337), Vector2(0.73890027237726374, 4.2521146078134828), Vector2(0.96696046795220358, 5.2571962539435217) },
            { Vector2(6.9052789408038939e-05, 4.2086194562619337), Vector2(0.96696046795220358, 5.2571962539435217), Vector2(0.11566175034306962, 5.1279741708433066) },
            { Vector2(0.73890027237726374, 4.2521146078134828), Vector2(1.9999999999999001, 3.9505641991024323), Vector2(1.888706864251285, 5.0318504084795075) },
            { Vector2(0.73890027237726374, 4.2521146078134828), Vector2(1.888706864251285, 5.0318504084795075), Vector2(0.96696046795220358, 5.2571962539435217) },
            { Vector2(1.9999999999999001, 3.9505641991024323), Vector2(3.1715178121479526, 3.8278296592946783), Vector2(3.0726433984595798, 4.8586285986181617) },
            { Vector2(1.9999999999999001, 3.9505641991024323), Vector2(3.0726433984595798, 4.8586285986181617), Vector2(1.888706864251285, 5.0318504084795075) },
            { Vector2(3.1715178121479526, 3.8278296592946783), Vector2(4.2219029003123012, 4.1795540696456346), Vector2(4.0723777633973208, 5.2244780024566833) },
            { Vector2(3.1715178121479526, 3.8278296592946783), Vector2(4.0723777633973208, 5.2244780024566833), Vector2(3.0726433984595798, 4.8586285986181617) },
            { Vector2(4.2219029003123012, 4.1795540696456346), Vector2(5.1161764382928849, 4.2022047927951807), Vector2(4.9576877468021445, 5.0341520587292896) },
            { Vector2(4.2219029003123012, 4.1795540696456346), Vector2(4.9576877468021445, 5.0341520587292896), Vector2(4.0723777633973208, 5.2244780024566833) },
            { Vector2(5.1161764382928849, 4.2022047927951807), Vector2(6.1014368679066573, 4.1236348604578152), Vector2(5.9644900564890238, 4.8774681095710575) },
            { Vector2(5.1161764382928849, 4.2022047927951807), Vector2(5.9644900564890238, 4.8774681095710575), Vector2(4.9576877468021445, 5.0341520587292896) },
            { Vector2(0.11566175034306962, 5.1279741708433066), Vector2(0.96696046795220358, 5.2571962539435217), Vector2(0.96051498671370161, 6.104219894882327) },
            { Vector2(0.11566175034306962, 5.1279741708433066), Vector2(0.96051498671370161, 6.104219894882327), Vector2(-0.061944063649158436, 6.0765967418979114) },
            { Vector2(0.96696046795220358, 5.2571962539435217), Vector2(1.888706864251285, 5.0318504084795075), Vector2(1.8952170988696189, 6.0726643819705233) },
            { Vector2(0.96696046795220358, 5.2571962539435217), Vector2(1.8952170988696189, 6.0726643819705233), Vector2(0.96051498671370161, 6.104219894882327) },
            { Vector2(1.888706864251285, 5.0318504084795075), Vector2(3.0726433984595798, 4.8586285986181617), Vector2(2.7815814249931701, 6.1270706415802341) },
            { Vector2(1.888706864251285, 5.0318504084795075), Vector2(2.7815814249931701, 6.1270706415802341), Vector2(1.8952170988696189, 6.0726643819705233) },
            { Vector2(3.0726433984595798, 4.8586285986181617), Vector2(4.0723777633973208, 5.2244780024566833), Vector2(4.1451171308115828, 5.917713497626317) },
            { Vector2(3.0726433984595798, 4.8586285986181617), Vector2(4.1451171308115828, 5.917713497626317), Vector2(2.7815814249931701, 6.1270706415802341) },
            { Vector2(4.0723777633973208, 5.2244780024566833), Vector2(4.9576877468021445, 5.0341520587292896), Vector2(4.9363961902170237, 6.094935370857228) },
            { Vector2(4.0723777633973208, 5.2244780024566833), Vector2(4.9363961902170237, 6.094935370857228), Vector2(4.1451171308115828, 5.917713497626317) },
            { Vector2(4.9576877468021445, 5.0341520587292896), Vector2(5.9644900564890238, 4.8774681095710575), Vector2(6.0108936468219625, 6.070186973518755) },
            { Vector2(4.9576877468021445, 5.0341520587292896), Vector2(6.0108936468219625, 6.070186973518755), Vector2(4.9363961902170237, 6.094935370857228) },
        };
        in.singular = { 16, 23, 11, 17, 7, 40 };
        in.computeUvs = false;
        in.origMode = 2;
        in.origUvs = {
            { Vector2(1e-13, -0.1595976954359119), Vector2(1.0190415849005634, -0.017955060539181014), Vector2(0.87556955037793094, 1.0319897706548649) },
            { Vector2(1e-13, -0.1595976954359119), Vector2(0.87556955037793094, 1.0319897706548649), Vector2(0.0077792872561632601, 1.0871771161685697) },
            { Vector2(1.0190415849005634, -0.017955060539181014), Vector2(2.107243038018058, -0.14807276831997629), Vector2(2.029780759724872, 1.1797679548496154) },
            { Vector2(1.0190415849005634, -0.017955060539181014), Vector2(2.029780759724872, 1.1797679548496154), Vector2(0.87556955037793094, 1.0319897706548649) },
            { Vector2(2.107243038018058, -0.14807276831997629), Vector2(3.0746740108817883, -0.050701081644243126), Vector2(2.7266139930284399, 0.88891593713809325) },
            { Vector2(2.107243038018058, -0.14807276831997629), Vector2(2.7266139930284399, 0.88891593713809325), Vector2(2.029780759724872, 1.1797679548496154) },
            { Vector2(3.0746740108817883, -0.050701081644243126), Vector2(4.146550474310521, -6.5289936922452441e-06), Vector2(4.0072619826577922, 1.1252434365172084) },
            { Vector2(3.0746740108817883, -0.050701081644243126), Vector2(4.0072619826577922, 1.1252434365172084), Vector2(2.7266139930284399, 0.88891593713809325) },
            { Vector2(4.146550474310521, -6.5289936922452441e-06), Vector2(4.8647801390277046, 0.13542350534551012), Vector2(4.8466548023844558, 1.1032461637019346) },
            { Vector2(4.146550474310521, -6.5289936922452441e-06), Vector2(4.8466548023844558, 1.1032461637019346), Vector2(4.0072619826577922, 1.1252434365172084) },
            { Vector2(4.8647801390277046, 0.13542350534551012), Vector2(5.9807699887463253, 0.0087962016049723882), Vector2(5.9317739764501853, 0.74599696900903867) },
            { Vector2(4.8647801390277046, 0.13542350534551012), Vector2(5.9317739764501853, 0.74599696900903867), Vector2(4.8466548023844558, 1.1032461637019346) },
            { Vector2(0.0077792872561632601, 1.0871771161685697), Vector2(0.87556955037793094, 1.0319897706548649), Vector2(0.85294695769379836, 2.0177316199749087) },
            { Vector2(0.0077792872561632601, 1.0871771161685697), Vector2(0.85294695769379836, 2.0177316199749087), Vector2(0.08798724629605946, 2.0877214040637742) },
            { Vector2(0.87556955037793094, 1.0319897706548649), Vector2(2.029780759724872, 1.1797679548496154), Vector2(2.2586018624542574, 2.2186831061723065) },
            { Vector2(0.87556955037793094, 1.0319897706548649), Vector2(2.2586018624542574, 2.2186831061723065), Vector2(0.85294695769379836, 2.0177316199749087) },
            { Vector2(2.029780759724872, 1.1797679548496154), Vector2(2.7266139930284399, 0.88891593713809325), Vector2(2.8823566356220747, 2.0292702265903091) },
            { Vector2(2.029780759724872, 1.1797679548496154), Vector2(2.8823566356220747, 2.0292702265903091), Vector2(2.2586018624542574, 2.2186831061723065) },
            { Vector2(2.7266139930284399, 0.88891593713809325), Vector2(4.0072619826577922, 1.1252434365172084), Vector2(3.8948944725869126, 2.1534177408799007) },
            { Vector2(2.7266139930284399, 0.88891593713809325), Vector2(3.8948944725869126, 2.1534177408799007), Vector2(2.8823566356220747, 2.0292702265903091) },
            { Vector2(4.0072619826577922, 1.1252434365172084), Vector2(4.8466548023844558, 1.1032461637019346), Vector2(4.9883569458610673, 2.193900372392346) },
            { Vector2(4.0072619826577922, 1.1252434365172084), Vector2(4.9883569458610673, 2.193900372392346), Vector2(3.8948944725869126, 2.1534177408799007) },
            { Vector2(4.8466548023844558, 1.1032461637019346), Vector2(5.9317739764501853, 0.74599696900903867), Vector2(5.8384700162640355, 1.9278256453808165) },
            { Vector2(4.8466548023844558, 1.1032461637019346), Vector2(5.8384700162640355, 1.9278256453808165), Vector2(4.9883569458610673, 2.193900372392346) },
            { Vector2(0.08798724629605946, 2.0877214040637742), Vector2(0.85294695769379836, 2.0177316199749087), Vector2(1.0862196354297247, 2.8061295197199492) },
            { Vector2(0.08798724629605946, 2.0877214040637742), Vector2(1.0862196354297247, 2.8061295197199492), Vector2(-0.094307917608920128, 3.0513771986844644) },
            { Vector2(0.85294695769379836, 2.0177316199749087), Vector2(2.2586018624542574, 2.2186831061723065), Vector2(2.1057437802792269, 2.8987920457946181) },
            { Vector2(0.85294695769379836, 2.0177316199749087), Vector2(2.1057437802792269, 2.8987920457946181), Vector2(1.0862196354297247, 2.8061295197199492) },
            { Vector2(3.8948944725869126, 2.1534177408799007), Vector2(5.2383569458610673, 2.068900372392346), Vector2(5.1748632938210877, 2.9790752490897847) },
            { Vector2(3.8948944725869126, 2.1534177408799007), Vector2(5.1748632938210877, 2.9790752490897847), Vector2(3.9569413204160147, 3.0722482802044149) },
            { Vector2(4.9883569458610673, 2.193900372392346), Vector2(5.8384700162640355, 1.9278256453808165), Vector2(6.2642115239424072, 3.2059599266950078) },
            { Vector2(4.9883569458610673, 2.193900372392346), Vector2(6.2642115239424072, 3.2059599266950078), Vector2(5.1748632938210877, 2.9790752490897847) },
            { Vector2(-0.094307917608920128, 3.0513771986844644), Vector2(1.0862196354297247, 2.8061295197199492), Vector2(0.73890027237726374, 4.2521146078134828) },
            { Vector2(-0.094307917608920128, 3.0513771986844644), Vector2(0.73890027237726374, 4.2521146078134828), Vector2(6.9052789408038939e-05, 4.2086194562619337) },
            { Vector2(1.0862196354297247, 2.8061295197199492), Vector2(2.1057437802792269, 2.8987920457946181), Vector2(1.9999999999999001, 3.9505641991024323) },
            { Vector2(1.0862196354297247, 2.8061295197199492), Vector2(1.9999999999999001, 3.9505641991024323), Vector2(0.73890027237726374, 4.2521146078134828) },
            { Vector2(3.9569413204160147, 3.0722482802044149), Vector2(5.1748632938210877, 2.9790752490897847), Vector2(5.1161764382928849, 4.2022047927951807) },
            { Vector2(3.9569413204160147, 3.0722482802044149), Vector2(5.1161764382928849, 4.2022047927951807), Vector2(4.2219029003123012, 4.1795540696456346) },
            { Vector2(5.1748632938210877, 2.9790752490897847), Vector2(6.2642115239424072, 3.2059599266950078), Vector2(6.1014368679066573, 4.1236348604578152) },
            { Vector2(5.1748632938210877, 2.9790752490897847), Vector2(6.1014368679066573, 4.1236348604578152), Vector2(5.1161764382928849, 4.2022047927951807) },
            { Vector2(6.9052789408038939e-05, 4.2086194562619337), Vector2(0.73890027237726374, 4.2521146078134828), Vector2(0.96696046795220358, 5.2571962539435217) },
            { Vector2(6.9052789408038939e-05, 4.2086194562619337), Vector2(0.96696046795220358, 5.2571962539435217), Vector2(0.11566175034306962, 5.1279741708433066) },
            { Vector2(0.73890027237726374, 4.2521146078134828), Vector2(1.9999999999999001, 3.9505641991024323), Vector2(1.888706864251285, 5.0318504084795075) },
            { Vector2(0.73890027237726374, 4.2521146078134828), Vector2(1.888706864251285, 5.0318504084795075), Vector2(0.96696046795220358, 5.2571962539435217) },
            { Vector2(1.9999999999999001, 3.9505641991024323), Vector2(3.1715178121479526, 3.8278296592946783), Vector2(3.0726433984595798, 4.8586285986181617) },
            { Vector2(1.9999999999999001, 3.9505641991024323), Vector2(3.0726433984595798, 4.8586285986181617), Vector2(1.888706864251285, 5.0318504084795075) },
            { Vector2(3.1715178121479526, 3.8278296592946783), Vector2(4.2219029003123012, 4.1795540696456346), Vector2(4.0723777633973208, 5.2244780024566833) },
            { Vector2(3.1715178121479526, 3.8278296592946783), Vector2(4.0723777633973208, 5.2244780024566833), Vector2(3.0726433984595798, 4.8586285986181617) },
            { Vector2(4.2219029003123012, 4.1795540696456346), Vector2(5.1161764382928849, 4.2022047927951807), Vector2(4.9576877468021445, 5.0341520587292896) },
            { Vector2(4.2219029003123012, 4.1795540696456346), Vector2(4.9576877468021445, 5.0341520587292896), Vector2(4.0723777633973208, 5.2244780024566833) },
            { Vector2(5.1161764382928849, 4.2022047927951807), Vector2(6.1014368679066573, 4.1236348604578152), Vector2(5.9644900564890238, 4.8774681095710575) },
            { Vector2(5.1161764382928849, 4.2022047927951807), Vector2(5.9644900564890238, 4.8774681095710575), Vector2(4.9576877468021445, 5.0341520587292896) },
            { Vector2(0.11566175034306962, 5.1279741708433066), Vector2(0.96696046795220358, 5.2571962539435217), Vector2(0.96051498671370161, 6.104219894882327) },
            { Vector2(0.11566175034306962, 5.1279741708433066), Vector2(0.96051498671370161, 6.104219894882327), Vector2(-0.061944063649158436, 6.0765967418979114) },
            { Vector2(0.96696046795220358, 5.2571962539435217), Vector2(1.888706864251285, 5.0318504084795075), Vector2(1.8952170988696189, 6.0726643819705233) },
            { Vector2(0.96696046795220358, 5.2571962539435217), Vector2(1.8952170988696189, 6.0726643819705233), Vector2(0.96051498671370161, 6.104219894882327) },
            { Vector2(1.888706864251285, 5.0318504084795075), Vector2(3.0726433984595798, 4.8586285986181617), Vector2(2.7815814249931701, 6.1270706415802341) },
            { Vector2(1.888706864251285, 5.0318504084795075), Vector2(2.7815814249931701, 6.1270706415802341), Vector2(1.8952170988696189, 6.0726643819705233) },
            { Vector2(3.0726433984595798, 4.8586285986181617), Vector2(4.0723777633973208, 5.2244780024566833), Vector2(4.1451171308115828, 5.917713497626317) },
            { Vector2(3.0726433984595798, 4.8586285986181617), Vector2(4.1451171308115828, 5.917713497626317), Vector2(2.7815814249931701, 6.1270706415802341) },
            { Vector2(4.0723777633973208, 5.2244780024566833), Vector2(4.9576877468021445, 5.0341520587292896), Vector2(4.9363961902170237, 6.094935370857228) },
            { Vector2(4.0723777633973208, 5.2244780024566833), Vector2(4.9363961902170237, 6.094935370857228), Vector2(4.1451171308115828, 5.917713497626317) },
            { Vector2(4.9576877468021445, 5.0341520587292896), Vector2(5.9644900564890238, 4.8774681095710575), Vector2(6.0108936468219625, 6.070186973518755) },
            { Vector2(4.9576877468021445, 5.0341520587292896), Vector2(6.0108936468219625, 6.070186973518755), Vector2(4.9363961902170237, 6.094935370857228) },
        };
        dumpCase(index++, in);
    }
    {
        // Lifted from a seeded run (seed-10 case 212 (singular walk)): fires the pass named below.
        CaseInput in;
        in.vertices = {
            Vector3(-0.15972240558436415, -0.19053242082345717, 0.019226517098732243),
            Vector3(0.81315972606491116, -0.19359247696159376, 0.013866593606467359),
            Vector3(1.9014836763806844, 0.038804123967133779, -0.065436099480868398),
            Vector3(2.8325187758444921, -0.0051314365795435361, -0.017392548714240785),
            Vector3(4.194329245514858, -0.037714749698778284, -0.040656060049246982),
            Vector3(4.9733110025216938, 0.18281368180129376, 0.057468690743045218),
            Vector3(6.0889828438847218, 0.19507075054562628, -0.01343505774460001),
            Vector3(0.07181661626068396, 0.86335679793497444, 0.050803822214037946),
            Vector3(0.9459316399166513, 1.0052052433350742, 0.096155001470227291),
            Vector3(2.0298748907517088, 1.0961592359368066, -0.032663047187301911),
            Vector3(3.0598323100600129, 1.1739337303135076, -0.059036596302839528),
            Vector3(3.9697618530097918, 1.0664295057495661, 0.076256994964364958),
            Vector3(5.1974708640713159, 1.0574459407577719, 0.081836950563819774),
            Vector3(5.9505721049324753, 0.93109573147348246, 0.017282604939865397),
            Vector3(-0.040309908837823732, 2.0175519487407048, 0.033207890780671566),
            Vector3(1.004748244700707, 1.9910020667849977, -0.0919188390349506),
            Vector3(2.0596770381304115, 2.1827225090364388, -0.023484010994647076),
            Vector3(3.0991512163094845, 1.9234344269854531, 0.024687713232204603),
            Vector3(4.0913578066511151, 1.8078544939861096, -0.072898570003500821),
            Vector3(5.0923437755653014, 2.0682039138021224, -0.054645605995273488),
            Vector3(6.0687790199207665, 1.8510151091722431, 0.0036303333933196181),
            Vector3(0.09755791097926525, 3.1312865804079104, 0.055687722307237356),
            Vector3(0.9500316510671224, 3.0397619687223183, -0.098912995802349787),
            Vector3(2.1652106680396326, 2.8909513072963042, 0.028551938240272759),
            Vector3(3.0302706878361958, 3.0123022408280891, 0.086624585187657463),
            Vector3(4.0166128010805586, 3.1178974915488618, 0.018537851097760693),
            Vector3(4.8333031184335482, 3.1774041088825986, 0.019898784029882346),
            Vector3(5.8882284925374151, 2.9229684237550742, -0.055487398169009897),
            Vector3(-0.065342892572239783, 4.0635764726231178, 0.090984478239205024),
            Vector3(0.97439643900008066, 4.0220818233907538, 0.031179951324878122),
            Vector3(1.9426137624583015, 3.8043542216119923, -0.077835569389723316),
            Vector3(2.9118701662171445, 4.1086640896649529, -0.035345299812165297),
            Vector3(3.8315794098807956, 4.0632720872809935, 0.037263813523625337),
            Vector3(5.1134010275268569, 4.1940239394600543, -0.035968677281708362),
            Vector3(5.8674139352053203, 3.8637548223366909, -0.042681820551601794),
            Vector3(0.12559165343903694, 4.8902557944384535, 0.067863042957772901),
            Vector3(0.94596114721635272, 5.0068684272971549, -0.071971511333177515),
            Vector3(1.857418328080443, 5.1939793263589165, 0.089934536962145872),
            Vector3(2.9562925905897814, 5.1805857833433544, -0.014292036568242628),
            Vector3(4.0121669502696733, 4.9062328768899768, -0.05901352241260549),
            Vector3(5.1301572253157026, 5.0608575217899983, -0.039087527501194756),
            Vector3(6.1125534450591079, 4.9914900502215174, -0.0741100289268454),
            Vector3(0.1108480885118711, 6.0831180960848386, -0.088262613910965626),
            Vector3(1.064404866690164, 6.0439807331674977, -0.00029112596203269004),
            Vector3(2.0043640483030343, 5.8147694161712309, 0.066997963864924959),
            Vector3(3.1122672219395566, 6.0800891336255285, -0.085149530666962764),
            Vector3(4.0159635939270366, 5.8361174114956196, 0.055373575746858285),
            Vector3(5.1514696014714572, 6.0443145942905696, -0.064302675483895885),
            Vector3(6.0087336486454035, 5.8406618987487215, -0.018724964002343091),
        };
        in.triangles = {
            { 0, 1, 8 },
            { 0, 8, 7 },
            { 1, 2, 9 },
            { 1, 9, 8 },
            { 2, 3, 10 },
            { 2, 10, 9 },
            { 3, 4, 11 },
            { 3, 11, 10 },
            { 4, 5, 12 },
            { 4, 12, 11 },
            { 5, 6, 13 },
            { 5, 13, 12 },
            { 7, 8, 15 },
            { 7, 15, 14 },
            { 8, 9, 16 },
            { 8, 16, 15 },
            { 9, 10, 17 },
            { 9, 17, 16 },
            { 10, 11, 18 },
            { 10, 18, 17 },
            { 11, 12, 19 },
            { 11, 19, 18 },
            { 12, 13, 20 },
            { 12, 20, 19 },
            { 14, 15, 22 },
            { 14, 22, 21 },
            { 15, 16, 23 },
            { 15, 23, 22 },
            { 18, 19, 26 },
            { 18, 26, 25 },
            { 19, 20, 27 },
            { 19, 27, 26 },
            { 21, 22, 29 },
            { 21, 29, 28 },
            { 22, 23, 30 },
            { 22, 30, 29 },
            { 25, 26, 33 },
            { 25, 33, 32 },
            { 26, 27, 34 },
            { 26, 34, 33 },
            { 28, 29, 36 },
            { 28, 36, 35 },
            { 29, 30, 37 },
            { 29, 37, 36 },
            { 30, 31, 38 },
            { 30, 38, 37 },
            { 31, 32, 39 },
            { 31, 39, 38 },
            { 32, 33, 40 },
            { 32, 40, 39 },
            { 33, 34, 41 },
            { 33, 41, 40 },
            { 35, 36, 43 },
            { 35, 43, 42 },
            { 36, 37, 44 },
            { 36, 44, 43 },
            { 37, 38, 45 },
            { 37, 45, 44 },
            { 38, 39, 46 },
            { 38, 46, 45 },
            { 39, 40, 47 },
            { 39, 47, 46 },
            { 40, 41, 48 },
            { 40, 48, 47 },
        };
        in.uvs = {
            { Vector2(-0.27212646677397767, -0.24281116499652783), Vector2(1.2346746651761988, -0.21843628021947764), Vector2(1.496415846742549, 1.4577227884451627) },
            { Vector2(-0.27212646677397767, -0.24281116499652783), Vector2(1.496415846742549, 1.4577227884451627), Vector2(0.17351041052361085, 1.2606456794563035) },
            { Vector2(1.2346746651761988, -0.21843628021947764), Vector2(2.8035520842222486, -0.01398415034145532), Vector2(2.9999999999999001, 1.5447119342086513) },
            { Vector2(1.2346746651761988, -0.21843628021947764), Vector2(2.9999999999999001, 1.5447119342086513), Vector2(1.496415846742549, 1.4577227884451627) },
            { Vector2(2.8035520842222486, -0.01398415034145532), Vector2(4.2749822569043578, 0.015711605018644593), Vector2(4.6648190489538361, 1.8220098326611416) },
            { Vector2(2.8035520842222486, -0.01398415034145532), Vector2(4.6648190489538361, 1.8220098326611416), Vector2(2.9999999999999001, 1.5447119342086513) },
            { Vector2(4.2749822569043578, 0.015711605018644593), Vector2(6.2731443333971955, 0.0031738560509082976), Vector2(5.9844545758557528, 1.5015745421036759) },
            { Vector2(4.2749822569043578, 0.015711605018644593), Vector2(5.9844545758557528, 1.5015745421036759), Vector2(4.6648190489538361, 1.8220098326611416) },
            { Vector2(6.2731443333971955, 0.0031738560509082976), Vector2(7.4953280796675061, 0.28404700220098122), Vector2(7.7837191704620929, 1.5169327395267906) },
            { Vector2(6.2731443333971955, 0.0031738560509082976), Vector2(7.7837191704620929, 1.5169327395267906), Vector2(5.9844545758557528, 1.5015745421036759) },
            { Vector2(7.4953280796675061, 0.28404700220098122), Vector2(9.2274868266983106, 0.20605367103092975), Vector2(8.9807533723408302, 1.3474351736210448) },
            { Vector2(7.4953280796675061, 0.28404700220098122), Vector2(8.9807533723408302, 1.3474351736210448), Vector2(7.7837191704620929, 1.5169327395267906) },
            { Vector2(0.17351041052361085, 1.2606456794563035), Vector2(1.496415846742549, 1.4577227884451627), Vector2(1.575892620019802, 2.9084173421719202) },
            { Vector2(0.17351041052361085, 1.2606456794563035), Vector2(1.575892620019802, 2.9084173421719202), Vector2(-0.051803244725838582, 3.0867968835655848) },
            { Vector2(1.496415846742549, 1.4577227884451627), Vector2(2.9999999999999001, 1.5447119342086513), Vector2(2.9944245892489225, 3.3699156141621573) },
            { Vector2(1.496415846742549, 1.4577227884451627), Vector2(2.9944245892489225, 3.3699156141621573), Vector2(1.575892620019802, 2.9084173421719202) },
            { Vector2(2.9999999999999001, 1.5447119342086513), Vector2(4.6648190489538361, 1.8220098326611416), Vector2(4.6144046069913323, 2.923104011489706) },
            { Vector2(2.9999999999999001, 1.5447119342086513), Vector2(4.6144046069913323, 2.923104011489706), Vector2(2.9944245892489225, 3.3699156141621573) },
            { Vector2(4.6648190489538361, 1.8220098326611416), Vector2(5.9844545758557528, 1.5015745421036759), Vector2(6.2183279590542604, 2.7473698287914523) },
            { Vector2(4.6648190489538361, 1.8220098326611416), Vector2(6.2183279590542604, 2.7473698287914523), Vector2(4.6144046069913323, 2.923104011489706) },
            { Vector2(5.9844545758557528, 1.5015745421036759), Vector2(7.7837191704620929, 1.5169327395267906), Vector2(7.7269446768704411, 3.0524336342769165) },
            { Vector2(5.9844545758557528, 1.5015745421036759), Vector2(7.7269446768704411, 3.0524336342769165), Vector2(6.2183279590542604, 2.7473698287914523) },
            { Vector2(7.7837191704620929, 1.5169327395267906), Vector2(8.9807533723408302, 1.3474351736210448), Vector2(9.135062434695941, 2.7118899643451484) },
            { Vector2(7.7837191704620929, 1.5169327395267906), Vector2(9.135062434695941, 2.7118899643451484), Vector2(7.7269446768704411, 3.0524336342769165) },
            { Vector2(-0.051803244725838582, 3.0867968835655848), Vector2(1.575892620019802, 2.9084173421719202), Vector2(1.4818924718135162, 4.5788208893517233) },
            { Vector2(-0.051803244725838582, 3.0867968835655848), Vector2(1.4818924718135162, 4.5788208893517233), Vector2(0.099503612531513297, 4.7672603587214324) },
            { Vector2(1.575892620019802, 2.9084173421719202), Vector2(2.9944245892489225, 3.3699156141621573), Vector2(3.180984658762636, 4.3994111527915747) },
            { Vector2(1.575892620019802, 2.9084173421719202), Vector2(3.180984658762636, 4.3994111527915747), Vector2(1.4818924718135162, 4.5788208893517233) },
            { Vector2(6.2183279590542604, 2.7473698287914523), Vector2(7.7269446768704411, 3.0524336342769165), Vector2(7.2021032519803416, 4.7593654051260845) },
            { Vector2(6.2183279590542604, 2.7473698287914523), Vector2(7.2021032519803416, 4.7593654051260845), Vector2(6.043962807205868, 4.5966165086010404) },
            { Vector2(7.7269446768704411, 3.0524336342769165), Vector2(9.135062434695941, 2.7118899643451484), Vector2(8.845086730990289, 4.3454699522382008) },
            { Vector2(7.7269446768704411, 3.0524336342769165), Vector2(8.845086730990289, 4.3454699522382008), Vector2(7.2021032519803416, 4.7593654051260845) },
            { Vector2(0.099503612531513297, 4.7672603587214324), Vector2(1.4818924718135162, 4.5788208893517233), Vector2(1.5421349988792332, 6.0603071092296688) },
            { Vector2(0.099503612531513297, 4.7672603587214324), Vector2(1.5421349988792332, 6.0603071092296688), Vector2(-0.072765291016583675, 6.0754724387325378) },
            { Vector2(1.4818924718135162, 4.5788208893517233), Vector2(3.180984658762636, 4.3994111527915747), Vector2(3.0119808828687988, 5.7933439766163506) },
            { Vector2(1.4818924718135162, 4.5788208893517233), Vector2(3.0119808828687988, 5.7933439766163506), Vector2(1.5421349988792332, 6.0603071092296688) },
            { Vector2(6.043962807205868, 4.5966165086010404), Vector2(7.2021032519803416, 4.7593654051260845), Vector2(7.6090640879527127, 6.277966749119483) },
            { Vector2(6.043962807205868, 4.5966165086010404), Vector2(7.6090640879527127, 6.277966749119483), Vector2(5.7541403121465136, 6.0188798163742314) },
            { Vector2(7.2021032519803416, 4.7593654051260845), Vector2(8.845086730990289, 4.3454699522382008), Vector2(9.0000000000000995, 5.8511464234708477) },
            { Vector2(7.2021032519803416, 4.7593654051260845), Vector2(9.0000000000000995, 5.8511464234708477), Vector2(7.6090640879527127, 6.277966749119483) },
            { Vector2(-0.072765291016583675, 6.0754724387325378), Vector2(1.5421349988792332, 6.0603071092296688), Vector2(1.392036280770125, 7.5035926853011219) },
            { Vector2(-0.072765291016583675, 6.0754724387325378), Vector2(1.392036280770125, 7.5035926853011219), Vector2(0.18481336207589308, 7.2452531018547823) },
            { Vector2(1.5421349988792332, 6.0603071092296688), Vector2(3.0119808828687988, 5.7933439766163506), Vector2(2.6895954276330265, 7.772050287955576) },
            { Vector2(1.5421349988792332, 6.0603071092296688), Vector2(2.6895954276330265, 7.772050287955576), Vector2(1.392036280770125, 7.5035926853011219) },
            { Vector2(3.0119808828687988, 5.7933439766163506), Vector2(4.3850025489624009, 6.2128337518360777), Vector2(4.3599911405723004, 7.7477355261666601) },
            { Vector2(3.0119808828687988, 5.7933439766163506), Vector2(4.3599911405723004, 7.7477355261666601), Vector2(2.6895954276330265, 7.772050287955576) },
            { Vector2(4.3850025489624009, 6.2128337518360777), Vector2(5.7541403121465136, 6.0188798163742314), Vector2(5.9930606006139149, 7.3088535018825196) },
            { Vector2(4.3850025489624009, 6.2128337518360777), Vector2(5.9930606006139149, 7.3088535018825196), Vector2(4.3599911405723004, 7.7477355261666601) },
            { Vector2(5.7541403121465136, 6.0188798163742314), Vector2(7.6090640879527127, 6.277966749119483), Vector2(7.7201825687571599, 7.6307713989128869) },
            { Vector2(5.7541403121465136, 6.0188798163742314), Vector2(7.7201825687571599, 7.6307713989128869), Vector2(5.9930606006139149, 7.3088535018825196) },
            { Vector2(7.6090640879527127, 6.277966749119483), Vector2(9.0000000000000995, 5.8511464234708477), Vector2(9.2483935539312068, 7.4213297697655882) },
            { Vector2(7.6090640879527127, 6.277966749119483), Vector2(9.2483935539312068, 7.4213297697655882), Vector2(7.7201825687571599, 7.6307713989128869) },
            { Vector2(0.18481336207589308, 7.2452531018547823), Vector2(1.392036280770125, 7.5035926853011219), Vector2(2.0000000000000999, 9.0113464734824298) },
            { Vector2(0.18481336207589308, 7.2452531018547823), Vector2(2.0000000000000999, 9.0113464734824298), Vector2(0.066280095606537498, 9.0340958536435298) },
            { Vector2(1.392036280770125, 7.5035926853011219), Vector2(2.6895954276330265, 7.772050287955576), Vector2(3.0263632124465514, 8.723554225484154) },
            { Vector2(1.392036280770125, 7.5035926853011219), Vector2(3.0263632124465514, 8.723554225484154), Vector2(2.0000000000000999, 9.0113464734824298) },
            { Vector2(2.6895954276330265, 7.772050287955576), Vector2(4.3599911405723004, 7.7477355261666601), Vector2(4.6931883359992179, 9.0232074744664015) },
            { Vector2(2.6895954276330265, 7.772050287955576), Vector2(4.6931883359992179, 9.0232074744664015), Vector2(3.0263632124465514, 8.723554225484154) },
            { Vector2(4.3599911405723004, 7.7477355261666601), Vector2(5.9930606006139149, 7.3088535018825196), Vector2(6.0085092874477519, 8.7794892970355765) },
            { Vector2(4.3599911405723004, 7.7477355261666601), Vector2(6.0085092874477519, 8.7794892970355765), Vector2(4.6931883359992179, 9.0232074744664015) },
            { Vector2(5.9930606006139149, 7.3088535018825196), Vector2(7.7201825687571599, 7.6307713989128869), Vector2(7.7938005707122917, 9.1317834488489211) },
            { Vector2(5.9930606006139149, 7.3088535018825196), Vector2(7.7938005707122917, 9.1317834488489211), Vector2(6.0085092874477519, 8.7794892970355765) },
            { Vector2(7.7201825687571599, 7.6307713989128869), Vector2(9.2483935539312068, 7.4213297697655882), Vector2(8.9688580436971641, 8.8210142096030957) },
            { Vector2(7.7201825687571599, 7.6307713989128869), Vector2(8.9688580436971641, 8.8210142096030957), Vector2(7.7938005707122917, 9.1317834488489211) },
        };
        in.singular = { 23, 23, 32, 17, 34, 38 };
        in.computeUvs = true;
        in.origMode = 1;
        in.origUvs = {
            { Vector2(-0.27212646677397767, -0.24281116499652783), Vector2(1.2346746651761988, -0.21843628021947764), Vector2(1.496415846742549, 1.4577227884451627) },
            { Vector2(-0.27212646677397767, -0.24281116499652783), Vector2(1.496415846742549, 1.4577227884451627), Vector2(0.17351041052361085, 1.2606456794563035) },
            { Vector2(1.2346746651761988, -0.21843628021947764), Vector2(2.8035520842222486, -0.01398415034145532), Vector2(2.9999999999999001, 1.5447119342086513) },
            { Vector2(1.2346746651761988, -0.21843628021947764), Vector2(2.9999999999999001, 1.5447119342086513), Vector2(1.496415846742549, 1.4577227884451627) },
            { Vector2(2.8035520842222486, -0.01398415034145532), Vector2(4.2749822569043578, 0.015711605018644593), Vector2(4.6648190489538361, 1.8220098326611416) },
            { Vector2(2.8035520842222486, -0.01398415034145532), Vector2(4.6648190489538361, 1.8220098326611416), Vector2(2.9999999999999001, 1.5447119342086513) },
            { Vector2(4.2749822569043578, 0.015711605018644593), Vector2(6.2731443333971955, 0.0031738560509082976), Vector2(5.9844545758557528, 1.5015745421036759) },
            { Vector2(4.2749822569043578, 0.015711605018644593), Vector2(5.9844545758557528, 1.5015745421036759), Vector2(4.6648190489538361, 1.8220098326611416) },
            { Vector2(6.2731443333971955, 0.0031738560509082976), Vector2(7.4953280796675061, 0.28404700220098122), Vector2(7.7837191704620929, 1.5169327395267906) },
            { Vector2(6.2731443333971955, 0.0031738560509082976), Vector2(7.7837191704620929, 1.5169327395267906), Vector2(5.9844545758557528, 1.5015745421036759) },
            { Vector2(7.4953280796675061, 0.28404700220098122), Vector2(9.2274868266983106, 0.20605367103092975), Vector2(8.9807533723408302, 1.3474351736210448) },
            { Vector2(7.4953280796675061, 0.28404700220098122), Vector2(8.9807533723408302, 1.3474351736210448), Vector2(7.7837191704620929, 1.5169327395267906) },
            { Vector2(0.17351041052361085, 1.2606456794563035), Vector2(1.496415846742549, 1.4577227884451627), Vector2(1.575892620019802, 2.9084173421719202) },
            { Vector2(0.17351041052361085, 1.2606456794563035), Vector2(1.575892620019802, 2.9084173421719202), Vector2(-0.051803244725838582, 3.0867968835655848) },
            { Vector2(1.496415846742549, 1.4577227884451627), Vector2(2.9999999999999001, 1.5447119342086513), Vector2(2.9944245892489225, 3.3699156141621573) },
            { Vector2(1.496415846742549, 1.4577227884451627), Vector2(2.9944245892489225, 3.3699156141621573), Vector2(1.575892620019802, 2.9084173421719202) },
            { Vector2(2.9999999999999001, 1.5447119342086513), Vector2(4.6648190489538361, 1.8220098326611416), Vector2(4.6144046069913323, 2.923104011489706) },
            { Vector2(2.9999999999999001, 1.5447119342086513), Vector2(4.6144046069913323, 2.923104011489706), Vector2(2.9944245892489225, 3.3699156141621573) },
            { Vector2(4.6648190489538361, 1.8220098326611416), Vector2(5.9844545758557528, 1.5015745421036759), Vector2(6.2183279590542604, 2.7473698287914523) },
            { Vector2(4.6648190489538361, 1.8220098326611416), Vector2(6.2183279590542604, 2.7473698287914523), Vector2(4.6144046069913323, 2.923104011489706) },
            { Vector2(5.9844545758557528, 1.5015745421036759), Vector2(7.7837191704620929, 1.5169327395267906), Vector2(7.7269446768704411, 3.0524336342769165) },
            { Vector2(5.9844545758557528, 1.5015745421036759), Vector2(7.7269446768704411, 3.0524336342769165), Vector2(6.2183279590542604, 2.7473698287914523) },
            { Vector2(7.7837191704620929, 1.5169327395267906), Vector2(8.9807533723408302, 1.3474351736210448), Vector2(9.135062434695941, 2.7118899643451484) },
            { Vector2(7.7837191704620929, 1.5169327395267906), Vector2(9.135062434695941, 2.7118899643451484), Vector2(7.7269446768704411, 3.0524336342769165) },
            { Vector2(-0.051803244725838582, 3.0867968835655848), Vector2(1.575892620019802, 2.9084173421719202), Vector2(1.4818924718135162, 4.5788208893517233) },
            { Vector2(-0.051803244725838582, 3.0867968835655848), Vector2(1.4818924718135162, 4.5788208893517233), Vector2(0.099503612531513297, 4.7672603587214324) },
            { Vector2(1.575892620019802, 2.9084173421719202), Vector2(2.9944245892489225, 3.3699156141621573), Vector2(3.180984658762636, 4.3994111527915747) },
            { Vector2(1.575892620019802, 2.9084173421719202), Vector2(3.180984658762636, 4.3994111527915747), Vector2(1.4818924718135162, 4.5788208893517233) },
            { Vector2(6.2183279590542604, 2.7473698287914523), Vector2(7.7269446768704411, 3.0524336342769165), Vector2(7.2021032519803416, 4.7593654051260845) },
            { Vector2(6.2183279590542604, 2.7473698287914523), Vector2(7.2021032519803416, 4.7593654051260845), Vector2(6.043962807205868, 4.5966165086010404) },
            { Vector2(7.7269446768704411, 3.0524336342769165), Vector2(9.135062434695941, 2.7118899643451484), Vector2(8.845086730990289, 4.3454699522382008) },
            { Vector2(7.7269446768704411, 3.0524336342769165), Vector2(8.845086730990289, 4.3454699522382008), Vector2(7.2021032519803416, 4.7593654051260845) },
            { Vector2(0.099503612531513297, 4.7672603587214324), Vector2(1.4818924718135162, 4.5788208893517233), Vector2(1.5421349988792332, 6.0603071092296688) },
            { Vector2(0.099503612531513297, 4.7672603587214324), Vector2(1.5421349988792332, 6.0603071092296688), Vector2(-0.072765291016583675, 6.0754724387325378) },
            { Vector2(1.4818924718135162, 4.5788208893517233), Vector2(3.180984658762636, 4.3994111527915747), Vector2(3.0119808828687988, 5.7933439766163506) },
            { Vector2(1.4818924718135162, 4.5788208893517233), Vector2(3.0119808828687988, 5.7933439766163506), Vector2(1.5421349988792332, 6.0603071092296688) },
            { Vector2(6.043962807205868, 4.5966165086010404), Vector2(7.2021032519803416, 4.7593654051260845), Vector2(7.6090640879527127, 6.277966749119483) },
            { Vector2(6.043962807205868, 4.5966165086010404), Vector2(7.6090640879527127, 6.277966749119483), Vector2(5.7541403121465136, 6.0188798163742314) },
            { Vector2(7.2021032519803416, 4.7593654051260845), Vector2(8.845086730990289, 4.3454699522382008), Vector2(9.0000000000000995, 5.8511464234708477) },
            { Vector2(7.2021032519803416, 4.7593654051260845), Vector2(9.0000000000000995, 5.8511464234708477), Vector2(7.6090640879527127, 6.277966749119483) },
            { Vector2(-0.072765291016583675, 6.0754724387325378), Vector2(1.5421349988792332, 6.0603071092296688), Vector2(1.392036280770125, 7.5035926853011219) },
            { Vector2(-0.072765291016583675, 6.0754724387325378), Vector2(1.392036280770125, 7.5035926853011219), Vector2(0.18481336207589308, 7.2452531018547823) },
            { Vector2(1.5421349988792332, 6.0603071092296688), Vector2(3.0119808828687988, 5.7933439766163506), Vector2(2.6895954276330265, 7.772050287955576) },
            { Vector2(1.5421349988792332, 6.0603071092296688), Vector2(2.6895954276330265, 7.772050287955576), Vector2(1.392036280770125, 7.5035926853011219) },
            { Vector2(3.0119808828687988, 5.7933439766163506), Vector2(4.3850025489624009, 6.2128337518360777), Vector2(4.3599911405723004, 7.7477355261666601) },
            { Vector2(3.0119808828687988, 5.7933439766163506), Vector2(4.3599911405723004, 7.7477355261666601), Vector2(2.6895954276330265, 7.772050287955576) },
            { Vector2(4.3850025489624009, 6.2128337518360777), Vector2(5.7541403121465136, 6.0188798163742314), Vector2(5.9930606006139149, 7.3088535018825196) },
            { Vector2(4.3850025489624009, 6.2128337518360777), Vector2(5.9930606006139149, 7.3088535018825196), Vector2(4.3599911405723004, 7.7477355261666601) },
            { Vector2(5.7541403121465136, 6.0188798163742314), Vector2(7.6090640879527127, 6.277966749119483), Vector2(7.7201825687571599, 7.6307713989128869) },
            { Vector2(5.7541403121465136, 6.0188798163742314), Vector2(7.7201825687571599, 7.6307713989128869), Vector2(5.9930606006139149, 7.3088535018825196) },
            { Vector2(7.6090640879527127, 6.277966749119483), Vector2(9.0000000000000995, 5.8511464234708477), Vector2(9.2483935539312068, 7.4213297697655882) },
            { Vector2(7.6090640879527127, 6.277966749119483), Vector2(9.2483935539312068, 7.4213297697655882), Vector2(7.7201825687571599, 7.6307713989128869) },
            { Vector2(0.18481336207589308, 7.2452531018547823), Vector2(1.392036280770125, 7.5035926853011219), Vector2(2.0000000000000999, 9.0113464734824298) },
            { Vector2(0.18481336207589308, 7.2452531018547823), Vector2(2.0000000000000999, 9.0113464734824298), Vector2(0.066280095606537498, 9.0340958536435298) },
            { Vector2(1.392036280770125, 7.5035926853011219), Vector2(2.6895954276330265, 7.772050287955576), Vector2(3.0263632124465514, 8.723554225484154) },
            { Vector2(1.392036280770125, 7.5035926853011219), Vector2(3.0263632124465514, 8.723554225484154), Vector2(2.0000000000000999, 9.0113464734824298) },
            { Vector2(2.6895954276330265, 7.772050287955576), Vector2(4.3599911405723004, 7.7477355261666601), Vector2(4.6931883359992179, 9.0232074744664015) },
            { Vector2(2.6895954276330265, 7.772050287955576), Vector2(4.6931883359992179, 9.0232074744664015), Vector2(3.0263632124465514, 8.723554225484154) },
            { Vector2(4.3599911405723004, 7.7477355261666601), Vector2(5.9930606006139149, 7.3088535018825196), Vector2(6.0085092874477519, 8.7794892970355765) },
            { Vector2(4.3599911405723004, 7.7477355261666601), Vector2(6.0085092874477519, 8.7794892970355765), Vector2(4.6931883359992179, 9.0232074744664015) },
            { Vector2(5.9930606006139149, 7.3088535018825196), Vector2(7.7201825687571599, 7.6307713989128869), Vector2(7.7938005707122917, 9.1317834488489211) },
            { Vector2(5.9930606006139149, 7.3088535018825196), Vector2(7.7938005707122917, 9.1317834488489211), Vector2(6.0085092874477519, 8.7794892970355765) },
            { Vector2(7.7201825687571599, 7.6307713989128869), Vector2(9.2483935539312068, 7.4213297697655882), Vector2(8.9688580436971641, 8.8210142096030957) },
            { Vector2(7.7201825687571599, 7.6307713989128869), Vector2(8.9688580436971641, 8.8210142096030957), Vector2(7.7938005707122917, 9.1317834488489211) },
        };
        dumpCase(index++, in);
    }
    {
        // Lifted from a seeded run (seed-0 case 162 (five-merge)): fires the pass named below.
        CaseInput in;
        in.vertices = {
            Vector3(0.15041923556012687, -0.16336042599659784, 0.082777683818553285),
            Vector3(0.93757718189345085, 0.052006914133716636, -0.091509434120569444),
            Vector3(2.0900905926568485, 0.076170228230416998, 0.058076305108425166),
            Vector3(2.8415903084130338, -0.090408223617166439, 0.02137770996703019),
            Vector3(4.181795238075531, -0.10350202015731109, 0.087872162187749916),
            Vector3(4.9222605143846163, 0.026855276103757, -0.081251650001838094),
            Vector3(5.868773179111372, -0.03998009237814109, -0.0026316708572186043),
            Vector3(-0.18062358794347155, 1.1235723587612376, -0.059936641984065543),
            Vector3(0.92340709077851191, 0.81251025509009533, 0.059483777622863415),
            Vector3(1.8015999653538444, 0.8090392867007774, -0.075375230529244552),
            Vector3(3.0378475665603779, 0.85687150227379094, 0.050595215427352881),
            Vector3(4.1747235828045381, 0.98778575953127523, 0.058514658094233757),
            Vector3(5.0575573649954171, 0.88889761920199706, 0.028773482415346031),
            Vector3(5.8190469136124996, 0.80067767077307583, 0.088699562231736723),
            Vector3(0.034816792258939924, 2.0100839049912036, -0.082513892624191135),
            Vector3(0.8880945229111743, 1.9762324802295523, 0.036584299037593949),
            Vector3(1.8313676688015734, 1.8190434483320432, -0.015762925810837936),
            Vector3(2.8950503680353852, 1.9183878406705901, -0.093296317182890207),
            Vector3(4.0920378790336711, 2.0040200072504306, -0.071137210868916931),
            Vector3(5.0752220743918697, 2.1699434858447586, -0.058057191048898754),
            Vector3(5.9885663932212951, 1.8141147252379874, 0.023374163686429375),
            Vector3(0.12227059606335233, 2.917929305418645, 0.019344757113199541),
            Vector3(0.86643177220037204, 2.932491777706451, -0.082670557948737303),
            Vector3(2.1176471055398736, 3.1081222555234795, -0.0081172252301361381),
            Vector3(2.8998616340391523, 3.1183389552966898, -0.0054654793763793345),
            Vector3(3.8452744072127381, 2.8442574907109819, -0.088881428330511991),
            Vector3(5.0676990410557288, 2.9229830602292086, -0.085140693229204367),
            Vector3(5.890166547345876, 3.1875812435195536, -0.084032316955729464),
            Vector3(0.1575085109608852, 3.9841624230574912, -0.028991443769261039),
            Vector3(0.99534683158367976, 4.13106393526001, -0.0018820460322475309),
            Vector3(2.0081305474020206, 4.0047752898605706, 0.070394633270735585),
            Vector3(3.0109956741909474, 4.1547138334521749, 0.066640455701658774),
            Vector3(4.0887857797776999, 4.1055543393651677, -0.052203792937958272),
            Vector3(5.0734099246245252, 3.9976009170147817, -0.016572907982875983),
            Vector3(5.8677847529372267, 3.9176876633359035, 0.04678371647602695),
            Vector3(-0.040005312123336759, 4.895932614839853, -0.053450450452080883),
            Vector3(1.0121417646405693, 5.0552882508724348, -0.077015153586728419),
            Vector3(1.8346727499727411, 4.9407120523911114, 0.095586289154797083),
            Vector3(2.9150947846924309, 4.8390226514332424, 0.0082377273203399388),
            Vector3(3.9780526224211314, 5.0115500354498828, 0.083925311305082323),
            Vector3(4.972149523162309, 4.8940134715117711, -0.066515533809596558),
            Vector3(5.9841205219396265, 4.8764035128431313, 0.059002985484217167),
            Vector3(-0.015859833716401938, 5.8619684163505408, 0.0021310572212468459),
            Vector3(1.0004613794246962, 6.1105487020134435, -0.0074232574492645092),
            Vector3(1.8714291798166784, 5.9185187377517838, -0.012977668207214599),
            Vector3(3.0916303993423702, 6.0708945652537407, 0.030159235256784347),
            Vector3(3.8136669966456775, 6.1073438965127025, -0.04996988821177642),
            Vector3(5.0688293219650147, 5.9609570437529014, 0.080314962482704672),
            Vector3(6.0720325114385112, 5.9318263847028891, 0.035716760257106774),
        };
        in.triangles = {
            { 0, 1, 8 },
            { 0, 8, 7 },
            { 1, 2, 9 },
            { 1, 9, 8 },
            { 2, 3, 10 },
            { 2, 10, 9 },
            { 3, 4, 11 },
            { 3, 11, 10 },
            { 4, 5, 12 },
            { 4, 12, 11 },
            { 5, 6, 13 },
            { 5, 13, 12 },
            { 7, 8, 15 },
            { 7, 15, 14 },
            { 8, 9, 16 },
            { 8, 16, 15 },
            { 9, 10, 17 },
            { 9, 17, 16 },
            { 10, 11, 18 },
            { 10, 18, 17 },
            { 11, 12, 19 },
            { 11, 19, 18 },
            { 12, 13, 20 },
            { 12, 20, 19 },
            { 14, 15, 22 },
            { 14, 22, 21 },
            { 15, 16, 23 },
            { 15, 23, 22 },
            { 16, 17, 24 },
            { 16, 24, 23 },
            { 17, 18, 25 },
            { 17, 25, 24 },
            { 18, 19, 26 },
            { 18, 26, 25 },
            { 19, 20, 27 },
            { 19, 27, 26 },
            { 21, 22, 29 },
            { 21, 29, 28 },
            { 22, 23, 30 },
            { 22, 30, 29 },
            { 23, 24, 31 },
            { 23, 31, 30 },
            { 24, 25, 32 },
            { 24, 32, 31 },
            { 25, 26, 33 },
            { 25, 33, 32 },
            { 26, 27, 34 },
            { 26, 34, 33 },
            { 28, 29, 36 },
            { 28, 36, 35 },
            { 29, 30, 37 },
            { 29, 37, 36 },
            { 30, 31, 38 },
            { 30, 38, 37 },
            { 31, 32, 39 },
            { 31, 39, 38 },
            { 32, 33, 40 },
            { 32, 40, 39 },
            { 33, 34, 41 },
            { 33, 41, 40 },
            { 35, 36, 43 },
            { 35, 43, 42 },
            { 36, 37, 44 },
            { 36, 44, 43 },
            { 37, 38, 45 },
            { 37, 45, 44 },
            { 38, 39, 46 },
            { 38, 46, 45 },
            { 39, 40, 47 },
            { 39, 47, 46 },
            { 40, 41, 48 },
            { 40, 48, 47 },
        };
        in.uvs = {
            { Vector2(0.089947834703167348, -0.25429642474321418), Vector2(0.90578508910782862, 0.027038428650232541), Vector2(0.95656446579938714, 0.82273743619771467) },
            { Vector2(0.089947834703167348, -0.25429642474321418), Vector2(0.95656446579938714, 0.82273743619771467), Vector2(-0.17425645631551934, 1.1093927448630883) },
            { Vector2(0.90578508910782862, 0.027038428650232541), Vector2(2.0709392712746126, 0.11587085751762437), Vector2(1.8866781072713616, 0.86282891520407612) },
            { Vector2(0.90578508910782862, 0.027038428650232541), Vector2(1.8866781072713616, 0.86282891520407612), Vector2(0.95656446579938714, 0.82273743619771467) },
            { Vector2(2.0709392712746126, 0.11587085751762437), Vector2(-0.058123234339000802, 3.0858726056104118), Vector2(0.8151576088194894, 2.882951851863651) },
            { Vector2(2.0709392712746126, 0.11587085751762437), Vector2(0.8151576088194894, 2.882951851863651), Vector2(1.8866781072713616, 0.86282891520407612) },
            { Vector2(-0.058123234339000802, 3.0858726056104118), Vector2(-0.028088588107409621, 1.8635728397972484), Vector2(0.99443969700149326, 1.8549833402467217) },
            { Vector2(-0.058123234339000802, 3.0858726056104118), Vector2(0.99443969700149326, 1.8549833402467217), Vector2(0.8151576088194894, 2.882951851863651) },
            { Vector2(-0.028088588107409621, 1.8635728397972484), Vector2(0.12109246966120216, 1.0886969675834903), Vector2(0.9319361185936974, 0.92465245443837885) },
            { Vector2(-0.028088588107409621, 1.8635728397972484), Vector2(0.9319361185936974, 0.92465245443837885), Vector2(0.99443969700149326, 1.8549833402467217) },
            { Vector2(0.12109246966120216, 1.0886969675834903), Vector2(0.0041555904379266817, 0.058115174149109688), Vector2(0.77824537662971582, 0.20272730676830888) },
            { Vector2(0.12109246966120216, 1.0886969675834903), Vector2(0.77824537662971582, 0.20272730676830888), Vector2(0.9319361185936974, 0.92465245443837885) },
            { Vector2(-0.17425645631551934, 1.1093927448630883), Vector2(0.95656446579938714, 0.82273743619771467), Vector2(0.85783238178225396, 1.912175073627715) },
            { Vector2(-0.17425645631551934, 1.1093927448630883), Vector2(0.85783238178225396, 1.912175073627715), Vector2(-0.016992852923847022, 1.9975500780058209) },
            { Vector2(0.95656446579938714, 0.82273743619771467), Vector2(1.8866781072713616, 0.86282891520407612), Vector2(1.7715566306605235, 1.8044186473745312) },
            { Vector2(0.95656446579938714, 0.82273743619771467), Vector2(1.7715566306605235, 1.8044186473745312), Vector2(0.85783238178225396, 1.912175073627715) },
            { Vector2(1.8866781072713616, 0.86282891520407612), Vector2(0.8151576088194894, 2.882951851863651), Vector2(1.9192089145595108, 3.1115546695989909) },
            { Vector2(1.8866781072713616, 0.86282891520407612), Vector2(1.9192089145595108, 3.1115546695989909), Vector2(1.7715566306605235, 1.8044186473745312) },
            { Vector2(0.8151576088194894, 2.882951851863651), Vector2(0.99443969700149326, 1.8549833402467217), Vector2(2.0834228069567331, 1.8380614634824861) },
            { Vector2(0.8151576088194894, 2.882951851863651), Vector2(2.0834228069567331, 1.8380614634824861), Vector2(1.9192089145595108, 3.1115546695989909) },
            { Vector2(0.99443969700149326, 1.8549833402467217), Vector2(0.9319361185936974, 0.92465245443837885), Vector2(2.2249090866476848, 1.0216114607913651) },
            { Vector2(0.99443969700149326, 1.8549833402467217), Vector2(2.2249090866476848, 1.0216114607913651), Vector2(2.0834228069567331, 1.8380614634824861) },
            { Vector2(0.9319361185936974, 0.92465245443837885), Vector2(0.77824537662971582, 0.20272730676830888), Vector2(1.791592406584942, 0.018568829812358099) },
            { Vector2(0.9319361185936974, 0.92465245443837885), Vector2(1.791592406584942, 0.018568829812358099), Vector2(2.2249090866476848, 1.0216114607913651) },
            { Vector2(-0.016992852923847022, 1.9975500780058209), Vector2(0.85783238178225396, 1.912175073627715), Vector2(0.95550068433993696, 3.0163830761575614) },
            { Vector2(-0.016992852923847022, 1.9975500780058209), Vector2(0.95550068433993696, 3.0163830761575614), Vector2(0.12589344821217438, 2.967296306055156) },
            { Vector2(0.85783238178225396, 1.912175073627715), Vector2(1.7715566306605235, 1.8044186473745312), Vector2(2.0513524480030982, 3.1289326058632083) },
            { Vector2(0.85783238178225396, 1.912175073627715), Vector2(2.0513524480030982, 3.1289326058632083), Vector2(0.95550068433993696, 3.0163830761575614) },
            { Vector2(1.7715566306605235, 1.8044186473745312), Vector2(1.9192089145595108, 3.1115546695989909), Vector2(3.1959225076933695, 3.0180142200906257) },
            { Vector2(1.7715566306605235, 1.8044186473745312), Vector2(3.1959225076933695, 3.0180142200906257), Vector2(2.0513524480030982, 3.1289326058632083) },
            { Vector2(1.9192089145595108, 3.1115546695989909), Vector2(2.0834228069567331, 1.8380614634824861), Vector2(2.8412155499641694, 2.225168112284039) },
            { Vector2(1.9192089145595108, 3.1115546695989909), Vector2(2.8412155499641694, 2.225168112284039), Vector2(3.1959225076933695, 3.0180142200906257) },
            { Vector2(2.0834228069567331, 1.8380614634824861), Vector2(2.2249090866476848, 1.0216114607913651), Vector2(3.0046231599788822, 0.98652721154221601) },
            { Vector2(2.0834228069567331, 1.8380614634824861), Vector2(3.0046231599788822, 0.98652721154221601), Vector2(2.8412155499641694, 2.225168112284039) },
            { Vector2(2.2249090866476848, 1.0216114607913651), Vector2(1.791592406584942, 0.018568829812358099), Vector2(3.1487695416929005, 0.12426906415262925) },
            { Vector2(2.2249090866476848, 1.0216114607913651), Vector2(3.1487695416929005, 0.12426906415262925), Vector2(3.0046231599788822, 0.98652721154221601) },
            { Vector2(0.12589344821217438, 2.967296306055156), Vector2(0.95550068433993696, 3.0163830761575614), Vector2(1.0005376139374551, 4.0462060045585106) },
            { Vector2(0.12589344821217438, 2.967296306055156), Vector2(1.0005376139374551, 4.0462060045585106), Vector2(0.13887911310349932, 4.0283531704023892) },
            { Vector2(0.95550068433993696, 3.0163830761575614), Vector2(2.0513524480030982, 3.1289326058632083), Vector2(2.0120871649031966, 4.0793671564120269) },
            { Vector2(0.95550068433993696, 3.0163830761575614), Vector2(2.0120871649031966, 4.0793671564120269), Vector2(1.0005376139374551, 4.0462060045585106) },
            { Vector2(2.0513524480030982, 3.1289326058632083), Vector2(3.1959225076933695, 3.0180142200906257), Vector2(4.1563032124726602, 3.0187319810027278) },
            { Vector2(2.0513524480030982, 3.1289326058632083), Vector2(4.1563032124726602, 3.0187319810027278), Vector2(2.0120871649031966, 4.0793671564120269) },
            { Vector2(3.1959225076933695, 3.0180142200906257), Vector2(2.8412155499641694, 2.225168112284039), Vector2(4.2041083350885327, 1.9389019408539112) },
            { Vector2(3.1959225076933695, 3.0180142200906257), Vector2(4.2041083350885327, 1.9389019408539112), Vector2(4.1563032124726602, 3.0187319810027278) },
            { Vector2(2.8412155499641694, 2.225168112284039), Vector2(3.0046231599788822, 0.98652721154221601), Vector2(3.9644871895865554, 0.9951916139826863) },
            { Vector2(2.8412155499641694, 2.225168112284039), Vector2(3.9644871895865554, 0.9951916139826863), Vector2(4.2041083350885327, 1.9389019408539112) },
            { Vector2(3.0046231599788822, 0.98652721154221601), Vector2(3.1487695416929005, 0.12426906415262925), Vector2(3.970854503054456, 0.13876157407179091) },
            { Vector2(3.0046231599788822, 0.98652721154221601), Vector2(3.970854503054456, 0.13876157407179091), Vector2(3.9644871895865554, 0.9951916139826863) },
            { Vector2(0.13887911310349932, 4.0283531704023892), Vector2(1.0005376139374551, 4.0462060045585106), Vector2(0.97481177633604088, 4.960399348550558) },
            { Vector2(0.13887911310349932, 4.0283531704023892), Vector2(0.97481177633604088, 4.960399348550558), Vector2(0.00028267043649323614, 4.9853285312334839) },
            { Vector2(1.0005376139374551, 4.0462060045585106), Vector2(2.0120871649031966, 4.0793671564120269), Vector2(1.7986235693014496, 4.9463597580078389) },
            { Vector2(1.0005376139374551, 4.0462060045585106), Vector2(1.7986235693014496, 4.9463597580078389), Vector2(0.97481177633604088, 4.960399348550558) },
            { Vector2(2.0120871649031966, 4.0793671564120269), Vector2(4.1563032124726602, 3.0187319810027278), Vector2(4.7404477198945845, 3.1100404394665677) },
            { Vector2(2.0120871649031966, 4.0793671564120269), Vector2(4.7404477198945845, 3.1100404394665677), Vector2(1.7986235693014496, 4.9463597580078389) },
            { Vector2(4.1563032124726602, 3.0187319810027278), Vector2(4.2041083350885327, 1.9389019408539112), Vector2(5.034527757234633, 1.9446235573111263) },
            { Vector2(4.1563032124726602, 3.0187319810027278), Vector2(5.034527757234633, 1.9446235573111263), Vector2(4.7404477198945845, 3.1100404394665677) },
            { Vector2(4.2041083350885327, 1.9389019408539112), Vector2(3.9644871895865554, 0.9951916139826863), Vector2(4.9434811524938187, 1.0453968577504709) },
            { Vector2(4.2041083350885327, 1.9389019408539112), Vector2(4.9434811524938187, 1.0453968577504709), Vector2(5.034527757234633, 1.9446235573111263) },
            { Vector2(3.9644871895865554, 0.9951916139826863), Vector2(3.970854503054456, 0.13876157407179091), Vector2(4.861061946729472, -0.056641550395374773) },
            { Vector2(3.9644871895865554, 0.9951916139826863), Vector2(4.861061946729472, -0.056641550395374773), Vector2(4.9434811524938187, 1.0453968577504709) },
            { Vector2(0.00028267043649323614, 4.9853285312334839), Vector2(0.97481177633604088, 4.960399348550558), Vector2(1.0304597738322254, 6.1105825230111295) },
            { Vector2(0.00028267043649323614, 4.9853285312334839), Vector2(1.0304597738322254, 6.1105825230111295), Vector2(-0.090623825748532297, 5.7630494332024771) },
            { Vector2(0.97481177633604088, 4.960399348550558), Vector2(1.7986235693014496, 4.9463597580078389), Vector2(1.7789454650541436, 5.9338269240149168) },
            { Vector2(0.97481177633604088, 4.960399348550558), Vector2(1.7789454650541436, 5.9338269240149168), Vector2(1.0304597738322254, 6.1105825230111295) },
            { Vector2(1.7986235693014496, 4.9463597580078389), Vector2(4.7404477198945845, 3.1100404394665677), Vector2(6.0233362956080212, 2.8338033531588129) },
            { Vector2(1.7986235693014496, 4.9463597580078389), Vector2(6.0233362956080212, 2.8338033531588129), Vector2(1.7789454650541436, 5.9338269240149168) },
            { Vector2(4.7404477198945845, 3.1100404394665677), Vector2(5.034527757234633, 1.9446235573111263), Vector2(6.051684562055927, 2.2376762515618145) },
            { Vector2(4.7404477198945845, 3.1100404394665677), Vector2(6.051684562055927, 2.2376762515618145), Vector2(6.0233362956080212, 2.8338033531588129) },
            { Vector2(5.034527757234633, 1.9446235573111263), Vector2(4.9434811524938187, 1.0453968577504709), Vector2(5.8706701305018774, 0.941573189732684) },
            { Vector2(5.034527757234633, 1.9446235573111263), Vector2(5.8706701305018774, 0.941573189732684), Vector2(6.051684562055927, 2.2376762515618145) },
            { Vector2(4.9434811524938187, 1.0453968577504709), Vector2(4.861061946729472, -0.056641550395374773), Vector2(5.8689555226572416, -0.15638455311227994) },
            { Vector2(4.9434811524938187, 1.0453968577504709), Vector2(5.8689555226572416, -0.15638455311227994), Vector2(5.8706701305018774, 0.941573189732684) },
        };
        in.computeUvs = false;
        in.origMode = 0;
        dumpCase(index++, in);
    }
    {
        // Lifted from a seeded run (seed-0 case 23 (fan convert)): fires the pass named below.
        CaseInput in;
        in.vertices = {
            Vector3(-0.18344552262848618, 0.064968540852667372, -0.022359543138811767),
            Vector3(0.91156543624800235, 0.18296046498910359, 0.047039085626722169),
            Vector3(2.1882391977902085, 0.0024431706912361852, 0.038618632850650259),
            Vector3(3.1590073911381071, -0.087693542975310387, -0.012150188283574993),
            Vector3(4.048546466305349, -0.12335773388413857, 0.067138069619020163),
            Vector3(4.9759720261656426, -0.16933900175109323, -0.039423616015772871),
            Vector3(6.0298012395415759, -0.089705330582318832, 0.042992553712809682),
            Vector3(6.9150334605182371, 0.087447459999937263, 0.037344965345000668),
            Vector3(7.8507671675192396, -0.017622112151510685, -0.0036033573808209153),
            Vector3(0.17047838354909073, 1.01521532479275, 0.048285745489293479),
            Vector3(0.96370322690212806, 0.9132827640045349, -0.094523672064504183),
            Vector3(2.093112400836457, 1.0154121416617878, 0.063268248286253881),
            Vector3(2.9222448378073755, 1.0058727333645781, -0.055753065345921352),
            Vector3(4.0170148949146478, 1.1611102305002896, 0.080925917547294016),
            Vector3(5.0912088515493243, 0.95180458791212086, -0.035743219673558514),
            Vector3(6.1734867992144054, 1.1202795824527856, 0.092980223603011156),
            Vector3(6.8422399568296379, 1.078746916310229, -0.028829960347117004),
            Vector3(7.9595955428702387, 0.88748662391831934, -0.028434572325147324),
            Vector3(-0.12437174719688474, 2.0462030022506568, -0.047549889435148575),
            Vector3(1.0870052811118378, 1.946285487719281, -0.025159124110468813),
            Vector3(1.8314515828457996, 2.0908074287167757, 0.053317580628352235),
            Vector3(3.1152873572741706, 1.9045009435530893, -0.08865146311651538),
            Vector3(3.8900196324385141, 2.0973721324075796, 0.072010950851876365),
            Vector3(4.8916128211194687, 1.9335610102158989, 0.088122133663824667),
            Vector3(5.8791987417857126, 2.1255203452178568, -0.098751147703117909),
            Vector3(7.1804628057580286, 2.1746872942352136, 0.050574553469211692),
            Vector3(7.8973434383716219, 1.8289677124579704, -0.035895135147324231),
            Vector3(0.11623016429085982, 3.0487869632249209, -0.096188448860335313),
            Vector3(0.8888348953083034, 2.8542312418182001, -0.039858342387327217),
            Vector3(2.1825334044313314, 3.108901739737076, 0.089542944952194503),
            Vector3(2.8282581616319948, 3.1555105097548752, 0.055762635214857893),
            Vector3(3.9597049948431198, 2.8333765197781946, 0.019420305045377353),
            Vector3(4.8063822529068077, 2.8090459069069333, 0.080000336821813831),
            Vector3(5.9973119296733923, 2.9703413267007388, -0.043714367840855722),
            Vector3(7.1890117120846835, 3.0231629216965348, -0.085049035243891541),
            Vector3(8.040741419674033, 2.8349698749259256, -0.031202614392262729),
            Vector3(-0.17293117418550397, 3.8948218032213031, -0.040037585331846808),
            Vector3(0.97717857521485463, 4.0687547496569074, 0.081773878058820249),
            Vector3(2.1779319170395595, 4.0812574756792568, -0.012995995889080359),
            Vector3(2.8804599216056443, 3.9879877191461439, -0.018585997850369185),
            Vector3(3.8940587595522094, 3.9202974605272867, -0.068292397776357427),
            Vector3(5.0467760878970163, 4.0023203696485936, -0.048680600215367423),
            Vector3(5.8475311584143332, 4.0440787405632186, -0.048075005153489482),
            Vector3(7.1774429454369812, 3.8790909973530758, 0.014471463769225945),
            Vector3(8.1723612178742044, 4.0180692506747349, 0.083278748669781355),
            Vector3(-0.15587727936710186, 5.199222631092125, -0.097346161800977882),
            Vector3(1.174827734676388, 4.8184680845165468, 0.05940456539086008),
            Vector3(2.0410427887434088, 4.9146794231507682, -0.094399724986085898),
            Vector3(2.8304748442918219, 4.8092476180646733, 0.096279398448737286),
            Vector3(3.9760687593822643, 4.9834821752543963, -0.0087543609435408957),
            Vector3(5.0482917014300082, 4.8857359337851607, -0.083665452112059668),
            Vector3(5.9005595159556528, 4.9362373001244251, -0.030739742188855914),
            Vector3(6.8775002260549964, 5.0820517608771185, 0.053880996440069918),
            Vector3(8.1675863854959694, 5.185382439860116, 0.062133519718727472),
            Vector3(0.19639801691506087, 6.1986878998456669, -0.068898523857115465),
            Vector3(0.97997109366202673, 6.1125902252036388, -0.0064292741449191396),
            Vector3(2.1072339092441212, 5.8958104447041455, 0.018214836756655872),
            Vector3(2.8279726934295684, 6.0216237956884449, 0.01754787068713113),
            Vector3(3.8384088438019455, 5.8456684652147262, 0.072399222876588801),
            Vector3(4.9751118429309047, 6.1752623164703397, 0.065080080693294615),
            Vector3(6.1193624048376218, 6.182284224953511, -0.056429308855119299),
            Vector3(6.9422295759315542, 5.9009307719294775, 0.079907682718423176),
            Vector3(7.8819653456312002, 6.1800905026118054, 0.0071042796776971569),
        };
        in.triangles = {
            { 0, 1, 10 },
            { 0, 10, 9 },
            { 1, 2, 11 },
            { 1, 11, 10 },
            { 2, 3, 12 },
            { 2, 12, 11 },
            { 3, 4, 13 },
            { 3, 13, 12 },
            { 4, 5, 14 },
            { 4, 14, 13 },
            { 5, 6, 15 },
            { 5, 15, 14 },
            { 6, 7, 16 },
            { 6, 16, 15 },
            { 7, 8, 17 },
            { 7, 17, 16 },
            { 9, 10, 19 },
            { 9, 19, 18 },
            { 10, 11, 20 },
            { 10, 20, 19 },
            { 11, 12, 21 },
            { 11, 21, 20 },
            { 12, 13, 22 },
            { 12, 22, 21 },
            { 13, 14, 23 },
            { 13, 23, 22 },
            { 14, 15, 24 },
            { 14, 24, 23 },
            { 15, 16, 25 },
            { 15, 25, 24 },
            { 16, 17, 26 },
            { 16, 26, 25 },
            { 18, 19, 28 },
            { 18, 28, 27 },
            { 19, 20, 29 },
            { 19, 29, 28 },
            { 20, 21, 30 },
            { 20, 30, 29 },
            { 21, 22, 31 },
            { 21, 31, 30 },
            { 22, 23, 32 },
            { 22, 32, 31 },
            { 23, 24, 33 },
            { 23, 33, 32 },
            { 24, 25, 34 },
            { 24, 34, 33 },
            { 25, 26, 35 },
            { 25, 35, 34 },
            { 27, 28, 37 },
            { 27, 37, 36 },
            { 28, 29, 38 },
            { 28, 38, 37 },
            { 29, 30, 39 },
            { 29, 39, 38 },
            { 30, 31, 40 },
            { 30, 40, 39 },
            { 31, 32, 41 },
            { 31, 41, 40 },
            { 32, 33, 42 },
            { 32, 42, 41 },
            { 33, 34, 43 },
            { 33, 43, 42 },
            { 34, 35, 44 },
            { 34, 44, 43 },
            { 36, 37, 46 },
            { 36, 46, 45 },
            { 37, 38, 47 },
            { 37, 47, 46 },
            { 38, 39, 48 },
            { 38, 48, 47 },
            { 39, 40, 49 },
            { 39, 49, 48 },
            { 40, 41, 50 },
            { 40, 50, 49 },
            { 41, 42, 51 },
            { 41, 51, 50 },
            { 42, 43, 52 },
            { 42, 52, 51 },
            { 43, 44, 53 },
            { 43, 53, 52 },
            { 45, 46, 55 },
            { 45, 55, 54 },
            { 46, 47, 56 },
            { 46, 56, 55 },
            { 47, 48, 57 },
            { 47, 57, 56 },
            { 48, 49, 58 },
            { 48, 58, 57 },
            { 49, 50, 59 },
            { 49, 59, 58 },
            { 50, 51, 60 },
            { 50, 60, 59 },
            { 51, 52, 61 },
            { 51, 61, 60 },
            { 52, 53, 62 },
            { 52, 62, 61 },
        };
        in.uvs = {
            { Vector2(-0.17625250278227644, 0.11677462359339447), Vector2(0.96821654223116971, 0.2387867413148263), Vector2(0.98918220779736232, 0.95295819055164199) },
            { Vector2(-0.17625250278227644, 0.11677462359339447), Vector2(0.98918220779736232, 0.95295819055164199), Vector2(0.073893862978196528, 0.96975279719903784) },
            { Vector2(0.96821654223116971, 0.2387867413148263), Vector2(2.1187440160620401, -0.0013983679022050886), Vector2(2.0727382253485067, 0.94210650467000934) },
            { Vector2(0.96821654223116971, 0.2387867413148263), Vector2(2.0727382253485067, 0.94210650467000934), Vector2(0.98918220779736232, 0.95295819055164199) },
            { Vector2(2.1187440160620401, -0.0013983679022050886), Vector2(3.1717121120416567, -0.01051240653782419), Vector2(2.9328221311069838, 0.97264865704163894) },
            { Vector2(2.1187440160620401, -0.0013983679022050886), Vector2(2.9328221311069838, 0.97264865704163894), Vector2(2.0727382253485067, 0.94210650467000934) },
            { Vector2(3.1717121120416567, -0.01051240653782419), Vector2(-0.050256340959847712, 3.9828484382472071), Vector2(1.1168472826651907, 3.8883157250014104) },
            { Vector2(3.1717121120416567, -0.01051240653782419), Vector2(1.1168472826651907, 3.8883157250014104), Vector2(2.9328221311069838, 0.97264865704163894) },
            { Vector2(-0.050256340959847712, 3.9828484382472071), Vector2(-0.24442819533519256, 3.0640890940721084), Vector2(0.98537462727456326, 2.9743996209928452) },
            { Vector2(-0.050256340959847712, 3.9828484382472071), Vector2(0.98537462727456326, 2.9743996209928452), Vector2(1.1168472826651907, 3.8883157250014104) },
            { Vector2(-0.24442819533519256, 3.0640890940721084), Vector2(0.0074028780821612239, 1.9910648434468108), Vector2(1.0624105420340213, 1.8775214672517013) },
            { Vector2(-0.24442819533519256, 3.0640890940721084), Vector2(1.0624105420340213, 1.8775214672517013), Vector2(0.98537462727456326, 2.9743996209928452) },
            { Vector2(0.0074028780821612239, 1.9910648434468108), Vector2(0.13715893637162002, 1.1547744197732615), Vector2(1.1075783961783621, 1.2161774283760902) },
            { Vector2(0.0074028780821612239, 1.9910648434468108), Vector2(1.1075783961783621, 1.2161774283760902), Vector2(1.0624105420340213, 1.8775214672517013) },
            { Vector2(0.13715893637162002, 1.1547744197732615), Vector2(0.032259381404624435, 0.14592223422539), Vector2(0.78946946999968071, 0.057803117953874758) },
            { Vector2(0.13715893637162002, 1.1547744197732615), Vector2(0.78946946999968071, 0.057803117953874758), Vector2(1.1075783961783621, 1.2161774283760902) },
            { Vector2(0.073893862978196528, 0.96975279719903784), Vector2(0.98918220779736232, 0.95295819055164199), Vector2(1.110139361049604, 2.0258498491245449) },
            { Vector2(0.073893862978196528, 0.96975279719903784), Vector2(1.110139361049604, 2.0258498491245449), Vector2(-0.17576873296494289, 2.0612989443860252) },
            { Vector2(0.98918220779736232, 0.95295819055164199), Vector2(2.0727382253485067, 0.94210650467000934), Vector2(1.8307400439396795, 2.1333417066165064) },
            { Vector2(0.98918220779736232, 0.95295819055164199), Vector2(1.8307400439396795, 2.1333417066165064), Vector2(1.110139361049604, 2.0258498491245449) },
            { Vector2(2.0727382253485067, 0.94210650467000934), Vector2(2.9328221311069838, 0.97264865704163894), Vector2(3.2125743923363883, 1.9781614706049147) },
            { Vector2(2.0727382253485067, 0.94210650467000934), Vector2(3.2125743923363883, 1.9781614706049147), Vector2(1.8307400439396795, 2.1333417066165064) },
            { Vector2(2.9328221311069838, 0.97264865704163894), Vector2(1.1168472826651907, 3.8883157250014104), Vector2(2.0575000098238574, 4.170222964827806) },
            { Vector2(2.9328221311069838, 0.97264865704163894), Vector2(2.0575000098238574, 4.170222964827806), Vector2(3.2125743923363883, 1.9781614706049147) },
            { Vector2(1.1168472826651907, 3.8883157250014104), Vector2(0.98537462727456326, 2.9743996209928452), Vector2(1.9905794664086685, 3.2025917413824776) },
            { Vector2(1.1168472826651907, 3.8883157250014104), Vector2(1.9905794664086685, 3.2025917413824776), Vector2(2.0575000098238574, 4.170222964827806) },
            { Vector2(0.98537462727456326, 2.9743996209928452), Vector2(1.0624105420340213, 1.8775214672517013), Vector2(2.0581469870173352, 2.0430725184806842) },
            { Vector2(0.98537462727456326, 2.9743996209928452), Vector2(2.0581469870173352, 2.0430725184806842), Vector2(1.9905794664086685, 3.2025917413824776) },
            { Vector2(1.0624105420340213, 1.8775214672517013), Vector2(1.1075783961783621, 1.2161774283760902), Vector2(2.2386818517291585, 0.75697619415367934) },
            { Vector2(1.0624105420340213, 1.8775214672517013), Vector2(2.2386818517291585, 0.75697619415367934), Vector2(2.0581469870173352, 2.0430725184806842) },
            { Vector2(1.1075783961783621, 1.2161774283760902), Vector2(0.78946946999968071, 0.057803117953874758), Vector2(1.7914566024007799, 0.019751922790278376) },
            { Vector2(1.1075783961783621, 1.2161774283760902), Vector2(1.7914566024007799, 0.019751922790278376), Vector2(2.2386818517291585, 0.75697619415367934) },
            { Vector2(-0.17576873296494289, 2.0612989443860252), Vector2(1.110139361049604, 2.0258498491245449), Vector2(0.87367195024205546, 2.7643832530581478) },
            { Vector2(-0.17576873296494289, 2.0612989443860252), Vector2(0.87367195024205546, 2.7643832530581478), Vector2(0.03374680592185976, 2.9540670019564095) },
            { Vector2(1.110139361049604, 2.0258498491245449), Vector2(1.8307400439396795, 2.1333417066165064), Vector2(2.2757074946435059, 3.1539437539456276) },
            { Vector2(1.110139361049604, 2.0258498491245449), Vector2(2.2757074946435059, 3.1539437539456276), Vector2(0.87367195024205546, 2.7643832530581478) },
            { Vector2(1.8307400439396795, 2.1333417066165064), Vector2(3.2125743923363883, 1.9781614706049147), Vector2(2.7993390851111739, 3.1800242645073724) },
            { Vector2(1.8307400439396795, 2.1333417066165064), Vector2(2.7993390851111739, 3.1800242645073724), Vector2(2.2757074946435059, 3.1539437539456276) },
            { Vector2(3.2125743923363883, 1.9781614706049147), Vector2(2.0575000098238574, 4.170222964827806), Vector2(2.9186610239387076, 4.1316467986243728) },
            { Vector2(3.2125743923363883, 1.9781614706049147), Vector2(2.9186610239387076, 4.1316467986243728), Vector2(2.7993390851111739, 3.1800242645073724) },
            { Vector2(2.0575000098238574, 4.170222964827806), Vector2(1.9905794664086685, 3.2025917413824776), Vector2(2.8971241393089042, 3.2791119547671546) },
            { Vector2(2.0575000098238574, 4.170222964827806), Vector2(2.8971241393089042, 3.2791119547671546), Vector2(2.9186610239387076, 4.1316467986243728) },
            { Vector2(1.9905794664086685, 3.2025917413824776), Vector2(2.0581469870173352, 2.0430725184806842), Vector2(3.0584943346761717, 2.0404794185176858) },
            { Vector2(1.9905794664086685, 3.2025917413824776), Vector2(3.0584943346761717, 2.0404794185176858), Vector2(2.8971241393089042, 3.2791119547671546) },
            { Vector2(2.0581469870173352, 2.0430725184806842), Vector2(2.2386818517291585, 0.75697619415367934), Vector2(3.0332394101332203, 0.778029958239568) },
            { Vector2(2.0581469870173352, 2.0430725184806842), Vector2(3.0332394101332203, 0.778029958239568), Vector2(3.0584943346761717, 2.0404794185176858) },
            { Vector2(2.2386818517291585, 0.75697619415367934), Vector2(1.7914566024007799, 0.019751922790278376), Vector2(2.8913715538978257, -0.06649366763518387) },
            { Vector2(2.2386818517291585, 0.75697619415367934), Vector2(2.8913715538978257, -0.06649366763518387), Vector2(3.0332394101332203, 0.778029958239568) },
            { Vector2(0.03374680592185976, 2.9540670019564095), Vector2(0.87367195024205546, 2.7643832530581478), Vector2(1.0577585715726392, 3.9968020663895363) },
            { Vector2(0.03374680592185976, 2.9540670019564095), Vector2(1.0577585715726392, 3.9968020663895363), Vector2(-0.11466384511383747, 3.9710386399074529) },
            { Vector2(0.87367195024205546, 2.7643832530581478), Vector2(2.2757074946435059, 3.1539437539456276), Vector2(2.1826649326485286, 3.9969296636572853) },
            { Vector2(0.87367195024205546, 2.7643832530581478), Vector2(2.1826649326485286, 3.9969296636572853), Vector2(1.0577585715726392, 3.9968020663895363) },
            { Vector2(2.2757074946435059, 3.1539437539456276), Vector2(2.7993390851111739, 3.1800242645073724), Vector2(2.8065790057107698, 4.0652164380301148) },
            { Vector2(2.2757074946435059, 3.1539437539456276), Vector2(2.8065790057107698, 4.0652164380301148), Vector2(2.1826649326485286, 3.9969296636572853) },
            { Vector2(2.7993390851111739, 3.1800242645073724), Vector2(2.9186610239387076, 4.1316467986243728), Vector2(3.9491340910940709, 4.0699047066492779) },
            { Vector2(2.7993390851111739, 3.1800242645073724), Vector2(3.9491340910940709, 4.0699047066492779), Vector2(2.8065790057107698, 4.0652164380301148) },
            { Vector2(2.9186610239387076, 4.1316467986243728), Vector2(2.8971241393089042, 3.2791119547671546), Vector2(4.0979403239955303, 2.9210087262477931) },
            { Vector2(2.9186610239387076, 4.1316467986243728), Vector2(4.0979403239955303, 2.9210087262477931), Vector2(3.9491340910940709, 4.0699047066492779) },
            { Vector2(2.8971241393089042, 3.2791119547671546), Vector2(3.0584943346761717, 2.0404794185176858), Vector2(4.0879652912388114, 2.2345336045893736) },
            { Vector2(2.8971241393089042, 3.2791119547671546), Vector2(4.0879652912388114, 2.2345336045893736), Vector2(4.0979403239955303, 2.9210087262477931) },
            { Vector2(3.0584943346761717, 2.0404794185176858), Vector2(3.0332394101332203, 0.778029958239568), Vector2(3.8398677084982205, 0.80449296770808232) },
            { Vector2(3.0584943346761717, 2.0404794185176858), Vector2(3.8398677084982205, 0.80449296770808232), Vector2(4.0879652912388114, 2.2345336045893736) },
            { Vector2(3.0332394101332203, 0.778029958239568), Vector2(2.8913715538978257, -0.06649366763518387), Vector2(4.030340442888245, -0.19925845012373095) },
            { Vector2(3.0332394101332203, 0.778029958239568), Vector2(4.030340442888245, -0.19925845012373095), Vector2(3.8398677084982205, 0.80449296770808232) },
            { Vector2(-0.11466384511383747, 3.9710386399074529), Vector2(1.0577585715726392, 3.9968020663895363), Vector2(1.1891732579804852, 4.7830845747964412) },
            { Vector2(-0.11466384511383747, 3.9710386399074529), Vector2(1.1891732579804852, 4.7830845747964412), Vector2(-0.088465880527543478, 5.2100217779305842) },
            { Vector2(1.0577585715726392, 3.9968020663895363), Vector2(2.1826649326485286, 3.9969296636572853), Vector2(1.9734687030446296, 4.8577248633132069) },
            { Vector2(1.0577585715726392, 3.9968020663895363), Vector2(1.9734687030446296, 4.8577248633132069), Vector2(1.1891732579804852, 4.7830845747964412) },
            { Vector2(2.1826649326485286, 3.9969296636572853), Vector2(2.8065790057107698, 4.0652164380301148), Vector2(2.867788134534111, 4.7609035137943509) },
            { Vector2(2.1826649326485286, 3.9969296636572853), Vector2(2.867788134534111, 4.7609035137943509), Vector2(1.9734687030446296, 4.8577248633132069) },
            { Vector2(2.8065790057107698, 4.0652164380301148), Vector2(3.9491340910940709, 4.0699047066492779), Vector2(4.9417779839516092, 4.0186396340080641) },
            { Vector2(2.8065790057107698, 4.0652164380301148), Vector2(4.9417779839516092, 4.0186396340080641), Vector2(2.867788134534111, 4.7609035137943509) },
            { Vector2(3.9491340910940709, 4.0699047066492779), Vector2(4.0979403239955303, 2.9210087262477931), Vector2(4.8559396540070541, 2.9266314805588292) },
            { Vector2(3.9491340910940709, 4.0699047066492779), Vector2(4.8559396540070541, 2.9266314805588292), Vector2(4.9417779839516092, 4.0186396340080641) },
            { Vector2(4.0979403239955303, 2.9210087262477931), Vector2(4.0879652912388114, 2.2345336045893736), Vector2(4.8814444058545021, 2.0978269472072357) },
            { Vector2(4.0979403239955303, 2.9210087262477931), Vector2(4.8814444058545021, 2.0978269472072357), Vector2(4.8559396540070541, 2.9266314805588292) },
            { Vector2(4.0879652912388114, 2.2345336045893736), Vector2(3.8398677084982205, 0.80449296770808232), Vector2(5.0805772372733466, 1.1719850159782403) },
            { Vector2(4.0879652912388114, 2.2345336045893736), Vector2(5.0805772372733466, 1.1719850159782403), Vector2(4.8814444058545021, 2.0978269472072357) },
            { Vector2(3.8398677084982205, 0.80449296770808232), Vector2(4.030340442888245, -0.19925845012373095), Vector2(5.2217473797477059, -0.22546207285273762) },
            { Vector2(3.8398677084982205, 0.80449296770808232), Vector2(5.2217473797477059, -0.22546207285273762), Vector2(5.0805772372733466, 1.1719850159782403) },
            { Vector2(-0.088465880527543478, 5.2100217779305842), Vector2(1.1891732579804852, 4.7830845747964412), Vector2(0.89108385696962289, 6.1458281415981899) },
            { Vector2(-0.088465880527543478, 5.2100217779305842), Vector2(0.89108385696962289, 6.1458281415981899), Vector2(0.10199560652249923, 6.1797459436217146) },
            { Vector2(1.1891732579804852, 4.7830845747964412), Vector2(1.9734687030446296, 4.8577248633132069), Vector2(2.1252758166525028, 5.8385010800614037) },
            { Vector2(1.1891732579804852, 4.7830845747964412), Vector2(2.1252758166525028, 5.8385010800614037), Vector2(0.89108385696962289, 6.1458281415981899) },
            { Vector2(1.9734687030446296, 4.8577248633132069), Vector2(2.867788134534111, 4.7609035137943509), Vector2(2.9081521584910868, 6.0423771869888254) },
            { Vector2(1.9734687030446296, 4.8577248633132069), Vector2(2.9081521584910868, 6.0423771869888254), Vector2(2.1252758166525028, 5.8385010800614037) },
            { Vector2(2.867788134534111, 4.7609035137943509), Vector2(4.9417779839516092, 4.0186396340080641), Vector2(5.9216497991147135, 4.2116003969823002) },
            { Vector2(2.867788134534111, 4.7609035137943509), Vector2(5.9216497991147135, 4.2116003969823002), Vector2(2.9081521584910868, 6.0423771869888254) },
            { Vector2(4.9417779839516092, 4.0186396340080641), Vector2(4.8559396540070541, 2.9266314805588292), Vector2(6.1574207000918015, 3.0797918493639007) },
            { Vector2(4.9417779839516092, 4.0186396340080641), Vector2(6.1574207000918015, 3.0797918493639007), Vector2(5.9216497991147135, 4.2116003969823002) },
            { Vector2(4.8559396540070541, 2.9266314805588292), Vector2(4.8814444058545021, 2.0978269472072357), Vector2(6.1640235004874908, 1.9597288792656269) },
            { Vector2(4.8559396540070541, 2.9266314805588292), Vector2(6.1640235004874908, 1.9597288792656269), Vector2(6.1574207000918015, 3.0797918493639007) },
            { Vector2(4.8814444058545021, 2.0978269472072357), Vector2(5.0805772372733466, 1.1719850159782403), Vector2(5.9517528197859439, 1.0229916189077719) },
            { Vector2(4.8814444058545021, 2.0978269472072357), Vector2(5.9517528197859439, 1.0229916189077719), Vector2(6.1640235004874908, 1.9597288792656269) },
            { Vector2(5.0805772372733466, 1.1719850159782403), Vector2(5.2217473797477059, -0.22546207285273762), Vector2(6.1479006818043889, 0.12346739927643305) },
            { Vector2(5.0805772372733466, 1.1719850159782403), Vector2(6.1479006818043889, 0.12346739927643305), Vector2(5.9517528197859439, 1.0229916189077719) },
        };
        in.singular = { 10, 25, 37 };
        in.computeUvs = false;
        in.origMode = 1;
        in.origUvs = {
            { Vector2(-0.17625250278227644, 0.11677462359339447), Vector2(0.96821654223116971, 0.2387867413148263), Vector2(0.98918220779736232, 0.95295819055164199) },
            { Vector2(-0.17625250278227644, 0.11677462359339447), Vector2(0.98918220779736232, 0.95295819055164199), Vector2(0.073893862978196528, 0.96975279719903784) },
            { Vector2(0.96821654223116971, 0.2387867413148263), Vector2(2.1187440160620401, -0.0013983679022050886), Vector2(2.0727382253485067, 0.94210650467000934) },
            { Vector2(0.96821654223116971, 0.2387867413148263), Vector2(2.0727382253485067, 0.94210650467000934), Vector2(0.98918220779736232, 0.95295819055164199) },
            { Vector2(2.1187440160620401, -0.0013983679022050886), Vector2(3.1717121120416567, -0.01051240653782419), Vector2(2.9328221311069838, 0.97264865704163894) },
            { Vector2(2.1187440160620401, -0.0013983679022050886), Vector2(2.9328221311069838, 0.97264865704163894), Vector2(2.0727382253485067, 0.94210650467000934) },
            { Vector2(3.1717121120416567, -0.01051240653782419), Vector2(-0.050256340959847712, 3.9828484382472071), Vector2(1.1168472826651907, 3.8883157250014104) },
            { Vector2(3.1717121120416567, -0.01051240653782419), Vector2(1.1168472826651907, 3.8883157250014104), Vector2(2.9328221311069838, 0.97264865704163894) },
            { Vector2(-0.050256340959847712, 3.9828484382472071), Vector2(-0.24442819533519256, 3.0640890940721084), Vector2(0.98537462727456326, 2.9743996209928452) },
            { Vector2(-0.050256340959847712, 3.9828484382472071), Vector2(0.98537462727456326, 2.9743996209928452), Vector2(1.1168472826651907, 3.8883157250014104) },
            { Vector2(-0.24442819533519256, 3.0640890940721084), Vector2(0.0074028780821612239, 1.9910648434468108), Vector2(1.0624105420340213, 1.8775214672517013) },
            { Vector2(-0.24442819533519256, 3.0640890940721084), Vector2(1.0624105420340213, 1.8775214672517013), Vector2(0.98537462727456326, 2.9743996209928452) },
            { Vector2(0.0074028780821612239, 1.9910648434468108), Vector2(0.13715893637162002, 1.1547744197732615), Vector2(1.1075783961783621, 1.2161774283760902) },
            { Vector2(0.0074028780821612239, 1.9910648434468108), Vector2(1.1075783961783621, 1.2161774283760902), Vector2(1.0624105420340213, 1.8775214672517013) },
            { Vector2(0.13715893637162002, 1.1547744197732615), Vector2(0.032259381404624435, 0.14592223422539), Vector2(0.78946946999968071, 0.057803117953874758) },
            { Vector2(0.13715893637162002, 1.1547744197732615), Vector2(0.78946946999968071, 0.057803117953874758), Vector2(1.1075783961783621, 1.2161774283760902) },
            { Vector2(0.073893862978196528, 0.96975279719903784), Vector2(0.98918220779736232, 0.95295819055164199), Vector2(1.110139361049604, 2.0258498491245449) },
            { Vector2(0.073893862978196528, 0.96975279719903784), Vector2(1.110139361049604, 2.0258498491245449), Vector2(-0.17576873296494289, 2.0612989443860252) },
            { Vector2(0.98918220779736232, 0.95295819055164199), Vector2(2.0727382253485067, 0.94210650467000934), Vector2(1.8307400439396795, 2.1333417066165064) },
            { Vector2(0.98918220779736232, 0.95295819055164199), Vector2(1.8307400439396795, 2.1333417066165064), Vector2(1.110139361049604, 2.0258498491245449) },
            { Vector2(2.0727382253485067, 0.94210650467000934), Vector2(2.9328221311069838, 0.97264865704163894), Vector2(3.2125743923363883, 1.9781614706049147) },
            { Vector2(2.0727382253485067, 0.94210650467000934), Vector2(3.2125743923363883, 1.9781614706049147), Vector2(1.8307400439396795, 2.1333417066165064) },
            { Vector2(2.9328221311069838, 0.97264865704163894), Vector2(1.1168472826651907, 3.8883157250014104), Vector2(2.0575000098238574, 4.170222964827806) },
            { Vector2(2.9328221311069838, 0.97264865704163894), Vector2(2.0575000098238574, 4.170222964827806), Vector2(3.2125743923363883, 1.9781614706049147) },
            { Vector2(1.1168472826651907, 3.8883157250014104), Vector2(0.98537462727456326, 2.9743996209928452), Vector2(1.9905794664086685, 3.2025917413824776) },
            { Vector2(1.1168472826651907, 3.8883157250014104), Vector2(1.9905794664086685, 3.2025917413824776), Vector2(2.0575000098238574, 4.170222964827806) },
            { Vector2(0.98537462727456326, 2.9743996209928452), Vector2(1.0624105420340213, 1.8775214672517013), Vector2(2.0581469870173352, 2.0430725184806842) },
            { Vector2(0.98537462727456326, 2.9743996209928452), Vector2(2.0581469870173352, 2.0430725184806842), Vector2(1.9905794664086685, 3.2025917413824776) },
            { Vector2(1.0624105420340213, 1.8775214672517013), Vector2(1.1075783961783621, 1.2161774283760902), Vector2(2.2386818517291585, 0.75697619415367934) },
            { Vector2(1.0624105420340213, 1.8775214672517013), Vector2(2.2386818517291585, 0.75697619415367934), Vector2(2.0581469870173352, 2.0430725184806842) },
            { Vector2(1.1075783961783621, 1.2161774283760902), Vector2(0.78946946999968071, 0.057803117953874758), Vector2(1.7914566024007799, 0.019751922790278376) },
            { Vector2(1.1075783961783621, 1.2161774283760902), Vector2(1.7914566024007799, 0.019751922790278376), Vector2(2.2386818517291585, 0.75697619415367934) },
            { Vector2(-0.17576873296494289, 2.0612989443860252), Vector2(1.110139361049604, 2.0258498491245449), Vector2(0.87367195024205546, 2.7643832530581478) },
            { Vector2(-0.17576873296494289, 2.0612989443860252), Vector2(0.87367195024205546, 2.7643832530581478), Vector2(0.03374680592185976, 2.9540670019564095) },
            { Vector2(1.110139361049604, 2.0258498491245449), Vector2(1.8307400439396795, 2.1333417066165064), Vector2(2.2757074946435059, 3.1539437539456276) },
            { Vector2(1.110139361049604, 2.0258498491245449), Vector2(2.2757074946435059, 3.1539437539456276), Vector2(0.87367195024205546, 2.7643832530581478) },
            { Vector2(1.8307400439396795, 2.1333417066165064), Vector2(3.2125743923363883, 1.9781614706049147), Vector2(2.7993390851111739, 3.1800242645073724) },
            { Vector2(1.8307400439396795, 2.1333417066165064), Vector2(2.7993390851111739, 3.1800242645073724), Vector2(2.2757074946435059, 3.1539437539456276) },
            { Vector2(3.2125743923363883, 1.9781614706049147), Vector2(2.0575000098238574, 4.170222964827806), Vector2(2.9186610239387076, 4.1316467986243728) },
            { Vector2(3.2125743923363883, 1.9781614706049147), Vector2(2.9186610239387076, 4.1316467986243728), Vector2(2.7993390851111739, 3.1800242645073724) },
            { Vector2(2.0575000098238574, 4.170222964827806), Vector2(1.9905794664086685, 3.2025917413824776), Vector2(2.8971241393089042, 3.2791119547671546) },
            { Vector2(2.0575000098238574, 4.170222964827806), Vector2(2.8971241393089042, 3.2791119547671546), Vector2(2.9186610239387076, 4.1316467986243728) },
            { Vector2(1.9905794664086685, 3.2025917413824776), Vector2(2.0581469870173352, 2.0430725184806842), Vector2(3.0584943346761717, 2.0404794185176858) },
            { Vector2(1.9905794664086685, 3.2025917413824776), Vector2(3.0584943346761717, 2.0404794185176858), Vector2(2.8971241393089042, 3.2791119547671546) },
            { Vector2(2.0581469870173352, 2.0430725184806842), Vector2(2.2386818517291585, 0.75697619415367934), Vector2(3.0332394101332203, 0.778029958239568) },
            { Vector2(2.0581469870173352, 2.0430725184806842), Vector2(3.0332394101332203, 0.778029958239568), Vector2(3.0584943346761717, 2.0404794185176858) },
            { Vector2(2.2386818517291585, 0.75697619415367934), Vector2(1.7914566024007799, 0.019751922790278376), Vector2(2.8913715538978257, -0.06649366763518387) },
            { Vector2(2.2386818517291585, 0.75697619415367934), Vector2(2.8913715538978257, -0.06649366763518387), Vector2(3.0332394101332203, 0.778029958239568) },
            { Vector2(0.03374680592185976, 2.9540670019564095), Vector2(0.87367195024205546, 2.7643832530581478), Vector2(1.0577585715726392, 3.9968020663895363) },
            { Vector2(0.03374680592185976, 2.9540670019564095), Vector2(1.0577585715726392, 3.9968020663895363), Vector2(-0.11466384511383747, 3.9710386399074529) },
            { Vector2(0.87367195024205546, 2.7643832530581478), Vector2(2.2757074946435059, 3.1539437539456276), Vector2(2.1826649326485286, 3.9969296636572853) },
            { Vector2(0.87367195024205546, 2.7643832530581478), Vector2(2.1826649326485286, 3.9969296636572853), Vector2(1.0577585715726392, 3.9968020663895363) },
            { Vector2(2.2757074946435059, 3.1539437539456276), Vector2(2.7993390851111739, 3.1800242645073724), Vector2(2.8065790057107698, 4.0652164380301148) },
            { Vector2(2.2757074946435059, 3.1539437539456276), Vector2(2.8065790057107698, 4.0652164380301148), Vector2(2.1826649326485286, 3.9969296636572853) },
            { Vector2(2.7993390851111739, 3.1800242645073724), Vector2(2.9186610239387076, 4.1316467986243728), Vector2(3.9491340910940709, 4.0699047066492779) },
            { Vector2(2.7993390851111739, 3.1800242645073724), Vector2(3.9491340910940709, 4.0699047066492779), Vector2(2.8065790057107698, 4.0652164380301148) },
            { Vector2(2.9186610239387076, 4.1316467986243728), Vector2(2.8971241393089042, 3.2791119547671546), Vector2(4.0979403239955303, 2.9210087262477931) },
            { Vector2(2.9186610239387076, 4.1316467986243728), Vector2(4.0979403239955303, 2.9210087262477931), Vector2(3.9491340910940709, 4.0699047066492779) },
            { Vector2(2.8971241393089042, 3.2791119547671546), Vector2(3.0584943346761717, 2.0404794185176858), Vector2(4.0879652912388114, 2.2345336045893736) },
            { Vector2(2.8971241393089042, 3.2791119547671546), Vector2(4.0879652912388114, 2.2345336045893736), Vector2(4.0979403239955303, 2.9210087262477931) },
            { Vector2(3.0584943346761717, 2.0404794185176858), Vector2(3.0332394101332203, 0.778029958239568), Vector2(3.8398677084982205, 0.80449296770808232) },
            { Vector2(3.0584943346761717, 2.0404794185176858), Vector2(3.8398677084982205, 0.80449296770808232), Vector2(4.0879652912388114, 2.2345336045893736) },
            { Vector2(3.0332394101332203, 0.778029958239568), Vector2(2.8913715538978257, -0.06649366763518387), Vector2(4.030340442888245, -0.19925845012373095) },
            { Vector2(3.0332394101332203, 0.778029958239568), Vector2(4.030340442888245, -0.19925845012373095), Vector2(3.8398677084982205, 0.80449296770808232) },
            { Vector2(-0.11466384511383747, 3.9710386399074529), Vector2(1.0577585715726392, 3.9968020663895363), Vector2(1.1891732579804852, 4.7830845747964412) },
            { Vector2(-0.11466384511383747, 3.9710386399074529), Vector2(1.1891732579804852, 4.7830845747964412), Vector2(-0.088465880527543478, 5.2100217779305842) },
            { Vector2(1.0577585715726392, 3.9968020663895363), Vector2(2.1826649326485286, 3.9969296636572853), Vector2(1.9734687030446296, 4.8577248633132069) },
            { Vector2(1.0577585715726392, 3.9968020663895363), Vector2(1.9734687030446296, 4.8577248633132069), Vector2(1.1891732579804852, 4.7830845747964412) },
            { Vector2(2.1826649326485286, 3.9969296636572853), Vector2(2.8065790057107698, 4.0652164380301148), Vector2(2.867788134534111, 4.7609035137943509) },
            { Vector2(2.1826649326485286, 3.9969296636572853), Vector2(2.867788134534111, 4.7609035137943509), Vector2(1.9734687030446296, 4.8577248633132069) },
            { Vector2(2.8065790057107698, 4.0652164380301148), Vector2(3.9491340910940709, 4.0699047066492779), Vector2(4.9417779839516092, 4.0186396340080641) },
            { Vector2(2.8065790057107698, 4.0652164380301148), Vector2(4.9417779839516092, 4.0186396340080641), Vector2(2.867788134534111, 4.7609035137943509) },
            { Vector2(3.9491340910940709, 4.0699047066492779), Vector2(4.0979403239955303, 2.9210087262477931), Vector2(4.8559396540070541, 2.9266314805588292) },
            { Vector2(3.9491340910940709, 4.0699047066492779), Vector2(4.8559396540070541, 2.9266314805588292), Vector2(4.9417779839516092, 4.0186396340080641) },
            { Vector2(4.0979403239955303, 2.9210087262477931), Vector2(4.0879652912388114, 2.2345336045893736), Vector2(4.8814444058545021, 2.0978269472072357) },
            { Vector2(4.0979403239955303, 2.9210087262477931), Vector2(4.8814444058545021, 2.0978269472072357), Vector2(4.8559396540070541, 2.9266314805588292) },
            { Vector2(4.0879652912388114, 2.2345336045893736), Vector2(3.8398677084982205, 0.80449296770808232), Vector2(5.0805772372733466, 1.1719850159782403) },
            { Vector2(4.0879652912388114, 2.2345336045893736), Vector2(5.0805772372733466, 1.1719850159782403), Vector2(4.8814444058545021, 2.0978269472072357) },
            { Vector2(3.8398677084982205, 0.80449296770808232), Vector2(4.030340442888245, -0.19925845012373095), Vector2(5.2217473797477059, -0.22546207285273762) },
            { Vector2(3.8398677084982205, 0.80449296770808232), Vector2(5.2217473797477059, -0.22546207285273762), Vector2(5.0805772372733466, 1.1719850159782403) },
            { Vector2(-0.088465880527543478, 5.2100217779305842), Vector2(1.1891732579804852, 4.7830845747964412), Vector2(0.89108385696962289, 6.1458281415981899) },
            { Vector2(-0.088465880527543478, 5.2100217779305842), Vector2(0.89108385696962289, 6.1458281415981899), Vector2(0.10199560652249923, 6.1797459436217146) },
            { Vector2(1.1891732579804852, 4.7830845747964412), Vector2(1.9734687030446296, 4.8577248633132069), Vector2(2.1252758166525028, 5.8385010800614037) },
            { Vector2(1.1891732579804852, 4.7830845747964412), Vector2(2.1252758166525028, 5.8385010800614037), Vector2(0.89108385696962289, 6.1458281415981899) },
            { Vector2(1.9734687030446296, 4.8577248633132069), Vector2(2.867788134534111, 4.7609035137943509), Vector2(2.9081521584910868, 6.0423771869888254) },
            { Vector2(1.9734687030446296, 4.8577248633132069), Vector2(2.9081521584910868, 6.0423771869888254), Vector2(2.1252758166525028, 5.8385010800614037) },
            { Vector2(2.867788134534111, 4.7609035137943509), Vector2(4.9417779839516092, 4.0186396340080641), Vector2(5.9216497991147135, 4.2116003969823002) },
            { Vector2(2.867788134534111, 4.7609035137943509), Vector2(5.9216497991147135, 4.2116003969823002), Vector2(2.9081521584910868, 6.0423771869888254) },
            { Vector2(4.9417779839516092, 4.0186396340080641), Vector2(4.8559396540070541, 2.9266314805588292), Vector2(6.1574207000918015, 3.0797918493639007) },
            { Vector2(4.9417779839516092, 4.0186396340080641), Vector2(6.1574207000918015, 3.0797918493639007), Vector2(5.9216497991147135, 4.2116003969823002) },
            { Vector2(4.8559396540070541, 2.9266314805588292), Vector2(4.8814444058545021, 2.0978269472072357), Vector2(6.1640235004874908, 1.9597288792656269) },
            { Vector2(4.8559396540070541, 2.9266314805588292), Vector2(6.1640235004874908, 1.9597288792656269), Vector2(6.1574207000918015, 3.0797918493639007) },
            { Vector2(4.8814444058545021, 2.0978269472072357), Vector2(5.0805772372733466, 1.1719850159782403), Vector2(5.9517528197859439, 1.0229916189077719) },
            { Vector2(4.8814444058545021, 2.0978269472072357), Vector2(5.9517528197859439, 1.0229916189077719), Vector2(6.1640235004874908, 1.9597288792656269) },
            { Vector2(5.0805772372733466, 1.1719850159782403), Vector2(5.2217473797477059, -0.22546207285273762), Vector2(6.1479006818043889, 0.12346739927643305) },
            { Vector2(5.0805772372733466, 1.1719850159782403), Vector2(6.1479006818043889, 0.12346739927643305), Vector2(5.9517528197859439, 1.0229916189077719) },
        };
        dumpCase(index++, in);
    }
}

static void jitterGridPositions(CaseInput& in, double xy, double z)
{
    for (auto& v : in.vertices) {
        v = Vector3(v.x() + range(-xy, xy), v.y() + range(-xy, xy), range(-z, z));
    }
}

// Possibly snaps a UV coordinate near an integer (isoline-ratio cliff,
// FMA-sensitive interpolation).
static double maybeSnapNearInteger(double u)
{
    if (below(100) < 4) {
        double near = std::floor(u + 0.5);
        u = near + (below(2) ? 1e-13 : -1e-13);
    }
    return u;
}

static void paintScaledGridUvs(CaseInput& in, double su, double sv, double jitter)
{
    // Per-VERTEX UVs: isolines must continue across shared edges, so the
    // jitter is drawn once per vertex, not once per corner. (Per-corner
    // jitter tears every edge into a UV seam and nothing ever closes.)
    std::vector<Vector2> vertUv(in.vertices.size());
    for (size_t i = 0; i < in.vertices.size(); ++i) {
        const auto& v = in.vertices[i];
        vertUv[i] = Vector2(
            maybeSnapNearInteger(v.x() * su + range(-jitter, jitter)),
            v.y() * sv + range(-jitter, jitter));
    }
    for (size_t i = 0; i < in.triangles.size(); ++i) {
        const auto& t = in.triangles[i];
        for (size_t k = 0; k < 3; ++k)
            in.uvs[i][k] = vertUv[t[k]];
    }
}

static void finishRandomFlags(CaseInput& in)
{
    size_t nsing = below(6);
    for (size_t i = 0; i < nsing; ++i)
        in.singular.push_back((size_t)(nextU64() % in.vertices.size()));
    in.computeUvs = below(2) == 0;
    in.origMode = (int)below(3);
    finishOrig(in);
}

static void dumpRandomCase(size_t index)
{
    CaseInput in;
    std::uint64_t kind = below(100);
    if (kind < 50) {
        // Big jittered grid (4..10 cells/side, UV spans 1..3): dense
        // closed isoline loops, the downstream-plausible island shape.
        size_t w = 4 + below(7);
        size_t h = 4 + below(7);
        makeGrid(w, h, in);
        jitterGridPositions(in, 0.3, 0.15);
        double su = 1.0 + below(3) * 0.5 + range(0.0, 0.5);
        double sv = 1.0 + below(3) * 0.5 + range(0.0, 0.5);
        paintScaledGridUvs(in, su, sv, 0.15);
    } else if (kind < 58) {
        // Triangle soup: random points, random triples, per-corner random
        // UVs. Topologically chaotic on purpose: irregular isoline graphs
        // are what feed the defect-cleanup passes (pentagons, hexagons,
        // odd valences) that regular grids never produce.
        size_t npoints = 8 + below(23);
        for (size_t i = 0; i < npoints; ++i)
            in.vertices.emplace_back(range(0, 5), range(0, 5), range(-0.2, 0.2));
        size_t ntris = 8 + below(15);
        for (size_t i = 0; i < ntris; ++i) {
            size_t a = (size_t)(nextU64() % npoints);
            size_t b = (size_t)(nextU64() % npoints);
            size_t c = (size_t)(nextU64() % npoints);
            if (a == b || b == c || a == c)
                continue;
            in.triangles.push_back({ a, b, c });
        }
        std::vector<Vector2> vertUv(npoints);
        for (size_t i = 0; i < npoints; ++i) {
            vertUv[i] = Vector2(
                maybeSnapNearInteger(range(0, 3)), range(0, 3));
        }
        in.uvs.resize(in.triangles.size());
        for (size_t i = 0; i < in.triangles.size(); ++i) {
            const auto& t = in.triangles[i];
            for (size_t k = 0; k < 3; ++k) {
                if (below(100) < 10) {
                    // Torn corner: a UV seam inside the soup.
                    in.uvs[i].emplace_back(range(0, 3), range(0, 3));
                } else {
                    in.uvs[i].push_back(vertUv[t[k]]);
                }
            }
        }
    } else if (kind < 70) {
        // Pole: ring fan with ANGULAR u (M integers around the ring) and
        // radial v. The M spokes meet at the center in a valence-M pole,
        // the shape extractMesh turns into pentagons/hexagons/heptagons.
        size_t ringChoices[] = { 3, 5, 6, 7, 8 };
        size_t ring = ringChoices[below(5)];
        double span = (double)ring;
        in.vertices.emplace_back(range(-0.1, 0.1), range(-0.1, 0.1), range(0, 0.3));
        for (size_t i = 0; i < ring; ++i) {
            double a = 2.0 * M_PI * (double)i / (double)ring;
            double r = 2.0 + range(-0.2, 0.2);
            in.vertices.emplace_back(r * std::cos(a), r * std::sin(a), range(-0.1, 0.1));
        }
        for (size_t i = 0; i < ring; ++i)
            in.triangles.push_back({ 0, 1 + i, 1 + (i + 1) % ring });
        in.uvs.resize(in.triangles.size());
        for (size_t i = 0; i < in.triangles.size(); ++i) {
            double u0 = span * (double)i / (double)ring;
            double u1 = span * (double)(i + 1) / (double)ring;
            in.uvs[i].push_back(Vector2(0.5 * (u0 + u1), 0.0));
            in.uvs[i].push_back(Vector2(u0, 2.0));
            in.uvs[i].push_back(Vector2(u1, 2.0));
        }
    } else if (kind < 78) {
        // Hole grid: 6x6 grid with the center 2x2 cells removed. Isoline
        // ends dangle at the hole boundary (valence-1 stubs), which is
        // what starves a cone; singulars sit on the hole rim.
        makeGrid(6, 6, in);
        {
            std::vector<std::vector<size_t>> kept;
            std::vector<std::vector<Vector2>> keptUv;
            auto holeCell = [](size_t x, size_t y) {
                return x >= 2 && x < 4 && y >= 2 && y < 4;
            };
            for (size_t c = 0; c < in.triangles.size(); ++c) {
                size_t cell = c / 2;
                size_t x = cell % 6;
                size_t y = cell / 6;
                if (holeCell(x, y))
                    continue;
                kept.push_back(in.triangles[c]);
                keptUv.push_back(in.uvs[c]);
            }
            in.triangles = std::move(kept);
            in.uvs = std::move(keptUv);
        }
        jitterGridPositions(in, 0.2, 0.1);
        paintScaledGridUvs(in, 1.0 + below(2) * 0.5, 1.0 + below(2) * 0.5, 0.1);
        {
            // Singulars on the hole rim: vertices of removed cells that
            // still border kept ones.
            auto id = [](size_t x, size_t y) { return y * 7 + x; };
            size_t rim[] = { id(2, 2), id(3, 2), id(4, 2), id(4, 3), id(4, 4),
                id(3, 4), id(2, 4), id(2, 3) };
            size_t nrim = 1 + below(4);
            for (size_t i = 0; i < nrim; ++i)
                in.singular.push_back(rim[below(8)]);
        }
    } else if (kind < 88) {
        // Seam grid: one half UV=(x,y), the other rotated 90 degrees. The
        // seam column(s) are UV discontinuities full of T-junctions, the
        // richest source of pentagon/hexagon faces. Variants: vertical or
        // horizontal seam, one or two columns wide, sometimes a cross.
        size_t w = 6 + below(3);
        size_t h = 6 + below(3);
        makeGrid(w, h, in);
        jitterGridPositions(in, 0.2, 0.1);
        {
            bool horizontal = below(2) == 0;
            bool wide = below(100) < 40;
            bool cross = below(100) < 20;
            size_t mid = (horizontal ? h : w) / 2;
            size_t edge = mid + (wide ? 1 : 0);
            std::vector<Vector2> vertUv(in.vertices.size());
            for (size_t i = 0; i < in.vertices.size(); ++i) {
                const auto& v = in.vertices[i];
                size_t x = i % (w + 1);
                size_t y = i / (w + 1);
                size_t d = horizontal ? y : x;
                bool rotated = cross ? (x >= mid && y >= mid) : (d >= edge);
                if (!rotated) {
                    vertUv[i] = Vector2(v.x() + range(-0.1, 0.1),
                        v.y() + range(-0.1, 0.1));
                } else {
                    vertUv[i] = Vector2(v.y() + range(-0.1, 0.1),
                        ((double)(horizontal ? h : w) - (horizontal ? v.y() : v.x())) + range(-0.1, 0.1));
                }
            }
            for (size_t i = 0; i < in.triangles.size(); ++i) {
                const auto& t = in.triangles[i];
                for (size_t k = 0; k < 3; ++k)
                    in.uvs[i][k] = vertUv[t[k]];
            }
        }
    } else if (kind < 92) {
        // Star: high-valence center fan with radial UVs.
        size_t ring = 5 + below(5);
        in.vertices.emplace_back(range(-0.2, 0.2), range(-0.2, 0.2), range(0, 0.4));
        for (size_t i = 0; i < ring; ++i) {
            double a = 2.0 * M_PI * (double)i / (double)ring;
            double r = 2.0 + range(-0.3, 0.3);
            in.vertices.emplace_back(r * std::cos(a) + range(-0.2, 0.2),
                r * std::sin(a) + range(-0.2, 0.2), range(-0.2, 0.2));
        }
        for (size_t i = 0; i < ring; ++i) {
            in.triangles.push_back({ 0, 1 + i, 1 + (i + 1) % ring });
        }
        std::vector<Vector2> vertUv(in.vertices.size());
        for (size_t i = 0; i < in.vertices.size(); ++i) {
            const auto& v = in.vertices[i];
            vertUv[i] = Vector2(
                maybeSnapNearInteger((v.x() + 2.5) * 0.8 + range(-0.15, 0.15)),
                (v.y() + 2.5) * 0.8 + range(-0.15, 0.15));
        }
        in.uvs.resize(in.triangles.size());
        for (size_t i = 0; i < in.triangles.size(); ++i) {
            const auto& t = in.triangles[i];
            for (size_t k = 0; k < 3; ++k)
                in.uvs[i].push_back(vertUv[t[k]]);
        }
    } else if (below(3) < 2) {
        // Small grid (1..3 cells): sparse/open graphs, empty-output and
        // early-exit coverage.
        size_t w = 1 + below(3);
        size_t h = 1 + below(3);
        makeGrid(w, h, in);
        jitterGridPositions(in, 0.3, 0.1);
        static const double kSpans[] = { 0.4, 0.75, 1.0, 1.5, 2.0 };
        paintScaledGridUvs(in, kSpans[below(5)], kSpans[below(5)], 0.15);
    } else if (below(2)) {
        // Torn grid: per-CORNER UV jitter tears every edge into a seam.
        // Dangling isoline stubs everywhere is what starves cones into
        // completed singular-line walks (moved == 2 connections).
        size_t w = 4 + below(3);
        size_t h = 4 + below(3);
        makeGrid(w, h, in);
        jitterGridPositions(in, 0.2, 0.1);
        for (size_t i = 0; i < in.triangles.size(); ++i) {
            const auto& t = in.triangles[i];
            for (size_t k = 0; k < 3; ++k) {
                const auto& v = in.vertices[t[k]];
                in.uvs[i][k] = Vector2(v.x() + range(-0.4, 0.4),
                    v.y() + range(-0.4, 0.4));
            }
        }
        size_t extra = 2 + below(4);
        for (size_t i = 0; i < extra; ++i)
            in.singular.push_back((size_t)(nextU64() % in.vertices.size()));
    } else {
        // Walk farm: torn 6x8 grid with a dozen singulars. Mass walks
        // splice irregular branches into the graph (odd valences), the
        // raw material of the valence-driven defect passes.
        makeGrid(6, 8, in);
        jitterGridPositions(in, 0.2, 0.1);
        for (size_t i = 0; i < in.triangles.size(); ++i) {
            const auto& t = in.triangles[i];
            for (size_t k = 0; k < 3; ++k) {
                const auto& v = in.vertices[t[k]];
                in.uvs[i][k] = Vector2(v.x() + range(-0.4, 0.4),
                    v.y() + range(-0.4, 0.4));
            }
        }
        for (size_t i = 0; i < 12; ++i)
            in.singular.push_back((size_t)(nextU64() % in.vertices.size()));
    }
    finishRandomFlags(in);
    dumpCase(index, in);
}

// Large timing input: pure formula (no RNG), mirrored by the Rust timing
// test. 32x32 grid, UVs at half scale, vertex UVs on.
static CaseInput makeTimingInput()
{
    CaseInput in;
    makeGrid(32, 32, in);
    for (auto& row : in.uvs) {
        for (auto& uv : row)
            uv = Vector2(uv.x() * 0.5, uv.y() * 0.5);
    }
    in.computeUvs = true;
    return in;
}

static void timeLarge()
{
    for (int sample = 0; sample < 3; ++sample) {
        CaseInput in = makeTimingInput();
        QuadExtractor extractor(&in.vertices, &in.triangles, &in.uvs);
        extractor.setComputeVertexUvs(true);
        auto t0 = std::chrono::steady_clock::now();
        bool ok = extractor.extract();
        auto t1 = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::printf("T quadext tris=%zu ok=%d verts=%zu quads=%zu ms=%.3f\n",
            in.triangles.size(), ok ? 1 : 0,
            extractor.remeshedVertices().size(),
            extractor.remeshedQuads().size(), ms);
    }
}

static constexpr int kRandomCases = 200;

int main()
{
    // QX_SEED overrides the RNG seed for coverage experiments; the
    // committed fixture always uses the default seed above.
    if (const char* seed = std::getenv("QX_SEED"))
        g_state = 0x9E3779B97F4A7C15ull ^ (std::uint64_t)std::strtoull(seed, nullptr, 0);
    std::printf("QUADEXT1\n");
    size_t index = 0;
    dumpEdgeCases(index);
    for (int i = 0; i < kRandomCases; ++i)
        dumpRandomCase(index++);
    timeLarge();
    return 0;
}
