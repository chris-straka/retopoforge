// Differential oracle dump for the meshseparator Rust port (wave 1).
// Generates fixed edge-case meshes plus seeded random face soups, splits
// them with the C++ MeshSeparator, and prints inputs + outputs in a token
// format that rust/core/tests/mesh_separator_diff.rs replays. Also times a
// large grid split for the runtime ratio (3 samples).
//
// Build-only helper: not registered with ctest. Run it and redirect stdout
// to tests/fixtures/meshsep_diff.txt, then commit the fixture.
import retopo.core.mesh_separator;

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <map>
#include <utility>
#include <vector>

using AutoRemesher::MeshSeparator;
using Faces = std::vector<std::vector<size_t>>;

// splitmix64: the edge-case meshes below are fixed, the random cases draw
// from this stream with a fixed seed, so the fixture is reproducible.
static std::uint64_t g_state = 0x5EED5EED5EED5EEDull;

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

static void printFace(const std::vector<size_t>& face)
{
    std::printf("f %zu", face.size());
    for (size_t v : face)
        std::printf(" %zu", v);
    std::printf("\n");
}

static void dumpCase(const Faces& faces)
{
    std::printf("CASE %zu\n", faces.size());
    for (const auto& face : faces)
        printFace(face);
    std::map<std::pair<size_t, size_t>, size_t> edgeToFace;
    MeshSeparator::buildEdgeToFaceMap(faces, edgeToFace);
    std::printf("MAP %zu\n", edgeToFace.size());
    for (const auto& [edge, index] : edgeToFace)
        std::printf("m %zu %zu %zu\n", edge.first, edge.second, index);
    std::vector<Faces> islands;
    MeshSeparator::splitToIslands(faces, islands);
    std::printf("ISLANDS %zu\n", islands.size());
    for (const auto& island : islands) {
        std::printf("ISL %zu\n", island.size());
        for (const auto& face : island)
            printFace(face);
    }
}

// Fixed edge cases: empty input, single component, all-isolated,
// same-orientation (geometrically adjacent but NOT connected: connectivity
// needs the reversed directed edge), nonmanifold stars, duplicates,
// degenerate/empty faces, and >u32 vertex ids (packed-table fallback path).
static void dumpEdgeCases()
{
    dumpCase({}); // empty input -> zero islands
    dumpCase({ { 0, 1, 2 } }); // single triangle
    // Single component: two tris forming a quad across (1,2)/(2,1).
    dumpCase({ { 0, 1, 2 }, { 2, 1, 3 } });
    // All-isolated: disjoint vertex sets, no shared edges at all.
    dumpCase({ { 0, 1, 2 }, { 3, 4, 5 }, { 6, 7, 8 }, { 9, 10, 11 } });
    // Same orientation: shared geometric edge (0,1) in the same direction
    // on both faces, so no reversed edge exists -> two islands.
    dumpCase({ { 0, 1, 2 }, { 0, 1, 3 } });
    // Nonmanifold: three tris around edge (0,1), two forward one reversed.
    dumpCase({ { 0, 1, 2 }, { 0, 1, 3 }, { 1, 0, 4 } });
    // Duplicate faces: identical directed edges, no reversed edge -> apart.
    dumpCase({ { 0, 1, 2 }, { 0, 1, 2 } });
    // Mixed tri/quad/pent single island.
    dumpCase({ { 0, 1, 2 }, { 2, 1, 3, 4 }, { 4, 3, 2, 5, 6 } });
    // Degenerate faces: repeated vertices and an all-equal triple.
    dumpCase({ { 0, 1, 1 }, { 5, 5, 5 }, { 0, 1, 2 } });
    // Empty face mixed with a normal triangle.
    dumpCase({ {}, { 0, 1, 2 } });
    static const size_t big = static_cast<size_t>(0x100000000ull); // 2^32
    // >u32 ids sharing a reversed edge: fallback path, one island.
    dumpCase({ { big + 5, big + 6, big + 7 }, { big + 7, big + 6, big + 8 } });
    // >u32 mixed with small ids: fallback path, two islands.
    dumpCase({ { 0, 1, 2 }, { big, big + 1, big + 2 } });
    // Fan: five tris around center vertex 0, one island.
    dumpCase({ { 0, 1, 2 }, { 0, 2, 3 }, { 0, 3, 4 }, { 0, 4, 5 }, { 0, 5, 1 } });
    // Strip of quads with consistent winding, one island.
    dumpCase({ { 0, 1, 4, 3 }, { 1, 2, 5, 4 }, { 3, 4, 7, 6 }, { 4, 5, 8, 7 } });
    // Nonmanifold star: two faces each direction on edge (0,1).
    dumpCase({ { 0, 1, 2 }, { 0, 1, 3 }, { 1, 0, 4 }, { 1, 0, 5 } });
    // Two islands of uneven size (3 faces + 1 face).
    dumpCase({ { 0, 1, 2 }, { 2, 1, 3 }, { 3, 1, 4 }, { 10, 11, 12 } });
}

static std::vector<size_t> randomFace()
{
    // 3% empty face, 5% degenerate (a repeated vertex), else tri/quad/pent.
    std::uint64_t kind = below(100);
    if (kind < 3)
        return {};
    size_t pool = 1 + below(8);
    size_t k = 3 + below(3);
    std::vector<size_t> face;
    for (size_t i = 0; i < k; ++i) {
        // 3% of vertices are huge: 2^32-1 (still packable), 2^32, 2^32+7,
        // 2^33 (fallback path).
        if (below(100) < 3) {
            static const std::uint64_t kBig[] = {
                0xFFFFFFFFull, 0x100000000ull, 0x100000007ull, 0x200000000ull
            };
            face.push_back(static_cast<size_t>(kBig[below(4)]));
        } else {
            face.push_back(below(pool));
        }
    }
    if (kind < 8 && face.size() > 1)
        face[below(face.size())] = face[0];
    return face;
}

static void dumpRandomCase()
{
    size_t nfaces = below(13); // 0..12 faces on a tiny pool: collisions,
    // duplicates, degenerate windings, and multi-island soups all occur.
    Faces faces;
    for (size_t i = 0; i < nfaces; ++i) {
        // 10%: duplicate an earlier face verbatim.
        if (!faces.empty() && below(10) == 0)
            faces.push_back(faces[below(faces.size())]);
        else
            faces.push_back(randomFace());
    }
    dumpCase(faces);
}

// Large timing input: W(x)H quad grid split into consistently-wound
// triangles, one island. Pure formula, no RNG: the Rust timing test builds
// the identical mesh.
static Faces makeGrid(size_t w, size_t h)
{
    Faces faces;
    faces.reserve(w * h * 2);
    auto id = [w](size_t x, size_t y) { return y * (w + 1) + x; };
    for (size_t y = 0; y < h; ++y) {
        for (size_t x = 0; x < w; ++x) {
            size_t a = id(x, y);
            size_t b = id(x + 1, y);
            size_t c = id(x + 1, y + 1);
            size_t d = id(x, y + 1);
            faces.push_back({ a, b, c });
            faces.push_back({ a, c, d });
        }
    }
    return faces;
}

static void timeGrid()
{
    Faces faces = makeGrid(200, 200); // 80,000 triangles, one island
    for (int sample = 0; sample < 3; ++sample) {
        std::vector<Faces> islands;
        auto t0 = std::chrono::steady_clock::now();
        MeshSeparator::splitToIslands(faces, islands);
        auto t1 = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::printf("T meshsep faces=%zu islands=%zu ms=%.3f\n", faces.size(), islands.size(), ms);
    }
}

static constexpr int kRandomCases = 220;

int main()
{
    std::printf("MESHSEP1\n");
    dumpEdgeCases();
    for (int i = 0; i < kRandomCases; ++i)
        dumpRandomCase();
    timeGrid();
    return 0;
}
