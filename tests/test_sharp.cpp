// Unit + engine tests for explicit sharp/feature constraints.
// Plain assert-style main, no third-party framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
import retopo.core.auto_remesher;
import retopo.core.frame_field;
import retopo.core.surface_mesh;
import retopo.core.vector3;

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <tuple>
#include <vector>

static int g_failures = 0;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                             \
            ++g_failures;                                                                          \
        }                                                                                          \
    } while (0)

namespace {

using Remesher = AutoRemesher::AutoRemesher;
using FrameField = AutoRemesher::FrameField;
using SurfaceMesh = AutoRemesher::SurfaceMesh;
using Vector3 = AutoRemesher::Vector3;

// Closed box centered at the origin, outward winding, no degenerate faces.
// Vertices: 0..7 = (±1,±1,±1); triangles match the CLI cube (0-based).
void buildBox(std::vector<Vector3>& vertices, std::vector<std::vector<size_t>>& triangles)
{
    vertices = {
        Vector3(-1.0, -1.0, -1.0),
        Vector3(1.0, -1.0, -1.0),
        Vector3(1.0, 1.0, -1.0),
        Vector3(-1.0, 1.0, -1.0),
        Vector3(-1.0, -1.0, 1.0),
        Vector3(1.0, -1.0, 1.0),
        Vector3(1.0, 1.0, 1.0),
        Vector3(-1.0, 1.0, 1.0),
    };
    triangles = {
        { 0, 3, 2 }, { 0, 2, 1 },
        { 4, 5, 6 }, { 4, 6, 7 },
        { 0, 1, 5 }, { 0, 5, 4 },
        { 3, 7, 6 }, { 3, 6, 2 },
        { 0, 4, 7 }, { 0, 7, 3 },
        { 1, 2, 6 }, { 1, 6, 5 },
    };
}

// The 4 vertical box edges as 2-point polylines in input-mesh coordinates.
std::vector<std::vector<Vector3>> verticalSharps()
{
    return {
        { Vector3(-1.0, -1.0, -1.0), Vector3(-1.0, 1.0, -1.0) },
        { Vector3(1.0, -1.0, -1.0), Vector3(1.0, 1.0, -1.0) },
        { Vector3(-1.0, -1.0, 1.0), Vector3(-1.0, 1.0, 1.0) },
        { Vector3(1.0, -1.0, 1.0), Vector3(1.0, 1.0, 1.0) },
    };
}



// Closed UV sphere centered at the origin: single pole vertices, outward
// winding, no degenerate triangles. (Box remeshing is nondeterministic
// run-to-run — the CLI reproduces it with no flags at all — so the
// bit-identical no-op checks below run on the sphere, whose pipeline the
// guides lane proved deterministic.)
void buildSphere(int latBands, int lonBands, double radius,
    std::vector<Vector3>& vertices, std::vector<std::vector<size_t>>& triangles)
{
    vertices.clear();
    triangles.clear();
    vertices.push_back(Vector3(0.0, radius, 0.0));
    for (int j = 1; j < latBands; ++j) {
        const double theta = M_PI * j / latBands;
        const double y = radius * std::cos(theta);
        const double ringRadius = radius * std::sin(theta);
        for (int i = 0; i < lonBands; ++i) {
            const double phi = 2.0 * M_PI * i / lonBands;
            vertices.push_back(Vector3(ringRadius * std::cos(phi), y, ringRadius * std::sin(phi)));
        }
    }
    const size_t southPole = vertices.size();
    vertices.push_back(Vector3(0.0, -radius, 0.0));

    const auto ringIndex = [&](int j, int i) {
        return 1 + static_cast<size_t>(j - 1) * static_cast<size_t>(lonBands)
            + static_cast<size_t>((i + lonBands) % lonBands);
    };
    for (int i = 0; i < lonBands; ++i)
        triangles.push_back({ 0, ringIndex(1, i + 1), ringIndex(1, i) });
    for (int j = 1; j + 1 < latBands; ++j) {
        for (int i = 0; i < lonBands; ++i) {
            const size_t a = ringIndex(j, i), b = ringIndex(j, i + 1);
            const size_t c = ringIndex(j + 1, i + 1), d = ringIndex(j + 1, i);
            triangles.push_back({ a, b, d });
            triangles.push_back({ b, c, d });
        }
    }
    for (int i = 0; i < lonBands; ++i)
        triangles.push_back({ southPole, ringIndex(latBands - 1, i), ringIndex(latBands - 1, i + 1) });
}

// Equatorial sharp loop in the XZ plane, closed by repeating the first point.
std::vector<Vector3> equatorialLoop(double radius = 1.0, int points = 64)
{
    std::vector<Vector3> loop;
    for (int i = 0; i <= points; ++i) {
        const double phi = 2.0 * M_PI * i / points;
        loop.push_back(Vector3(radius * std::cos(phi), 0.0, radius * std::sin(phi)));
    }
    return loop;
}

// Diagonal guide in the x=2 plane: its tangent (0,1,1)/sqrt(2) sits 45
// degrees from vertical, the maximally different cross. On +-X box faces
// this fights the vertical sharps; on +-Z faces it projects to vertical
// and agrees. A horizontal loop would be 90 degrees off — the same cross
// under the 90-degree symmetry, useless for a tie test.
std::vector<Vector3> diagonalGuide()
{
    return { Vector3(2.0, -1.0, -1.0), Vector3(2.0, 1.0, 1.0) };
}

// Misalignment of a direction to the cross spanned by `tangent`: the angle
// to the nearest multiple of 90 degrees, in degrees.
double crossMisalignmentDegrees(const Vector3& direction, const Vector3& tangent)
{
    const double cosine = std::max(0.0, std::min(1.0,
        std::fabs(Vector3::dotProduct(direction.normalized(), tangent.normalized()))));
    const double angle = std::acos(cosine) * 180.0 / M_PI;
    return std::min(angle, 90.0 - angle);
}

double maxPositionDifference(const std::vector<Vector3>& a, const std::vector<Vector3>& b)
{
    if (a.size() != b.size())
        return -1.0;
    double worst = 0.0;
    for (size_t i = 0; i < a.size(); ++i)
        worst = std::max(worst, (a[i] - b[i]).length());
    return worst;
}

double pointSegmentDistance(const Vector3& p, const Vector3& a, const Vector3& b)
{
    const Vector3 delta = b - a;
    const double length = delta.length();
    if (length <= 1e-12)
        return (p - a).length();
    const Vector3 direction = delta / length;
    double along = Vector3::dotProduct(p - a, direction);
    along = std::max(0.0, std::min(length, along));
    return (p - (a + direction * along)).length();
}

}

int main()
{
    std::vector<Vector3> boxVertices;
    std::vector<std::vector<size_t>> boxTriangles;
    buildBox(boxVertices, boxTriangles);
    CHECK(!boxVertices.empty() && !boxTriangles.empty());
    for (const auto& triangle : boxTriangles) {
        const Vector3 normal = Vector3::normal(
            boxVertices[triangle[0]], boxVertices[triangle[1]], boxVertices[triangle[2]]);
        CHECK(normal.length() > 0.5);
        const Vector3 centroid = (boxVertices[triangle[0]] + boxVertices[triangle[1]]
            + boxVertices[triangle[2]])
            / 3.0;
        CHECK(Vector3::dotProduct(normal, centroid) > 0.0);
    }

    const std::vector<std::vector<Vector3>> sharps = verticalSharps();
    const std::vector<std::vector<Vector3>> guides = { diagonalGuide() };
    const Vector3 vertical(0.0, 1.0, 0.0);

    // FrameField level: side faces near a vertical sharp lock to vertical.
    // (Top/bottom faces stay unlocked: a vertical tangent runs into their
    // plane, so tangentNear correctly reports no flow direction for them.)
    {
        const SurfaceMesh mesh(boxVertices, boxTriangles);
        std::vector<Vector3> field;
        CHECK(FrameField::create(mesh, 90.0, &field, {}, sharps));
        CHECK(field.size() == mesh.faceCount());
        size_t sideFaces = 0;
        double worst = 0.0;
        for (size_t f = 0; f < mesh.faceCount(); ++f) {
            if (std::fabs(mesh.faceNormal(f).y()) > 0.5)
                continue;
            ++sideFaces;
            worst = std::max(worst, crossMisalignmentDegrees(field[f], vertical));
        }
        std::printf("sharp box: %zu side faces, worst cross misalignment %.3f deg\n",
            sideFaces, worst);
        CHECK(sideFaces == 8);
        CHECK(worst < 2.0);
    }

    // FrameField level: sharps win ties over guides. The +-X side faces
    // sit within radius of both the vertical sharps and the 45-degree
    // diagonal guide; with both present the field must follow the sharps,
    // while the guide alone pulls those faces ~45 degrees off vertical.
    {
        const SurfaceMesh mesh(boxVertices, boxTriangles);
        std::vector<Vector3> sharpWins, guideOnly;
        CHECK(FrameField::create(mesh, 90.0, &sharpWins, guides, sharps));
        CHECK(FrameField::create(mesh, 90.0, &guideOnly, guides));
        size_t sideFaces = 0;
        double worstSharp = 0.0;
        bool guideDiffers = false;
        for (size_t f = 0; f < mesh.faceCount(); ++f) {
            if (std::fabs(mesh.faceNormal(f).y()) > 0.5)
                continue;
            ++sideFaces;
            worstSharp = std::max(worstSharp, crossMisalignmentDegrees(sharpWins[f], vertical));
            if (crossMisalignmentDegrees(guideOnly[f], vertical) > 20.0)
                guideDiffers = true;
        }
        std::printf("tie: %zu side faces, sharp+guide worst %.3f deg, guide-only claims one %.0f\n",
            sideFaces, worstSharp, guideDiffers ? 1.0 : 0.0);
        CHECK(sideFaces == 8);
        CHECK(worstSharp < 2.0);
        CHECK(guideDiffers);
    }

    // FrameField level: degenerate-only sharps are a no-op.
    {
        const SurfaceMesh mesh(boxVertices, boxTriangles);
        std::vector<Vector3> plain, degenerate;
        CHECK(FrameField::create(mesh, 90.0, &plain));
        const std::vector<std::vector<Vector3>> junk = {
            {},
            { Vector3(1.0, 0.0, 0.0) },
            { Vector3(0.0, 0.0, 1.0), Vector3(0.0, 0.0, 1.0) },
            { Vector3(100.0, 0.0, 0.0), Vector3(101.0, 0.0, 0.0) },
        };
        CHECK(FrameField::create(mesh, 90.0, &degenerate, {}, junk));
        CHECK(plain.size() == degenerate.size());
        const double junkDiff = maxPositionDifference(plain, degenerate);
        std::printf("degenerate-vs-plain field diff %.3e\n", junkDiff);
        CHECK(junkDiff >= 0.0 && junkDiff < 1e-9);
    }

    // End to end: output quad edges near the 4 disjoint verticals follow
    // them. (A full 12-edge cage is left to the CLI smoke path: where three
    // marked edges meet at a box corner the integer cover distorts — corner
    // singularities under crossing sharps are follow-up engine work. The
    // disjoint verticals assert the straight-run crispness this lane owns.)
    const std::vector<std::vector<Vector3>> cage = verticalSharps();
    std::vector<Vector3> sharpVertices, plainVertices;
    std::vector<std::vector<size_t>> sharpQuads, plainQuads;
    {
        Remesher remesher(boxVertices, boxTriangles);
        remesher.setTargetTriangleCount(1600);
        remesher.setSharpPolylines(cage);
        CHECK(remesher.remesh());
        sharpVertices = remesher.remeshedVertices();
        sharpQuads = remesher.remeshedQuads();
        CHECK(!sharpVertices.empty() && !sharpQuads.empty());
    }
    {
        Remesher remesher(boxVertices, boxTriangles);
        remesher.setTargetTriangleCount(1600);
        CHECK(remesher.remesh());
        plainVertices = remesher.remeshedVertices();
        plainQuads = remesher.remeshedQuads();
        CHECK(!plainVertices.empty() && !plainQuads.empty());
    }
    const auto cageStats = [&](const std::vector<Vector3>& vertices,
                               const std::vector<std::vector<size_t>>& quads) {
        double totalEdge = 0.0;
        size_t edgeCount = 0;
        for (const auto& quad : quads) {
            for (size_t i = 0; i < quad.size(); ++i)
                totalEdge += (vertices[quad[i]] - vertices[quad[(i + 1) % quad.size()]]).length(),
                    ++edgeCount;
        }
        const double band = 1.5 * totalEdge / std::max<size_t>(edgeCount, 1);
        double sum = 0.0, worst = 0.0;
        size_t quadsInBand = 0, edgesInBand = 0;
        for (const auto& quad : quads) {
            Vector3 centroid;
            for (const size_t index : quad)
                centroid += vertices[index];
            centroid = centroid / static_cast<double>(quad.size());
            double best = -1.0;
            Vector3 bestTangent;
            for (const auto& polyline : cage) {
                for (size_t i = 0; i + 1 < polyline.size(); ++i) {
                    const double d = pointSegmentDistance(centroid, polyline[i], polyline[i + 1]);
                    if (best < 0.0 || d < best) {
                        best = d;
                        bestTangent = (polyline[i + 1] - polyline[i]).normalized();
                    }
                }
            }
            if (best > band)
                continue;
            // Triangles cannot align to a cross by construction (60-degree
            // angles), so only true quads count toward crispness.
            if (quad.size() != 4)
                continue;
            // Polyline endpoints are singular (the feature direction
            // terminates), so quads near the verticals' top/bottom ends
            // are excluded; straight-run crispness is the assertion.
            if (std::fabs(centroid.y()) > 0.75)
                continue;
            // Skip quads the feature direction runs into (e.g. top/bottom
            // faces near a vertical's endpoint): crispness only applies
            // where the tangent lies in the surface, mirroring the 60-degree
            // rejection in Guides::tangentNear.
            if (quad.size() >= 3) {
                const Vector3 quadNormal = Vector3::normal(
                    vertices[quad[0]], vertices[quad[1]], vertices[quad[2]]);
                if (std::fabs(Vector3::dotProduct(bestTangent, quadNormal)) > 0.5)
                    continue;
            }
            ++quadsInBand;
            for (size_t i = 0; i < quad.size(); ++i) {
                const Vector3 edge = vertices[quad[(i + 1) % quad.size()]] - vertices[quad[i]];
                if (edge.length() <= 1e-12)
                    continue;
                const double mis = crossMisalignmentDegrees(edge, bestTangent);
                sum += mis;
                worst = std::max(worst, mis);
                ++edgesInBand;
            }
        }
        std::vector<double> all;
        all.reserve(edgesInBand);
        for (const auto& quad : quads) {
            Vector3 centroid;
            for (const size_t index : quad)
                centroid += vertices[index];
            centroid = centroid / static_cast<double>(quad.size());
            double best = -1.0;
            Vector3 bestTangent;
            for (const auto& polyline : cage) {
                for (size_t i = 0; i + 1 < polyline.size(); ++i) {
                    const double d = pointSegmentDistance(centroid, polyline[i], polyline[i + 1]);
                    if (best < 0.0 || d < best) {
                        best = d;
                        bestTangent = (polyline[i + 1] - polyline[i]).normalized();
                    }
                }
            }
            if (best > band || quad.size() != 4 || std::fabs(centroid.y()) > 0.75)
                continue;
            if (quad.size() >= 3) {
                const Vector3 quadNormal = Vector3::normal(
                    vertices[quad[0]], vertices[quad[1]], vertices[quad[2]]);
                if (std::fabs(Vector3::dotProduct(bestTangent, quadNormal)) > 0.5)
                    continue;
            }
            for (size_t i = 0; i < quad.size(); ++i) {
                const Vector3 edge = vertices[quad[(i + 1) % quad.size()]] - vertices[quad[i]];
                if (edge.length() <= 1e-12)
                    continue;
                all.push_back(crossMisalignmentDegrees(edge, bestTangent));
            }
        }
        std::sort(all.begin(), all.end());
        const double p95 = all.empty() ? -1.0 : all[std::min(all.size() - 1, all.size() * 95 / 100)];
        const double p99 = all.empty() ? -1.0 : all[std::min(all.size() - 1, all.size() * 99 / 100)];
        return std::make_tuple(quadsInBand, edgesInBand,
            edgesInBand > 0 ? sum / edgesInBand : -1.0, worst, p95, p99);
    };
    {
        const auto [sharpBands, sharpEdges, sharpMean, sharpWorst, sharpP95, sharpP99] = cageStats(sharpVertices, sharpQuads);
        const auto [plainBands, plainEdges, plainMean, plainWorst, plainP95, plainP99] = cageStats(plainVertices, plainQuads);
        std::printf("sharp: %zu quads / %zu edges in band, mean %.2f deg, worst %.2f deg, p95 %.2f, p99 %.2f\n",
            sharpBands, sharpEdges, sharpMean, sharpWorst, sharpP95, sharpP99);
        std::printf("plain: %zu quads / %zu edges in band, mean %.2f deg, worst %.2f deg, p95 %.2f, p99 %.2f\n",
            plainBands, plainEdges, plainMean, plainWorst, plainP95, plainP99);
        CHECK(sharpBands >= 8);
        CHECK(sharpMean >= 0.0 && sharpMean < 15.0);
    }

    // End to end contrast: a diagonal feature across the +Z face carries no
    // dihedral cue, so the plain run ignores it while the marked run follows
    // it. This is the case explicit sharps exist for: plain must fail the
    // crispness bar the marked run clears. Partial compliance is expected
    // (the guides helix precedent): the marked run must clearly beat plain,
    // not reach the straight-edge bar.
    const std::vector<std::vector<Vector3>> diagonal = {
        { Vector3(-1.0, -1.0, 1.0), Vector3(1.0, 1.0, 1.0) },
    };
    std::vector<Vector3> diagSharpVertices, diagPlainVertices;
    std::vector<std::vector<size_t>> diagSharpQuads, diagPlainQuads;
    {
        Remesher remesher(boxVertices, boxTriangles);
        remesher.setTargetTriangleCount(1600);
        remesher.setSharpPolylines(diagonal);
        CHECK(remesher.remesh());
        diagSharpVertices = remesher.remeshedVertices();
        diagSharpQuads = remesher.remeshedQuads();
        CHECK(!diagSharpVertices.empty() && !diagSharpQuads.empty());
    }
    {
        Remesher remesher(boxVertices, boxTriangles);
        remesher.setTargetTriangleCount(1600);
        CHECK(remesher.remesh());
        diagPlainVertices = remesher.remeshedVertices();
        diagPlainQuads = remesher.remeshedQuads();
        CHECK(!diagPlainVertices.empty() && !diagPlainQuads.empty());
    }
    const auto diagStats = [&](const std::vector<Vector3>& vertices,
                               const std::vector<std::vector<size_t>>& quads) {
        double totalEdge = 0.0;
        size_t edgeCount = 0;
        for (const auto& quad : quads) {
            for (size_t i = 0; i < quad.size(); ++i)
                totalEdge += (vertices[quad[i]] - vertices[quad[(i + 1) % quad.size()]]).length(),
                    ++edgeCount;
        }
        const double band = 1.5 * totalEdge / std::max<size_t>(edgeCount, 1);
        const Vector3 a = diagonal[0][0], b = diagonal[0][1];
        const Vector3 tangent = (b - a).normalized();
        std::vector<double> all;
        size_t quadsInBand = 0;
        for (const auto& quad : quads) {
            Vector3 centroid;
            for (const size_t index : quad)
                centroid += vertices[index];
            centroid = centroid / static_cast<double>(quad.size());
            if (pointSegmentDistance(centroid, a, b) > band)
                continue;
            if (quad.size() != 4)
                continue;
            // Endpoints terminate mid-face (singular): exclude quads near them.
            if ((centroid - a).length() < 0.35 || (centroid - b).length() < 0.35)
                continue;
            const Vector3 quadNormal = Vector3::normal(
                vertices[quad[0]], vertices[quad[1]], vertices[quad[2]]);
            if (std::fabs(Vector3::dotProduct(tangent, quadNormal)) > 0.5)
                continue;
            ++quadsInBand;
            for (size_t i = 0; i < quad.size(); ++i) {
                const Vector3 edge = vertices[quad[(i + 1) % quad.size()]] - vertices[quad[i]];
                if (edge.length() <= 1e-12)
                    continue;
                all.push_back(crossMisalignmentDegrees(edge, tangent));
            }
        }
        std::sort(all.begin(), all.end());
        double sum = 0.0;
        for (double v : all)
            sum += v;
        const double mean = all.empty() ? -1.0 : sum / all.size();
        const double p95 = all.empty() ? -1.0 : all[std::min(all.size() - 1, all.size() * 95 / 100)];
        return std::make_tuple(quadsInBand, all.size(), mean, p95);
    };
    {
        const auto [sharpBands, sharpEdges, sharpMean, sharpP95] = diagStats(diagSharpVertices, diagSharpQuads);
        const auto [plainBands, plainEdges, plainMean, plainP95] = diagStats(diagPlainVertices, diagPlainQuads);
        std::printf("diagonal marked: %zu quads / %zu edges, mean %.2f deg, p95 %.2f\n",
            sharpBands, sharpEdges, sharpMean, sharpP95);
        std::printf("diagonal plain:  %zu quads / %zu edges, mean %.2f deg, p95 %.2f\n",
            plainBands, plainEdges, plainMean, plainP95);
        CHECK(sharpBands >= 8);
        CHECK(sharpMean >= 0.0 && sharpMean < 28.0);
        CHECK(plainMean > 30.0);
        CHECK(sharpMean + 10.0 < plainMean);
    }

    // Box remeshing is nondeterministic run-to-run (the CLI reproduces
    // 176/180/178 quads on the same cube with no flags at all), so exact
    // no-op and alias comparisons run on the sphere instead. A marked-vs-
    // plain `differ` check on the box would prove nothing — any two plain
    // runs already differ — so the box asserts crispness only (above), and
    // the sphere below asserts no-op/alias/differ deterministically.
    std::vector<Vector3> sphereVertices;
    std::vector<std::vector<size_t>> sphereTriangles;
    buildSphere(16, 32, 1.0, sphereVertices, sphereTriangles);
    const std::vector<std::vector<Vector3>> sphereSharps = { equatorialLoop(1.0) };
    std::vector<Vector3> spherePlainVertices, sphereSharpVertices;
    std::vector<std::vector<size_t>> spherePlainQuads, sphereSharpQuads;
    {
        Remesher remesher(sphereVertices, sphereTriangles);
        remesher.setTargetTriangleCount(2000);
        CHECK(remesher.remesh());
        spherePlainVertices = remesher.remeshedVertices();
        spherePlainQuads = remesher.remeshedQuads();
        CHECK(!spherePlainVertices.empty() && !spherePlainQuads.empty());
    }
    {
        Remesher remesher(sphereVertices, sphereTriangles);
        remesher.setTargetTriangleCount(2000);
        remesher.setSharpPolylines(sphereSharps);
        CHECK(remesher.remesh());
        sphereSharpVertices = remesher.remeshedVertices();
        sphereSharpQuads = remesher.remeshedQuads();
        CHECK(!sphereSharpVertices.empty() && !sphereSharpQuads.empty());
    }

    // End to end on the sphere: marked output differs from unmarked.
    {
        bool differ = sphereSharpVertices.size() != spherePlainVertices.size()
            || sphereSharpQuads.size() != spherePlainQuads.size();
        for (size_t i = 0; !differ && i < sphereSharpVertices.size() && i < spherePlainVertices.size(); ++i) {
            if ((sphereSharpVertices[i] - spherePlainVertices[i]).length() > 1e-9)
                differ = true;
        }
        std::printf("sphere: plain %zu verts / %zu quads, sharp %zu verts / %zu quads, differ=%d\n",
            spherePlainVertices.size(), spherePlainQuads.size(),
            sphereSharpVertices.size(), sphereSharpQuads.size(), differ ? 1 : 0);
        CHECK(differ);
    }

    // End to end on the sphere: empty sharps are a no-op vs the default run.
    {
        Remesher remesher(sphereVertices, sphereTriangles);
        remesher.setTargetTriangleCount(2000);
        remesher.setSharpPolylines({});
        CHECK(remesher.remesh());
        CHECK(remesher.remeshedQuads().size() == spherePlainQuads.size());
        CHECK(remesher.remeshedVertices().size() == spherePlainVertices.size());
        const double emptyDiff = maxPositionDifference(remesher.remeshedVertices(), spherePlainVertices);
        std::printf("empty-sharps end-to-end position diff %.3e\n", emptyDiff);
        CHECK(emptyDiff >= 0.0 && emptyDiff < 1e-9);
    }

    // End to end on the sphere: degenerate-only sharps are a no-op
    // (far-away points are dropped by the post-resample snap gate, the
    // rest carry no direction).
    {
        Remesher remesher(sphereVertices, sphereTriangles);
        remesher.setTargetTriangleCount(2000);
        remesher.setSharpPolylines({ {}, { Vector3(1.0, 0.0, 0.0) },
            { Vector3(100.0, 0.0, 0.0), Vector3(101.0, 0.0, 0.0) } });
        CHECK(remesher.remesh());
        CHECK(remesher.remeshedQuads().size() == spherePlainQuads.size());
        CHECK(remesher.remeshedVertices().size() == spherePlainVertices.size());
        const double junkDiff = maxPositionDifference(remesher.remeshedVertices(), spherePlainVertices);
        std::printf("degenerate-sharps end-to-end position diff %.3e\n", junkDiff);
        CHECK(junkDiff >= 0.0 && junkDiff < 1e-9);
    }

    // End to end on the sphere: setFeaturePolylines reaches the engine
    // like setSharpPolylines does. (Marked runs are not bit-identical
    // run-to-run — the sharp path exposes the same pre-existing ordering
    // race as plain box runs — so the alias asserts an effect, not
    // equality: it must differ from plain, as the sharp run does. Both
    // setters assign the same member; see autoremesher.cppm.)
    {
        Remesher remesher(sphereVertices, sphereTriangles);
        remesher.setTargetTriangleCount(2000);
        remesher.setFeaturePolylines(sphereSharps);
        CHECK(remesher.remesh());
        const std::vector<Vector3> aliasVertices = remesher.remeshedVertices();
        const std::vector<std::vector<size_t>> aliasQuads = remesher.remeshedQuads();
        CHECK(!aliasVertices.empty() && !aliasQuads.empty());
        bool differ = aliasVertices.size() != spherePlainVertices.size()
            || aliasQuads.size() != spherePlainQuads.size();
        for (size_t i = 0; !differ && i < aliasVertices.size() && i < spherePlainVertices.size(); ++i) {
            if ((aliasVertices[i] - spherePlainVertices[i]).length() > 1e-9)
                differ = true;
        }
        std::printf("alias: %zu verts / %zu quads, differs-from-plain=%d\n",
            aliasVertices.size(), aliasQuads.size(), differ ? 1 : 0);
        CHECK(differ);
    }

    if (g_failures == 0)
        std::printf("PASS test_sharp\n");
    return g_failures == 0 ? 0 : 1;
}
