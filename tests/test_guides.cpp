// Unit + engine tests for user guide-curve constraints (FrameField guides).
// Plain assert-style main, no third-party framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
import retopo.core.auto_remesher;
import retopo.core.frame_field;
import retopo.core.surface_mesh;
import retopo.core.vector3;

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

// Closed UV sphere centered at the origin: single pole vertices, outward
// winding, no degenerate triangles.
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

// Equatorial guide loop in the XZ plane, closed by repeating the first point.
std::vector<Vector3> equatorialGuide(double radius, int points = 64)
{
    std::vector<Vector3> loop;
    for (int i = 0; i <= points; ++i) {
        const double phi = 2.0 * M_PI * i / points;
        loop.push_back(Vector3(radius * std::cos(phi), 0.0, radius * std::sin(phi)));
    }
    return loop;
}

// Tangent of the equatorial loop at the longitude of `point`.
Vector3 equatorialTangent(const Vector3& point)
{
    return Vector3(-point.z(), 0.0, point.x()).normalized();
}

// Closed cylinder centered at the origin (axis = Y), outward winding.
void buildCylinder(int around, int rows, double radius, double height,
    std::vector<Vector3>& vertices, std::vector<std::vector<size_t>>& triangles)
{
    vertices.clear();
    triangles.clear();
    for (int j = 0; j <= rows; ++j) {
        const double y = -height / 2.0 + height * j / rows;
        for (int i = 0; i < around; ++i) {
            const double phi = 2.0 * M_PI * i / around;
            vertices.push_back(Vector3(radius * std::cos(phi), y, radius * std::sin(phi)));
        }
    }
    const auto ringIndex = [&](int j, int i) {
        return static_cast<size_t>(j) * static_cast<size_t>(around)
            + static_cast<size_t>((i + around) % around);
    };
    for (int j = 0; j < rows; ++j) {
        for (int i = 0; i < around; ++i) {
            const size_t a = ringIndex(j, i), b = ringIndex(j, i + 1);
            const size_t c = ringIndex(j + 1, i + 1), d = ringIndex(j + 1, i);
            triangles.push_back({ a, d, c });
            triangles.push_back({ a, c, b });
        }
    }
    const size_t topCenter = vertices.size();
    vertices.push_back(Vector3(0.0, height / 2.0, 0.0));
    for (int i = 0; i < around; ++i)
        triangles.push_back({ topCenter, ringIndex(rows, i + 1), ringIndex(rows, i) });
    const size_t bottomCenter = vertices.size();
    vertices.push_back(Vector3(0.0, -height / 2.0, 0.0));
    for (int i = 0; i < around; ++i)
        triangles.push_back({ bottomCenter, ringIndex(0, i), ringIndex(0, i + 1) });
}

struct Helix {
    double radius = 1.0;
    double yBegin = -3.5;
    double yEnd = 3.5;
    double thetaTotal = 7.0;
    std::vector<Vector3> points;
    std::vector<double> thetas;
};

Helix buildHelix(int samples = 240)
{
    Helix helix;
    for (int i = 0; i < samples; ++i) {
        const double t = static_cast<double>(i) / (samples - 1);
        const double y = helix.yBegin + t * (helix.yEnd - helix.yBegin);
        const double theta = t * helix.thetaTotal;
        helix.points.push_back(
            Vector3(helix.radius * std::cos(theta), y, helix.radius * std::sin(theta)));
        helix.thetas.push_back(theta);
    }
    return helix;
}

Vector3 helixTangent(double theta)
{
    return Vector3(-std::sin(theta), 1.0, std::cos(theta)).normalized();
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

}

int main()
{
    // Hand-built sphere: outward winding, no degenerate faces.
    std::vector<Vector3> sphereVertices;
    std::vector<std::vector<size_t>> sphereTriangles;
    buildSphere(16, 32, 1.0, sphereVertices, sphereTriangles);
    CHECK(!sphereVertices.empty() && !sphereTriangles.empty());
    for (const auto& triangle : sphereTriangles) {
        const Vector3 normal = Vector3::normal(
            sphereVertices[triangle[0]], sphereVertices[triangle[1]], sphereVertices[triangle[2]]);
        CHECK(normal.length() > 0.5);
        const Vector3 centroid = (sphereVertices[triangle[0]] + sphereVertices[triangle[1]]
            + sphereVertices[triangle[2]])
            / 3.0;
        CHECK(Vector3::dotProduct(normal, centroid) > 0.0);
    }

    const std::vector<std::vector<Vector3>> guides = { equatorialGuide(1.0) };

    // FrameField level: faces near the guide lock to the guide tangent.
    {
        const SurfaceMesh mesh(sphereVertices, sphereTriangles);
        std::vector<Vector3> field;
        CHECK(FrameField::create(mesh, 90.0, &field, guides));
        CHECK(field.size() == mesh.faceCount());
        const double radius = 2.0 * mesh.averageEdgeLength();
        size_t locked = 0;
        double worst = 0.0;
        for (size_t f = 0; f < mesh.faceCount(); ++f) {
            const auto& triangle = mesh.triangle(f);
            const Vector3 centroid = (mesh.position(triangle[0]) + mesh.position(triangle[1])
                + mesh.position(triangle[2]))
                / 3.0;
            if (std::fabs(centroid.y()) > radius)
                continue;
            ++locked;
            worst = std::max(worst, crossMisalignmentDegrees(field[f], equatorialTangent(centroid)));
        }
        std::printf("guide band: %zu faces within %.4f of equator, worst cross misalignment %.3f deg\n",
            locked, radius, worst);
        CHECK(locked > 0);
        CHECK(worst < 2.0);
    }

    // FrameField level: degenerate-only guides are a no-op.
    {
        const SurfaceMesh mesh(sphereVertices, sphereTriangles);
        std::vector<Vector3> plain, degenerate;
        CHECK(FrameField::create(mesh, 90.0, &plain));
        const std::vector<std::vector<Vector3>> junk = {
            {},
            { Vector3(1.0, 0.0, 0.0) },
            { Vector3(0.0, 0.0, 1.0), Vector3(0.0, 0.0, 1.0) },
            { Vector3(100.0, 0.0, 0.0), Vector3(101.0, 0.0, 0.0) },
        };
        CHECK(FrameField::create(mesh, 90.0, &degenerate, junk));
        CHECK(plain.size() == degenerate.size());
        const double junkDiff = maxPositionDifference(plain, degenerate);
        std::vector<Vector3> plainAgain;
        CHECK(FrameField::create(mesh, 90.0, &plainAgain));
        const double rerunDiff = maxPositionDifference(plain, plainAgain);
        std::printf("degenerate-vs-plain field diff %.3e, plain rerun-vs-plain field diff %.3e\n",
            junkDiff, rerunDiff);
        // No-op means: indistinguishable from the pipeline's own rerun noise.
        CHECK(junkDiff >= 0.0 && junkDiff < 1e-9);
    }

    // End to end: output quad edges near the guide follow the guide tangent.
    std::vector<Vector3> guidedVertices, unguidedVertices;
    std::vector<std::vector<size_t>> guidedQuads, unguidedQuads;
    {
        Remesher remesher(sphereVertices, sphereTriangles);
        remesher.setTargetTriangleCount(2000);
        remesher.setGuidePolylines(guides);
        CHECK(remesher.remesh());
        guidedVertices = remesher.remeshedVertices();
        guidedQuads = remesher.remeshedQuads();
        CHECK(!guidedVertices.empty() && !guidedQuads.empty());
    }
    {
        Remesher remesher(sphereVertices, sphereTriangles);
        remesher.setTargetTriangleCount(2000);
        CHECK(remesher.remesh());
        unguidedVertices = remesher.remeshedVertices();
        unguidedQuads = remesher.remeshedQuads();
        CHECK(!unguidedVertices.empty() && !unguidedQuads.empty());
    }
    const auto bandStats = [](const std::vector<Vector3>& vertices,
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
            if (std::fabs(centroid.y()) > band)
                continue;
            ++quadsInBand;
            const Vector3 tangent = equatorialTangent(centroid);
            for (size_t i = 0; i < quad.size(); ++i) {
                const Vector3 edge = vertices[quad[(i + 1) % quad.size()]] - vertices[quad[i]];
                if (edge.length() <= 1e-12)
                    continue;
                const double mis = crossMisalignmentDegrees(edge, tangent);
                sum += mis;
                worst = std::max(worst, mis);
                ++edgesInBand;
            }
        }
        return std::make_tuple(quadsInBand, edgesInBand,
            edgesInBand > 0 ? sum / edgesInBand : -1.0, worst);
    };
    {
        const auto [guidedBands, guidedEdges, guidedMean, guidedWorst] = bandStats(guidedVertices, guidedQuads);
        const auto [plainBands, plainEdges, plainMean, plainWorst] = bandStats(unguidedVertices, unguidedQuads);
        std::printf("guided:   %zu quads / %zu edges in band, mean %.2f deg, worst %.2f deg\n",
            guidedBands, guidedEdges, guidedMean, guidedWorst);
        std::printf("unguided: %zu quads / %zu edges in band, mean %.2f deg, worst %.2f deg\n",
            plainBands, plainEdges, plainMean, plainWorst);
        CHECK(guidedBands >= 8);
        CHECK(guidedMean >= 0.0 && guidedMean < 12.0);
        CHECK(guidedWorst < 30.0);
    }

    // End to end: empty guides are a no-op vs the default run.
    {
        Remesher remesher(sphereVertices, sphereTriangles);
        remesher.setTargetTriangleCount(2000);
        remesher.setGuidePolylines({});
        CHECK(remesher.remesh());
        CHECK(remesher.remeshedQuads().size() == unguidedQuads.size());
        CHECK(remesher.remeshedVertices().size() == unguidedVertices.size());
        const double emptyDiff = maxPositionDifference(remesher.remeshedVertices(), unguidedVertices);
        std::printf("empty-guides end-to-end position diff %.3e\n", emptyDiff);
        CHECK(emptyDiff >= 0.0 && emptyDiff < 1e-9);
    }

    // End to end: degenerate-only guides are a no-op vs the default run.
    {
        Remesher remesher(sphereVertices, sphereTriangles);
        remesher.setTargetTriangleCount(2000);
        remesher.setGuidePolylines({ {}, { Vector3(1.0, 0.0, 0.0) },
            { Vector3(100.0, 0.0, 0.0), Vector3(101.0, 0.0, 0.0) } });
        CHECK(remesher.remesh());
        CHECK(remesher.remeshedQuads().size() == unguidedQuads.size());
        CHECK(remesher.remeshedVertices().size() == unguidedVertices.size());
        const double junkDiff = maxPositionDifference(remesher.remeshedVertices(), unguidedVertices);
        std::printf("degenerate-guides end-to-end position diff %.3e\n", junkDiff);
        CHECK(junkDiff >= 0.0 && junkDiff < 1e-9);
    }

    // End to end: rerun determinism baseline (same input, same settings).
    {
        Remesher remesher(sphereVertices, sphereTriangles);
        remesher.setTargetTriangleCount(2000);
        CHECK(remesher.remesh());
        std::printf("plain rerun end-to-end position diff %.3e\n",
            maxPositionDifference(remesher.remeshedVertices(), unguidedVertices));
    }

    // End to end on a cylinder: a 45-degree helix guide fights the natural
    // curvature-driven (circumferential) flow, so guided vs unguided contrast
    // is large. This is the adversarial case: partial compliance is expected,
    // but the guided run must clearly beat the unguided one.
    {
        std::vector<Vector3> cylinderVertices;
        std::vector<std::vector<size_t>> cylinderTriangles;
        buildCylinder(36, 72, 1.0, 9.0, cylinderVertices, cylinderTriangles);
        for (const auto& triangle : cylinderTriangles) {
            const Vector3 normal = Vector3::normal(cylinderVertices[triangle[0]],
                cylinderVertices[triangle[1]], cylinderVertices[triangle[2]]);
            CHECK(normal.length() > 0.5);
            const Vector3 centroid = (cylinderVertices[triangle[0]] + cylinderVertices[triangle[1]]
                + cylinderVertices[triangle[2]])
                / 3.0;
            CHECK(Vector3::dotProduct(normal, centroid) > 0.0);
        }
        const Helix helix = buildHelix();
        const std::vector<std::vector<Vector3>> helixGuides = { helix.points };
        std::vector<Vector3> helixGuidedVertices, helixPlainVertices;
        std::vector<std::vector<size_t>> helixGuidedQuads, helixPlainQuads;
        {
            Remesher remesher(cylinderVertices, cylinderTriangles);
            remesher.setTargetTriangleCount(4000);
            remesher.setGuidePolylines(helixGuides);
            CHECK(remesher.remesh());
            helixGuidedVertices = remesher.remeshedVertices();
            helixGuidedQuads = remesher.remeshedQuads();
        }
        {
            Remesher remesher(cylinderVertices, cylinderTriangles);
            remesher.setTargetTriangleCount(4000);
            CHECK(remesher.remesh());
            helixPlainVertices = remesher.remeshedVertices();
            helixPlainQuads = remesher.remeshedQuads();
        }
        const auto helixStats = [&](const std::vector<Vector3>& vertices,
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
                size_t bestIndex = 0;
                for (size_t i = 0; i < helix.points.size(); ++i) {
                    const double d = (centroid - helix.points[i]).length();
                    if (best < 0.0 || d < best)
                        best = d, bestIndex = i;
                }
                if (best > band)
                    continue;
                ++quadsInBand;
                const Vector3 tangent = helixTangent(helix.thetas[bestIndex]);
                for (size_t i = 0; i < quad.size(); ++i) {
                    const Vector3 edge = vertices[quad[(i + 1) % quad.size()]] - vertices[quad[i]];
                    if (edge.length() <= 1e-12)
                        continue;
                    const double mis = crossMisalignmentDegrees(edge, tangent);
                    sum += mis;
                    worst = std::max(worst, mis);
                    ++edgesInBand;
                }
            }
            return std::make_tuple(quadsInBand, edgesInBand,
                edgesInBand > 0 ? sum / edgesInBand : -1.0, worst);
        };
        const auto [hgBands, hgEdges, hgMean, hgWorst] = helixStats(helixGuidedVertices, helixGuidedQuads);
        const auto [hpBands, hpEdges, hpMean, hpWorst] = helixStats(helixPlainVertices, helixPlainQuads);
        std::printf("helix guided:   %zu quads / %zu edges in band, mean %.2f deg, worst %.2f deg\n",
            hgBands, hgEdges, hgMean, hgWorst);
        std::printf("helix unguided: %zu quads / %zu edges in band, mean %.2f deg, worst %.2f deg\n",
            hpBands, hpEdges, hpMean, hpWorst);
        CHECK(hgBands >= 8);
        CHECK(hgMean >= 0.0 && hgMean < 25.0);
        CHECK(hgMean < hpMean);
    }

    if (g_failures == 0)
        std::printf("PASS test_guides\n");
    return g_failures == 0 ? 0 : 1;
}
