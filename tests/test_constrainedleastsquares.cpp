// Unit tests for retopo.core.constrained_least_squares
// (core/constrainedleastsquares.cppm). Solver goldens on small
// deterministic systems: minimize sum weight*(c.x - rhs)^2 subject to
// hard constraints c.x = rhs.
// Plain assert-style main, no third-party framework.
// NOTE: CHECK instead of <cassert> assert(), which is a no-op under NDEBUG.
import retopo.core.constrained_least_squares;

#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>
#include <vector>

static int g_failures = 0;

#define CHECK(cond)                                                                                  \
    do {                                                                                             \
        if (!(cond)) {                                                                               \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                               \
            ++g_failures;                                                                            \
        }                                                                                            \
    } while (0)

static bool near(double a, double b, double tol = 1e-6)
{
    return std::fabs(a - b) <= tol;
}

int main()
{
    using AutoRemesher::ConstrainedLeastSquares;
    using Row = std::vector<std::pair<size_t, double>>;

    // Unconstrained diagonal system: each energy pins one variable.
    {
        ConstrainedLeastSquares s(2);
        s.addEnergy(Row { { 0, 1.0 } }, 3.0);
        s.addEnergy(Row { { 1, 1.0 } }, -1.0);
        std::vector<double> x;
        CHECK(s.solve(&x));
        CHECK(x.size() == 2);
        CHECK(near(x[0], 3.0));
        CHECK(near(x[1], -1.0));
    }

    // Weighted single variable: (0*1 + 10*3) / (1+3) = 7.5.
    {
        ConstrainedLeastSquares s(1);
        s.addEnergy(Row { { 0, 1.0 } }, 0.0, 1.0);
        s.addEnergy(Row { { 0, 1.0 } }, 10.0, 3.0);
        std::vector<double> x;
        CHECK(s.solve(&x));
        CHECK(x.size() == 1);
        CHECK(near(x[0], 7.5));
    }

    // Overdetermined: two energies on x0 average, x1 pinned directly.
    {
        ConstrainedLeastSquares s(2);
        s.addEnergy(Row { { 0, 1.0 } }, 1.0);
        s.addEnergy(Row { { 0, 1.0 } }, 3.0);
        s.addEnergy(Row { { 1, 1.0 } }, 5.0);
        std::vector<double> x;
        CHECK(s.solve(&x));
        CHECK(near(x[0], 2.0));
        CHECK(near(x[1], 5.0));
    }

    // Two-variable hard constraint (substitution path): energies pull both
    // variables to 0, x0 + x1 = 4 splits the difference.
    {
        ConstrainedLeastSquares s(2);
        s.addEnergy(Row { { 0, 1.0 } }, 0.0);
        s.addEnergy(Row { { 1, 1.0 } }, 0.0);
        s.addConstraint(Row { { 0, 1.0 }, { 1, 1.0 } }, 4.0);
        std::vector<double> x;
        CHECK(s.solve(&x));
        CHECK(near(x[0], 2.0));
        CHECK(near(x[1], 2.0));
        CHECK(near(x[0] + x[1], 4.0));
    }

    // Single-variable hard constraint pins x0; x1 stays at its energy.
    {
        ConstrainedLeastSquares s(2);
        s.addEnergy(Row { { 0, 1.0 } }, 0.0);
        s.addEnergy(Row { { 1, 1.0 } }, 0.0);
        s.addConstraint(Row { { 0, 1.0 } }, 7.0);
        std::vector<double> x;
        CHECK(s.solve(&x));
        CHECK(near(x[0], 7.0));
        CHECK(near(x[1], 0.0));
    }

    // Three-variable hard constraint (Lagrange path): equal energies at 0
    // with x0 + x1 + x2 = 6 spread evenly.
    {
        ConstrainedLeastSquares s(3);
        s.addEnergy(Row { { 0, 1.0 } }, 0.0);
        s.addEnergy(Row { { 1, 1.0 } }, 0.0);
        s.addEnergy(Row { { 2, 1.0 } }, 0.0);
        s.addConstraint(Row { { 0, 1.0 }, { 1, 1.0 }, { 2, 1.0 } }, 6.0);
        std::vector<double> x;
        CHECK(s.solve(&x));
        CHECK(near(x[0], 2.0));
        CHECK(near(x[1], 2.0));
        CHECK(near(x[2], 2.0));
    }

    // setEnergyRightHandSide re-solves the same system with a new target.
    {
        ConstrainedLeastSquares s(1);
        const size_t index = s.addEnergy(Row { { 0, 1.0 } }, 1.0);
        CHECK(index == 0);
        std::vector<double> x;
        CHECK(s.solve(&x));
        CHECK(near(x[0], 1.0));
        s.setEnergyRightHandSide(index, 5.0);
        CHECK(s.solve(&x));
        CHECK(near(x[0], 5.0));
    }

    // clearConstraints drops the hard constraint; energies alone remain.
    {
        ConstrainedLeastSquares s(2);
        s.addEnergy(Row { { 0, 1.0 } }, 0.0);
        s.addEnergy(Row { { 1, 1.0 } }, 0.0);
        s.addConstraint(Row { { 0, 1.0 }, { 1, 1.0 } }, 4.0);
        std::vector<double> x;
        CHECK(s.solve(&x));
        CHECK(near(x[0], 2.0));
        s.clearConstraints();
        CHECK(s.solve(&x));
        CHECK(near(x[0], 0.0));
        CHECK(near(x[1], 0.0));
    }

    // Degenerate inputs: empty coefficients and non-positive weights are
    // refused with noIndex, solving zero variables fails.
    {
        ConstrainedLeastSquares s(1);
        const size_t none = std::numeric_limits<size_t>::max();
        CHECK(s.addEnergy(Row {}, 0.0) == none);
        CHECK(s.addEnergy(Row { { 0, 1.0 } }, 0.0, 0.0) == none);
        CHECK(s.addEnergy(Row { { 0, 1.0 } }, 0.0, -2.0) == none);
        ConstrainedLeastSquares empty(0);
        std::vector<double> x;
        CHECK(!empty.solve(&x));
    }

    if (g_failures == 0)
        std::printf("PASS test_constrainedleastsquares\n");
    return g_failures == 0 ? 0 : 1;
}
