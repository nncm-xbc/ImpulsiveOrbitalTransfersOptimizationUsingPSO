#include <catch2/catch_all.hpp>
#include "optimization/PSO.hpp"
#include "core/OrbitProblem.hpp"
#include <functional>
#include <cmath>

// Simple test function for PSO validation
double sphereFunction(const std::vector<double>& x) {
    double sum = 0.0;
    for (double xi : x) {
        sum += xi * xi;
    }
    return sum;
}

double rosenbrockFunction(const std::vector<double>& x) {
    if (x.size() < 2) return 1e6;
    double sum = 0.0;
    for (size_t i = 0; i < x.size() - 1; i++) {
        double term1 = x[i+1] - x[i]*x[i];
        double term2 = 1.0 - x[i];
        sum += 100.0 * term1 * term1 + term2 * term2;
    }
    return sum;
}

TEST_CASE("PSO - Basic Algorithm Functionality", "[pso][optimization]") {

    SECTION("PSO constructor and solve execution") {
        auto testFunc = [](double* x) -> double {
            std::vector<double> vec(x, x + 2);
            return sphereFunction(vec);
        };

        PSO<double, std::function<double(double*)>> pso(
            20, 2, 50, 1e-6, 0.7, 1.5, 1.5,
            testFunc, {-5.0, -5.0}, {5.0, 5.0}
        );

        // Should execute without crashing
        REQUIRE_NOTHROW(pso.solve());

        // Should find a reasonable solution
        double best_value = pso.getBestValue();
        REQUIRE(best_value >= 0.0);
        REQUIRE(std::isfinite(best_value));

        std::vector<double> best_pos = pso.getBestPosition();
        REQUIRE(best_pos.size() == 2);
        REQUIRE(std::isfinite(best_pos[0]));
        REQUIRE(std::isfinite(best_pos[1]));
    }

    SECTION("PSO convergence on sphere function") {
        auto testFunc = [](double* x) -> double {
            std::vector<double> vec(x, x + 2);
            return sphereFunction(vec);
        };

        PSO<double, std::function<double(double*)>> pso(
            30, 2, 100, 1e-6, 0.7, 1.5, 1.5,
            testFunc, {-10.0, -10.0}, {10.0, 10.0}
        );

        pso.solve();

        double best_value = pso.getBestValue();
        std::vector<double> best_pos = pso.getBestPosition();

        // Should converge close to global minimum at origin
        REQUIRE(best_value < 1.0);
        REQUIRE(abs(best_pos[0]) < 1.0);
        REQUIRE(abs(best_pos[1]) < 1.0);
    }

    SECTION("PSO with different dimensions") {
        // Test 1D problem
        auto func1D = [](double* x) -> double {
            return (x[0] - 2.0) * (x[0] - 2.0);
        };

        PSO<double, std::function<double(double*)>> pso1D(
            20, 1, 50, 1e-6, 0.5, 1.0, 1.0,
            func1D, {-5.0}, {5.0}
        );

        REQUIRE_NOTHROW(pso1D.solve());

        double best_value = pso1D.getBestValue();
        std::vector<double> best_pos = pso1D.getBestPosition();

        REQUIRE(best_pos.size() == 1);
        REQUIRE(best_value < 1.0);
        REQUIRE(abs(best_pos[0] - 2.0) < 1.0);

        // Test 3D problem
        auto func3D = [](double* x) -> double {
            return x[0]*x[0] + x[1]*x[1] + x[2]*x[2];
        };

        PSO<double, std::function<double(double*)>> pso3D(
            30, 3, 50, 1e-6, 0.5, 1.0, 1.0,
            func3D, {-5.0, -5.0, -5.0}, {5.0, 5.0, 5.0}
        );

        REQUIRE_NOTHROW(pso3D.solve());
        REQUIRE(pso3D.getBestPosition().size() == 3);
    }
}

TEST_CASE("PSO - Parameter Validation", "[pso][validation]") {

    SECTION("Boundary constraint enforcement") {
        auto testFunc = [](double* x) -> double {
            return x[0] * x[0];
        };

        std::vector<double> lower = {1.0};
        std::vector<double> upper = {2.0};

        PSO<double, std::function<double(double*)>> pso(
            20, 1, 50, 1e-6, 0.5, 1.0, 1.0,
            testFunc, lower, upper
        );

        pso.solve();

        std::vector<double> best_pos = pso.getBestPosition();

        // Solution should respect bounds
        REQUIRE(best_pos[0] >= lower[0]);
        REQUIRE(best_pos[0] <= upper[0]);

        // Should find minimum at lower bound
        REQUIRE(best_pos[0] == Catch::Approx(1.0).epsilon(1e-2));
    }

    SECTION("Different swarm sizes") {
        auto testFunc = [](double* x) -> double {
            return x[0]*x[0] + x[1]*x[1];
        };

        // Small swarm
        PSO<double, std::function<double(double*)>> small_pso(
            5, 2, 30, 1e-6, 0.5, 1.0, 1.0,
            testFunc, {-2.0, -2.0}, {2.0, 2.0}
        );

        REQUIRE_NOTHROW(small_pso.solve());

        // Large swarm
        PSO<double, std::function<double(double*)>> large_pso(
            100, 2, 30, 1e-6, 0.5, 1.0, 1.0,
            testFunc, {-2.0, -2.0}, {2.0, 2.0}
        );

        REQUIRE_NOTHROW(large_pso.solve());

        // Large swarm should generally perform better
        double small_result = small_pso.getBestValue();
        double large_result = large_pso.getBestValue();

        REQUIRE(large_result <= small_result + 0.1); // Allow some tolerance
    }

    SECTION("PSO parameter sensitivity") {
        auto testFunc = [](double* x) -> double {
            return x[0]*x[0] + x[1]*x[1];
        };

        // High inertia (more exploration)
        PSO<double, std::function<double(double*)>> high_inertia(
            30, 2, 50, 1e-6, 0.9, 1.0, 1.0,
            testFunc, {-5.0, -5.0}, {5.0, 5.0}
        );

        // Low inertia (more exploitation)
        PSO<double, std::function<double(double*)>> low_inertia(
            30, 2, 50, 1e-6, 0.1, 1.0, 1.0,
            testFunc, {-5.0, -5.0}, {5.0, 5.0}
        );

        REQUIRE_NOTHROW(high_inertia.solve());
        REQUIRE_NOTHROW(low_inertia.solve());

        // Both should find reasonable solutions
        REQUIRE(high_inertia.getBestValue() < 5.0);
        REQUIRE(low_inertia.getBestValue() < 5.0);
    }
}

TEST_CASE("PSO - Orbital Transfer Optimization", "[pso][orbital]") {

    SECTION("Simple coplanar transfer") {
        OrbitTransferObjective<double, std::function<double(double*)>> orbitObjective(
            1.0, 1.5, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        );

        std::vector<double> lowerBounds = {0.0, 0.0, 0.1};
        std::vector<double> upperBounds = {2*M_PI, 2*M_PI, 5.0};

        PSO<double, std::function<double(double*)>> pso(
            50, 3, 100, 1e-6, 0.6, 1.5, 1.5,
            orbitObjective, lowerBounds, upperBounds
        );

        REQUIRE_NOTHROW(pso.solve());

        double best_deltaV = pso.getBestValue();
        std::vector<double> best_params = pso.getBestPosition();

        REQUIRE(best_deltaV > 0.0);
        REQUIRE(best_deltaV < 10.0); // Reasonable upper bound
        REQUIRE(best_params.size() == 3);

        // Parameters should be within bounds
        REQUIRE(best_params[0] >= lowerBounds[0]);
        REQUIRE(best_params[0] <= upperBounds[0]);
        REQUIRE(best_params[1] >= lowerBounds[1]);
        REQUIRE(best_params[1] <= upperBounds[1]);
        REQUIRE(best_params[2] >= lowerBounds[2]);
        REQUIRE(best_params[2] <= upperBounds[2]);
    }

    SECTION("Non-coplanar transfer optimization") {
        OrbitTransferObjective<double, std::function<double(double*)>> orbitObjective(
            1.0, 1.8, 3.0,     // LEO to MEO
            0.0, 0.0,          // circular orbits
            0.0, 0.5236,       // 0° to 30° inclination
            0.0, 0.0, 0.0, 0.0
        );

        std::vector<double> lowerBounds = {0.0, 0.0, 1.0};
        std::vector<double> upperBounds = {2*M_PI, 2*M_PI, 8.0};

        PSO<double, std::function<double(double*)>> pso(
            30, 3, 80, 1e-6, 0.6, 1.5, 1.5,
            orbitObjective, lowerBounds, upperBounds
        );

        REQUIRE_NOTHROW(pso.solve());

        double best_deltaV = pso.getBestValue();

        REQUIRE(best_deltaV > 0.0);
        REQUIRE(std::isfinite(best_deltaV));

        // Should be higher than coplanar due to plane change
        REQUIRE(best_deltaV > 1.0);
    }
}

TEST_CASE("PSO - Robustness and Edge Cases", "[pso][edge_cases]") {

    SECTION("Narrow search bounds") {
        auto testFunc = [](double* x) -> double {
            return (x[0] - 1.5) * (x[0] - 1.5);
        };

        // Very narrow bounds around optimum
        PSO<double, std::function<double(double*)>> narrow_pso(
            20, 1, 30, 1e-6, 0.5, 1.0, 1.0,
            testFunc, {1.49}, {1.51}
        );

        REQUIRE_NOTHROW(narrow_pso.solve());

        double best_value = narrow_pso.getBestValue();
        std::vector<double> best_pos = narrow_pso.getBestPosition();

        REQUIRE(best_value < 0.01);
        REQUIRE(abs(best_pos[0] - 1.5) < 0.01);
    }

    SECTION("Multiple local minima function") {
        auto multiModal = [](double* x) -> double {
            return sin(x[0]) * sin(x[0]) + 0.1 * cos(10 * x[0]) + 1.0;
        };

        PSO<double, std::function<double(double*)>> pso(
            50, 1, 100, 1e-6, 0.7, 2.0, 2.0,
            multiModal, {-5.0}, {5.0}
        );

        REQUIRE_NOTHROW(pso.solve());

        double best_value = pso.getBestValue();

        REQUIRE(best_value >= 0.0); // Function minimum is around 0
        REQUIRE(best_value < 2.0);
    }

    SECTION("PSO state consistency") {
        auto testFunc = [](double* x) -> double {
            return x[0]*x[0] + x[1]*x[1];
        };

        PSO<double, std::function<double(double*)>> pso(
            20, 2, 50, 1e-6, 0.5, 1.0, 1.0,
            testFunc, {-2.0, -2.0}, {2.0, 2.0}
        );

        // Multiple solve calls should be consistent
        pso.solve();
        double result1 = pso.getBestValue();

        pso.solve();
        double result2 = pso.getBestValue();

        // Second run should be same or better
        REQUIRE(result2 <= result1);
        REQUIRE(std::isfinite(result1));
        REQUIRE(std::isfinite(result2));
    }
}
