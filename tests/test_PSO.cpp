#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include "optimization/PSO.hpp"
#include "core/OrbitProblem.hpp"
#include <functional>
#include <cmath>
#include <iostream>

// Simple test functions for PSO validation
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

TEST_CASE("PSO Basic Algorithm Tests", "[pso_basic]") {

    SECTION("PSO constructor and solve execution") {
        // Create wrapper function that converts double* to vector<double>
        auto testFunc = [](double* x) -> double {
            std::vector<double> vec(x, x + 2);  // Convert double* to vector
            return sphereFunction(vec);
        };

        size_t numParticles = 20;
        size_t dimension = 2;
        size_t maxIterations = 100;
        double tolerance = 1e-6;
        double inertiaWeight = 0.7;
        double cognitiveWeight = 1.5;
        double socialWeight = 1.5;

        std::vector<double> lowerBounds = {-5.0, -5.0};
        std::vector<double> upperBounds = {5.0, 5.0};

        PSO<double, std::function<double(double*)>> pso(
            numParticles, dimension, maxIterations, tolerance,
            inertiaWeight, cognitiveWeight, socialWeight,
            testFunc, lowerBounds, upperBounds
        );

        // Test that PSO runs without crashing
        pso.solve();

        std::cout << "PSO sphere function test completed successfully" << std::endl;
    }

    SECTION("PSO with different problem sizes") {
        std::vector<size_t> dimensions = {1, 2, 3, 5};

        for (size_t dim : dimensions) {
            // Create wrapper function for this dimension
            auto testFunc = [dim](double* x) -> double {
                std::vector<double> vec(x, x + dim);  // Convert double* to vector
                return sphereFunction(vec);
            };

            std::vector<double> lowerBounds(dim, -10.0);
            std::vector<double> upperBounds(dim, 10.0);

            PSO<double, std::function<double(double*)>> pso(
                30, dim, 50, 1e-6, 0.7, 1.5, 1.5,
                testFunc, lowerBounds, upperBounds
            );

            // Should execute without error
            pso.solve();

            std::cout << "PSO test with dimension " << dim << " completed" << std::endl;
        }
    }
}

TEST_CASE("PSO Parameter Validation", "[pso_parameters]") {

    SECTION("PSO with various parameter combinations") {
        // Create wrapper function that converts double* to vector<double>
        auto testFunc = [](double* x) -> double {
            std::vector<double> vec(x, x + 2);  // Convert double* to vector
            return sphereFunction(vec);
        };

        struct PSOParams {
            size_t particles;
            double inertia;
            double cognitive;
            double social;
        };

        std::vector<PSOParams> paramSets = {
            {10, 0.5, 1.0, 1.0},
            {50, 0.7, 1.5, 1.5},
            {20, 0.9, 2.0, 2.0},
            {100, 0.4, 1.2, 1.8}
        };

        for (const auto& params : paramSets) {
            PSO<double, std::function<double(double*)>> pso(
                params.particles, 2, 100, 1e-6,
                params.inertia, params.cognitive, params.social,
                testFunc, {-5.0, -5.0}, {5.0, 5.0}
            );

            pso.solve();

            std::cout << "PSO test with " << params.particles << " particles completed" << std::endl;
        }
    }

    SECTION("PSO with boundary cases") {
        // Create wrapper function that converts double* to vector<double>
        auto testFunc = [](double* x) -> double {
            std::vector<double> vec(x, x + 2);  // Convert double* to vector
            return sphereFunction(vec);
        };

        // Test very narrow bounds
        PSO<double, std::function<double(double*)>> narrow_pso(
            20, 2, 50, 1e-6, 0.5, 1.0, 1.0,
            testFunc, {1.0, 1.0}, {1.001, 1.001}
        );

        narrow_pso.solve();

        // Test wide bounds
        PSO<double, std::function<double(double*)>> wide_pso(
            20, 2, 50, 1e-6, 0.5, 1.0, 1.0,
            testFunc, {-100.0, -100.0}, {100.0, 100.0}
        );

        wide_pso.solve();

        std::cout << "PSO boundary case tests completed" << std::endl;
    }
}

TEST_CASE("PSO with Orbital Transfer Problems", "[pso_orbital]") {

    SECTION("PSO with coplanar orbital transfer") {
        // Set up coplanar transfer problem
        OrbitTransferObjective<double, std::function<double(double*)>> orbitObjective(
            1.0, 1.5, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        );

        // PSO bounds for orbital transfer
        std::vector<double> lowerBounds = {0.0, 0.0, 0.1};      // TA1, TA2, transfer time
        std::vector<double> upperBounds = {2*M_PI, 2*M_PI, 10.0};

        PSO<double, std::function<double(double*)>> pso(
            30, 3, 100, 1e-6, 0.6, 1.8, 1.8,
            orbitObjective, lowerBounds, upperBounds
        );

        // Should execute without error
        pso.solve();

        std::cout << "PSO coplanar orbital transfer test completed" << std::endl;
    }

    SECTION("PSO with non-coplanar orbital transfer") {
        // Set up non-coplanar transfer problem (your actual problem)
        OrbitTransferObjective<double, std::function<double(double*)>> orbitObjective(
            1.0, 1.5, 2.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0  // 28.6° inclination difference
        );

        std::vector<double> lowerBounds = {0.0, 0.0, 0.1};
        std::vector<double> upperBounds = {2*M_PI, 2*M_PI, 10.0};

        PSO<double, std::function<double(double*)>> pso(
            50, 3, 200, 1e-6, 0.6, 1.8, 1.8,
            orbitObjective, lowerBounds, upperBounds
        );

        // This is the critical test - should execute without instant convergence
        pso.solve();

        std::cout << "PSO non-coplanar orbital transfer test completed" << std::endl;
        std::cout << "If this test passes, PSO can handle your problem without crashing" << std::endl;
    }
}

TEST_CASE("PSO Execution Diagnostics", "[pso_diagnostics]") {

    SECTION("PSO multiple execution consistency") {
        // Create wrapper function that converts double* to vector<double>
        auto testFunc = [](double* x) -> double {
            std::vector<double> vec(x, x + 2);  // Convert double* to vector
            return sphereFunction(vec);
        };

        int numRuns = 3;
        bool allRunsSucceeded = true;

        for (int run = 0; run < numRuns; run++) {
            try {
                PSO<double, std::function<double(double*)>> pso(
                    20, 2, 50, 1e-6, 0.7, 1.5, 1.5,
                    testFunc, {-5.0, -5.0}, {5.0, 5.0}
                );

                pso.solve();
                std::cout << "Run " << (run + 1) << " completed successfully" << std::endl;

            } catch (const std::exception& e) {
                std::cout << "Run " << (run + 1) << " failed: " << e.what() << std::endl;
                allRunsSucceeded = false;
            }
        }

        REQUIRE(allRunsSucceeded);
    }

    SECTION("PSO with problematic objective function") {
        // Function that returns NaN for some inputs
        auto problematicFunc = [](double* x) -> double {
            if (x[0] < 0.1 && x[1] < 0.1) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            return x[0]*x[0] + x[1]*x[1];
        };

        PSO<double, std::function<double(double*)>> pso(
            20, 2, 50, 1e-6, 0.7, 1.5, 1.5,
            problematicFunc, {0.0, 0.0}, {2.0, 2.0}
        );

        // Should handle NaN gracefully without crashing
        pso.solve();

        std::cout << "PSO handled problematic function without crashing" << std::endl;
    }
}

TEST_CASE("PSO Instant Convergence Debug", "[pso_convergence_debug]") {

    SECTION("Test that reveals instant convergence behavior") {
        // Use orbital transfer to test instant convergence
        OrbitTransferObjective<double, std::function<double(double*)>> orbitObjective(
            1.0, 1.5, 2.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0
        );

        std::vector<double> lowerBounds = {0.0, 0.0, 0.1};
        std::vector<double> upperBounds = {2*M_PI, 2*M_PI, 10.0};

        // Test with fewer iterations to see early behavior
        PSO<double, std::function<double(double*)>> pso(
            100, 3, 10, 1e-6, 0.6, 1.8, 1.8,  // Only 10 iterations
            orbitObjective, lowerBounds, upperBounds
        );

        pso.solve();

        std::cout << "SHORT PSO run completed - check if it finds reasonable solution in just 10 iterations" << std::endl;
        std::cout << "If it does, then instant convergence issue is confirmed" << std::endl;

        // Now test with more iterations
        PSO<double, std::function<double(double*)>> pso_long(
            100, 3, 1000, 1e-6, 0.6, 1.8, 1.8,  // 1000 iterations
            orbitObjective, lowerBounds, upperBounds
        );

        pso_long.solve();

        std::cout << "LONG PSO run completed - compare results with short run" << std::endl;
    }
}
