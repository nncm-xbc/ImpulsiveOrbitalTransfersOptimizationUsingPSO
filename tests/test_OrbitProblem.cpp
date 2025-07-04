#include <catch2/catch_all.hpp>
#include "core/OrbitProblem.hpp"
#include <functional>
#include <cmath>
#include <random>

TEST_CASE("OrbitTransferObjective - Basic Functionality", "[orbit_problem][core]") {

    SECTION("Simple coplanar transfer") {
        // LEO to GEO transfer (simplified)
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            1.0, 6.6, 8.0,  // r1=LEO, r2=GEO, transfer_time
            0.0, 0.0,       // e1=0, e2=0 (circular)
            0.0, 0.0,       // i1=0, i2=0 (equatorial)
            0.0, 0.0,       // RAAN1=0, RAAN2=0
            0.0, 0.0        // omega1=0, omega2=0
        );

        std::vector<double> params = {0.0, M_PI, 5.5}; // TA1, TA2, time
        double deltaV = objective(params.data());

        REQUIRE(deltaV > 0.0);
        REQUIRE(deltaV < 1000.0); // Reasonable upper bound
        REQUIRE(std::isfinite(deltaV));
    }

    SECTION("Hohmann transfer approximation") {
        // Classic Hohmann transfer
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            1.0, 1.5, 2.0,  // r1=1, r2=1.5, reasonable time
            0.0, 0.0,       // circular orbits
            0.0, 0.0,       // coplanar
            0.0, 0.0, 0.0, 0.0
        );

        std::vector<double> params = {0.0, M_PI, 1.8}; // Optimal Hohmann timing
        double deltaV = objective(params.data());

        REQUIRE(deltaV > 0.0);
        REQUIRE(deltaV < 5.0);

        // Test different departure positions
        std::vector<double> params2 = {M_PI/2, 3*M_PI/2, 1.8};
        double deltaV2 = objective(params2.data());

        REQUIRE(deltaV2 > 0.0);
        REQUIRE(std::isfinite(deltaV2));
    }

    SECTION("Non-coplanar transfer") {
        // Transfer with inclination change
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            1.0, 1.5, 3.0,     // radii and time
            0.0, 0.0,          // circular
            0.0, 0.5,          // inclination change (0 to ~28.6 deg)
            0.0, 0.0, 0.0, 0.0
        );

        std::vector<double> params = {0.0, M_PI, 2.5};
        double deltaV = objective(params.data());

        REQUIRE(deltaV > 0.0);
        REQUIRE(std::isfinite(deltaV));

        // Should be higher than coplanar transfer due to plane change
        OrbitTransferObjective<double, std::function<double(double*)>> coplanar_obj(
            1.0, 1.5, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        );
        double coplanar_dv = coplanar_obj(params.data());

        REQUIRE(deltaV >= coplanar_dv); // Non-coplanar should cost more
    }
}

TEST_CASE("OrbitTransferObjective - Parameter Validation", "[orbit_problem][validation]") {

    SECTION("Parameter bounds checking") {
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            1.0, 1.5, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        );

        // Test negative true anomaly
        std::vector<double> negative_params = {-0.5, M_PI, 1.5};
        double result1 = objective(negative_params.data());
        REQUIRE(std::isfinite(result1));

        // Test large true anomaly (should wrap)
        std::vector<double> large_params = {3*M_PI, M_PI, 1.5};
        double result2 = objective(large_params.data());
        REQUIRE(std::isfinite(result2));

        // Test very small transfer time
        std::vector<double> small_time = {0.0, M_PI, 0.01};
        double result3 = objective(small_time.data());
        REQUIRE(std::isfinite(result3)); // May be high penalty
    }

    SECTION("Invalid orbital elements") {
        // Test with high eccentricity
        OrbitTransferObjective<double, std::function<double(double*)>> high_ecc_obj(
            1.0, 1.5, 2.0,
            0.9, 0.0,  // Very elliptical initial orbit
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        );

        std::vector<double> params = {0.0, M_PI, 1.5};
        double result = high_ecc_obj(params.data());

        REQUIRE(std::isfinite(result));
        REQUIRE(result >= 0.0);
    }

    SECTION("Constraint violations") {
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            1.0, 1.5, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        );

        // Test various parameter combinations that might violate constraints
        std::vector<std::vector<double>> test_cases = {
            {0.0, 0.0, 0.1},      // Very short time
            {0.0, 2*M_PI, 1.0},   // Full revolution
            {M_PI, 0.0, 1.0},     // Different phase
            {0.0, M_PI, 10.0}     // Very long time
        };

        for (const auto& params : test_cases) {
            std::vector<double> param_copy = params;
            double result = objective(param_copy.data());

            REQUIRE(std::isfinite(result));
            REQUIRE(result >= 0.0);
        }
    }
}

TEST_CASE("OrbitTransferObjective - Edge Cases", "[orbit_problem][edge_cases]") {

    SECTION("Same orbit transfer") {
        // Transfer from orbit to itself
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            1.0, 1.0, 2.0,  // Same radius
            0.0, 0.0,       // Same eccentricity
            0.0, 0.0,       // Same inclination
            0.0, 0.0, 0.0, 0.0
        );

        std::vector<double> params = {0.0, 0.0, 0.1}; // Same position, short time
        double deltaV = objective(params.data());

        REQUIRE(std::isfinite(deltaV));
        // Should be very small for same orbit
        if (deltaV < 100.0) { // If not penalty value
            REQUIRE(deltaV < 1.0);
        }
    }

    SECTION("Large radius ratio") {
        // Earth to Moon distance ratio (~60)
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            1.0, 60.0, 100.0, // Very large ratio
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        );

        std::vector<double> params = {0.0, M_PI, 50.0};
        double deltaV = objective(params.data());

        REQUIRE(std::isfinite(deltaV));
        REQUIRE(deltaV > 0.0);
    }

    SECTION("Multiple evaluations consistency") {
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            1.0, 1.5, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        );

        std::vector<double> params = {M_PI/4, M_PI/2, 1.5};

        // Multiple evaluations should give same result
        double result1 = objective(params.data());
        double result2 = objective(params.data());
        double result3 = objective(params.data());

        REQUIRE(result1 == Catch::Approx(result2).epsilon(1e-12));
        REQUIRE(result2 == Catch::Approx(result3).epsilon(1e-12));
    }
}

TEST_CASE("OrbitTransferObjective - Performance", "[orbit_problem][performance]") {

    SECTION("Rapid evaluation") {
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            1.0, 1.5, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        );

        std::vector<double> params = {0.0, M_PI, 1.5};

        // Time multiple evaluations
        auto start = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < 1000; ++i) {
            double result = objective(params.data());
            REQUIRE(std::isfinite(result));
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        // Should be reasonably fast (less than 1ms per evaluation on average)
        REQUIRE(duration.count() < 1000000); // 1 second for 1000 evaluations
    }

    SECTION("Random parameter robustness") {
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            1.0, 2.0, 5.0, 0.0, 0.0, 0.2, 0.1, 0.0, 0.0, 0.0, 0.0
        );

        // Test with random parameters within reasonable bounds
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> angle_dist(0.0, 2*M_PI);
        std::uniform_real_distribution<> time_dist(0.1, 10.0);

        for (int i = 0; i < 100; ++i) {
            std::vector<double> params = {
                angle_dist(gen),
                angle_dist(gen),
                time_dist(gen)
            };

            double result = objective(params.data());
            REQUIRE(std::isfinite(result));
            REQUIRE(result >= 0.0);
        }
    }
}
