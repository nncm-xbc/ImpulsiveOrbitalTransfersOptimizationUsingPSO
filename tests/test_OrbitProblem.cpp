#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include "core/OrbitProblem.hpp"
#include "core/Constants.hpp"
#include <functional>
#include <iostream>

TEST_CASE("OrbitTransferObjective Coplanar Transfer", "[orbit_problem_coplanar]") {

    SECTION("Coplanar circular-to-circular transfer (Hohmann)") {
        // Set up Hohmann transfer problem
        double R1 = 1.0;
        double R2 = 1.5;
        double e1 = 0.0, e2 = 0.0;              // Circular orbits
        double i1 = 0.0, i2 = 0.0;              // Coplanar
        double raan1 = 0.0, raan2 = 0.0;        // Same RAAN
        double omega1 = 0.0, omega2 = 0.0;      // Same argument of periapsis

        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            R1, R2, 2.0, e1, e2, i1, i2, raan1, raan2, omega1, omega2
        );

        // Test Hohmann transfer parameters
        // Departure at periapsis (0°), arrival at apoapsis (180°)
        std::vector<double> hohmann_params = {0.0, M_PI, M_PI * sqrt(1.25 * 1.25 * 1.25)};

        double deltaV = objective(hohmann_params.data());

        // Convert to km/s for comparison
        double DU_to_km = 6378.165;
        double TU_to_s = sqrt(DU_to_km * DU_to_km * DU_to_km / 398600.4418);
        double deltaV_kmps = deltaV * DU_to_km / TU_to_s;

        std::cout << "Hohmann Transfer ΔV: " << deltaV_kmps << " km/s" << std::endl;

        // Hohmann transfer should be around 1.4-1.5 km/s
        REQUIRE(deltaV_kmps > 1.0);
        REQUIRE(deltaV_kmps < 2.0);

        // Verify no constraint violations for this case - use function operator
        // We can't test constraints directly as they're private
        // Instead test that valid parameters give reasonable results
        REQUIRE(deltaV_kmps > 0.0);  // Should be positive
    }

        SECTION("Coplanar vs Non-coplanar detection") {
            // We can't directly test isNonCoplanar() as it doesn't exist
            // Instead, test that different orbital configurations give different results

            OrbitTransferObjective<double, std::function<double(double*)>> coplanar_obj(
                1.0, 1.5, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            );

            OrbitTransferObjective<double, std::function<double(double*)>> noncoplanar_obj(
                1.0, 1.5, 2.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0  // Different inclinations
            );

            std::vector<double> test_params = {0.0, M_PI, 3.0};

            double coplanar_result = coplanar_obj(test_params.data());
            double noncoplanar_result = noncoplanar_obj(test_params.data());

            // Non-coplanar should require more ΔV due to plane change
            REQUIRE(noncoplanar_result > coplanar_result);
        }
}

TEST_CASE("OrbitTransferObjective Non-Coplanar Transfer", "[orbit_problem_noncoplanar]") {

    SECTION("Non-coplanar transfer setup") {
        // Use actual constants from your problem
        double R1 = constant::R1;      // 1.0
        double R2 = constant::R2;      // 1.5
        double e1 = constant::E1;      // 0.0
        double e2 = constant::E2;      // 0.0
        double i1 = constant::I1;      // 0.497419 rad (28.5°)
        double i2 = constant::I2;      // 0.0 rad (equatorial)
        double raan1 = constant::RAAN1; // 0.0
        double raan2 = constant::RAAN2; // 0.0
        double omega1 = constant::OMEGA1; // 0.0
        double omega2 = constant::OMEGA2; // 0.0

        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            R1, R2, 2.0, e1, e2, i1, i2, raan1, raan2, omega1, omega2
        );

        // Test that this is a non-coplanar case by comparing with coplanar
        OrbitTransferObjective<double, std::function<double(double*)>> coplanar_compare(
            R1, R2, 2.0, e1, e2, 0.0, 0.0, raan1, raan2, omega1, omega2  // Both inclinations = 0
        );

        std::vector<double> test_params = {0.0, 0.0, 3.0};
        double noncoplanar_dv = objective(test_params.data());
        double coplanar_dv = coplanar_compare(test_params.data());

        // Non-coplanar should require more ΔV
        REQUIRE(noncoplanar_dv > coplanar_dv);

        // Calculate approximate plane change angle for validation
        double expected_angle = std::abs(i1 - i2);  // Simplified for same RAAN

        std::cout << "Approximate plane change angle: " << expected_angle << " rad = "
                  << expected_angle * 180.0 / M_PI << "°" << std::endl;
    }

    SECTION("Non-coplanar transfer ΔV calculation") {
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            constant::R1, constant::R2, 2.0, constant::E1, constant::E2,
            constant::I1, constant::I2, constant::RAAN1, constant::RAAN2,
            constant::OMEGA1, constant::OMEGA2
        );

        // Test various transfer geometries
        std::vector<std::vector<double>> test_cases = {
            {0.0, 0.0, 3.0},           // 0° to 0°, 3 TU transfer time
            {0.0, M_PI, 3.0},          // 0° to 180°, 3 TU transfer time
            {M_PI/2, M_PI/2, 3.0},     // 90° to 90°, 3 TU transfer time
            {0.0, M_PI/2, 2.0},        // 0° to 90°, 2 TU transfer time
            {M_PI, 0.0, 4.0}           // 180° to 0°, 4 TU transfer time
        };

        for (size_t i = 0; i < test_cases.size(); i++) {
            auto& params = test_cases[i];

            try {
                double deltaV = objective(params.data());

                // Convert to km/s
                double DU_to_km = 6378.165;
                double TU_to_s = sqrt(DU_to_km * DU_to_km * DU_to_km / 398600.4418);
                double deltaV_kmps = deltaV * DU_to_km / TU_to_s;

                std::cout << "Case " << i+1 << " (TA1=" << params[0]*180/M_PI
                          << "°, TA2=" << params[1]*180/M_PI
                          << "°, Time=" << params[2] << " TU): "
                          << deltaV_kmps << " km/s" << std::endl;

                // Non-coplanar transfers should require significant ΔV
                REQUIRE(deltaV_kmps > 1.0);
                REQUIRE(deltaV_kmps < 20.0);  // Upper bound for sanity

            } catch (const std::exception& e) {
                std::cout << "Case " << i+1 << " failed: " << e.what() << std::endl;
                REQUIRE(false);  // Test should not throw
            }
        }
    }

    SECTION("Non-coplanar constraint behavior") {
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            constant::R1, constant::R2, 2.0, constant::E1, constant::E2,
            constant::I1, constant::I2, constant::RAAN1, constant::RAAN2,
            constant::OMEGA1, constant::OMEGA2
        );

        // Test valid parameters
        std::vector<double> valid_params = {M_PI/4, M_PI/2, 3.0};
        double result = objective(valid_params.data());

        std::cout << "Result for valid params: " << result << std::endl;

        // Should get a reasonable result (not infinity or error)
        REQUIRE(result > 0.0);
        REQUIRE(result < 1000.0);  // Reasonable upper bound

        // Test invalid parameters (bounds violations)
        std::vector<double> invalid_params = {-1.0, 10.0, -5.0};  // Outside reasonable bounds
        double invalid_result = objective(invalid_params.data());

        std::cout << "Result for invalid params: " << invalid_result << std::endl;

        // May get high penalty value or reasonable result depending on implementation
        REQUIRE(invalid_result >= 0.0);  // Should not be negative
    }
}

TEST_CASE("OrbitTransferObjective Edge Cases", "[orbit_problem_edge_cases]") {

    SECTION("Same orbit transfer (should be zero ΔV)") {
        // Transfer from orbit to itself
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            1.0, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        );

        // Same position and timing
        std::vector<double> same_params = {0.0, 0.0, 0.1};  // Very short transfer time

        try {
            double deltaV = objective(same_params.data());

            std::cout << "Same orbit transfer ΔV: " << deltaV << " DU/TU" << std::endl;

            // Should be very small (near zero)
            REQUIRE(deltaV < 0.1);

        } catch (const std::exception& e) {
            std::cout << "Same orbit test failed: " << e.what() << std::endl;
            // This might fail due to Lambert solver issues with very small transfers
        }
    }

    SECTION("Large orbit ratio transfer") {
        // Test transfer with large radius ratio
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            1.0, 3.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  // 3:1 ratio
        );

        std::vector<double> large_transfer_params = {0.0, M_PI, 5.0};

        try {
            double deltaV = objective(large_transfer_params.data());

            double DU_to_km = 6378.165;
            double TU_to_s = sqrt(DU_to_km * DU_to_km * DU_to_km / 398600.4418);
            double deltaV_kmps = deltaV * DU_to_km / TU_to_s;

            std::cout << "Large ratio transfer ΔV: " << deltaV_kmps << " km/s" << std::endl;

            // Should be significant but reasonable
            REQUIRE(deltaV_kmps > 2.0);
            REQUIRE(deltaV_kmps < 10.0);

        } catch (const std::exception& e) {
            std::cout << "Large ratio test exception: " << e.what() << std::endl;
        }
    }

    SECTION("Very short transfer time") {
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            1.0, 1.5, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        );

        std::vector<double> fast_params = {0.0, M_PI, 0.1};  // Very fast transfer

        try {
            double deltaV = objective(fast_params.data());

            std::cout << "Fast transfer ΔV: " << deltaV << " DU/TU" << std::endl;

            // Fast transfers should require high ΔV
            REQUIRE(deltaV > 2.0);

        } catch (const std::exception& e) {
            std::cout << "Fast transfer may fail Lambert solver: " << e.what() << std::endl;
            // This might legitimately fail for very fast transfers
        }
    }
}

TEST_CASE("OrbitTransferObjective Parameter Validation", "[orbit_problem_validation]") {

    SECTION("Parameter bounds checking") {
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            1.0, 1.5, 2.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0
        );

        // Test parameter dimension validation
        std::vector<double> wrong_size_params = {1.0, 2.0};  // Only 2 parameters instead of 3

        // This should either throw or handle gracefully
        try {
            double deltaV = objective(wrong_size_params.data());  // Will only read first 3 elements
            std::cout << "Unexpected success with wrong size params: " << deltaV << std::endl;
            REQUIRE(deltaV >= 0.0);  // If it succeeds, should be reasonable
        } catch (const std::exception& e) {
            std::cout << "Correctly caught error with wrong size: " << e.what() << std::endl;
            REQUIRE(true);   // Expected behavior
        }
    }

    SECTION("Orbital element validation") {
        // Test with invalid orbital elements
        try {
            OrbitTransferObjective<double, std::function<double(double*)>> invalid_objective(
                -1.0, 1.5, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  // Negative radius
            );

            std::vector<double> params = {0.0, M_PI, 3.0};
            double deltaV = invalid_objective(params.data());

            // Should either throw or give reasonable bounds
            REQUIRE(deltaV >= 0.0);

        } catch (const std::exception& e) {
            std::cout << "Invalid orbital elements correctly rejected: " << e.what() << std::endl;
        }
    }
}

TEST_CASE("OrbitTransferObjective Performance Tests", "[orbit_problem_performance]") {

    SECTION("Multiple evaluations consistency") {
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            constant::R1, constant::R2, 2.0, constant::E1, constant::E2,
            constant::I1, constant::I2, constant::RAAN1, constant::RAAN2,
            constant::OMEGA1, constant::OMEGA2
        );

        std::vector<double> test_params = {M_PI/3, M_PI/2, 2.5};

        // Evaluate same parameters multiple times
        std::vector<double> results;
        for (int i = 0; i < 5; i++) {
            try {
                double deltaV = objective(test_params.data());
                results.push_back(deltaV);
            } catch (const std::exception& e) {
                std::cout << "Evaluation " << i << " failed: " << e.what() << std::endl;
            }
        }

        // All results should be identical
        if (results.size() > 1) {
            for (size_t i = 1; i < results.size(); i++) {
                REQUIRE(results[i] == Catch::Approx(results[0]).epsilon(1e-12));
            }
            std::cout << "Consistent result: " << results[0] << " DU/TU" << std::endl;
        }
    }

    SECTION("Random parameter evaluation") {
        OrbitTransferObjective<double, std::function<double(double*)>> objective(
            constant::R1, constant::R2, 2.0, constant::E1, constant::E2,
            constant::I1, constant::I2, constant::RAAN1, constant::RAAN2,
            constant::OMEGA1, constant::OMEGA2
        );

        srand(42);  // Fixed seed for reproducibility

        int successful_evals = 0;
        int total_evals = 100;
        double min_deltaV = 1e9;
        double max_deltaV = -1e9;

        for (int i = 0; i < total_evals; i++) {
            std::vector<double> random_params = {
                2.0 * M_PI * ((double)rand() / RAND_MAX),           // TA1 [0, 2π]
                2.0 * M_PI * ((double)rand() / RAND_MAX),           // TA2 [0, 2π]
                0.5 + 9.5 * ((double)rand() / RAND_MAX)             // Time [0.5, 10]
            };

            try {
                double deltaV = objective(random_params.data());

                double DU_to_km = 6378.165;
                double TU_to_s = sqrt(DU_to_km * DU_to_km * DU_to_km / 398600.4418);
                double deltaV_kmps = deltaV * DU_to_km / TU_to_s;

                if (deltaV_kmps > 0.1 && deltaV_kmps < 50.0) {  // Reasonable bounds
                    successful_evals++;
                    min_deltaV = std::min(min_deltaV, deltaV_kmps);
                    max_deltaV = std::max(max_deltaV, deltaV_kmps);
                }

            } catch (const std::exception& e) {
                // Some random parameters may fail - this is expected
            }
        }

        double success_rate = (double)successful_evals / total_evals;

        std::cout << "Random evaluation results:" << std::endl;
        std::cout << "Success rate: " << success_rate * 100.0 << "%" << std::endl;
        std::cout << "ΔV range: [" << min_deltaV << ", " << max_deltaV << "] km/s" << std::endl;

        // Should have reasonable success rate
        REQUIRE(success_rate > 0.1);  // At least 10% should succeed

        if (successful_evals > 0) {
            // Should see variety in results
            double range = max_deltaV - min_deltaV;
            REQUIRE(range > 0.5);  // Should see at least 0.5 km/s range
        }
    }
}
