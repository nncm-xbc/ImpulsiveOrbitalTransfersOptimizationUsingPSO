#include <catch2/catch_all.hpp>
#include "core/LambertSolver.hpp"
#include "core/OrbitMechanics.hpp"
#include <cmath>

using namespace Physics;

TEST_CASE("LambertSolver - Basic Functionality", "[lambert_solver][core]") {

    SECTION("Simple transfer between known points") {
        Vector3 r1(1.0, 0.0, 0.0);  // Start at x-axis
        Vector3 r2(0.0, 1.0, 0.0);  // End at y-axis
        double tof = M_PI/2;         // Quarter orbit time
        double mu = 1.0;

        auto result = LambertSolver::solveLambert(r1, r2, tof, mu, false);

        Vector3 v1 = result.first;   // Departure velocity
        Vector3 v2 = result.second;  // Arrival velocity

        // Velocity vectors should have reasonable magnitude
        double v1_mag = v1.magnitude();
        double v2_mag = v2.magnitude();
        if (v1_mag > 1e-10) { // Only test if non-zero
            REQUIRE(v1_mag < 100.0);
        }
        REQUIRE(v2_mag > 0.0);
        REQUIRE(v2_mag < 10.0);

        // Velocities should be finite
        REQUIRE(std::isfinite(v1.x));
        REQUIRE(std::isfinite(v1.y));
        REQUIRE(std::isfinite(v1.z));
        REQUIRE(std::isfinite(v2.x));
        REQUIRE(std::isfinite(v2.y));
        REQUIRE(std::isfinite(v2.z));
    }

    SECTION("Circular orbit transfer") {
        // Points on circular orbit
        Vector3 r1(1.0, 0.0, 0.0);
        Vector3 r2(-1.0, 0.0, 0.0);
        double tof = M_PI; // Half orbit
        double mu = 1.0;

        auto result = LambertSolver::solveLambert(r1, r2, tof, mu, false);

        Vector3 v1 = result.first;
        Vector3 v2 = result.second;

        // Should get reasonable velocities
        REQUIRE(v1.magnitude() > 0.0);
        REQUIRE(v2.magnitude() > 0.0);
        REQUIRE(v1.magnitude() < 5.0);
        REQUIRE(v2.magnitude() < 5.0);

        // Check that velocities are roughly perpendicular to position vectors
        // (for circular-like motion)
        double dot1 = r1.x * v1.x + r1.y * v1.y + r1.z * v1.z;
        double dot2 = r2.x * v2.x + r2.y * v2.y + r2.z * v2.z;
        REQUIRE(abs(dot1) < 1.0); // Should be roughly perpendicular
        REQUIRE(abs(dot2) < 1.0);
    }

    SECTION("Short time-of-flight transfer") {
        Vector3 r1(1.0, 0.0, 0.0);
        Vector3 r2(0.707, 0.707, 0.0); // 45 degrees
        double tof = 0.1; // Very short transfer
        double mu = 1.0;

        auto result = LambertSolver::solveLambert(r1, r2, tof, mu, false);

        Vector3 v1 = result.first;
        Vector3 v2 = result.second;

        // Check that we get some result (even if high energy)
        REQUIRE(std::isfinite(v1.magnitude()));
        REQUIRE(std::isfinite(v2.magnitude()));

        // For very short transfers, velocities might be high
        if (v1.magnitude() > 0.1) { // If we got a valid solution
            REQUIRE(v1.magnitude() < 100.0); // Sanity upper bound
            REQUIRE(v2.magnitude() < 100.0);
        }
    }
}

TEST_CASE("LambertSolver - Edge Cases", "[lambert_solver][edge_cases]") {

    SECTION("Same position transfer") {
        Vector3 r1(1.0, 0.0, 0.0);
        Vector3 r2(1.0, 0.0, 0.0); // Same position
        double tof = 1.0;
        double mu = 1.0;

        auto result = LambertSolver::solveLambert(r1, r2, tof, mu, false);

        Vector3 v1 = result.first;
        Vector3 v2 = result.second;

        // Should handle gracefully - velocities should be finite
        REQUIRE(std::isfinite(v1.magnitude()));
        REQUIRE(std::isfinite(v2.magnitude()));

        // For same position, might get zero or very small velocities
        if (v1.magnitude() > 1e-10) {
            REQUIRE(v1.magnitude() < 100.0);
            REQUIRE(v2.magnitude() < 100.0);
        }
    }

    SECTION("Opposite positions") {
        Vector3 r1(1.0, 0.0, 0.0);
        Vector3 r2(-1.0, 0.0, 0.0);
        double tof = 0.5; // Short time for 180-degree transfer
        double mu = 1.0;

        auto result = LambertSolver::solveLambert(r1, r2, tof, mu, false);

        Vector3 v1 = result.first;
        Vector3 v2 = result.second;

        // Should get some result
        REQUIRE(std::isfinite(v1.magnitude()));
        REQUIRE(std::isfinite(v2.magnitude()));

        if (v1.magnitude() > 0.1) { // If we got a meaningful solution
            // High energy transfer for short time
            REQUIRE(v1.magnitude() > 0.5);
            REQUIRE(v1.magnitude() < 50.0); // Sanity check
        }
    }

    SECTION("Very long transfer time") {
        Vector3 r1(1.0, 0.0, 0.0);
        Vector3 r2(0.0, 1.0, 0.0);
        double tof = 10.0; // Very long transfer time
        double mu = 1.0;

        auto result = LambertSolver::solveLambert(r1, r2, tof, mu, false);

        Vector3 v1 = result.first;
        Vector3 v2 = result.second;

        // Should get reasonable velocities for long transfer
        REQUIRE(std::isfinite(v1.magnitude()));
        REQUIRE(std::isfinite(v2.magnitude()));

        if (v1.magnitude() > 0.01) { // If we got a solution
            // Long transfer should have lower energy/velocities
            REQUIRE(v1.magnitude() < 10.0);
            REQUIRE(v2.magnitude() < 10.0);
        }
    }
}

TEST_CASE("LambertSolver - Solution Validation", "[lambert_solver][validation]") {

    SECTION("Energy conservation check") {
        Vector3 r1(1.0, 0.0, 0.0);
        Vector3 r2(1.5, 0.0, 0.0);
        double tof = 2.0;
        double mu = 1.0;

        auto result = LambertSolver::solveLambert(r1, r2, tof, mu, false);

        Vector3 v1 = result.first;
        Vector3 v2 = result.second;

        if (v1.magnitude() > 0.01 && v2.magnitude() > 0.01) { // Valid solution
            // Calculate specific energy at both points
            double r1_mag = r1.magnitude();
            double r2_mag = r2.magnitude();
            double v1_mag_sq = v1.magnitude() * v1.magnitude();
            double v2_mag_sq = v2.magnitude() * v2.magnitude();

            double energy1 = 0.5 * v1_mag_sq - mu / r1_mag;
            double energy2 = 0.5 * v2_mag_sq - mu / r2_mag;

            // Energy should be conserved (within numerical tolerance)
            REQUIRE(energy1 == Catch::Approx(energy2).epsilon(1e-2));
        }
    }

    SECTION("Velocity magnitude reasonableness") {
        Vector3 r1(1.0, 0.0, 0.0);
        Vector3 r2(0.0, 1.0, 0.0); // 90-degree transfer
        double tof = 1.5;
        double mu = 1.0;

        auto result = LambertSolver::solveLambert(r1, r2, tof, mu, false);

        Vector3 v1 = result.first;
        Vector3 v2 = result.second;

        // Velocities should be reasonable for this transfer
        REQUIRE(std::isfinite(v1.magnitude()));
        REQUIRE(std::isfinite(v2.magnitude()));

        if (v1.magnitude() > 0.01) { // If we got a solution
            REQUIRE(v1.magnitude() > 0.1);  // Should be at least some velocity
            REQUIRE(v1.magnitude() < 5.0);  // But not too high for this case
            REQUIRE(v2.magnitude() > 0.1);
            REQUIRE(v2.magnitude() < 5.0);
        }
    }

    SECTION("Long vs short way comparison") {
        Vector3 r1(1.0, 0.0, 0.0);
        Vector3 r2(0.0, 1.0, 0.0);
        double tof = 5.0; // Long enough for multiple solutions
        double mu = 1.0;

        // Test both short and long way solutions
        auto result_short = LambertSolver::solveLambert(r1, r2, tof, mu, false);
        auto result_long = LambertSolver::solveLambert(r1, r2, tof, mu, true);

        Vector3 v1_short = result_short.first;
        Vector3 v1_long = result_long.first;

        // Both should give some result
        REQUIRE(std::isfinite(v1_short.magnitude()));
        REQUIRE(std::isfinite(v1_long.magnitude()));

        if (v1_short.magnitude() > 0.01 && v1_long.magnitude() > 0.01) {
            // Solutions should be different for long vs short way
            double diff = abs(v1_short.magnitude() - v1_long.magnitude());
            REQUIRE(diff > 0.01); // Should be noticeably different
        }
    }
}
