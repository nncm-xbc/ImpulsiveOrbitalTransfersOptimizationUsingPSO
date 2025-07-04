#include <catch2/catch_all.hpp>
#include "core/OrbitMechanics.hpp"
#include <cmath>
#include <chrono>
#include <random>

using namespace Physics;

TEST_CASE("OrbitMechanics - Basic Position Calculations", "[orbit_mechanics][core]") {

    SECTION("Circular orbit position") {
        double radius = 1.0;
        double inclination = 0.0;
        double raan = 0.0;
        double eccentricity = 0.0;
        double arg_periapsis = 0.0;
        double true_anomaly = 0.0;

        Vector3 pos = OrbitMechanics::calculatePosition3D(
            radius, eccentricity, inclination, raan, arg_periapsis, true_anomaly
        );

        // At true anomaly 0, should be at periapsis (for circular orbit, at distance = radius)
        REQUIRE(pos.x == Catch::Approx(radius).epsilon(1e-6));
        REQUIRE(pos.y == Catch::Approx(0.0).epsilon(1e-6));
        REQUIRE(pos.z == Catch::Approx(0.0).epsilon(1e-6));
    }

    SECTION("Circular orbit at 90 degrees") {
        double radius = 1.0;
        double true_anomaly = M_PI/2;

        Vector3 pos = OrbitMechanics::calculatePosition3D(
            radius, 0.0, 0.0, 0.0, 0.0, true_anomaly
        );

        // At 90 degrees, should be on y-axis
        REQUIRE(pos.x == Catch::Approx(0.0).epsilon(1e-6));
        REQUIRE(pos.y == Catch::Approx(radius).epsilon(1e-6));
        REQUIRE(pos.z == Catch::Approx(0.0).epsilon(1e-6));
    }

    SECTION("Position magnitude consistency") {
        double radius = 2.5;
        double eccentricity = 0.3;
        double true_anomaly = M_PI/4;

        Vector3 pos = OrbitMechanics::calculatePosition3D(
            radius, eccentricity, 0.0, 0.0, 0.0, true_anomaly
        );

        double expected_r = OrbitMechanics::calculateRadius(radius, eccentricity, true_anomaly);
        double actual_r = pos.magnitude();

        REQUIRE(actual_r == Catch::Approx(expected_r).epsilon(1e-6));
    }
}

TEST_CASE("OrbitMechanics - Radius Calculations", "[orbit_mechanics][core]") {

    SECTION("Circular orbit radius") {
        double semi_major = 1.0;
        double eccentricity = 0.0;

        for (double theta = 0; theta < 2*M_PI; theta += M_PI/4) {
            double r = OrbitMechanics::calculateRadius(semi_major, eccentricity, theta);
            REQUIRE(r == Catch::Approx(semi_major).epsilon(1e-6));
        }
    }

    SECTION("Elliptical orbit extremes") {
        double semi_major = 2.0;
        double eccentricity = 0.5;

        // At periapsis (true anomaly = 0)
        double r_peri = OrbitMechanics::calculateRadius(semi_major, eccentricity, 0.0);
        double expected_peri = semi_major * (1 - eccentricity);
        REQUIRE(r_peri == Catch::Approx(expected_peri).epsilon(1e-6));

        // At apoapsis (true anomaly = π)
        double r_apo = OrbitMechanics::calculateRadius(semi_major, eccentricity, M_PI);
        double expected_apo = semi_major * (1 + eccentricity);
        REQUIRE(r_apo == Catch::Approx(expected_apo).epsilon(1e-6));
    }
}

TEST_CASE("OrbitMechanics - Velocity Calculations", "[orbit_mechanics][core]") {

    SECTION("Circular orbit velocity magnitude") {
        double radius = 1.0;
        double mu = 1.0; // From constants

        Vector3 velocity = OrbitMechanics::calculateVelocity3D(
            radius, 0.0, 0.0, 0.0, 0.0, 0.0
        );

        double speed = velocity.magnitude();
        double expected_speed = sqrt(mu / radius);

        REQUIRE(speed == Catch::Approx(expected_speed).epsilon(1e-6));
    }

    SECTION("Velocity perpendicular to position for circular orbits") {
        double radius = 1.5;

        Vector3 pos = OrbitMechanics::calculatePosition3D(radius, 0.0, 0.0, 0.0, 0.0, 0.0);
        Vector3 vel = OrbitMechanics::calculateVelocity3D(radius, 0.0, 0.0, 0.0, 0.0, 0.0);

        // Dot product should be zero for circular orbits
        double dot_product = pos.dot(vel);
        REQUIRE(dot_product == Catch::Approx(0.0).epsilon(1e-6));
    }
}

TEST_CASE("OrbitMechanics - Impulse Calculations", "[orbit_mechanics][core]") {

    SECTION("Simple impulse vector calculation") {
        Vector3 v_initial(0.0, 1.0, 0.0);  // Circular velocity
        Vector3 v_final(0.0, 1.414, 0.0);  // Higher circular velocity

        Vector3 impulse = v_final - v_initial; // Simple delta-V calculation

        // Impulse should be in velocity direction
        REQUIRE(impulse.x == Catch::Approx(0.0).epsilon(1e-6));
        REQUIRE(impulse.y == Catch::Approx(0.414).epsilon(1e-3));
        REQUIRE(impulse.z == Catch::Approx(0.0).epsilon(1e-6));
    }

    SECTION("Zero impulse for same velocity") {
        Vector3 velocity(1.0, 2.0, 3.0);

        Vector3 impulse = velocity - velocity; // Same velocity

        REQUIRE(impulse.x == Catch::Approx(0.0).epsilon(1e-6));
        REQUIRE(impulse.y == Catch::Approx(0.0).epsilon(1e-6));
        REQUIRE(impulse.z == Catch::Approx(0.0).epsilon(1e-6));
    }
}

TEST_CASE("OrbitMechanics - Edge Cases", "[orbit_mechanics][edge_cases]") {

    SECTION("High eccentricity orbit") {
        double semi_major = 1.0;
        double eccentricity = 0.95; // Very elliptical

        // Should not crash and give reasonable values
        double r_peri = OrbitMechanics::calculateRadius(semi_major, eccentricity, 0.0);
        double r_apo = OrbitMechanics::calculateRadius(semi_major, eccentricity, M_PI);

        REQUIRE(r_peri > 0.0);
        REQUIRE(r_apo > r_peri);
        REQUIRE(r_peri == Catch::Approx(0.05).epsilon(1e-6));
        REQUIRE(r_apo == Catch::Approx(1.95).epsilon(1e-6));
    }

    SECTION("Inclined orbit position") {
        double inclination = M_PI/4; // 45 degrees

        Vector3 pos = OrbitMechanics::calculatePosition3D(
            1.0, 0.0, inclination, 0.0, 0.0, M_PI/2
        );

        // Should have z-component due to inclination
        REQUIRE(abs(pos.z) > 1e-6);

        // Total radius should still be correct
        double total_r = pos.magnitude();
        REQUIRE(total_r == Catch::Approx(1.0).epsilon(1e-6));
    }
}
