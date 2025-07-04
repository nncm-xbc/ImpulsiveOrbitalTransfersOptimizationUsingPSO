#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include "core/OrbitMechanics.hpp"
#include "core/Constants.hpp"
#include <cmath>
#include <iostream>

TEST_CASE("OrbitMechanics Basic Calculations", "[orbit_mechanics]") {

    SECTION("Calculate radius for circular orbit") {
        double a = 1.0;  // Semi-major axis
        double e = 0.0;  // Circular orbit
        double true_anomaly = 0.0;  // At periapsis

        double radius = Physics::OrbitMechanics::calculateRadius(a, e, true_anomaly);

        REQUIRE(radius == Catch::Approx(1.0).epsilon(1e-10));
    }

    SECTION("Calculate radius for elliptical orbit") {
        double a = 1.0;
        double e = 0.5;  // Elliptical

        // At periapsis (true anomaly = 0)
        double r_periapsis = Physics::OrbitMechanics::calculateRadius(a, e, 0.0);
        double expected_periapsis = a * (1.0 - e);
        REQUIRE(r_periapsis == Catch::Approx(expected_periapsis).epsilon(1e-10));

        // At apoapsis (true anomaly = π)
        double r_apoapsis = Physics::OrbitMechanics::calculateRadius(a, e, M_PI);
        double expected_apoapsis = a * (1.0 + e);
        REQUIRE(r_apoapsis == Catch::Approx(expected_apoapsis).epsilon(1e-10));
    }

    SECTION("Calculate 2D velocity for circular orbit") {
        double a = 1.0;
        double e = 0.0;
        double true_anomaly = 0.0;

        auto velocity = Physics::OrbitMechanics::calculateVelocity(a, e, true_anomaly);
        double v_radial = velocity.first;
        double v_tangential = velocity.second;

        // For circular orbit, radial velocity should be zero
        REQUIRE(v_radial == Catch::Approx(0.0).epsilon(1e-10));

        // Tangential velocity should match orbital speed
        double expected_v_tangential = sqrt(constant::MU / a);
        REQUIRE(v_tangential == Catch::Approx(expected_v_tangential).epsilon(1e-10));
    }
}

TEST_CASE("OrbitMechanics 3D Position Calculations", "[orbit_mechanics_3d]") {

    SECTION("3D position for equatorial orbit") {
        double a = 1.0;
        double e = 0.0;
        double i = 0.0;      // Equatorial
        double raan = 0.0;
        double omega = 0.0;

        // Test multiple true anomalies
        Physics::Vector3 pos_0 = Physics::OrbitMechanics::calculatePosition3D(a, e, i, raan, omega, 0.0);
        Physics::Vector3 pos_90 = Physics::OrbitMechanics::calculatePosition3D(a, e, i, raan, omega, M_PI/2);
        Physics::Vector3 pos_180 = Physics::OrbitMechanics::calculatePosition3D(a, e, i, raan, omega, M_PI);
        Physics::Vector3 pos_270 = Physics::OrbitMechanics::calculatePosition3D(a, e, i, raan, omega, 3*M_PI/2);

        // Check positions
        REQUIRE(pos_0.x == Catch::Approx(1.0).epsilon(1e-10));
        REQUIRE(pos_0.y == Catch::Approx(0.0).epsilon(1e-10));
        REQUIRE(pos_0.z == Catch::Approx(0.0).epsilon(1e-10));

        REQUIRE(pos_90.x == Catch::Approx(0.0).epsilon(1e-10));
        REQUIRE(pos_90.y == Catch::Approx(1.0).epsilon(1e-10));
        REQUIRE(pos_90.z == Catch::Approx(0.0).epsilon(1e-10));

        REQUIRE(pos_180.x == Catch::Approx(-1.0).epsilon(1e-10));
        REQUIRE(pos_180.y == Catch::Approx(0.0).epsilon(1e-10));
        REQUIRE(pos_180.z == Catch::Approx(0.0).epsilon(1e-10));

        REQUIRE(pos_270.x == Catch::Approx(0.0).epsilon(1e-10));
        REQUIRE(pos_270.y == Catch::Approx(-1.0).epsilon(1e-10));
        REQUIRE(pos_270.z == Catch::Approx(0.0).epsilon(1e-10));
    }

    SECTION("3D position for inclined orbit") {
        double a = 1.0;
        double e = 0.0;
        double i = M_PI/4;   // 45° inclination
        double raan = 0.0;
        double omega = 0.0;

        // At true anomaly = 0° (should be in XZ plane)
        Physics::Vector3 pos_0 = Physics::OrbitMechanics::calculatePosition3D(a, e, i, raan, omega, 0.0);
        REQUIRE(pos_0.x == Catch::Approx(1.0).epsilon(1e-10));
        REQUIRE(pos_0.y == Catch::Approx(0.0).epsilon(1e-10));
        REQUIRE(pos_0.z == Catch::Approx(0.0).epsilon(1e-10));

        // At true anomaly = 90° (should show inclination effect)
        Physics::Vector3 pos_90 = Physics::OrbitMechanics::calculatePosition3D(a, e, i, raan, omega, M_PI/2);
        REQUIRE(pos_90.x == Catch::Approx(0.0).epsilon(1e-10));
        REQUIRE(pos_90.y == Catch::Approx(cos(i)).epsilon(1e-10));  // cos(45°) = √2/2
        REQUIRE(pos_90.z == Catch::Approx(sin(i)).epsilon(1e-10));  // sin(45°) = √2/2

        // Verify magnitude is still 1.0
        double magnitude = sqrt(pos_90.x*pos_90.x + pos_90.y*pos_90.y + pos_90.z*pos_90.z);
        REQUIRE(magnitude == Catch::Approx(1.0).epsilon(1e-10));
    }

    SECTION("3D position for polar orbit") {
        double a = 1.0;
        double e = 0.0;
        double i = M_PI/2;   // 90° inclination (polar)
        double raan = 0.0;
        double omega = 0.0;

        // At true anomaly = 90° (maximum Z displacement)
        Physics::Vector3 pos_90 = Physics::OrbitMechanics::calculatePosition3D(a, e, i, raan, omega, M_PI/2);
        REQUIRE(pos_90.x == Catch::Approx(0.0).epsilon(1e-10));
        REQUIRE(pos_90.y == Catch::Approx(0.0).epsilon(1e-10));
        REQUIRE(pos_90.z == Catch::Approx(1.0).epsilon(1e-10));

        // At true anomaly = 270° (minimum Z displacement)
        Physics::Vector3 pos_270 = Physics::OrbitMechanics::calculatePosition3D(a, e, i, raan, omega, 3*M_PI/2);
        REQUIRE(pos_270.x == Catch::Approx(0.0).epsilon(1e-10));
        REQUIRE(pos_270.y == Catch::Approx(0.0).epsilon(1e-10));
        REQUIRE(pos_270.z == Catch::Approx(-1.0).epsilon(1e-10));
    }
}

TEST_CASE("OrbitMechanics 3D Velocity Calculations", "[orbit_mechanics_3d_velocity]") {

    SECTION("3D velocity for equatorial circular orbit") {
        double a = 1.0;
        double e = 0.0;
        double i = 0.0;      // Equatorial
        double raan = 0.0;
        double omega = 0.0;

        // At true anomaly = 0° (periapsis)
        Physics::Vector3 vel_0 = Physics::OrbitMechanics::calculateVelocity3D(a, e, i, raan, omega, 0.0);
        double expected_speed = sqrt(constant::MU / a);

        REQUIRE(vel_0.x == Catch::Approx(0.0).epsilon(1e-10));
        REQUIRE(vel_0.y == Catch::Approx(expected_speed).epsilon(1e-10));
        REQUIRE(vel_0.z == Catch::Approx(0.0).epsilon(1e-10));

        // Verify magnitude
        double speed = vel_0.magnitude();
        REQUIRE(speed == Catch::Approx(expected_speed).epsilon(1e-10));
    }

    SECTION("3D velocity magnitude conservation") {
        double a = 1.0;
        double e = 0.0;
        double i = M_PI/6;   // 30° inclination
        double raan = 0.0;
        double omega = 0.0;

        std::vector<double> test_anomalies = {0.0, M_PI/4, M_PI/2, 3*M_PI/4, M_PI, 5*M_PI/4, 3*M_PI/2, 7*M_PI/4};
        double expected_speed = sqrt(constant::MU / a);

        for (double nu : test_anomalies) {
            Physics::Vector3 velocity = Physics::OrbitMechanics::calculateVelocity3D(a, e, i, raan, omega, nu);
            double speed = velocity.magnitude();

            REQUIRE(speed == Catch::Approx(expected_speed).epsilon(1e-9));
        }
    }
}

TEST_CASE("OrbitMechanics Non-Coplanar Transfer Test Case", "[orbit_mechanics_noncoplanar]") {

    SECTION("Reproduce actual problem geometry") {
        // Test the actual parameters from your constants
        double R1 = constant::R1;
        double R2 = constant::R2;
        double I1 = constant::I1;  // 28.5° inclined
        double I2 = constant::I2;  // 0° equatorial
        double E1 = constant::E1;  // 0.0 circular
        double E2 = constant::E2;  // 0.0 circular

        // Test positions at different true anomalies
        Physics::Vector3 r1_0 = Physics::OrbitMechanics::calculatePosition3D(R1, E1, I1, 0.0, 0.0, 0.0);
        Physics::Vector3 r1_90 = Physics::OrbitMechanics::calculatePosition3D(R1, E1, I1, 0.0, 0.0, M_PI/2);

        Physics::Vector3 r2_0 = Physics::OrbitMechanics::calculatePosition3D(R2, E2, I2, 0.0, 0.0, 0.0);
        Physics::Vector3 r2_90 = Physics::OrbitMechanics::calculatePosition3D(R2, E2, I2, 0.0, 0.0, M_PI/2);

        // Verify initial orbit characteristics
        REQUIRE(r1_0.magnitude() == Catch::Approx(R1).epsilon(1e-10));
        REQUIRE(r1_90.magnitude() == Catch::Approx(R1).epsilon(1e-10));

        // Verify target orbit characteristics
        REQUIRE(r2_0.magnitude() == Catch::Approx(R2).epsilon(1e-10));
        REQUIRE(r2_90.magnitude() == Catch::Approx(R2).epsilon(1e-10));

        // Check Z-separation for inclined vs equatorial
        REQUIRE(r1_0.z == Catch::Approx(0.0).epsilon(1e-10));  // At 0°, Z should be 0
        REQUIRE(r2_0.z == Catch::Approx(0.0).epsilon(1e-10));  // Equatorial always Z=0
        REQUIRE(r2_90.z == Catch::Approx(0.0).epsilon(1e-10)); // Equatorial always Z=0

        // At 90°, inclined orbit should have significant Z component
        double expected_z_90 = R1 * sin(I1);
        REQUIRE(abs(r1_90.z) == Catch::Approx(expected_z_90).epsilon(1e-10));

        std::cout << "Initial orbit (28.5° inclined) at 0°:  " << r1_0.x << ", " << r1_0.y << ", " << r1_0.z << std::endl;
        std::cout << "Initial orbit (28.5° inclined) at 90°: " << r1_90.x << ", " << r1_90.y << ", " << r1_90.z << std::endl;
        std::cout << "Target orbit (0° equatorial) at 0°:    " << r2_0.x << ", " << r2_0.y << ", " << r2_0.z << std::endl;
        std::cout << "Target orbit (0° equatorial) at 90°:   " << r2_90.x << ", " << r2_90.y << ", " << r2_90.z << std::endl;
    }
}

TEST_CASE("OrbitMechanics Vector3 Operations", "[vector3]") {

    SECTION("Vector3 basic operations") {
        Physics::Vector3 v1(1.0, 2.0, 3.0);
        Physics::Vector3 v2(4.0, 5.0, 6.0);

        // Addition
        Physics::Vector3 sum = v1 + v2;
        REQUIRE(sum.x == Catch::Approx(5.0).epsilon(1e-10));
        REQUIRE(sum.y == Catch::Approx(7.0).epsilon(1e-10));
        REQUIRE(sum.z == Catch::Approx(9.0).epsilon(1e-10));

        // Subtraction
        Physics::Vector3 diff = v2 - v1;
        REQUIRE(diff.x == Catch::Approx(3.0).epsilon(1e-10));
        REQUIRE(diff.y == Catch::Approx(3.0).epsilon(1e-10));
        REQUIRE(diff.z == Catch::Approx(3.0).epsilon(1e-10));

        // Magnitude
        double mag1 = v1.magnitude();
        double expected_mag1 = sqrt(1.0 + 4.0 + 9.0);
        REQUIRE(mag1 == Catch::Approx(expected_mag1).epsilon(1e-10));
    }

    SECTION("Vector3 dot product") {
        Physics::Vector3 v1(1.0, 0.0, 0.0);
        Physics::Vector3 v2(0.0, 1.0, 0.0);
        Physics::Vector3 v3(1.0, 0.0, 0.0);

        // Orthogonal vectors
        double dot_orthogonal = v1.dot(v2);
        REQUIRE(dot_orthogonal == Catch::Approx(0.0).epsilon(1e-10));

        // Parallel vectors
        double dot_parallel = v1.dot(v3);
        REQUIRE(dot_parallel == Catch::Approx(1.0).epsilon(1e-10));
    }

    SECTION("Vector3 cross product") {
        Physics::Vector3 v1(1.0, 0.0, 0.0);
        Physics::Vector3 v2(0.0, 1.0, 0.0);

        Physics::Vector3 cross = v1.cross(v2);
        REQUIRE(cross.x == Catch::Approx(0.0).epsilon(1e-10));
        REQUIRE(cross.y == Catch::Approx(0.0).epsilon(1e-10));
        REQUIRE(cross.z == Catch::Approx(1.0).epsilon(1e-10));
    }

    SECTION("Vector3 normalization") {
        Physics::Vector3 v(3.0, 4.0, 0.0);  // Magnitude = 5.0

        Physics::Vector3 normalized = v.normalized();
        REQUIRE(normalized.x == Catch::Approx(0.6).epsilon(1e-10));
        REQUIRE(normalized.y == Catch::Approx(0.8).epsilon(1e-10));
        REQUIRE(normalized.z == Catch::Approx(0.0).epsilon(1e-10));

        // Normalized vector should have magnitude 1
        REQUIRE(normalized.magnitude() == Catch::Approx(1.0).epsilon(1e-10));
    }
}
