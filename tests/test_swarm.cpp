#include <catch2/catch_all.hpp>
#include "optimization/Swarm.hpp"
#include "optimization/Logger.hpp"
#include <functional>
#include <fstream>
#include <cmath>

// Simple test function for swarm testing
auto simpleQuadratic = [](double* x) -> double {
    return x[0]*x[0] + x[1]*x[1];
};

TEST_CASE("Swarm - Basic Functionality", "[swarm][optimization]") {

    SECTION("Swarm initialization") {
        std::vector<double> lowerBounds = {-5.0, -5.0};
        std::vector<double> upperBounds = {5.0, 5.0};

        Swarm<double, std::function<double(double*)>> swarm(
            20, 2, simpleQuadratic,
            0.5, 1.2, 1.8,  // inertia, cognitive, social weights
            lowerBounds, upperBounds
        );

        REQUIRE(swarm.getNumParticles() == 20);
        REQUIRE(swarm.getInertiaWeight() == Catch::Approx(0.5));
        REQUIRE(swarm.getCognitiveWeight() == Catch::Approx(1.2));
        REQUIRE(swarm.getSocialWeight() == Catch::Approx(1.8));

        std::vector<double> globalBest = swarm.getGlobalBestPosition();
        // Should be initialized but may be NaN initially
        REQUIRE(globalBest.size() == 2);
    }

    SECTION("Swarm parameter setters") {
        std::vector<double> bounds = {-1.0, -1.0};
        std::vector<double> upper = {1.0, 1.0};

        Swarm<double, std::function<double(double*)>> swarm(
            10, 2, simpleQuadratic,
            0.5, 1.0, 1.0,  // inertia, cognitive, social weights
            bounds, upper
        );

        swarm.setInertiaWeight(0.3);
        swarm.setCognitiveWeight(2.5);
        swarm.setSocialWeight(2.8);

        REQUIRE(swarm.getInertiaWeight() == Catch::Approx(0.3));
        REQUIRE(swarm.getCognitiveWeight() == Catch::Approx(2.5));
        REQUIRE(swarm.getSocialWeight() == Catch::Approx(2.8));
    }
}

TEST_CASE("Swarm - Evolution and Updates", "[swarm][evolution]") {

    SECTION("Swarm initialization and basic operations") {
        std::vector<double> lowerBounds = {-2.0, -2.0};
        std::vector<double> upperBounds = {2.0, 2.0};

        Swarm<double, std::function<double(double*)>> swarm(
            15, 2, simpleQuadratic,
            0.6, 1.5, 1.5,  // weights
            lowerBounds, upperBounds
        );

        // Initialize the swarm
        swarm.init();

        // Check that particles were created
        REQUIRE(swarm.particles.size() == 15);

        // Test individual particle operations
        if (!swarm.particles.empty()) {
            auto& particle = swarm.particles[0];

            // Test position and velocity updates
            REQUIRE_NOTHROW(swarm.updateVelocity(particle));
            REQUIRE_NOTHROW(swarm.updatePosition(particle));
            REQUIRE_NOTHROW(swarm.updatePBestPos(particle));
            REQUIRE_NOTHROW(swarm.updateGBestPos());
        }
    }

    SECTION("Multiple iterations convergence simulation") {
        std::vector<double> lowerBounds = {-10.0, -10.0};
        std::vector<double> upperBounds = {10.0, 10.0};

        Swarm<double, std::function<double(double*)>> swarm(
            30, 2, simpleQuadratic,
            0.7, 2.0, 2.0,  // weights
            lowerBounds, upperBounds
        );

        swarm.init();

        double initial_best = swarm.getGlobalBestValue();

        // Simulate several iterations
        for (int i = 0; i < 20; ++i) {
            for (auto& particle : swarm.particles) {
                swarm.updateVelocity(particle);
                swarm.updatePosition(particle);
                swarm.updatePBestPos(particle);
            }
            swarm.updateGBestPos();
        }

        double final_best = swarm.getGlobalBestValue();
        std::vector<double> best_pos = swarm.getGlobalBestPosition();

        // Should improve or stay same over iterations
        REQUIRE(final_best <= initial_best);

        // Results should be finite
        REQUIRE(std::isfinite(final_best));
        REQUIRE(std::isfinite(best_pos[0]));
        REQUIRE(std::isfinite(best_pos[1]));

        // For quadratic function, should converge towards origin
        if (final_best < 100.0) { // If not penalty value
            REQUIRE(final_best >= 0.0);
        }
    }

    SECTION("Boundary enforcement") {
        std::vector<double> lowerBounds = {0.0, 0.0};
        std::vector<double> upperBounds = {1.0, 1.0};

        Swarm<double, std::function<double(double*)>> swarm(
            20, 2, simpleQuadratic,
            0.5, 1.0, 1.0,  // weights
            lowerBounds, upperBounds
        );

        swarm.init();

        // Run many iterations to test boundary handling
        for (int i = 0; i < 50; ++i) {
            for (auto& particle : swarm.particles) {
                swarm.updateVelocity(particle);
                swarm.updatePosition(particle);
                swarm.updatePBestPos(particle);
            }
            swarm.updateGBestPos();
        }

        std::vector<double> best_pos = swarm.getGlobalBestPosition();

        // Should respect bounds (if positions are not NaN)
        if (std::isfinite(best_pos[0]) && std::isfinite(best_pos[1])) {
            REQUIRE(best_pos[0] >= lowerBounds[0]);
            REQUIRE(best_pos[0] <= upperBounds[0]);
            REQUIRE(best_pos[1] >= lowerBounds[1]);
            REQUIRE(best_pos[1] <= upperBounds[1]);
        }
    }
}

TEST_CASE("Swarm - Edge Cases", "[swarm][edge_cases]") {

    SECTION("Single particle swarm") {
        std::vector<double> bounds = {-1.0};
        std::vector<double> upper = {1.0};

        auto func1D = [](double* x) -> double { return x[0]*x[0]; };

        Swarm<double, std::function<double(double*)>> swarm(
            1, 1, func1D,
            0.5, 1.0, 1.0,  // weights
            bounds, upper
        );

        REQUIRE(swarm.getNumParticles() == 1);

        swarm.init();

        // Should still work with single particle
        if (!swarm.particles.empty()) {
            auto& particle = swarm.particles[0];
            REQUIRE_NOTHROW(swarm.updateVelocity(particle));
            REQUIRE_NOTHROW(swarm.updatePosition(particle));
            REQUIRE_NOTHROW(swarm.updatePBestPos(particle));
            REQUIRE_NOTHROW(swarm.updateGBestPos());
        }

        REQUIRE(std::isfinite(swarm.getGlobalBestValue()));
    }

    SECTION("Large dimensional problem") {
        const size_t dim = 5; // Reduced from 10 for simpler testing
        std::vector<double> lowerBounds(dim, -1.0);
        std::vector<double> upperBounds(dim, 1.0);

        auto highDimFunc = [](double* x) -> double {
            double sum = 0.0;
            for (int i = 0; i < 5; ++i) {
                sum += x[i] * x[i];
            }
            return sum;
        };

        Swarm<double, std::function<double(double*)>> swarm(
            30, dim, highDimFunc,
            0.6, 1.5, 1.5,  // weights
            lowerBounds, upperBounds
        );

        REQUIRE(swarm.getGlobalBestPosition().size() == dim);

        swarm.init();

        // Should handle higher dimensions
        for (int i = 0; i < 10; ++i) {
            for (auto& particle : swarm.particles) {
                REQUIRE_NOTHROW(swarm.updateVelocity(particle));
                REQUIRE_NOTHROW(swarm.updatePosition(particle));
                REQUIRE_NOTHROW(swarm.updatePBestPos(particle));
            }
            REQUIRE_NOTHROW(swarm.updateGBestPos());
        }
    }
}

TEST_CASE("Logger - Basic Functionality", "[logger][optimization]") {

    SECTION("Logger creation and destruction") {
        std::string filename = "test_log.csv";

        {
            Logger logger(filename);
            // Logger should create file
        } // Logger destructor should flush and close

        // File should exist and be readable
        std::ifstream file(filename);
        REQUIRE(file.is_open());

        std::string line;
        REQUIRE(std::getline(file, line)); // Should have header
        file.close();

        // Clean up
        std::remove(filename.c_str());
    }

    SECTION("Logging data entries") {
        std::string filename = "test_log_data.csv";

        {
            Logger logger(filename);

            logger.log(1, 10.0, 0.8, 1.5, 1.2);   // iter, value, inertia, social, cognitive
            logger.log(2, 8.0, 0.7, 1.5, 1.2);
            logger.log(3, 6.0, 0.6, 1.5, 1.2);
        }

        // Read and verify file contents
        std::ifstream file(filename);
        REQUIRE(file.is_open());

        std::string line;
        std::getline(file, line); // Skip header

        int line_count = 0;
        while (std::getline(file, line)) {
            REQUIRE(!line.empty());

            // Check that line contains expected number of commas
            int comma_count = std::count(line.begin(), line.end(), ',');
            REQUIRE(comma_count >= 4); // At least 5 fields (iter, value, inertia, social, cognitive)

            line_count++;
        }

        REQUIRE(line_count == 3); // Should have 3 data lines
        file.close();

        // Clean up
        std::remove(filename.c_str());
    }

    SECTION("Buffer flushing") {
        std::string filename = "test_log_flush.csv";

        Logger logger(filename);

        // Add several entries
        for (int i = 1; i <= 5; ++i) {
            logger.log(i, 10.0 - i, 0.8 - i*0.1, 1.5, 1.2);
        }

        // Manually flush
        logger.flushBuffer();

        // File should contain data even before destructor
        std::ifstream file(filename);
        REQUIRE(file.is_open());

        std::string line;
        int line_count = 0;
        while (std::getline(file, line)) {
            line_count++;
        }
        file.close();

        REQUIRE(line_count >= 6); // Header + 5 data lines

        // Clean up
        std::remove(filename.c_str());
    }
}
