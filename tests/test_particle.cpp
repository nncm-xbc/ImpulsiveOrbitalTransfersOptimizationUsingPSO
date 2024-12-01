#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include "Particle.hpp"
#include "Functions.hpp"
#include <vector>
#include <functional>

TEST_CASE("Particle tests", "[particle]") {
    std::function<double(double *, size_t)> fun = &Function::Sphere<double>;
    const size_t dimension = 2;
    std::mt19937 rng;
    std::uniform_real_distribution<> dis;
    std::vector<double> lowerBounds(dimension);
    std::vector<double> upperBounds(dimension);

    lowerBounds[0] = 6678.0;
    upperBounds[0] = 42164.0;
    lowerBounds[1] = 42164.0;
    upperBounds[1] = 2*M_PI;

    SECTION("Constructor initializes correctly") {
        Particle<double, std::function<double(double *, size_t)>> particle(fun, dimension, lowerBounds, upperBounds, rng, dis);
        REQUIRE(particle.getPosition().size() == dimension);
        REQUIRE(particle.getVelocity().size() == dimension);
        REQUIRE(particle.getBestPosition().size() == dimension);
    }

    SECTION("Setter methods update correctly") {
        Particle<double, std::function<double(double *, size_t)>> particle(fun, dimension, lowerBounds, upperBounds, rng, dis);
        SECTION("Set position") {
            std::vector<double> position = {1.0, 2.0};
            particle.setPosition(position, fun);
            REQUIRE(particle.getPosition() == position);
        }

        SECTION("Set velocity") {
            std::vector<double> velocity = {1.0, 2.0};
            particle.setVelocity(velocity);
            REQUIRE(particle.getVelocity() == velocity);
        }

        SECTION("Set best position") {
            std::vector<double> bestPosition = {1.0, 2.0};
            particle.setBestPosition(bestPosition);
            REQUIRE(particle.getBestPosition() == bestPosition);
        }

        SECTION("Set value") {
            double value = 1.0;
            particle.setValue(value);
            REQUIRE(particle.getValue() == Catch::Approx(value));
        }

        SECTION("Set best value") {
            double bestValue = 1.0;
            particle.setBestValue(bestValue);
            REQUIRE(particle.getBestValue() == Catch::Approx(bestValue));
        }
    }

    SECTION("Handle incorrect dimension input") {
        Particle<double, std::function<double(double *, size_t)>> particle(fun, dimension, lowerBounds, upperBounds, rng, dis);
        std::vector<double> incorrectDimension = {1.0, 2.0, 3.0};
        REQUIRE_THROWS_AS(particle.setPosition(incorrectDimension, fun), std::invalid_argument);
        REQUIRE_THROWS_AS(particle.setVelocity(incorrectDimension), std::invalid_argument);
        REQUIRE_THROWS_AS(particle.setBestPosition(incorrectDimension), std::invalid_argument);
    }

    SECTION("Objective function used correctly") {
        Particle<double, std::function<double(double *, size_t)>> particle(fun, dimension, lowerBounds, upperBounds, rng, dis);
        std::vector<double> position = {1.0, 2.0};
        particle.setPosition(position, fun);
        double expectedValue = Function::Sphere<double>(position.data(), dimension);
        REQUIRE(particle.getValue() == Catch::Approx(expectedValue));
    }
}
