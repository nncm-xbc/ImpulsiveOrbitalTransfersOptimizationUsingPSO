#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include "Swarm.hpp"
#include "Functions.hpp"

TEST_CASE("Swarm tests", "[swarm]") {
    using SwarmType = Swarm<double, std::function<double(double*, size_t)>>;
    const size_t numParticles = 30;
    const size_t dimension = 2;
    std::function<double(double*, size_t)> testFunction = &Function::Sphere<double>;
    const double inertiaWeight = 0.7;
    const double cognitiveWeight = 1.5;
    const double socialWeight = 1.5;

    SwarmType* swarm = new SwarmType(numParticles, dimension, testFunction, inertiaWeight, cognitiveWeight, socialWeight);

    SECTION("Constructor and initialization") {
        REQUIRE(swarm->getNumParticles() == numParticles);
        REQUIRE(swarm->getDimension() == dimension);

        std::vector<double> testInput = {1.0, 1.0};
        REQUIRE(swarm->getObjectiveFunction()(testInput.data(), testInput.size()) ==
                Catch::Approx(testFunction(testInput.data(), testInput.size())));
    }

    SECTION("Particle initialization") {
        swarm->init();
        for (size_t i = 0; i < numParticles; ++i) {
            auto position = swarm->getPosition(swarm->particles[i]);
            REQUIRE(position.size() == dimension);
            for (const auto& pos : position) {
                REQUIRE(pos >= -100.0);
                REQUIRE(pos <= 100.0);
            }
        }
    }
/*
    SECTION("Global best initialization") {
        auto gBestPos = swarm->getGlobalBestPosition();
        REQUIRE(gBestPos.size() == dimension);
        REQUIRE(swarm->getGlobalBestValue() != std::numeric_limits<double>::max());
    }
*/
    SECTION("Position update") {
        swarm->init();
        auto& particle = swarm->particles[0];
        auto oldPosition = swarm->getPosition(particle);
        swarm->updatePosition(particle);
        auto newPosition = swarm->getPosition(particle);
        REQUIRE(oldPosition != newPosition);
    }

    SECTION("Velocity update") {
        swarm->init();
        auto& particle = swarm->particles[0];
        auto oldVelocity = swarm->getVelocity(particle);
        swarm->updateVelocity(particle);
        auto newVelocity = swarm->getVelocity(particle);
        REQUIRE(oldVelocity != newVelocity);
    }

    SECTION("Personal best update") {
        swarm->init();
        auto& particle = swarm->particles[0];

        // init best personal val and pos
        particle.setBestPosition(std::vector<double>{10.0, 10.0});
        particle.setBestValue(10.0);
        auto oldPBestPos = particle.getBestPosition();
        auto oldPBestVal = particle.getBestValue();

        //New best personal pos and updates
        particle.setPosition(std::vector<double>{1.0, 1.0}, testFunction);
        swarm->updatePBestPos(particle);
        swarm->updatePBestVal(particle);

        auto newPBestPos = particle.getBestPosition();
        auto newPBestVal = particle.getBestValue();

        //Checks
        REQUIRE(newPBestPos == std::vector<double>{1.0, 1.0});
        REQUIRE(oldPBestVal > newPBestVal);
        double* values = new double[2] {1.0, 1.0};
        REQUIRE(newPBestVal == testFunction(values, dimension));
        delete[] values;
    }

    SECTION("Global best update") {
        swarm->init();
        auto oldGBest = swarm->getGlobalBestPosition();
        swarm->updateGBestPos();
        auto newGBest = swarm->getGlobalBestPosition();
        REQUIRE(oldGBest != newGBest);
    }

    SECTION("Swarm convergence") {
        swarm->init();
        const int iterations = 100;
        double initialBest = swarm->getGlobalBestValue();
        for (int i = 0; i < iterations; ++i) {
            for (auto& particle : swarm->particles) {
                swarm->updateVelocity(particle);
                swarm->updatePosition(particle);
                swarm->updatePBestPos(particle);
                swarm->updatePBestVal(particle);
            }
            swarm->updateGBestPos();
        }
        double finalBest = swarm->getGlobalBestValue();
        REQUIRE(finalBest < initialBest);
    }

    SECTION("Memory management") {
        swarm->init();
        swarm->deallocateMemory();
        REQUIRE(swarm->particles.size() == 0);
    }

    delete swarm;
}
