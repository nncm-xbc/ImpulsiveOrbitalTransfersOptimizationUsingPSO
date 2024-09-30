#include <gtest/gtest.h>
#include "Swarm.hpp"
#include "Functions.hpp"

// Test fixture for Swarm
class SwarmTest : public ::testing::Test {
protected:
    using SwarmType = Swarm<double, std::function<double(double*, size_t)>>;
    SwarmType* swarm;
    const size_t numParticles = 30;
    const size_t dimension = 2;
    std::function<double(double*, size_t)> testFunction;
    const double inertiaWeight = 0.7;
    const double cognitiveWeight = 1.5;
    const double socialWeight = 1.5;

    void SetUp() override {
        testFunction = Function::Sphere<double>;
        swarm = new SwarmType(numParticles, dimension, testFunction, inertiaWeight, cognitiveWeight, socialWeight);
    }

    void TearDown() override {
        delete swarm;
    }
};

TEST_F(SwarmTest, ConstructorAndInitialization) {
    EXPECT_EQ(swarm->getNumParticles(), numParticles);
    EXPECT_EQ(swarm->getDimension(), dimension);
    
    std::vector<double> testInput = {1.0, 1.0}; // Adjust based on your function's domain
    EXPECT_DOUBLE_EQ(swarm->getObjectiveFunction()(testInput.data(), testInput.size()), 
                     testFunction(testInput.data(), testInput.size()));
}

// Test particle initialization
TEST_F(SwarmTest, ParticleInitialization) {
    for (size_t i = 0; i < numParticles; ++i) {
        auto position = swarm->getPosition(swarm->particles[i]);
        EXPECT_EQ(position.size(), dimension);
        for (const auto& pos : position) {
            EXPECT_GE(pos, -100.0); 
            EXPECT_LE(pos, 100.0);
        }
    }
}

// Test global best initialization
TEST_F(SwarmTest, GlobalBestInitialization) {
    auto gBestPos = swarm->getGlobalBestPosition();
    EXPECT_EQ(gBestPos.size(), dimension);
    EXPECT_NE(swarm->getGlobalBestValue(), std::numeric_limits<double>::max());
}

// Test position update
TEST_F(SwarmTest, PositionUpdate) {
    auto& particle = swarm->particles[0];
    auto oldPosition = swarm->getPosition(particle);
    swarm->updatePosition(particle);
    auto newPosition = swarm->getPosition(particle);
    EXPECT_NE(oldPosition, newPosition);
}

// Test velocity update
TEST_F(SwarmTest, VelocityUpdate) {
    auto& particle = swarm->particles[0];
    auto oldVelocity = swarm->getVelocity(particle);
    swarm->updateVelocity(particle);
    auto newVelocity = swarm->getVelocity(particle);
    EXPECT_NE(oldVelocity, newVelocity);
}

// Test personal best update
TEST_F(SwarmTest, PersonalBestUpdate) {
    auto& particle = swarm->particles[0];
    auto oldPBest = particle.getBestPosition();
    swarm->updatePBestPos(particle);
    swarm->updatePBestVal(particle);
    auto newPBest = particle.getBestPosition();
    EXPECT_NE(oldPBest, newPBest);
}

// Test global best update
TEST_F(SwarmTest, GlobalBestUpdate) {
    auto oldGBest = swarm->getGlobalBestPosition();
    swarm->updateGBestPos();
    auto newGBest = swarm->getGlobalBestPosition();
    EXPECT_NE(oldGBest, newGBest);
}

// Test swarm behavior over multiple iterations
TEST_F(SwarmTest, SwarmConvergence) {
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
    EXPECT_LT(finalBest, initialBest); // Expect improvement
}

/*
// Test with different objective functions
TEST_F(SwarmTest, DifferentObjectiveFunctions) {
    std::vector<std::function<double(const std::vector<double>&)>> functions = {
        Functions::sphere,
        Functions::rastrigin,
        Functions::rosenbrock
    };
    
    for (const auto& func : functions) {
        swarm->setObjectiveFunction(func);
        EXPECT_EQ(swarm->getObjectiveFunction(), func);
        // Run a few iterations to ensure it works with the new function
        for (int i = 0; i < 10; ++i) {
            for (auto& particle : swarm->particles) {
                swarm->updateVelocity(particle);
                swarm->updatePosition(particle);
                swarm->updatePBestPos(particle);
                swarm->updatePBestVal(particle);
            }
            swarm->updateGBestPos();
        }
        EXPECT_NO_THROW(swarm->getGlobalBestValue());
    }
}*/

// Test memory management
TEST_F(SwarmTest, MemoryManagement) {
    swarm->deallocateMemory();
    EXPECT_EQ(swarm->getNumParticles(), 0);
}
