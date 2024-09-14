#include <gtest/gtest.h>
#include "Swarm.hpp"
#include "Functions.hpp"

// Test fixture for Swarm
class SwarmTest : public ::testing::Test {
protected:
    using SwarmType = Swarm<double, std::function<double(const std::vector<double>&)>>;
    SwarmType* swarm;
    const size_t numParticles = 30;
    const size_t dimension = 2;
    std::function<double(const std::vector<double>&)> testFunction;

    void SetUp() override {
        testFunction = Functions::sphere; // Using sphere function from Functions namespace
        swarm = new SwarmType(numParticles, dimension, testFunction);
    }

    void TearDown() override {
        delete swarm;
    }
};

// Test constructor and initialization
TEST_F(SwarmTest, ConstructorAndInitialization) {
    EXPECT_EQ(swarm->getNumParticles(), numParticles);
    EXPECT_EQ(swarm->getDimension(), dimension);
    EXPECT_EQ(swarm->getObjectiveFunction(), testFunction);
}

// Test particle initialization
TEST_F(SwarmTest, ParticleInitialization) {
    for (size_t i = 0; i < numParticles; ++i) {
        auto position = swarm->getPosition(swarm->_particles[i]);
        EXPECT_EQ(position.size(), dimension);
        for (const auto& pos : position) {
            EXPECT_GE(pos, -100.0); // Assuming initialization range [-100, 100]
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
    auto& particle = swarm->_particles[0];
    auto oldPosition = swarm->getPosition(particle);
    swarm->updatePosition(particle);
    auto newPosition = swarm->getPosition(particle);
    EXPECT_NE(oldPosition, newPosition);
}

// Test velocity update
TEST_F(SwarmTest, VelocityUpdate) {
    auto& particle = swarm->_particles[0];
    auto oldVelocity = swarm->getVelocity(particle);
    swarm->updateVelocity(particle);
    auto newVelocity = swarm->getVelocity(particle);
    EXPECT_NE(oldVelocity, newVelocity);
}

// Test personal best update
TEST_F(SwarmTest, PersonalBestUpdate) {
    auto& particle = swarm->_particles[0];
    auto oldPBest = particle.getPBestPos();
    swarm->updatePBestPos(particle);
    swarm->updatePBestVal(particle);
    auto newPBest = particle.getPBestPos();
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
        for (auto& particle : swarm->_particles) {
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
            for (auto& particle : swarm->_particles) {
                swarm->updateVelocity(particle);
                swarm->updatePosition(particle);
                swarm->updatePBestPos(particle);
                swarm->updatePBestVal(particle);
            }
            swarm->updateGBestPos();
        }
        EXPECT_NO_THROW(swarm->getGlobalBestValue());
    }
}

// Test memory management
TEST_F(SwarmTest, MemoryManagement) {
    swarm->deallocateMemory();
    EXPECT_EQ(swarm->getNumParticles(), 0);
    swarm->allocateMemory();
    EXPECT_EQ(swarm->getNumParticles(), numParticles);
}