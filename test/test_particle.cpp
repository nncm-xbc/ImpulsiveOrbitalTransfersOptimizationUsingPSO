#include "Particle.hpp"
#include "Function.hpp"
#include <gtest/gtest.h>
#include <vector>
#include <functional>

class ParticleTest : public ::testing::Test {
protected:
    std::function<double(double *, size_t)> fun = &Function::Rosenbrock<double>;
    const size_t dimension = 2;
};

// Initialization test
TEST_F(ParticleTest, ConstructorInitializesCorrectly) {
    Particle<double, std::function<double(double *, size_t)>> particle(fun, dimension);
    EXPECT_EQ(particle.getPosition().size(), dimension);
    EXPECT_EQ(particle.getVelocity().size(), dimension);
    EXPECT_EQ(particle.getBestPosition().size(), dimension);
    EXPECT_DOUBLE_EQ(particle.getValue(), 0.0);
    EXPECT_DOUBLE_EQ(particle.getBestValue(), 0.0);
}

// Setter tests
TEST_F(ParticleTest, SetPositionUpdatesCorrectly) {
    Particle<double, std::function<double(double *, size_t)>> particle(fun, dimension);
    std::vector<double> position = {1.0, 2.0};
    particle.setPosition(position);
    EXPECT_EQ(particle.getPosition(), position);
}

TEST_F(ParticleTest, SetVelocityUpdatesCorrectly) {
    Particle<double, std::function<double(double *, size_t)>> particle(fun, dimension);
    std::vector<double> velocity = {1.0, 2.0};
    particle.setVelocity(velocity);
    EXPECT_EQ(particle.getVelocity(), velocity);
}

TEST_F(ParticleTest, SetBestPositionUpdatesCorrectly) {
    Particle<double, std::function<double(double *, size_t)>> particle(fun, dimension);
    std::vector<double> bestPosition = {1.0, 2.0};
    particle.setBestPosition(bestPosition);
    EXPECT_EQ(particle.getBestPosition(), bestPosition);
}

TEST_F(ParticleTest, SetValueUpdatesCorrectly) {
    Particle<double, std::function<double(double *, size_t)>> particle(fun, dimension);
    double value = 1.0;
    particle.setValue(value);
    EXPECT_DOUBLE_EQ(particle.getValue(), value);
}

TEST_F(ParticleTest, SetBestValueUpdatesCorrectly) {
    Particle<double, std::function<double(double *, size_t)>> particle(fun, dimension);
    double bestValue = 1.0;
    particle.setBestValue(bestValue);
    EXPECT_DOUBLE_EQ(particle.getBestValue(), bestValue);
}

// Corner case tests
TEST_F(ParticleTest, HandleZeroDimensionParticle) {
    EXPECT_THROW(Particle<double, std::function<double(double *, size_t)>>(fun, 0), std::invalid_argument);
}

TEST_F(ParticleTest, HandleIncorrectDimensionInput) {
    Particle<double, std::function<double(double *, size_t)>> particle(fun, dimension);
    std::vector<double> incorrectDimension = {1.0, 2.0, 3.0};
    EXPECT_THROW(particle.setPosition(incorrectDimension), std::invalid_argument);
    EXPECT_THROW(particle.setVelocity(incorrectDimension), std::invalid_argument);
    EXPECT_THROW(particle.setBestPosition(incorrectDimension), std::invalid_argument);
}

// Function object test
TEST_F(ParticleTest, ObjectiveFunctionUsedCorrectly) {
    Particle<double, std::function<double(double *, size_t)>> particle(fun, dimension);
    std::vector<double> position = {1.0, 2.0};
    particle.setPosition(position);
    double expectedValue = Function::Rosenbrock<double>(position.data(), dimension);
    EXPECT_DOUBLE_EQ(particle.getValue(), expectedValue);
}