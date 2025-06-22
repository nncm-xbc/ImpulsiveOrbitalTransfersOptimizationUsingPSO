/**
 * @file Particle.hpp
 * @brief Individual particle implementation for PSO algorithm
 * @author PSO Orbit Transfer Team
 * @date 2025
 *
 * Defines the Particle class representing a single candidate solution
 * in the Particle Swarm Optimization algorithm. Each particle maintains
 * position, velocity, and personal best information.
 */

#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <cstddef>
#include <functional>
#include <iostream>
#include <random>
#include <vector>

/**
 * @class Particle
 * @brief Individual particle in the PSO swarm
 * @tparam T Floating point type (float or double)
 * @tparam Fun Function type for objective function evaluation
 *
 * Represents a single candidate solution in the particle swarm optimization
 * algorithm. Each particle has:
 * - Current position (candidate solution)
 * - Current velocity (search direction and magnitude)
 * - Personal best position found so far
 * - Objective function values
 *
 * Particles move through the search space by updating their velocity based on:
 * - Inertia (current velocity)
 * - Cognitive component (attraction to personal best)
 * - Social component (attraction to global best)
 */
template <typename T, typename Fun> class Particle
{
    public:
        /**
         * @brief Constructor for particle initialization
         * @param objectiveFunction Function to evaluate solution quality
         * @param dimension Dimensionality of the search space
         * @param rng Random number generator reference
         * @param dis Vector of uniform distributions for each dimension
         * @param lowerBounds Lower bounds for each dimension
         * @param upperBounds Upper bounds for each dimension
         *
         * Initializes particle with random position within bounds and
         * small random velocity. Evaluates initial objective function value.
         */
        Particle(const Fun objectiveFunction,
                const size_t &dimension,
                std::mt19937 &rng,
                std::vector<std::uniform_real_distribution<T>> &dis,
                std::vector<T> lowerBounds,
                std::vector<T> upperBounds);

        /** @brief Default constructor */
        Particle() = default;

        /**
         * @brief Set particle position and evaluate objective function
         * @param position New position vector
         * @param objectiveFunction Function to evaluate solution quality
         *
         * Updates particle position and automatically evaluates the
         * objective function at the new position.
         */
        void setPosition(const std::vector<T> &position,
                        const Fun &objectiveFunction);

        /**
         * @brief Set particle velocity vector
         * @param velocity New velocity vector
         *
         * Updates the particle's velocity, which determines its movement
         * direction and magnitude in the next iteration.
         */
        void setVelocity(const std::vector<T> &velocity);

        /**
         * @brief Set particle's personal best position
         * @param best_position New personal best position
         *
         * Updates the best position this particle has ever found.
         * Should only be called when a better solution is discovered.
         */
        void setBestPosition(const std::vector<T> &best_position);

        /**
         * @brief Set current objective function value
         * @param value Current objective function value
         */
        void setValue(const T &value);

        /**
         * @brief Set personal best objective function value
         * @param best_value Best objective function value found by this particle
         */
        void setBestValue(const T &best_value);

        /**
         * @brief Get current particle position
         * @return Reference to current position vector
         */
        std::vector<T> &getPosition();

        /**
         * @brief Get current particle velocity
         * @return Reference to current velocity vector
         */
        std::vector<T> &getVelocity();

        /**
         * @brief Get particle's personal best position
         * @return Reference to personal best position vector
         */
        std::vector<T> &getBestPosition();

        /**
         * @brief Get current objective function value
         * @return Current objective function value
         */
        T &getValue();

        /**
         * @brief Get personal best objective function value
         * @return Best objective function value found by this particle
         */
        T &getBestValue();

    private:
        /** @brief Dimensionality of the search space */
        size_t _dimension;

        /** @brief Current position in search space */
        std::vector<T> _position;

        /** @brief Current velocity vector */
        std::vector<T> _velocity;

        /** @brief Personal best position found */
        std::vector<T> _bestPosition;

        /** @brief Current objective function value */
        T _value;

        /** @brief Personal best objective function value */
        T _bestValue;
};

// ============================================
// TEMPLATE IMPLEMENTATION
// ============================================

/**
 * @brief Constructor implementation
 *
 * Initializes particle with random position within bounds and velocity
 * scaled to 10% of the position range. Evaluates initial objective
 * function and sets personal best to initial values.
 */
template <typename T, typename Fun>
Particle<T, Fun>::Particle(const Fun objectiveFunction, const size_t &dimension,
                           std::mt19937 &rng,
                           std::vector<std::uniform_real_distribution<T>> &dis,
                           std::vector<T> lowerBounds,
                           std::vector<T> upperBounds)
    : _dimension(dimension),
    _position(dimension),
    _velocity(dimension),
    _bestPosition(dimension)
{
    double velocityScale = 0.1; // 10% of position range

    for (size_t i = 0; i < _dimension; ++i)
    {
        _position[i] = dis[i](rng);
        _velocity[i] = velocityScale * (dis[i](rng)-0.5);
    }

    _bestPosition = _position;
    _value = objectiveFunction(_position.data());
    _bestValue = _value;
}

// Setters
template <typename T, typename Fun>
void Particle<T, Fun>::setPosition(const std::vector<T> &position,
                                   const Fun &objectiveFunction)
{
    if (position.size() != _dimension)
    {
        throw std::invalid_argument("Position dimension mismatch");
    }
    _position = position;
    _value = objectiveFunction(_position.data());
}

template <typename T, typename Fun>
void Particle<T, Fun>::setVelocity(const std::vector<T> &velocity)
{
    if (velocity.size() != _dimension)
    {
        throw std::invalid_argument("Velocity dimension mismatch");
    }
    _velocity = velocity;
}

template <typename T, typename Fun>
void Particle<T, Fun>::setBestPosition(const std::vector<T> &best_position)
{
    if (best_position.size() != _dimension)
    {
        throw std::invalid_argument("Position dimension mismatch");
    }
    _bestPosition = best_position;
}

template <typename T, typename Fun>
void Particle<T, Fun>::setValue(const T &value)
{
  _value = value;
}

template <typename T, typename Fun>
void Particle<T, Fun>::setBestValue(const T &best_value)
{
  _bestValue = best_value;
}

// Getters
template <typename T, typename Fun>
std::vector<T> &Particle<T, Fun>::getPosition()
{
  return _position;
}

template <typename T, typename Fun>
std::vector<T> &Particle<T, Fun>::getVelocity()
{
  return _velocity;
}

template <typename T, typename Fun>
std::vector<T> &Particle<T, Fun>::getBestPosition()
{
  return _bestPosition;
}

template <typename T, typename Fun> T &Particle<T, Fun>::getValue()
{
  return _value;
}

template <typename T, typename Fun> T &Particle<T, Fun>::getBestValue()
{
  return _bestValue;
}

#endif // !PARTICLE_HPP
