/**
 * @file PSO.hpp
 * @brief Main Particle Swarm Optimization algorithm implementation
 * @author PSO Orbit Transfer Team
 * @date 2025
 *
 * Implements the core PSO algorithm for solving orbital transfer optimization
 * problems. Manages the swarm of particles and orchestrates the optimization
 * process with adaptive parameter updates.
 */

#ifndef PSO_HPP
#define PSO_HPP

#include "core/OrbitProblem.hpp"
#include "optimization/Swarm.hpp"

/**
 * @class PSO
 * @brief Particle Swarm Optimization algorithm controller
 * @tparam T Floating point type (typically double)
 * @tparam Fun Function type for objective function
 *
 * Main PSO algorithm implementation that coordinates the optimization process.
 * Features include:
 * - Adaptive parameter control (inertia weight scheduling)
 * - Convergence monitoring and logging
 * - Integration with orbital transfer problems
 * - Results analysis and output formatting
 *
 * The algorithm follows the standard PSO velocity update equation:
 * v(t+1) = w*v(t) + c1*r1*(pbest - x(t)) + c2*r2*(gbest - x(t))
 *
 * Where:
 * - w: inertia weight (decreases linearly over iterations)
 * - c1: cognitive weight (attraction to personal best)
 * - c2: social weight (attraction to global best)
 * - r1, r2: random numbers [0,1]
 */
template <typename T, typename Fun>
class PSO
{
    public:
        /**
         * @brief Constructor for PSO algorithm
         * @param numParticles Number of particles in the swarm
         * @param dimension Dimensionality of the optimization problem
         * @param maxIterations Maximum number of iterations to perform
         * @param tolerance Convergence tolerance (currently unused)
         * @param inertiaWeight Initial inertia weight parameter
         * @param cognitiveWeight Cognitive (personal best) weight parameter
         * @param socialWeight Social (global best) weight parameter
         * @param objectiveFunction Function to optimize
         * @param lowerBounds Lower bounds for each dimension
         * @param upperBounds Upper bounds for each dimension
         *
         * Initializes the PSO algorithm with specified parameters and creates
         * the particle swarm. The swarm handles individual particle management.
         */
        PSO(size_t numParticles,
            size_t dimension,
            size_t maxIterations,
            T tolerance,
            T inertiaWeight,
            T cognitiveWeight,
            T socialWeight,
            const Fun &objectiveFunction,
            const std::vector<T> lowerBounds,
            const std::vector<T> upperBounds);

        /**
         * @brief Execute the main PSO optimization loop
         *
         * Performs the complete optimization process:
         * 1. Initialize swarm and display information
         * 2. Main iteration loop:
         *    - Update particle velocities
         *    - Update particle positions
         *    - Update personal best positions
         *    - Update global best position
         *    - Adapt algorithm parameters
         * 3. Log convergence data periodically
         *
         * The algorithm uses adaptive inertia weight that decreases linearly
         * from 0.9 to 0.4 over the course of optimization for improved
         * exploration early and exploitation later.
         */
        void solve();

        /**
         * @brief Print formatted optimization results to console
         *
         * Displays a comprehensive summary including:
         * - Best solution vector found
         * - Best objective function value (total ΔV)
         * - Iteration statistics
         * - Additional orbital transfer specific information
         */
        void printResults() const;

        /**
         * @brief Save optimization results to file
         * @param filename Path to output file
         * @param orbitProblem Reference to orbit problem for detailed analysis
         *
         * Saves detailed results in structured format including:
         * - Orbital parameters for initial and target orbits
         * - Optimal transfer parameters
         * - Delta-V breakdown
         * - Transfer classification (two vs three impulse)
         * - Plane change information for non-coplanar cases
         */
        void saveResults(const std::string& filename, OrbitTransferObjective<double, std::function<double(double*)>> orbitProblem);

        /**
         * @brief Get the best value of the objective function
         * @return Best value of the objective function
         */
        T getBestValue() const;

        /**
         * @brief Get the best position of the objective function
         * @return Best position of the objective function
         */
        std::vector<T> getBestPosition() const;

    private:
        /** @brief Particle swarm containing all candidate solutions */
        Swarm<T, Fun> swarm;

        /** @brief Maximum number of iterations to perform */
        size_t _maxIterations;

        /** @brief Convergence tolerance (currently unused) */
        T _tolerance;

        /**
         * @brief Update algorithm parameters based on iteration progress
         * @param GBPos_previous Previous global best position (currently unused)
         * @param iter Current iteration number
         *
         * Implements adaptive parameter control strategy:
         * - Linear decrease of inertia weight from 0.9 to 0.4
         * - Helps balance exploration (early) vs exploitation (late)
         * - Can be extended for other adaptive strategies
         */
        void updateWC(std::vector<T> GBPos_previous, size_t iter);
};
#endif
