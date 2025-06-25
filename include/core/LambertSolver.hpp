/**
 * @file LambertSolver.hpp
 * @brief Lambert's problem solver for orbital trajectory determination
 * @author PSO Orbit Transfer Team
 * @date 2025
 *
 * Solves Lambert's problem: determining the orbit that connects two position
 * vectors in a specified time of flight. This is fundamental for orbital
 * transfer trajectory design and optimization.
 */

#ifndef LAMBERT_SOLVER_HPP
#define LAMBERT_SOLVER_HPP

#include <glm/glm.hpp>
#include <utility>
#include <optional>

#include "core/OrbitMechanics.hpp"

namespace Physics {

/**
 * @class LambertSolver
 * @brief Solves Lambert's problem for orbital trajectory determination
 *
 * Lambert's problem: Given two position vectors and a time of flight,
 * find the orbit that connects them. This is a fundamental problem in
 * orbital mechanics used for:
 * - Interplanetary trajectory design
 * - Orbital rendezvous planning
 * - Transfer orbit optimization
 *
 * This implementation uses a robust numerical approach with universal
 * variables and Stumpff functions for improved convergence.
 */
class LambertSolver {
public:
    /**
     * @struct Solution
     * @brief Complete solution to Lambert's problem
     *
     * Contains all relevant information about the transfer orbit,
     * including velocity vectors and orbital elements.
     */
    struct Solution {
        /** @brief Velocity vector at departure point (DU/TU) */
        glm::dvec3 v1;

        /** @brief Velocity vector at arrival point (DU/TU) */
        glm::dvec3 v2;

        /** @brief Semi-major axis of transfer orbit (DU) */
        double semi_major_axis;

        /** @brief Eccentricity of transfer orbit (dimensionless) */
        double eccentricity;

        /** @brief Whether solution is physically valid */
        bool is_valid;

        // Optional debug information
        /** @brief True anomaly change during transfer (radians) */
        double transfer_angle;

        /** @brief Computed time of flight for validation (TU) */
        double time_of_flight_check;
    };

    /**
     * @struct Config
     * @brief Configuration parameters for Lambert solver
     *
     * Allows fine-tuning of solver behavior for different applications
     * and convergence requirements.
     */
    struct Config {
        /** @brief Convergence tolerance for iterative solver */
        double tolerance;

        /** @brief Maximum iterations for solver convergence */
        int max_iterations;

        /** @brief Take the long way around (>180 degrees transfer) */
        bool long_way;

        /** @brief Enable debug output during solving */
        bool debug_mode;

        /**
         * @brief Default constructor with standard values
         *
         * Sets reasonable defaults for most orbital transfer applications:
         * - tolerance: 1e-8 (high precision)
         * - max_iterations: 100 (sufficient for convergence)
         * - long_way: false (short path transfer)
         * - debug_mode: false (quiet operation)
         */
        Config() : tolerance(1e-8), max_iterations(100), long_way(false), debug_mode(false) {}
    };

    /**
     * @brief Construct Lambert solver with gravitational parameter
     * @param mu Gravitational parameter (DU³/TU²)
     *
     * The gravitational parameter defines the central body's gravitational
     * field strength and the unit system used for calculations.
     */
    explicit LambertSolver(double mu);

    /**
     * @brief Solve Lambert's problem for given boundary conditions
     * @param r1 Initial position vector (DU)
     * @param r2 Final position vector (DU)
     * @param tof Time of flight (TU)
     * @param config Solver configuration parameters
     * @return Solution if found, empty optional if no solution exists
     *
     * Uses universal variable formulation with Stumpff functions for
     * robust convergence across all conic sections (elliptical, parabolic,
     * and hyperbolic trajectories).
     */
    std::optional<Solution> solve(const glm::dvec3& r1,
                                 const glm::dvec3& r2,
                                 double tof,
                                 const Config& config = Config()) const;

    /**
     * @brief Static method for simple Lambert solution
     * @param r1 Initial position vector (DU)
     * @param r2 Final position vector (DU)
     * @param tof Time of flight (TU)
     * @param mu Gravitational parameter (DU³/TU²)
     * @param isLongWay Whether to take the long way around (default false)
     * @return Pair of velocity vectors (departure, arrival)
     *
     * Simplified interface that returns only velocity vectors like the
     * original implementation. Maintained for backward compatibility
     * with existing code.
     */
    static std::pair<Physics::Vector3, Physics::Vector3> solveLambert(
       const Physics::Vector3& r1,
        const Physics::Vector3& r2,
        double tof,
        double mu,
        bool isLongWay = false);

    /**
     * @brief Calculate minimum energy transfer time between two positions
     * @param r1 Initial position vector (DU)
     * @param r2 Final position vector (DU)
     * @param long_way Whether to consider the long way transfer
     * @return Minimum time of flight for the transfer (TU)
     *
     * Computes the minimum energy (Hohmann-like) transfer time, which
     * provides a lower bound for any feasible transfer between the
     * given positions.
     */
    double calculateMinimumTOF(const glm::dvec3& r1,
                              const glm::dvec3& r2,
                              bool long_way = false) const;

    /**
     * @brief Validate a solution by checking conservation laws
     * @param solution Solution to validate
     * @param r1 Initial position vector
     * @param r2 Final position vector
     * @param expected_tof Expected time of flight
     * @return true if solution satisfies physical constraints
     *
     * Performs validation checks including:
     * - Energy conservation between departure and arrival
     * - Angular momentum conservation
     * - Time of flight consistency
     */
    bool validateSolution(const Solution& solution,
                         const glm::dvec3& r1,
                         const glm::dvec3& r2,
                         double expected_tof) const;

    
private:
    /** @brief Gravitational parameter for the central body */
    double mu_;

    /**
     * @brief Stumpff function C(z) for universal variable formulation
     * @param z Universal variable argument
     * @return Value of C(z)
     *
     * Stumpff function C(z) = (1 - cos(√z))/z for z > 0
     * Used in universal variable formulation for robust convergence.
     */
    double stumpffC(double z) const;

    /**
     * @brief Stumpff function S(z) for universal variable formulation
     * @param z Universal variable argument
     * @return Value of S(z)
     *
     * Stumpff function S(z) = (√z - sin(√z))/(√z)³ for z > 0
     * Used in universal variable formulation for robust convergence.
     */
    double stumpffS(double z) const;
};
} // namespace Physics

#endif // LAMBERT_SOLVER_HPP
