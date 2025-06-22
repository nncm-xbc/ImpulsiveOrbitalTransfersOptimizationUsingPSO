#ifndef LAMBERT_SOLVER_HPP
#define LAMBERT_SOLVER_HPP

#include <glm/glm.hpp>
#include <utility>
#include <optional>

#include "core/OrbitMechanics.hpp"

namespace Physics {

/**
 * @brief Solves Lambert's problem: finding the orbit connecting two positions in a given time
 *
 * This is a pure physics implementation with no visualization dependencies.
 * All calculations are in computational units (DU for distance, TU for time).
 */
class LambertSolver {
public:
    /**
     * @brief Solution to Lambert's problem
     */
    struct Solution {
        glm::dvec3 v1;          ///< Velocity at departure point
        glm::dvec3 v2;          ///< Velocity at arrival point
        double semi_major_axis; ///< Semi-major axis of transfer orbit
        double eccentricity;    ///< Eccentricity of transfer orbit
        bool is_valid;          ///< Whether solution is physically valid

        // Optional debug information
        double transfer_angle;  ///< True anomaly change
        double time_of_flight_check; ///< Computed TOF for validation
    };

    /**
     * @brief Configuration for Lambert solver
     */
    struct Config {
        double tolerance;      ///< Convergence tolerance
        int max_iterations;     ///< Maximum iterations for solver
        bool long_way;        ///< Take the long way around (>180 degrees)
        bool debug_mode;      ///< Enable debug output

        Config() : tolerance(1e-8), max_iterations(100), long_way(false), debug_mode(false) {}
    };

    /**
     * @brief Construct Lambert solver with gravitational parameter
     * @param mu Gravitational parameter (DU³/TU²)
     */
    explicit LambertSolver(double mu);

    /**
     * @brief Solve Lambert's problem
     * @param r1 Initial position vector (DU)
     * @param r2 Final position vector (DU)
     * @param tof Time of flight (TU)
     * @param config Solver configuration
     * @return Solution if found, empty optional otherwise
     */
    std::optional<Solution> solve(const glm::dvec3& r1,
                                 const glm::dvec3& r2,
                                 double tof,
                                 const Config& config = Config()) const;

    /**
     * @brief Static method for simple Lambert solution (backward compatibility)
     * Returns pair of velocity vectors like the original implementation
     */
    static std::pair<glm::dvec3, glm::dvec3> solveLambert(
        const glm::dvec3& r1,
        const glm::dvec3& r2,
        double tof,
        double mu,
        bool isLongWay = false);

    /**
     * @brief Calculate minimum energy transfer time
     * @param r1 Initial position vector
     * @param r2 Final position vector
     * @param long_way Whether to take the long way
     * @return Minimum time of flight for the transfer
     */
    double calculateMinimumTOF(const glm::dvec3& r1,
                              const glm::dvec3& r2,
                              bool long_way = false) const;

    /**
     * @brief Validate a solution by checking time of flight
     * @param solution Solution to validate
     * @param r1 Initial position
     * @param r2 Final position
     * @param expected_tof Expected time of flight
     * @return true if solution is valid within tolerance
     */
    bool validateSolution(const Solution& solution,
                         const glm::dvec3& r1,
                         const glm::dvec3& r2,
                         double expected_tof) const;

private:
    double mu_;  ///< Gravitational parameter

    // Helper functions for universal variable formulation
    double stumpffC(double z) const;
    double stumpffS(double z) const;
};
} // namespace Physics

#endif // LAMBERT_SOLVER_HPP
