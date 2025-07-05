/**
 * @file OrbitProblem.hpp
 * @brief Orbital transfer optimization problem definition
 * @author PSO Orbit Transfer Team
 * @date 2025
 *
 * Defines the orbital transfer optimization problem as an objective function
 * for PSO optimization. Handles various orbital configurations including
 * coplanar and non-coplanar transfers between circular and elliptical orbits.
 */

#ifndef ORBIT_HPP
#define ORBIT_HPP

#include <cmath>
#include <vector>
#include <map>
#include <string>
#include "core/Constants.hpp"
#include "core/OrbitMechanics.hpp"

template <typename T, typename Fun>
class PSO;  // Forward declaration

/**
 * @class OrbitTransferObjective
 * @brief Objective function for orbital transfer optimization using PSO
 * @tparam T Floating point type (typically double)
 * @tparam Fun Function type for the objective function
 *
 * This class encapsulates the orbital transfer optimization problem as an
 * objective function that can be minimized by PSO. It handles:
 * - Coplanar circular-to-circular transfers
 * - Non-coplanar circular-to-circular transfers
 * - Elliptical orbit transfers
 * - Constraint handling for feasible trajectories
 *
 * The optimization variables represent:
 * - x[0]: Departure true anomaly (0 to 2π)
 * - x[1]: Arrival true anomaly (0 to 2π)
 * - x[2]: Time of flight
 */
template<typename T, typename Fun>
class OrbitTransferObjective
{
    private:
        /** @brief Gravitational parameter (normalized) */
        const double _MU = constant::MU;

        /** @brief Initial orbit radius (DU) */
        double _R1;

        /** @brief Target orbit radius (DU) */
        double _R2;

        /** @brief Maximum allowed orbital radius (DU) */
        double _Rmax;

        /** @brief Initial orbit eccentricity */
        double _e1;

        /** @brief Target orbit eccentricity */
        double _e2;

        /** @brief Initial orbit inclination (radians) */
        double _i1;

        /** @brief Target orbit inclination (radians) */
        double _i2;

        /** @brief Initial orbit right ascension of ascending node (radians) */
        double _raan1;

        /** @brief Target orbit right ascension of ascending node (radians) */
        double _raan2;

        /** @brief Initial orbit argument of periapsis (radians) */
        double _omega1;

        /** @brief Target orbit argument of periapsis (radians) */
        double _omega2;

        /** @brief Current PSO iteration (for adaptive constraints) */
        int _currentIteration = 0;

        /** @brief Maximum PSO iterations (for adaptive constraints) */
        int _maxIterations = 1;

        /**
         * @brief Calculate total delta-V for a given parameter set
         * @param x Vector of optimization parameters
         * @return Total velocity change required (DU/TU)
         *
         * Core objective function calculation. Uses Lambert solver for
         * trajectory computation and handles both coplanar and non-coplanar cases.
         */
        double calculateDeltaV(const std::vector<double>& x);

        /**
         * @brief Check all trajectory constraints
         * @param x Vector of optimization parameters
         * @return Total constraint violation (0 = feasible)
         *
         * Comprehensive constraint checking including:
         * - Parameter bounds
         * - Transfer time limits
         * - Impulse magnitude limits
         * - Orbital intersection requirements
         * - Plane change requirements
         */
        double checkConstraints(const std::vector<double>& x, double deltaV);

        //PSO class is a friend :)
        template<typename Y, typename Fin>
        friend class PSO;

    public:
        /**
         * @brief Constructor for orbital transfer problem
         * @param R1 Initial orbit radius (DU)
         * @param R2 Target orbit radius (DU)
         * @param Rmax Maximum allowed radius (DU)
         * @param E1 Initial orbit eccentricity (default 0.0)
         * @param E2 Target orbit eccentricity (default 0.0)
         * @param I1 Initial orbit inclination in radians (default 0.0)
         * @param I2 Target orbit inclination in radians (default 0.0)
         * @param RAAN1 Initial orbit RAAN in radians (default 0.0)
         * @param RAAN2 Target orbit RAAN in radians (default 0.0)
         * @param OMEGA1 Initial orbit argument of periapsis in radians (default 0.0)
         * @param OMEGA2 Target orbit argument of periapsis in radians (default 0.0)
         *
         * Sets up the orbital transfer problem with specified initial and
         * target orbit parameters. Defaults to circular coplanar case.
         */
        OrbitTransferObjective(double R1, double R2, double Rmax,
                    double E1 = 0.0, double E2 = 0.0,
                    double I1 = 0.0, double I2 = 0.0,
                    double RAAN1 = 0.0, double RAAN2 = 0.0,
                    double OMEGA1 = 0.0, double OMEGA2 = 0.0);

        /**
         * @brief Function call operator for PSO optimization
         * @param x Array of optimization parameters
         * @return Objective function value (total ΔV)
         *
         * Main interface called by PSO algorithm. Converts parameter
         * array to vector and calls calculateDeltaV.
         */
        double operator()(double* x);

        /**
         * @brief Get initial orbit eccentricity
         * @return Initial orbit eccentricity
         */
        double getE1() const;

        /**
         * @brief Get target orbit eccentricity
         * @return Target orbit eccentricity
         */
        double getE2() const;

        /**
         * @brief Get initial orbit inclination
         * @return Initial orbit inclination (radians)
         */
        double getI1() const;

        /**
         * @brief Get target orbit inclination
         * @return Target orbit inclination (radians)
         */
        double getI2() const;

        /**
         * @brief Extract detailed transfer information from optimal solution
         * @param x Vector of optimization parameters (optimal solution)
         * @return Map containing detailed transfer characteristics
         *
         * Provides comprehensive information about the optimal transfer including:
         * - Transfer orbit elements
         * - Impulse details
         * - Plane change distribution
         * - Transfer type classification
         */
        std::map<std::string, double> getTransferDetails(const std::vector<double>& x);

        /**
         * @brief Get constraint relaxation factor for adaptive optimization
         * @return Relaxation factor (decreases from 2.0 to 1.0 during optimization)
         *
         * Implements adaptive constraint handling that starts with relaxed
         * constraints and gradually tightens them as optimization progresses.
         * Helps avoid premature convergence to infeasible regions.
         */
        double getRelaxationFactor() const;

        double calculateMinimumTransferTime(double theta1, double theta2);

        bool orbitsIntersect();

};

#endif
