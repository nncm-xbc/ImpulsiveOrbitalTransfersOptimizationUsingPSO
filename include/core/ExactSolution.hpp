/**
 * @file ExactSolution.hpp
 * @brief Analytical solutions for orbital transfer problems
 * @author PSO Orbit Transfer Team
 * @date 2025
 *
 * Provides exact analytical solutions for specific orbital transfer cases,
 * primarily used for validation of numerical optimization results.
 */

#include "core/OrbitProblem.hpp"

/**
 * @class HohmannSolution
 * @brief Calculates the exact Hohmann transfer solution between two circular coplanar orbits
 * @tparam T Floating point type (float or double)
 *
 * The Hohmann transfer is the minimum energy two-impulse transfer between two circular
 * coplanar orbits. This class provides the analytical solution for validation purposes.
 *
 * The Hohmann transfer is globally optimal only when the radius ratio is within
 * specific bounds (approximately 0.08376 ≤ R₂/R₁ ≤ 15.582). Outside these bounds,
 * a three-impulse bi-elliptic transfer may be more efficient.
 */
template<typename T>
class HohmannSolution {
private:
    /** @brief Flag indicating if Hohmann transfer is globally optimal for given radii */
    bool _isValid;

public:
    /** @brief Initial orbit radius */
    T _R1;

    /** @brief Target orbit radius */
    T _R2;

    /** @brief Gravitational parameter */
    T _MU;

    /**
     * @brief Constructor for Hohmann transfer solution
     * @param R1 Initial orbit radius
     * @param R2 Target orbit radius
     * @param MU Gravitational parameter (default 1.0)
     *
     * Automatically determines if the Hohmann transfer is globally optimal
     * based on the radius ratio bounds.
     */
    HohmannSolution(T R1, T R2, T MU = 1.0): _R1(R1), _R2(R2), _MU(MU) {
        T ratio = R2/R1;
        _isValid = (ratio >= 0.08376 && ratio <= 15.582);
    }

    /**
     * @struct Solution
     * @brief Contains the complete Hohmann transfer solution
     */
    struct Solution {
        /** @brief Total velocity change required (km/s) */
        T deltaV;

        /** @brief First impulse magnitude (km/s) */
        T deltaV1;

        /** @brief Second impulse magnitude (km/s) */
        T deltaV2;

        /** @brief Whether this solution is globally optimal */
        bool isOptimal;
    };

    /**
     * @brief Compute the Hohmann transfer solution
     * @return Complete solution structure with ΔV components
     *
     * Calculates the analytical Hohmann transfer using:
     * - ΔV₁ = √(μ(2/r₁ - 1/a)) - √(μ/r₁)
     * - ΔV₂ = √(μ/r₂) - √(μ(2/r₂ - 1/a))
     * where a = (r₁ + r₂)/2 is the transfer orbit semi-major axis
     */
    Solution compute() const {
        Solution sol;
        sol.isOptimal = _isValid;

        // Circular orbit velocities
        T v1 = std::sqrt(_MU/_R1);
        T v2 = std::sqrt(_MU/_R2);

        // Hohmann transfer orbit semi-major axis
        T a = (_R1 + _R2)/2.0;

        sol.deltaV1 = std::sqrt(_MU*(2.0/_R1 - 1.0/a)) - v1;
        sol.deltaV2 = v2 - std::sqrt(_MU*(2.0/_R2 - 1.0/a));
        sol.deltaV = std::abs(sol.deltaV1) + std::abs(sol.deltaV2);

        return sol;
    }

    /**
     * @brief Calculate the relative error between numerical and analytical solutions
     * @param numericalDeltaV Total ΔV from numerical optimization
     * @return Relative error as a fraction (0.0 = perfect match)
     *
     * Returns -1.0 if the Hohmann transfer is not globally optimal for this case.
     * Used to validate PSO optimization results against known analytical solutions.
     */
    T getError(T numericalDeltaV) const {
        if (!_isValid) return -1.0;
        Solution exactSol = compute();
        return std::abs(numericalDeltaV - exactSol.deltaV)/exactSol.deltaV;
    }
};

//explicit instantiation
template class HohmannSolution<double>;
