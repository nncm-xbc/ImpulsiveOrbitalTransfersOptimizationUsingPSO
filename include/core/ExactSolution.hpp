/**
 * @file ExactSolution.hpp
 * @brief Analytical solutions for orbital transfer problems
 * @author PSO Orbit Transfer Team
 * @date 2025
 *
 * Provides exact analytical solutions for specific orbital transfer cases,
 * primarily used for validation of numerical optimization results.
 */

#ifndef EXACT_SOLUTION_HPP
#define EXACT_SOLUTION_HPP

#include <cmath>
#include <iostream>
#include <iomanip>
#include "core/Constants.hpp"

/**
 * @class HohmannSolution
 * @brief Calculates the exact Hohmann transfer solution between two circular coplanar orbits
 * @tparam T Floating point type (float or double)
 *
 * The Hohmann transfer is the minimum energy two-impulse transfer between two circular
 * coplanar orbits. This class provides the analytical solution for validation purposes.
 */
template<typename T>
class HohmannSolution {
private:
    /** @brief Initial orbit radius (normalized DU) */
    T _R1;

    /** @brief Target orbit radius (normalized DU) */
    T _R2;

    /** @brief Gravitational parameter (normalized) */
    T _MU;

    /** @brief Flag indicating if Hohmann transfer is globally optimal */
    bool _isValid;

public:
    /**
     * @struct Solution
     * @brief Contains the complete Hohmann transfer solution
     */
    struct Solution {
        /** @brief Total velocity change required (normalized DU/TU) */
        T deltaV;

        /** @brief First impulse magnitude (normalized DU/TU) */
        T deltaV1;

        /** @brief Second impulse magnitude (normalized DU/TU) */
        T deltaV2;

        /** @brief Transfer time (normalized TU) */
        T transferTime;

        /** @brief Transfer orbit semi-major axis (normalized DU) */
        T transferSemiMajor;

        /** @brief Transfer orbit eccentricity */
        T transferEccentricity;

        /** @brief Transfer orbit period (normalized TU) */
        T transferPeriod;

        /** @brief Whether this solution is globally optimal */
        bool isOptimal;

        /** @brief Radius ratio R2/R1 */
        T radiusRatio;

        /** @brief Efficiency compared to single impulse */
        T efficiency;
    };

    /**
     * @brief Constructor for Hohmann transfer solution
     * @param R1 Initial orbit radius (normalized DU)
     * @param R2 Target orbit radius (normalized DU)
     * @param MU Gravitational parameter (normalized, default 1.0)
     */
    HohmannSolution(T R1, T R2, T MU = 1.0) : _R1(R1), _R2(R2), _MU(MU) {
        T ratio = R2/R1;
        // Hohmann is globally optimal when:
        // - For inner transfers: ratio >= 0.08376 (approximately)
        // - For outer transfers: ratio <= 15.582 (approximately)
        _isValid = (ratio >= 0.08376 && ratio <= 15.582);
    }

    /**
     * @brief Compute the complete Hohmann transfer solution
     * @return Complete solution structure with all transfer parameters
     */
    Solution compute() const {
        Solution sol;
        sol.isOptimal = _isValid;
        sol.radiusRatio = _R2 / _R1;

        T v1 = std::sqrt(_MU / _R1);
        T v2 = std::sqrt(_MU / _R2);

        sol.transferSemiMajor = (_R1 + _R2) / 2.0;
        sol.transferEccentricity = std::abs(_R2 - _R1) / (_R1 + _R2);
        sol.transferPeriod = 2.0 * M_PI * std::sqrt(std::pow(sol.transferSemiMajor, 3) / _MU);
        sol.transferTime = sol.transferPeriod / 2.0;

        // At periapsis (R1): v = sqrt(μ * (2/r - 1/a))
        T v1_transfer = std::sqrt(_MU * (2.0/_R1 - 1.0/sol.transferSemiMajor));

        // At apoapsis (R2): v = sqrt(μ * (2/r - 1/a))
        T v2_transfer = std::sqrt(_MU * (2.0/_R2 - 1.0/sol.transferSemiMajor));

        sol.deltaV1 = v1_transfer - v1;
        sol.deltaV2 = v2 - v2_transfer;
        sol.deltaV = (std::abs(sol.deltaV1) + std::abs(sol.deltaV2)) * constant::VU;

        T singleImpulse = std::abs(v2 - v1);
        sol.efficiency = singleImpulse / sol.deltaV;

        return sol;
    }

    /**
     * @brief Calculate the relative error between numerical and analytical solutions
     * @param numericalDeltaV Total ΔV from numerical optimization (normalized DU/TU)
     * @return Relative error as a fraction (0.0 = perfect match)
     */
    T getDeltaVError(T numericalDeltaV) const {
        if (!_isValid) return -1.0;
        Solution exactSol = compute();
        return std::abs(numericalDeltaV - exactSol.deltaV) / exactSol.deltaV;
    }

    /**
     * @brief Calculate the relative error in transfer time
     * @param numericalTime Transfer time from numerical optimization (normalized TU)
     * @return Relative error as a fraction (0.0 = perfect match)
     */
    T getTimeError(T numericalTime) const {
        if (!_isValid) return -1.0;
        Solution exactSol = compute();
        return std::abs(numericalTime - exactSol.transferTime) / exactSol.transferTime;
    }

    /**
     * @brief Validate numerical solution against analytical Hohmann
     * @param numericalDeltaV Total ΔV from PSO (normalized DU/TU)
     * @param numericalTime Transfer time from PSO (normalized TU)
     * @param toleranceDeltaV Tolerance for ΔV error (default 0.1 = 10%)
     * @param toleranceTime Tolerance for time error (default 0.2 = 20%)
     * @return true if numerical solution is within tolerance of analytical
     */
    bool validateSolution(T numericalDeltaV, T numericalTime,
                         T toleranceDeltaV = 0.1, T toleranceTime = 0.2) const {
        if (!_isValid) return false;

        T deltaVError = getDeltaVError(numericalDeltaV);
        T timeError = getTimeError(numericalTime);

        return (deltaVError <= toleranceDeltaV) && (timeError <= toleranceTime);
    }

    /**
     * @brief Print detailed comparison between numerical and analytical solutions
     * @param numericalDeltaV Total ΔV from PSO (normalized DU/TU)
     * @param numericalTime Transfer time from PSO (normalized TU)
     * @param numericalTheta0 Departure true anomaly from PSO (radians)
     * @param numericalThetaF Arrival true anomaly from PSO (radians)
     */
    void printValidationReport(T numericalDeltaV, T numericalTime,
                              T numericalTheta0, T numericalThetaF) const {

        Solution analytical = compute();

        std::cout << "\n╔══════════════════════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║                        ANALYTICAL VALIDATION                         ║" << std::endl;
        std::cout << "╠══════════════════════════════════════════════════════════════════════╣" << std::endl;
        std::cout << "║ TRANSFER CONFIGURATION:                                              ║" << std::endl;
        std::cout << "║   • Initial Radius (R₁):       " << std::setw(12) << std::fixed << std::setprecision(3)
                  << _R1 << " DU" << std::setw(26) << " ║" << std::endl;
        std::cout << "║   • Target Radius (R₂):        " << std::setw(12) << std::fixed << std::setprecision(3)
                  << _R2 << " DU" << std::setw(26) << " ║" << std::endl;
        std::cout << "║   • Radius Ratio (R₂/R₁):       " << std::setw(12) << std::fixed << std::setprecision(3)
                  << analytical.radiusRatio << std::setw(28) << " ║" << std::endl;
        std::cout << "║   • Hohmann Globally Optimal:  " << std::setw(12)
                  << (_isValid ? "YES" : "NO") << std::setw(29) << " ║" << std::endl;
        std::cout << "╠══════════════════════════════════════════════════════════════════════╣" << std::endl;
        std::cout << "║ DELTA-V COMPARISON:                                                  ║" << std::endl;
        std::cout << "║   • Analytical Total ΔV:       " << std::setw(12) << std::fixed << std::setprecision(3)
                  << analytical.deltaV << " TU/DU" << std::setw(23) << " ║" << std::endl;
        std::cout << "║   • PSO Total ΔV:              " << std::setw(12) << std::fixed << std::setprecision(3)
                  << numericalDeltaV << " TU/DU" << std::setw(23) << " ║" << std::endl;
        T deltaVError = getDeltaVError(numericalDeltaV);
        std::cout << "║   • ΔV Error:                  " << std::setw(12) << std::fixed << std::setprecision(2)
                  << deltaVError * 100.0 << "%" << std::setw(28) << " ║" << std::endl;
        std::cout << "╠══════════════════════════════════════════════════════════════════════╣" << std::endl;
        std::cout << "║ TRANSFER TIME COMPARISON:                                            ║" << std::endl;
        std::cout << "║   • Analytical Time:           " << std::setw(12) << std::fixed << std::setprecision(3)
                  << analytical.transferTime * constant::TU / 3600.0 << " hours" << std::setw(23) << " ║" << std::endl;
        std::cout << "║   • PSO Time:                  " << std::setw(12) << std::fixed << std::setprecision(3)
                  << numericalTime * constant::TU / 3600.0 << " hours" << std::setw(23) << " ║" << std::endl;
        T timeError = getTimeError(numericalTime);
        std::cout << "║   • Time Error:                " << std::setw(12) << std::fixed << std::setprecision(2)
                  << timeError * 100.0 << "%" << std::setw(28) << " ║" << std::endl;
        std::cout << "╠══════════════════════════════════════════════════════════════════════╣" << std::endl;
        std::cout << "║ TRANSFER GEOMETRY:                                                   ║" << std::endl;
        std::cout << "║   • Analytical θ₀ (optimal):   " << std::setw(12) << std::fixed << std::setprecision(1)
                  << "0.0°" << std::setw(30) << " ║" << std::endl;
        std::cout << "║   • Analytical θf (optimal):   " << std::setw(12) << std::fixed << std::setprecision(1)
                  << "180.0°" << std::setw(30) << " ║" << std::endl;
        std::cout << "║   • PSO θ₀:                    " << std::setw(12) << std::fixed << std::setprecision(1)
                  << numericalTheta0 * 180.0 / M_PI << "°" << std::setw(28) << " ║" << std::endl;
        std::cout << "║   • PSO θf:                    " << std::setw(12) << std::fixed << std::setprecision(1)
                  << numericalThetaF * 180.0 / M_PI << "°" << std::setw(28) << " ║" << std::endl;
        std::cout << "╚══════════════════════════════════════════════════════════════════════╝" << std::endl;
    }

    /**
     * @brief Get transfer orbit parameters
     * @return Solution containing transfer orbit details
     */
    Solution getTransferOrbitParams() const {
        return compute();
    }
};

// Explicit instantiation
template class HohmannSolution<double>;
template class HohmannSolution<float>;

#endif // EXACT_SOLUTION_HPP
