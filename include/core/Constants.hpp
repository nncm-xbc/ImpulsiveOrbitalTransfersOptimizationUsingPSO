/**
 * @file Constants.hpp
 * @brief Physical and mathematical constants for orbital mechanics calculations
 * @author PSO Orbit Transfer Team
 * @date 2025
 */

#ifndef CONSTANT_HPP
#define CONSTANT_HPP

/**
 * @namespace constant
 * @brief Contains physical constants and orbital parameters for the transfer problem
 *
 * All distance units are normalized: 1 DU = 6378.165 km (Earth's equatorial radius)
 * All time units are normalized using the gravitational parameter
 */
namespace constant {
    /** @brief Maximum orbital radius constraint (normalized) */
    inline constexpr double Rmax = (1.5 * 6378.165)/6378.165;

    /** @brief Normalized gravitational parameter (DU³/TU²) */
    inline constexpr double MU = 1.0;

    // Init orbit transfer problem
    //inline constexpr double R1 = 16678.0/6378.165;
    //inline constexpr double R2 = 42164.0/6378.165;
    //inline constexpr double R1 = 7000.0/6378.165;
    //inline constexpr double R2 = 10000.0/6378.165;

    /** @brief Initial orbit radius (normalized) */
    inline constexpr double R1 = 6378.165/6378.165;

    /** @brief Target orbit radius (normalized) */
    inline constexpr double R2 = (1.5 * 6378.165)/6378.165;

    // Eccentricities
    /** @brief Initial orbit eccentricity */
    inline constexpr double E1 = 0.0;

    /** @brief Target orbit eccentricity */
    inline constexpr double E2 = 0.0;

    // Inclinations
    /** @brief Initial orbit inclination (radians) */
    //inline constexpr double I1 = 0.497419;
    inline constexpr double I1  = 0.0;

    /** @brief Target orbit inclination (radians) */
    inline constexpr double I2 = 0.0;

    //Raan
    /** @brief Initial orbit right ascension of ascending node (radians) */
    inline constexpr double RAAN1 = 0.0;

    /** @brief Target orbit right ascension of ascending node (radians) */
    inline constexpr double RAAN2 = 0.0;

    //Omega
    /** @brief Initial orbit argument of periapsis (radians) */
    inline constexpr double OMEGA1 = 0.0;

    /** @brief Target orbit argument of periapsis (radians) */
    inline constexpr double OMEGA2 = 0.0;

}
#endif
