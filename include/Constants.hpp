#ifndef CONSTANT_HPP
#define CONSTANT_HPP

namespace constant {
    //Distance Unit : 1DU = 6378.165 km
    inline constexpr double Rmax = (1.5 * 6378.165)/6378.165;
    inline constexpr double MU = 1.0;

    // Init orbit transfer problem
    //inline constexpr double R1 = 16678.0/6378.165;
    //inline constexpr double R2 = 42164.0/6378.165;
    //inline constexpr double R1 = 7000.0/6378.165;
    //inline constexpr double R2 = 10000.0/6378.165;
    inline constexpr double R1 = 6378.165/6378.165;
    inline constexpr double R2 = (1.5 * 6378.165)/6378.165;

    // Eccentricities
    inline constexpr double E1 = 0.0;
    inline constexpr double E2 = 0.0;

    // Inclinations
    //inline constexpr double I1 = 0.497419;
    inline constexpr double I1 = 0.0;
    inline constexpr double I2 = 0.0;

    //Raan
    inline constexpr double RAAN1 = 0.0;
    inline constexpr double RAAN2 = 0.0;

    //Omega
    inline constexpr double OMEGA1 = 0.0;
    inline constexpr double OMEGA2 = 0.0;

}
#endif
