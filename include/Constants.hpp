#ifndef CONSTANT_HPP
#define CONSTANT_HPP

namespace constant {
    //Distance Unit : 1DU = 6378.165 km
    inline constexpr double Rmax = 42164.0/6378.165;
    inline constexpr double MU = 398600.4418;

    // Init orbit transfer problem
    //inline constexpr double R1 = 16678.0/6378.165;
    //inline constexpr double R2 = 42164.0/6378.165;
    inline constexpr double R1 = 7000.0/6378.165;
    inline constexpr double R2 = 10000.0/6378.165;

    // Eccentricities
    inline constexpr double E1 = 0.0;
    inline constexpr double E2 = 0.0;

    // Inclinations
    inline constexpr double I1 = 0.0;
    inline constexpr double I2 = 0.0;

}
#endif
