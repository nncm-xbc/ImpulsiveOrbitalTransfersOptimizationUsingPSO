#ifndef CONSTANT_HPP
#define CONSTANT_HPP

namespace constant {
    // Init orbit transfer problem
    //Distance Unit : 1DU = 6378.165 km
    inline constexpr double R1 = 16678.0/6378.165;
    inline constexpr double R2 = 42164.0/6378.165;
    inline constexpr double Rmax = 42164.0/6378.165;
    inline constexpr double MU = 398600.4418;
}
#endif
