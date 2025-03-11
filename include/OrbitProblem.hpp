#ifndef ORBIT_HPP
#define ORBIT_HPP

#include <vector>
#include "constants.hpp"

template<typename T, typename Fun>
class OrbitTransferObjective
{
    private:
        std::pair<T, T> calculateVelocity(double r);

    public:
        double _R1; // Initial radius
        double _R2; // Final radius
        double _MU;
        double _Rmax; // Maximum allowed

        OrbitTransferObjective(double r1, double r2, double rmax);

        double operator()(double* x);
        double calculateDeltaV(const std::vector<double>& x);
        bool checkConstraints(const std::vector<double>& x);
        double computePeriapsis(const std::vector<T>& x);
        double computeApoapsis(const std::vector<T>& x);
};

#endif
