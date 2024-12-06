#ifndef ORBIT_HPP
#define ORBIT_HPP

#include <vector>

template<typename T, typename Fun>
class OrbitTransferObjective
{
    private:
        double _R1; // Initial radius
        double _R2; // Final radius
        double _Rmax; // Maximum allowed radius
        double _MU = 398600.4418;

        std::pair<T, T> calculateVelocity(double r);

    public:
        OrbitTransferObjective(double r1, double r2, double rmax);

        double operator()(double* x);
        double calculateDeltaV(const std::vector<double>& x);
        bool checkConstraints(const std::vector<double>& x);
        double computePeriapsis(const std::vector<T>& x);
        double computeApoapsis(const std::vector<T>& x);
};

#endif
