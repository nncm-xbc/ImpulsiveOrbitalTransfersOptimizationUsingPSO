#ifndef ORBIT_HPP
#define ORBIT_HPP

#include <vector>

template<typename T, typename Fun>
class OrbitTransferObjective {
private:
    double _R1; // Initial radius
    double _R2; // Final radius
    double _Rmax; // Maximum allowed radius
    double _mu = 398600.4418;

    double calculateImpulse(double r1, double r2, double theta);

public:
    OrbitTransferObjective(double r1, double r2, double rmax);

    double calculateDeltaV(const std::vector<double>& x, size_t dim);
    bool checkConstraints(const std::vector<double>& x);
};

#endif
