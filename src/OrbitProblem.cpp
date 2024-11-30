#include "OrbitProblem.hpp"

#include <vector>
#include <cmath>

template <typename T, typename Fun>
OrbitTransferObjective<T, Fun>::OrbitTransferObjective(double r1, double r2, double rmax): _R1(r1), _R2(r2), _Rmax(rmax) {}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::calculateImpulse(double r1, double r2, double theta) {
    double v1 = std::sqrt(_mu/r1);
    double v2 = std::sqrt(_mu/r2) * sqrt((2*theta)/(r1 + r2));
    return std::abs(v2 - v1);
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::calculateDeltaV(const std::vector<double>& x, size_t dim) {
    double deltaV = 0.0;
    if (!checkConstraints(x)) {
        return std::numeric_limits<double>::max(); // Penalty
    }

    // Calculate velocity changes at each impulse point
    double deltaV1 = calculateImpulse(_R1, x[0], x[2]);
    double deltaV2 = calculateImpulse(x[0], x[1], x[3]);
    double deltaV3 = calculateImpulse(x[1], _R2, x[4]);

    return deltaV1 + deltaV2 + deltaV3;
}

template <typename T, typename Fun>
bool OrbitTransferObjective<T, Fun>::checkConstraints(const std::vector<double>& x) {
    // Constraint 1: Maximum radius constraint
    if (x[0] > _Rmax || x[1] > _Rmax) {
        return false;
    }

    // Constraint 2: Impulses must occur at apse points
    /*
    if (x[2] < 0 || x[2] > M_PI || x[3] < 0 || x[3] > M_PI || x[4] < 0 || x[4] > M_PI) {
        return false;
    }
    */

    // Constraint 3: Ensure R2/R1 ratio is within proper bounds
    double ratio = _R2/_R1;
    if (ratio < 0.08376 || ratio > 11.939) {
        // This range determines if Hohmann transfer is optimal
        return false;
    }

    return true;
}
