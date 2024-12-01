#include "OrbitProblem.hpp"

#include <vector>
#include <cmath>
#include <functional>

template <typename T, typename Fun>
OrbitTransferObjective<T, Fun>::OrbitTransferObjective(double _R1, double _R2, double _Rmax): _R1(_R1), _R2(_R2), _Rmax(_Rmax) {}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::operator()(double* x, size_t dim) {
    std::vector<double> params(x, x + dim);
    return calculateDeltaV(params, dim);
}

template<typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::calculateVelocity(double r) {
    return std::sqrt(_mu/r);
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::calculateTransferVelocity(double _R1, double _R2, double theta) {
    return std::sqrt(_mu/_R2) * sqrt((2*theta)/(_R1 + _R2));
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::calculateDeltaV(const std::vector<double>& x, size_t dim) {
    double deltaV = 0.0;
    if (!checkConstraints(x)) {
        // Penalty
        return 1e6;
    }

    if (dim==2) {
        // First impulse
        double v1_init = calculateVelocity(_R1);
        double v1_transfer = calculateTransferVelocity(_R1, x[0], x[1]);
        deltaV += std::abs(v1_transfer - v1_init);

        // Second impulse
        double v2_transfer = calculateTransferVelocity(_R2, x[0], x[1]);
        double v2_final = calculateVelocity(_R2);
        deltaV += std::abs(v2_final - v2_transfer);
    }
    else if (dim=3) {
        // First impulse at initial orbit
        double v1_init = calculateVelocity(_R1);
        double v1_transfer = calculateTransferVelocity(_R1, x[0], x[1]);
        deltaV += std::abs(v1_transfer - v1_init);

        // Second impulse at intermediate orbit
        double v2_transfer = calculateTransferVelocity(x[0], x[1], x[2]);
        deltaV += std::abs(v2_transfer - v1_transfer);

        // Third impulse to final orbit
        double v3_transfer = calculateTransferVelocity(_R2, x[2], 0);
        double v3_final = calculateVelocity(_R2);
        deltaV += std::abs(v3_final - v3_transfer);
    }

    return deltaV;
}

template <typename T, typename Fun>
bool OrbitTransferObjective<T, Fun>::checkConstraints(const std::vector<double>& x) {

    if (x[0] < _R1 || x[0] > _Rmax) return false;
    if (x[1] < 0.0 || x[1] > 2*M_PI) return false;

    // Feasibility check for transfer orbit
    double transferPeriapsis = x[0] * (1.0 - std::cos(x[1]));
    double transferApoapsis = x[0] * (1.0 + std::cos(x[1]));

    // Check if transfer orbit intersects both initial and final orbits
    if (transferPeriapsis > std::min(_R1, _R2) ||
        transferApoapsis < std::max(_R1, _R2)) return false;

    return true;
}

// Explicit instantiation
template class OrbitTransferObjective<double, std::function<double(double*, size_t)>>;
