#include "OrbitProblem.hpp"

#include <vector>
#include <cmath>
#include <functional>
#include <iostream>

template <typename T, typename Fun>
OrbitTransferObjective<T, Fun>::OrbitTransferObjective(double _R1, double _R2, double _Rmax): _R1(_R1), _R2(_R2), _Rmax(_Rmax) {}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::operator()(double* x)
{
    std::vector<double> params(x, x + 4);
    return calculateDeltaV(params);
}

template<typename T, typename Fun>
std::pair<T, T> OrbitTransferObjective<T, Fun>::calculateVelocity(double r)
{
    double v_r = 0.0;
    double v_theta = std::sqrt(_MU/r);
    return std::make_pair(v_r, v_theta);
}

template <typename T, typename Fun>

double OrbitTransferObjective<T, Fun>::calculateDeltaV(const std::vector<double>& x)
{
    double deltaV = 0.0;
    if (!checkConstraints(x))
    {
        // Penalty
        return 1e6;
    }

    auto v_init = calculateVelocity(_R1); // Initial orbit velocity

    double v1_r = v_init.first + x[0] * std::sin(x[2]); // Radial velocity after first impulse
    double v1_theta = v_init.second + x[0] * std::cos(x[2]); // elocity after first impulse

    auto v_final = calculateVelocity(_R2); // Final orbit velocity

    // Calculate total deltaV
    double deltaV1 = std::sqrt(std::pow(v1_r - v_init.first, 2) +
                                std::pow(v1_theta - v_init.second, 2));

    double deltaV2 = std::sqrt(std::pow(v_final.first - v1_r, 2) +
                                std::pow(v_final.second - v1_theta, 2));

    return deltaV1 + deltaV2;
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::computePeriapsis(const std::vector<T>& x)
{
    auto v_init = calculateVelocity(_R1); // Initial orbit velocity
    double v1_r = v_init.first + x[0] * std::sin(x[2]); // Radial velocity after first impulse
    double v1_theta = v_init.second + x[0] * std::cos(x[2]); // Tangential velocity after first impulse
    double vt = std::sqrt(v1_r*v1_r + v1_theta*v1_theta); // Transfer orbit velocity at periapsis
    double h = _R1 * v1_theta; // specific angular momentum of transfer orbit
    double e = std::sqrt(1 + (h*h/(_MU*_R1))*(vt*vt/(2*_MU) - 1/_R1)); // eccentricity of transfer orbit
    double a = h*h/(_MU*(1-e*e)); // semi-major axis

    return a * (1-e);
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::computeApoapsis(const std::vector<T>& x)
{
    auto v_init = calculateVelocity(_R1); // Initial orbit velocity
    double v1_r = v_init.first + x[0] * std::sin(x[2]); // Radial velocity after first impulse
    double v1_theta = v_init.second + x[0] * std::cos(x[2]); // Tangential velocity after first impulse
    double vt = std::sqrt(v1_r*v1_r + v1_theta*v1_theta); // Transfer orbit velocity at periapsis
    double h = _R1 * v1_theta; // specific angular momentum of transfer orbit
    double e = std::sqrt(1 + (h*h/(_MU*_R1))*(vt*vt/(2*_MU) - 1/_R1)); // eccentricity of transfer orbit
    double a = h*h/(_MU*(1-e*e)); // semi-major axis

    return a * (1+e);
}

template <typename T, typename Fun>
bool OrbitTransferObjective<T, Fun>::checkConstraints(const std::vector<double>& x)
{
    if (_R2 <= _R1) return false;

    double transferPeriapsis = computePeriapsis(x);
    double transferApoapsis = computeApoapsis(x);

    // Check if transfer orbit intersects both initial and final orbits
    if (transferPeriapsis > std::min(_R1, _R2) ||
        transferApoapsis < std::max(_R1, _R2)) return false;

    return true;
}

// Explicit instantiation
template class OrbitTransferObjective<double, std::function<double(double*)>>;
