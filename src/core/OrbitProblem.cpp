#include <cmath>
#include <vector>
#include <functional>
#include <iostream>
#include <map>
#include <assert.h>
#include <glm/glm.hpp>

#include "core/OrbitProblem.hpp"
#include "optimization/PSO.hpp"
#include "optimization/PSOGlobals.hpp"
#include "visualization/TransferModel.hpp"
#include "core/LambertSolver.hpp"
#include "core/OrbitMechanics.hpp"

template <typename T, typename Fun>
OrbitTransferObjective<T, Fun>::OrbitTransferObjective(double R1, double R2, double Rmax,
                                            double e1, double e2,
                                            double i1, double i2,
                                            double raan1, double raan2,
                                            double omega1, double omega2):
                                            _R1(R1), _R2(R2), _Rmax(Rmax),
                                            _e1(e1), _e2(e2),
                                            _i1(i1), _i2(i2),
                                            _raan1(raan1), _raan2(raan2),
                                            _omega1(omega1), _omega2(omega2){}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::operator()(double* x)
{
    std::vector<double> params(x, x + 6);

    return calculateDeltaV(params);
}


template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::calculateDeltaV(const std::vector<double>& x)
{
    double deltaV = 0.0;

    double departureTrueAnomaly = x[0];
    double arrivalTrueAnomaly = x[1];
    double firstImpulseMagnitude = x[2];
    double secondImpulseMagnitude = x[3];
    double firstImpulseDirection = x[4];
    double transferTime = x[5];

    if (_i1 != 0 || _i2 != 0)
    {
        // Non-coplanar case
        double r_init = Physics::OrbitMechanics::calculateRadius(_R1, _e1, departureTrueAnomaly);
        double r_final = Physics::OrbitMechanics::calculateRadius(_R2, _e2, arrivalTrueAnomaly);

        // Position vectors in 3D space
        Physics::Vector3 r_init_vec = Physics::OrbitMechanics::calculatePosition3D(_R1, _e1, _i1, _raan1, _omega1, departureTrueAnomaly);
        Physics::Vector3 r_final_vec = Physics::OrbitMechanics::calculatePosition3D(_R2, _e2, _i2, _raan2, _omega2, arrivalTrueAnomaly);

        Physics::Vector3 v_init_vec = Physics::OrbitMechanics::calculateVelocity3D(_R1, _e1, _i1, _raan1, _omega1, departureTrueAnomaly);
        Physics::Vector3 v_final_vec = Physics::OrbitMechanics::calculateVelocity3D(_R2, _e2, _i2, _raan2, _omega2, arrivalTrueAnomaly);

        // Solve for transfer orbit velocities
        std::pair<Physics::Vector3, Physics::Vector3> lambert_result = Physics::LambertSolver::solveLambert(
            r_init_vec,
            r_final_vec,
            transferTime,
            _MU,
            false
        );

        Physics::Vector3 v_trans_dep = lambert_result.first;
        Physics::Vector3 v_trans_arr = lambert_result.second;

        // Impulse vectors and magnitudes
        Physics::Vector3 delta_v1 = v_trans_dep - v_init_vec;
        Physics::Vector3 delta_v2 = v_final_vec - v_trans_arr;

        double deltaV1 = delta_v1.magnitude();
        double deltaV2 = delta_v2.magnitude();

        deltaV = deltaV1 + deltaV2;

        double violation = checkConstraints(x);
        if (violation > 1e-4) {
            return deltaV + 1e3 * violation;
        }

        return deltaV;

    } else {
        // Coplanar case
        double r_init = Physics::OrbitMechanics::calculateRadius(_R1, _e1, departureTrueAnomaly);
        double r_init_x = r_init * std::cos(departureTrueAnomaly);
        double r_init_y = r_init * std::sin(departureTrueAnomaly);

        auto v_init = Physics::OrbitMechanics::calculateVelocity(_R1, _e1, departureTrueAnomaly);

        // First impulse
        double v1_r = v_init.first + firstImpulseMagnitude * std::sin(firstImpulseDirection);
        double v1_theta = v_init.second + firstImpulseMagnitude * std::cos(firstImpulseDirection);

        double vt = std::sqrt(v1_r*v1_r + v1_theta*v1_theta);
        double h = r_init * v1_theta;
        double energyT = 0.5 * vt*vt - _MU/r_init;
        double a_transfer = -_MU/(2 * energyT);
        double e_transfer = std::sqrt(1 + 2 * energyT * h*h/(_MU*_MU));

        // Arrival position
        double r_final = Physics::OrbitMechanics::calculateRadius(_R2, _e2, arrivalTrueAnomaly);
        double r_final_x = r_final * std::cos(arrivalTrueAnomaly);
        double r_final_y = r_final * std::sin(arrivalTrueAnomaly);

        std::pair<Physics::Vector3, Physics::Vector3> lambert_result = Physics::LambertSolver::solveLambert(
            Physics::Vector3(r_init_x, r_init_y, 0),
            Physics::Vector3(r_final_x, r_final_y, 0),
            transferTime,
            _MU,
            false
        );

        Physics::Vector3 v_trans_dep = lambert_result.first;
        Physics::Vector3 v_trans_arr = lambert_result.second;

        auto v_target = Physics::OrbitMechanics::calculateVelocity(_R2, _e2, arrivalTrueAnomaly);

        // delta-Vs
        Physics::Vector3 v_init_vec(v_init.first, v_init.second, 0);
        Physics::Vector3 v_final_vec(v_target.first, v_target.second, 0);

        double deltaV1 = (v_trans_dep - v_init_vec).magnitude();
        double deltaV2 = (v_final_vec - v_trans_arr).magnitude();

        deltaV = deltaV1 + deltaV2;

        double violation = checkConstraints(x);
        if (violation > 1e-4) {
            return deltaV + 1e3 * violation;
        }

        return deltaV;
    }
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::computePeriapsis(const std::vector<T>& x)
{
    double departureTrueAnomaly = x[0];
    double firstImpulseMagnitude = x[2];
    double firstImpulseDirection = x[4];

    // Radius at departure point
    double r_departure = _R1 * (1 - _e1*_e1) / (1 + _e1 * std::cos(departureTrueAnomaly));

    auto v_init = Physics::OrbitMechanics::calculateVelocity(_R1, _e1, departureTrueAnomaly);

    // Post-impulse velocity components
    double v1_r = v_init.first + firstImpulseMagnitude * std::sin(firstImpulseDirection);
    double v1_theta = v_init.second + firstImpulseMagnitude * std::cos(firstImpulseDirection);

    // Transfer orbit parameters
    double vt = std::sqrt(v1_r*v1_r + v1_theta*v1_theta);
    double h = r_departure * v1_theta;
    double e = std::sqrt(1 + (h*h/(_MU*r_departure))*(vt*vt/(2*_MU) - 1/r_departure));
    double a = h*h/(_MU*(1-e*e));

    return a * (1-e);
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::computeApoapsis(const std::vector<T>& x)
{
    double departureTrueAnomaly = x[0];
    double firstImpulseMagnitude = x[2];
    double firstImpulseDirection = x[4];

    // Radius at departure point
    double r_departure = _R1 * (1 - _e1*_e1) / (1 + _e1 * std::cos(departureTrueAnomaly));

    auto v_init = Physics::OrbitMechanics::calculateVelocity(_R1, _e1, departureTrueAnomaly);

    // Post-impulse velocity components
    double v1_r = v_init.first + firstImpulseMagnitude * std::sin(firstImpulseDirection);
    double v1_theta = v_init.second + firstImpulseMagnitude * std::cos(firstImpulseDirection);

    // Transfer orbit parameters
    double vt = std::sqrt(v1_r*v1_r + v1_theta*v1_theta);
    double h = r_departure * v1_theta;
    double e = std::sqrt(1 + (h*h/(_MU*r_departure))*(vt*vt/(2*_MU) - 1/r_departure));
    double a = h*h/(_MU*(1-e*e));

    return a * (1+e);
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::checkConstraints(const std::vector<double>& x)
{
    double totalViolation = 0.0;
    double relaxationFactor = getRelaxationFactor();

    // Check parameter bounds
    if (x[0] < 0) totalViolation += std::abs(x[0]);
    if (x[0] > 2*M_PI) totalViolation += (x[0] - 2*M_PI);
    if (x[1] < 0) totalViolation += std::abs(x[1]);
    if (x[1] > 2*M_PI) totalViolation += (x[1] - 2*M_PI);
    if (x[2] < 0) totalViolation += std::abs(x[2]);
    if (x[5] <= 0) totalViolation += (0.1 - x[5]);

    // Non-coplanar case constraints
    if (_i1 != 0 || _i2 != 0) {
        Physics::Vector3 r_init = Physics::OrbitMechanics::calculatePosition3D(_R1, _e1, _i1, _raan1, _omega1, x[0]);
        Physics::Vector3 r_final = Physics::OrbitMechanics::calculatePosition3D(_R2, _e2, _i2, _raan2, _omega2, x[1]);

        double r_init_mag = r_init.magnitude();
        double r_final_mag = r_final.magnitude();
        double semi_major = (r_init_mag + r_final_mag) / 2.0;

        // Transfer time constraints
        double min_time = M_PI * sqrt(pow(semi_major, 3) / _MU);
        if (x[5] < min_time / relaxationFactor)
            totalViolation += (min_time / relaxationFactor - x[5]);

        double max_time = 5.0 * min_time;
        if (x[5] > max_time / relaxationFactor)
            totalViolation += (x[5] - max_time / relaxationFactor);

        // Impulse magnitude constraints
        double escape_velocity = sqrt(2.0 * _MU / r_init_mag);
        if (x[2] > escape_velocity)
            totalViolation += (x[2] - escape_velocity);
        if (x[3] > escape_velocity)
            totalViolation += (x[3] - escape_velocity);

        // Impulse ratio constraint
        double impulse_ratio = std::max(x[2], x[3]) / (std::min(x[2], x[3]) + 1e-10);
        if (impulse_ratio > 20.0 / relaxationFactor)
            totalViolation += (impulse_ratio - 20.0 / relaxationFactor);

        // Max impulse magnitude constraints
        double v_circ1 = sqrt(_MU / _R1);
        double v_circ2 = sqrt(_MU / _R2);
        double v_max = 2.0 * std::max(v_circ1, v_circ2);

        if (x[2] > v_max * relaxationFactor)
            totalViolation += (x[2] - v_max * relaxationFactor);
        if (x[3] > v_max * relaxationFactor)
            totalViolation += (x[3] - v_max * relaxationFactor);

        // Min impulse magnitude constraints
        double min_impulse = 1e-4 * std::min(v_circ1, v_circ2);
        if (x[2] < min_impulse / relaxationFactor)
            totalViolation += (min_impulse / relaxationFactor - x[2]);
        if (x[3] < min_impulse / relaxationFactor)
            totalViolation += (min_impulse / relaxationFactor - x[3]);

        // Plane change constraint
        if (_i1 != _i2) {
            double min_plane_change_dv = 0.8 * v_circ1 * sin(fabs(_i1 - _i2) / 2.0);
            if (x[2] + x[3] < min_plane_change_dv / relaxationFactor)
                totalViolation += (min_plane_change_dv / relaxationFactor - (x[2] + x[3]));
        }

        // Intersection constraint
        if (!doesIntersect(x)) {
            totalViolation += 200.0;
        }

    }
    else {
        // Coplanar case
        double transferPeriapsis = computePeriapsis(x);
        double transferApoapsis = computeApoapsis(x);

        if (transferPeriapsis > std::min(_R1, _R2))
            totalViolation += (transferPeriapsis - std::min(_R1, _R2));
        if (transferApoapsis < std::max(_R1, _R2))
            totalViolation += (std::max(_R1, _R2) - transferApoapsis);
        if (transferApoapsis > _Rmax)
            totalViolation += (transferApoapsis - _Rmax);
    }

    return totalViolation;
}


template<typename T, typename Fun>
bool OrbitTransferObjective<T, Fun>::doesIntersect(const std::vector<double>& x) {
    double departureTrueAnomaly = x[0];
    double firstImpulseMagnitude = x[2];
    double firstImpulseDirection = x[4];

    Physics::Vector3 r_init = Physics::OrbitMechanics::calculatePosition3D(_R1, _e1, _i1, _raan1, _omega1, departureTrueAnomaly);
    Physics::Vector3 v_init = Physics::OrbitMechanics::calculateVelocity3D(_R1, _e1, _i1, _raan1, _omega1, departureTrueAnomaly);

    Physics::Vector3 impulse = Physics::OrbitMechanics::calculateImpulseVector(
        Physics::Vector3(r_init.x, r_init.y, r_init.z),
        Physics::Vector3(v_init.x, v_init.y, v_init.z),
        firstImpulseMagnitude,
        firstImpulseDirection
    );
    Physics::Vector3 v_transfer = v_init;
    v_transfer.x += impulse.x;
    v_transfer.y += impulse.y;
    v_transfer.z += impulse.z;

    // Orbital elements
    double mu = _MU;
    double r_mag = sqrt(r_init.x*r_init.x + r_init.y*r_init.y + r_init.z*r_init.z);
    double v_mag = sqrt(v_transfer.x*v_transfer.x + v_transfer.y*v_transfer.y + v_transfer.z*v_transfer.z);

    // Specific angular momentum
    Physics::Vector3 h;
    h.x = r_init.y * v_transfer.z - r_init.z * v_transfer.y;
    h.y = r_init.z * v_transfer.x - r_init.x * v_transfer.z;
    h.z = r_init.x * v_transfer.y - r_init.y * v_transfer.x;
    double h_mag = sqrt(h.x*h.x + h.y*h.y + h.z*h.z);

    // Eccentricity vector and magnitude
    double r_dot_v = r_init.x*v_transfer.x + r_init.y*v_transfer.y + r_init.z*v_transfer.z;
    Physics::Vector3 e_vec;
    e_vec.x = ((v_mag*v_mag - mu/r_mag) * r_init.x - r_dot_v * v_transfer.x) / mu;
    e_vec.y = ((v_mag*v_mag - mu/r_mag) * r_init.y - r_dot_v * v_transfer.y) / mu;
    e_vec.z = ((v_mag*v_mag - mu/r_mag) * r_init.z - r_dot_v * v_transfer.z) / mu;
    double ecc = sqrt(e_vec.x*e_vec.x + e_vec.y*e_vec.y + e_vec.z*e_vec.z);

    // Semi-major axis
    double sma = h_mag*h_mag / (mu * (1.0 - ecc*ecc));

    double periapsis = sma * (1.0 - ecc);
    double apoapsis = sma * (1.0 + ecc);

    // Check if target orbit is between periapsis and apoapsis
    return (_R2 >= periapsis - 1e-6 && _R2 <= apoapsis + 1e-6);
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::getE1() const
{
    return _e1;
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::getE2() const
{
    return _e2;
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::getI1() const
{
    return _i1;
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::getI2() const
{
    return _i2;
}

template <typename T, typename Fun>
std::map<std::string, double> OrbitTransferObjective<T, Fun>::getTransferDetails(const std::vector<double>& x)
{
    std::map<std::string, double> results;

    double departureTrueAnomaly = x[0];
    double arrivalTrueAnomaly = x[1];
    double firstImpulseMagnitude = x[2];
    double secondImpulseMagnitude = x[3];
    double firstImpulseDirection = x[4];
    double transferTime = x[5];

    results["initial_true_anomaly"] = departureTrueAnomaly;
    results["final_true_anomaly"] = arrivalTrueAnomaly;
    results["transfer_time"] = transferTime;
    results["impulse_mag_1"] = firstImpulseMagnitude;
    results["impulse_mag_2"] = secondImpulseMagnitude;
    results["impulse_dir_1"] = firstImpulseDirection;

    double r_init = Physics::OrbitMechanics::calculateRadius(_R1, _e1, departureTrueAnomaly);
    auto v_init = Physics::OrbitMechanics::calculateVelocity(_R1, _e1, departureTrueAnomaly);

    //  first impulse components
    double v1_r = v_init.first + firstImpulseMagnitude * std::sin(firstImpulseDirection);
    double v1_theta = v_init.second + firstImpulseMagnitude * std::cos(firstImpulseDirection);

    //  transfer orbit parameters
    double vt = std::sqrt(v1_r*v1_r + v1_theta*v1_theta);
    double h = r_init * v1_theta;
    double energyT = 0.5 * vt*vt - _MU/r_init;
    double a_transfer = -_MU/(2 * energyT);
    double e_transfer = std::sqrt(1 + 2 * energyT * h*h/(_MU*_MU));

    results["transfer_semimajor_axis"] = a_transfer;
    results["transfer_eccentricity"] = e_transfer;
    results["transfer_periapsis"] = a_transfer * (1 - e_transfer);
    results["transfer_apoapsis"] = a_transfer * (1 + e_transfer);

    // non-coplanar
    if (_i1 != 0.0 || _i2 != 0.0) {
        double total_inc_change = std::abs(_i2 - _i1);
        double inc_change_1 = firstImpulseDirection; // This is a simplification
        double inc_change_2 = total_inc_change - inc_change_1;

        results["plane_change_1"] = inc_change_1;
        results["plane_change_2"] = inc_change_2;
    }

    // Check if this is potentially a three-impulse transfer
    double ratio = _R2/_R1;
    results["is_three_impulse"] = (ratio < 0.06418 || ratio > 15.582) ? 1.0 : 0.0;

    return results;
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::getRelaxationFactor() const {
    return 2.0 - (static_cast<double>(PSOGlobals::currentIteration) / PSOGlobals::maxIterations);
}

template class OrbitTransferObjective<double, std::function<double(double*)>>;
