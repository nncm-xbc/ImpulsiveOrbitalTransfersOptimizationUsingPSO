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
    std::vector<double> params(x, x + 3);

    return calculateDeltaV(params);
}


template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::calculateDeltaV(const std::vector<double>& x)
{
    double deltaV = 0.0;

    double departureTrueAnomaly = x[0];
    double arrivalTrueAnomaly = x[1];
    double transferTime = x[2];

    if (_i1 != 0 || _i2 != 0)
    {
        // Non-coplanar case
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
            return deltaV + violation;
        }

        return deltaV;
    }
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::checkConstraints(const std::vector<double>& x) 
{
    double totalViolation = 0.0;
    double relaxationFactor = getRelaxationFactor();
    
    if (x[0] < 0.0) totalViolation += std::abs(x[0]) * 0.001;
    if (x[0] > 2*M_PI) totalViolation += (x[0] - 2*M_PI) * 0.001;
    if (x[1] < 0.0) totalViolation += std::abs(x[1]) * 0.001;
    if (x[1] > 2*M_PI) totalViolation += (x[1] - 2*M_PI) * 0.001;
    if (x[2] <= 0.0) totalViolation += (0.1 - x[2]) * 0.01;

    try {
        // Calculate positions
        Physics::Vector3 r_init = Physics::OrbitMechanics::calculatePosition3D(
            _R1, _e1, _i1, _raan1, _omega1, x[0]);
        Physics::Vector3 r_final = Physics::OrbitMechanics::calculatePosition3D(
            _R2, _e2, _i2, _raan2, _omega2, x[1]);
        
        double r_init_mag = r_init.magnitude();
        double r_final_mag = r_final.magnitude();
        
        double semi_major_avg = (r_init_mag + r_final_mag) / 2.0;
        double min_transfer_time = 0.05 * M_PI * sqrt(pow(semi_major_avg, 3) / _MU);
        double max_transfer_time = 50.0 * M_PI * sqrt(pow(semi_major_avg, 3) / _MU);
        
        if (x[2] < min_transfer_time * relaxationFactor) {
            totalViolation += (min_transfer_time * relaxationFactor - x[2]) * 0.0001;
        }
        if (x[2] > max_transfer_time / relaxationFactor) {
            totalViolation += (x[2] - max_transfer_time / relaxationFactor) * 0.00001;
        }
        
        // Angular separation guidance
        double dot_product = (r_init.x * r_final.x + r_init.y * r_final.y + r_init.z * r_final.z);
        double cos_separation = dot_product / (r_init_mag * r_final_mag);
        cos_separation = std::max(-1.0, std::min(1.0, cos_separation));
        double angular_separation = std::acos(cos_separation) * 180.0 / M_PI;
        
        // Very gentle guidance away from collinear cases
        if (angular_separation < 5.0) {
            totalViolation += (5.0 - angular_separation) * 0.01 * relaxationFactor;
        }
        if (angular_separation > 175.0) {
            totalViolation += (angular_separation - 175.0) * 0.005 * relaxationFactor;
        }
        
        // Lambert solver
        std::pair<Physics::Vector3, Physics::Vector3> lambert_result = 
            Physics::LambertSolver::solveLambert(r_init, r_final, x[2], _MU, false);
        
        Physics::Vector3 v1 = lambert_result.first;
        Physics::Vector3 v2 = lambert_result.second;
        double v1_mag = v1.magnitude();
        double v2_mag = v2.magnitude();
        
        if (v1_mag < 1e-12 || v2_mag < 1e-12) {
            totalViolation += 0.1 * relaxationFactor;
            return totalViolation;
        }
        
        // Energy constraint - prevent escape
        double specific_energy = 0.5 * v1_mag * v1_mag - _MU / r_init_mag;
        if (specific_energy >= -0.01) {  // Bound orbit requirement
            totalViolation += std::abs(specific_energy + 0.01) * 0.1 * relaxationFactor;
        }
        
        Physics::Vector3 h_transfer = r_init.cross(v1);
        double h_mag = h_transfer.magnitude();
        
        if (h_mag < 1e-10) {
            // Alternative calculation for zero angular momentum
            Physics::Vector3 r_unit = r_init.normalized();
            double v_radial = v1.dot(r_unit);
            Physics::Vector3 v_radial_vec = Physics::Vector3(r_unit.x * v_radial, r_unit.y * v_radial, r_unit.z * v_radial);
            Physics::Vector3 v_tangential_vec = v1 - v_radial_vec;
            double v_tangential = v_tangential_vec.magnitude();
            h_mag = r_init_mag * v_tangential;
            
            if (h_mag < 1e-10) h_mag = r_init_mag * 0.001;  // Minimum value
            
            // Gentle penalty for purely radial motion
            double radial_fraction = std::abs(v_radial) / v1_mag;
            if (radial_fraction > 0.95) {
                totalViolation += (radial_fraction - 0.95) * 1.0 * relaxationFactor;
            }
        }
        
        double transfer_eccentricity = std::sqrt(1.0 + 2.0 * specific_energy * h_mag * h_mag / (_MU * _MU));
        
        if (transfer_eccentricity > 0.85 * relaxationFactor) {
            totalViolation += (transfer_eccentricity - 0.85 * relaxationFactor) * 2.0;
        }
        
        double transfer_semi_major = -_MU / (2.0 * specific_energy);

        // Basic safety constraints (very lenient)
        double transfer_periapsis = transfer_semi_major * (1.0 - transfer_eccentricity);
        double transfer_apoapsis = transfer_semi_major * (1.0 + transfer_eccentricity);
        
        if (transfer_periapsis < 0.9) {  // Don't crash into central body
            totalViolation += (0.9 - transfer_periapsis) * 0.1 * relaxationFactor;
        }
        
        if (transfer_apoapsis > _Rmax) {
            totalViolation += (transfer_apoapsis - _Rmax) * 0.01 * relaxationFactor;
        }
        
        // Very gentle intersection check
        if (transfer_periapsis > std::max(_R1, _R2) * 1.1 || 
            transfer_apoapsis < std::min(_R1, _R2) * 0.9) {
            totalViolation += 1.0 * relaxationFactor;
        }
        
    } catch (...) {
        totalViolation += 1 * relaxationFactor;
    }
    
    return totalViolation;
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::calculateAngularSeparation(
    const Physics::Vector3& r1, const Physics::Vector3& r2) 
{
    double dot_product = r1.dot(r2);
    double r1_mag = r1.magnitude();
    double r2_mag = r2.magnitude();
    
    double cos_angle = dot_product / (r1_mag * r2_mag);
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle)); // Clamp to valid range
    
    return std::acos(cos_angle);
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::calculateParabolicTime(
    double r1, double r2, double theta) 
{
    double a_parabolic = (r1 + r2 + 2.0 * sqrt(r1 * r2) * cos(theta / 2.0)) / 4.0;
    return M_PI * sqrt(pow(a_parabolic, 3) / _MU);
}

template<typename T, typename Fun>
bool OrbitTransferObjective<T, Fun>::doesIntersect(const std::vector<double>& x) 
{
    double departureTrueAnomaly = x[0];
    double arrivalTrueAnomaly = x[1];
    double transferTime = x[2];

    Physics::Vector3 r_init, r_final;

    if (_i1 != 0.0 || _i2 != 0.0 || _raan1 != _raan2 || _omega1 != _omega2) {
        // NON-COPLANAR CASE
        r_init = Physics::OrbitMechanics::calculatePosition3D(
            _R1,
            _e1, 
            _i1,
            _raan1,
            _omega1,
            departureTrueAnomaly
        );

        r_final = Physics::OrbitMechanics::calculatePosition3D(
            _R2,
            _e2, 
            _i2,
            _raan2,
            _omega2,
            arrivalTrueAnomaly
        );
        
    } else {
        // COPLANAR CASE
        
        double r_init_magnitude = _R1;
        double r_final_magnitude = _R2;
        
        if (_e1 != 0.0) {
            r_init_magnitude = _R1 * (1.0 - _e1*_e1) / (1.0 + _e1 * cos(departureTrueAnomaly));
        }
        if (_e2 != 0.0) {
            r_final_magnitude = _R2 * (1.0 - _e2*_e2) / (1.0 + _e2 * cos(arrivalTrueAnomaly));
        }
        
        r_init = Physics::Vector3(
            r_init_magnitude * cos(departureTrueAnomaly),
            r_init_magnitude * sin(departureTrueAnomaly),  
            0.0);
        
        r_final = Physics::Vector3(
            r_final_magnitude * cos(arrivalTrueAnomaly),
            r_final_magnitude * sin(arrivalTrueAnomaly),
            0.0);    

        try {
            // Use Lambert solver instead of manual impulse calculation
            std::pair<Physics::Vector3, Physics::Vector3> lambert_result = Physics::LambertSolver::solveLambert(r_init, r_final, x[2], _MU, false);
            
            // Check if Lambert returned valid velocities
            return (lambert_result.first.magnitude() > 1e-10 && 
                    lambert_result.second.magnitude() > 1e-10);
                    
        } catch (...) {
            return false;
        }
    }
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
    double transferTime = x[2];

    results["initial_true_anomaly"] = departureTrueAnomaly;
    results["final_true_anomaly"] = arrivalTrueAnomaly;
    results["transfer_time"] = transferTime;

    try {
        Physics::Vector3 r_init_vec, r_final_vec, v_init_vec, v_final_vec;
        
        if (_i1 != 0.0 || _i2 != 0.0 || _raan1 != _raan2 || _omega1 != _omega2) {
            // Non-coplanar case
            r_init_vec = Physics::OrbitMechanics::calculatePosition3D(_R1, _e1, _i1, _raan1, _omega1, departureTrueAnomaly);
            r_final_vec = Physics::OrbitMechanics::calculatePosition3D(_R2, _e2, _i2, _raan2, _omega2, arrivalTrueAnomaly);
            v_init_vec = Physics::OrbitMechanics::calculateVelocity3D(_R1, _e1, _i1, _raan1, _omega1, departureTrueAnomaly);
            v_final_vec = Physics::OrbitMechanics::calculateVelocity3D(_R2, _e2, _i2, _raan2, _omega2, arrivalTrueAnomaly);
        } else {
            // Coplanar case
            double r_init = Physics::OrbitMechanics::calculateRadius(_R1, _e1, departureTrueAnomaly);
            double r_final = Physics::OrbitMechanics::calculateRadius(_R2, _e2, arrivalTrueAnomaly);
            auto v_init_2d = Physics::OrbitMechanics::calculateVelocity(_R1, _e1, departureTrueAnomaly);
            auto v_final_2d = Physics::OrbitMechanics::calculateVelocity(_R2, _e2, arrivalTrueAnomaly);
            
            r_init_vec = Physics::Vector3(r_init * cos(departureTrueAnomaly), r_init * sin(departureTrueAnomaly), 0.0);
            r_final_vec = Physics::Vector3(r_final * cos(arrivalTrueAnomaly), r_final * sin(arrivalTrueAnomaly), 0.0);
            v_init_vec = Physics::Vector3(v_init_2d.first, v_init_2d.second, 0.0);
            v_final_vec = Physics::Vector3(v_final_2d.first, v_final_2d.second, 0.0);
        }

        std::pair<Physics::Vector3, Physics::Vector3> lambert_result = 
            Physics::LambertSolver::solveLambert(r_init_vec, r_final_vec, transferTime, _MU, false);

        Physics::Vector3 v_trans_dep = lambert_result.first;
        Physics::Vector3 v_trans_arr = lambert_result.second;

        Physics::Vector3 delta_v1 = v_trans_dep - v_init_vec;
        Physics::Vector3 delta_v2 = v_final_vec - v_trans_arr;

        double impulse_mag_1 = delta_v1.magnitude();
        double impulse_mag_2 = delta_v2.magnitude();

        results["impulse_mag_1"] = impulse_mag_1;
        results["impulse_mag_2"] = impulse_mag_2;
        results["total_delta_v"] = impulse_mag_1 + impulse_mag_2;

        Physics::Vector3 r_init_unit = r_init_vec.normalized();
        Physics::Vector3 h_init = r_init_vec.cross(v_init_vec);
        Physics::Vector3 theta_init_unit = h_init.cross(r_init_unit).normalized();
        
        double delta_v1_radial = delta_v1.dot(r_init_unit);
        double delta_v1_tangential = delta_v1.dot(theta_init_unit);
        double impulse_dir_1 = atan2(delta_v1_radial, delta_v1_tangential);
        
        Physics::Vector3 r_final_unit = r_final_vec.normalized();
        Physics::Vector3 h_final = r_final_vec.cross(v_final_vec);
        Physics::Vector3 theta_final_unit = h_final.cross(r_final_unit).normalized();
        
        double delta_v2_radial = delta_v2.dot(r_final_unit);
        double delta_v2_tangential = delta_v2.dot(theta_final_unit);
        double impulse_dir_2 = atan2(delta_v2_radial, delta_v2_tangential);

        results["impulse_dir_1"] = impulse_dir_1;
        results["impulse_dir_2"] = impulse_dir_2;
        
        double r_init_mag = r_init_vec.magnitude();
        double v_trans_mag = v_trans_dep.magnitude();
        
        double specific_energy = 0.5 * v_trans_mag * v_trans_mag - _MU / r_init_mag;
        Physics::Vector3 h_transfer = r_init_vec.cross(v_trans_dep);
        double h_mag = h_transfer.magnitude();
        
        if (specific_energy < 0) {
            double a_transfer = -_MU / (2.0 * specific_energy);
            double e_transfer = sqrt(1.0 + 2.0 * specific_energy * h_mag * h_mag / (_MU * _MU));
            
            results["transfer_semimajor_axis"] = a_transfer;
            results["transfer_eccentricity"] = e_transfer;
            results["transfer_periapsis"] = a_transfer * (1.0 - e_transfer);
            results["transfer_apoapsis"] = a_transfer * (1.0 + e_transfer);
            results["transfer_period"] = 2.0 * M_PI * sqrt(a_transfer * a_transfer * a_transfer / _MU);
        } else {
            results["transfer_semimajor_axis"] = -1.0;
            results["transfer_eccentricity"] = -1.0;
            results["transfer_periapsis"] = -1.0;
            results["transfer_apoapsis"] = -1.0;
            results["transfer_period"] = -1.0;
        }
        
        if (_i1 != _i2 || _raan1 != _raan2) {
            double cos_plane_change = cos(_i1) * cos(_i2) + sin(_i1) * sin(_i2) * cos(_raan2 - _raan1);
            double total_plane_change = acos(std::max(-1.0, std::min(1.0, cos_plane_change)));
            
            Physics::Vector3 h_init_unit = (r_init_vec.cross(v_init_vec)).normalized();
            Physics::Vector3 h_final_unit = (r_final_vec.cross(v_final_vec)).normalized();
            Physics::Vector3 h_transfer_unit = h_transfer.normalized();
            
            double plane_change_1 = acos(std::max(-1.0, std::min(1.0, h_init_unit.dot(h_transfer_unit))));
            double plane_change_2 = acos(std::max(-1.0, std::min(1.0, h_transfer_unit.dot(h_final_unit))));
            
            results["total_plane_change"] = total_plane_change;
            results["plane_change_1"] = plane_change_1;
            results["plane_change_2"] = plane_change_2;
            results["inclination_change"] = std::abs(_i2 - _i1);
            results["raan_change"] = std::abs(_raan2 - _raan1);
        } else {
            // Coplanar transfer
            results["total_plane_change"] = 0.0;
            results["plane_change_1"] = 0.0;
            results["plane_change_2"] = 0.0;
            results["inclination_change"] = 0.0;
            results["raan_change"] = 0.0;
        }
        
        double radius_ratio = _R2 / _R1;
        bool potentially_three_impulse = (radius_ratio < 0.06418 || radius_ratio > 15.582);
        
        results["is_three_impulse"] = potentially_three_impulse ? 1.0 : 0.0;
        results["radius_ratio"] = radius_ratio;
        
        if (impulse_mag_1 > 0.001 && impulse_mag_2 > 0.001) {
            results["transfer_type"] = 2.0;
        } else if (impulse_mag_1 > 0.001 || impulse_mag_2 > 0.001) {
            results["transfer_type"] = 1.0;
        } else {
            results["transfer_type"] = 0.0;
        }

        if (_e1 == 0.0 && _e2 == 0.0 && _i1 == _i2) {
            double hohmann_dv = sqrt(_MU/_R1) * (sqrt(2.0*_R2/(_R1+_R2)) - 1.0) + 
                               sqrt(_MU/_R2) * (1.0 - sqrt(2.0*_R1/(_R1+_R2)));
            results["hohmann_delta_v"] = hohmann_dv;
            results["efficiency_ratio"] = hohmann_dv / (impulse_mag_1 + impulse_mag_2);
        } else {
            results["hohmann_delta_v"] = -1.0;
            results["efficiency_ratio"] = -1.0;
        }

    } catch (const std::exception& e) {
        results["error"] = 1.0;
        results["impulse_mag_1"] = -1.0;
        results["impulse_mag_2"] = -1.0;
        results["total_delta_v"] = 1e6;
        results["transfer_semimajor_axis"] = -1.0;
        results["transfer_eccentricity"] = -1.0;
    }

    return results;
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::getRelaxationFactor() const {
    double progress = static_cast<double>(PSOGlobals::currentIteration) / PSOGlobals::maxIterations;
    
    if (progress < 0.3) {
        return 1.0 + 0.3 * progress / 0.3;
    } else if (progress < 0.7) {
        return 1.3;
    } else {
        return 1.3 - 0.5 * (progress - 0.7) / 0.3;
    }}

template class OrbitTransferObjective<double, std::function<double(double*)>>;
