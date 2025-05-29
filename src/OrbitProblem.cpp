#include <cmath>
#include <vector>
#include <functional>
#include <iostream>
#include <map>

#include "OrbitProblem.hpp"

template <typename T, typename Fun>
OrbitTransferObjective<T, Fun>::OrbitTransferObjective(double R1, double R2, double Rmax,   
                                            double e1, double e2, double i1, double i2): 
                                            _R1(R1), _R2(R2), _Rmax(Rmax),
                                            _e1(e1), _e2(e2), _i1(i1), _i2(i2){}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::operator()(double* x)
{
    std::vector<double> params(x, x + 6);
    return calculateDeltaV(params);
}

template<typename T, typename Fun>
std::pair<T, T> OrbitTransferObjective<T, Fun>::calculateVelocity(double a, double e, double trueAnomaly)
{
    double r = a * (1 - e*e) / (1 + e * std::cos(trueAnomaly));
    double v_r = std::sqrt(_MU/a) * e * std::sin(trueAnomaly) / std::sqrt(1 - e*e);
    double v_theta = std::sqrt(_MU/a) * (1 + e * std::cos(trueAnomaly)) / std::sqrt(1 - e*e);
    return std::make_pair(v_r, v_theta);
}

template<typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::calculateRadius(double a, double e, double trueAnomaly)
{
    return a * (1 - e*e) / (1 + e * std::cos(trueAnomaly));
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

    double departureTrueAnomaly = x[0];     // Departure true anomaly
    double arrivalTrueAnomaly = x[1];       // Arrival true anomaly
    double firstImpulseMagnitude = x[2];    // First impulse magnitude
    double secondImpulseMagnitude = x[3];   // Second impulse magnitude
    double firstImpulseDirection = x[4];    // First impulse direction
    double transferTime = x[5];             // Transfer time
    
    // Iinitial position and velocity
    double r_init = calculateRadius(_R1, _e1, departureTrueAnomaly);
    double r_init_x = r_init * std::cos(departureTrueAnomaly);
    double r_init_y = r_init * std::sin(departureTrueAnomaly);
    
    auto v_init = calculateVelocity(_R1, _e1, departureTrueAnomaly);
    
    // First impulse
    double v1_r = v_init.first + firstImpulseMagnitude * std::sin(firstImpulseDirection);
    double v1_theta = v_init.second + firstImpulseMagnitude * std::cos(firstImpulseDirection);
    
    double vt = std::sqrt(v1_r*v1_r + v1_theta*v1_theta);               // transfer orbit velocity at periapsis
    double h = r_init * v1_theta;                                       // specific angular momentum of transfer orbit
    double energyT = 0.5 * vt*vt - _MU/r_init;                          // specific energy of transfer orbit
    double a_transfer = -_MU/(2 * energyT);                             // semi-major axis of transfer orbit
    double e_transfer = std::sqrt(1 + 2 * energyT * h*h/(_MU*_MU));     // eccentricity
    
    // Arrival position using Lambert's theorem
    double r_final = calculateRadius(_R2, _e2, arrivalTrueAnomaly);
    double r_final_x = r_final * std::cos(arrivalTrueAnomaly);
    double r_final_y = r_final * std::sin(arrivalTrueAnomaly);
    
    double transfer_angle = std::acos((r_init_x * r_final_x + r_init_y * r_final_y)/(r_init * r_final));
    
    // Find the velocity vectors at departure and arrival
    std::pair<Vector3, Vector3> lambert_result = solveLambert(
        Vector3(r_init_x, r_init_y, 0),
        Vector3(r_final_x, r_final_y, 0),
        transferTime,
        _MU,
        false
    );
    
    Vector3 v_trans_dep = lambert_result.first;
    Vector3 v_trans_arr = lambert_result.second;
    
    auto v_target = calculateVelocity(_R2, _e2, arrivalTrueAnomaly);
    
    // delta-Vs
    Vector3 v_init_vec(v_init.first, v_init.second, 0);
    Vector3 v_final_vec(v_target.first, v_target.second, 0);
    
    double deltaV1 = (v_trans_dep - v_init_vec).magnitude();
    double deltaV2 = (v_final_vec - v_trans_arr).magnitude();
    
    return deltaV1 + deltaV2;
}

template <typename T, typename Fun>
std::pair<Vector3, Vector3> OrbitTransferObjective<T, Fun>::solveLambert(
    const Vector3& r1, const Vector3& r2, double tof, double mu, bool isLongWay)
{
    // Magnitudes of position vectors
    double r1_mag = r1.magnitude();
    double r2_mag = r2.magnitude();
    
    // transfer angle
    double cos_dnu = (r1.x * r2.x + r1.y * r2.y + r1.z * r2.z) / (r1_mag * r2_mag);
    double dnu = std::acos(std::min(1.0, std::max(-1.0, cos_dnu)));
    
    if (isLongWay) {
        dnu = 2 * M_PI - dnu;
    }
    
    //  chord and semi-perimeter
    double chord = std::sqrt(r1_mag*r1_mag + r2_mag*r2_mag - 2*r1_mag*r2_mag*std::cos(dnu));
    double s = (r1_mag + r2_mag + chord) / 2.0;
    
    //  min energy ellipse semi-major axis
    double a_min = s / 2.0;
    
    //  minimum time of flight
    double alpha = 2.0 * std::asin(std::sqrt((s - chord) / s));
    double beta = 2.0 * std::asin(std::sqrt((s) / s));
    if (dnu > M_PI) {
        alpha = 2.0 * M_PI - alpha;
    }
    double t_min = std::sqrt(a_min*a_min*a_min / mu) * ((alpha - std::sin(alpha)) - (beta - std::sin(beta)));
    
    // If the time of flight is less than minimum, return error
    if (tof < t_min) {
        return std::make_pair(Vector3(), Vector3());
    }
    
    // Iteratively solve for semi-major axis that gives desired time of flight
    double a = a_min;
    double tof_current = t_min;
    
    int max_iter = 100;
    double tolerance = 1e-8;
    
    for (int i = 0; i < max_iter && std::abs(tof_current - tof) > tolerance; ++i) {
        a = a * 1.02;
        
        //  time of flight with current a
        double alpha = 2.0 * std::asin(std::sqrt(s / (2.0 * a)));
        double beta = 2.0 * std::asin(std::sqrt((s - chord) / (2.0 * a)));
        
        if (dnu > M_PI) {
            alpha = 2.0 * M_PI - alpha;
        }
        
        tof_current = std::sqrt(a*a*a / mu) * ((alpha - std::sin(alpha)) - (beta - std::sin(beta)));
    }
    
    //  Lagrange coeffs
    double f = 1.0 - r2_mag / a * (1.0 - std::cos(dnu));
    double g = r1_mag * r2_mag * std::sin(dnu) / std::sqrt(mu * a);
    double g_dot = 1.0 - r1_mag / a * (1.0 - std::cos(dnu));
    double f_dot = -std::sqrt(mu / a) * std::sin(dnu) / (r1_mag * r2_mag) * r2_mag;
    
    //  velocity vectors
    Vector3 v1, v2;
    v1.x = (r2.x - f * r1.x) / g;
    v1.y = (r2.y - f * r1.y) / g;
    v1.z = (r2.z - f * r1.z) / g;
    
    v2.x = f_dot * r1.x + g_dot * v1.x;
    v2.y = f_dot * r1.y + g_dot * v1.y;
    v2.z = f_dot * r1.z + g_dot * v1.z;
    
    return std::make_pair(v1, v2);
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::computePeriapsis(const std::vector<T>& x)
{
    double departureTrueAnomaly = x[0];
    double firstImpulseMagnitude = x[2];
    double firstImpulseDirection = x[4];
    
    //  radius at departure point
    double r_departure = _R1 * (1 - _e1*_e1) / (1 + _e1 * std::cos(departureTrueAnomaly));
    
    auto v_init = calculateVelocity(_R1, _e1, departureTrueAnomaly);
    
    //  post-impulse velocity components
    double v1_r = v_init.first + firstImpulseMagnitude * std::sin(firstImpulseDirection);
    double v1_theta = v_init.second + firstImpulseMagnitude * std::cos(firstImpulseDirection);
    
    //  transfer orbit parameters
    double vt = std::sqrt(v1_r*v1_r + v1_theta*v1_theta);
    double h = r_departure * v1_theta; // Angular momentum
    double e = std::sqrt(1 + (h*h/(_MU*r_departure))*(vt*vt/(2*_MU) - 1/r_departure));
    double a = h*h/(_MU*(1-e*e)); // Semi-major axis

    return a * (1-e); // Periapsis radius
}

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::computeApoapsis(const std::vector<T>& x)
{
    double departureTrueAnomaly = x[0];
    double firstImpulseMagnitude = x[2];
    double firstImpulseDirection = x[4];
    
    //  radius at departure point
    double r_departure = _R1 * (1 - _e1*_e1) / (1 + _e1 * std::cos(departureTrueAnomaly));
    
    auto v_init = calculateVelocity(_R1, _e1, departureTrueAnomaly);
    
    //  post-impulse velocity components
    double v1_r = v_init.first + firstImpulseMagnitude * std::sin(firstImpulseDirection);
    double v1_theta = v_init.second + firstImpulseMagnitude * std::cos(firstImpulseDirection);
    
    //  transfer orbit parameters
    double vt = std::sqrt(v1_r*v1_r + v1_theta*v1_theta);
    double h = r_departure * v1_theta; // Angular momentum
    double e = std::sqrt(1 + (h*h/(_MU*r_departure))*(vt*vt/(2*_MU) - 1/r_departure));
    double a = h*h/(_MU*(1-e*e)); // Semi-major axis

    return a * (1+e); // Periapsis radius
}

template <typename T, typename Fun>
bool OrbitTransferObjective<T, Fun>::checkConstraints(const std::vector<double>& x)
{
    if (_R2 <= _R1) return false;

    double transferPeriapsis = computePeriapsis(x);
    double transferApoapsis = computeApoapsis(x);

    // Check if transfer orbit intersects both initial and final orbits
    if (transferPeriapsis > std::min(_R1, _R2) ||
        transferApoapsis < std::max(_R1, _R2) ||
        transferApoapsis < _Rmax) return false;

    return true;
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
    
    //  initial position and velocity
    double r_init = calculateRadius(_R1, _e1, departureTrueAnomaly);
    auto v_init = calculateVelocity(_R1, _e1, departureTrueAnomaly);
    
    //  first impulse components
    double v1_r = v_init.first + firstImpulseMagnitude * std::sin(firstImpulseDirection);
    double v1_theta = v_init.second + firstImpulseMagnitude * std::cos(firstImpulseDirection);
    
    //  transfer orbit parameters
    double vt = std::sqrt(v1_r*v1_r + v1_theta*v1_theta);
    double h = r_init * v1_theta;
    double energyT = 0.5 * vt*vt - _MU/r_init;
    double a_transfer = -_MU/(2 * energyT);
    double e_transfer = std::sqrt(1 + 2 * energyT * h*h/(_MU*_MU));
    
    // Store transfer orbit elements
    results["transfer_semimajor_axis"] = a_transfer;
    results["transfer_eccentricity"] = e_transfer;
    results["transfer_periapsis"] = a_transfer * (1 - e_transfer);
    results["transfer_apoapsis"] = a_transfer * (1 + e_transfer);
    
    // For non-coplanar transfers,  inclination changes
    if (_i1 != 0.0 || _i2 != 0.0) {
        // Simplified calculation of inclination distribution
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

template class OrbitTransferObjective<double, std::function<double(double*)>>;
