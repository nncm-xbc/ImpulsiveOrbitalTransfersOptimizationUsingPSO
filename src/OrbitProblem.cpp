#include <cmath>
#include <vector>
#include <functional>
#include <iostream>
#include <map>
#include <assert.h>

#include "OrbitProblem.hpp"
#include "PSO.hpp"
#include "PSOGlobals.hpp"

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

template<typename T, typename Fun>
std::pair<T, T> OrbitTransferObjective<T, Fun>::calculateVelocity(double a, double e, double trueAnomaly)
{
    double r = a * (1 - e*e) / (1 + e * std::cos(trueAnomaly));
    double v_r = std::sqrt(_MU/a) * e * std::sin(trueAnomaly) / std::sqrt(1 - e*e);
    double v_theta = std::sqrt(_MU/a) * (1 + e * std::cos(trueAnomaly)) / std::sqrt(1 - e*e);
    return std::make_pair(v_r, v_theta);
}

template<typename T, typename Fun>
Vector3 OrbitTransferObjective<T, Fun>::calculateVelocity3D(double a, double e, double i, double raan, double omega, double true_anomaly) {
    // Trigonometric values
    const double cos_nu = cos(true_anomaly);
    const double sin_nu = sin(true_anomaly);
    const double cos_omega = cos(omega);
    const double sin_omega = sin(omega);
    const double cos_i = cos(i);
    const double sin_i = sin(i);
    const double cos_raan = cos(raan);
    const double sin_raan = sin(raan);

    const double p = a * (1 - e * e);
    const double r = p / (1 + e * cos_nu);
    const double sqrt_mu_p = sqrt(_MU / p);

    // Velocity components in perifocal frame (radial and transverse)
    const double v_r = sqrt_mu_p * e * sin_nu;
    const double v_theta = sqrt_mu_p * (1 + e * cos_nu);

    // Combined rotation matrix application (perifocal to inertial frame)
    const double v_P = v_r * cos_nu - v_theta * sin_nu;
    const double v_Q = v_r * sin_nu + v_theta * cos_nu;

    // Rotations
    double v_x = (v_P * cos_omega - v_Q * sin_omega) * cos_raan - ((v_P * sin_omega + v_Q * cos_omega) * cos_i) * sin_raan;
    double v_y = (v_P * cos_omega - v_Q * sin_omega) * sin_raan + ((v_P * sin_omega + v_Q * cos_omega) * cos_i) * cos_raan;
    double v_z = (v_P * sin_omega + v_Q * cos_omega) * sin_i;

    return Vector3(v_x, v_y, v_z);
}

template<typename T, typename Fun>
Vector3 OrbitTransferObjective<T, Fun>::calculatePosition3D(double a, double e, double i, double raan, double omega, double true_anomaly) {
    // Trigo values
    const double cos_nu = cos(true_anomaly);
    const double sin_nu = sin(true_anomaly);
    const double cos_omega = cos(omega);
    const double sin_omega = sin(omega);
    const double cos_i = cos(i);
    const double sin_i = sin(i);
    const double cos_raan = cos(raan);
    const double sin_raan = sin(raan);

    const double p = a * (1 - e * e);
    const double r = p / (1 + e * cos_nu);

    // Position in orbital plane (perifocal coordinates)
    const double x_peri = r * cos_nu;
    const double y_peri = r * sin_nu;

    // Combined rotation matrix application (perifocal to inertial frame)
    const double x_omega = x_peri * cos_omega - y_peri * sin_omega;
    const double y_omega = x_peri * sin_omega + y_peri * cos_omega;

    double x = x_omega * cos_raan - y_omega * cos_i * sin_raan;
    double y = x_omega * sin_raan + y_omega * cos_i * cos_raan;
    double z = y_omega * sin_i;

    return Vector3(x, y, z);
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

    double departureTrueAnomaly = x[0];     // Departure true anomaly
    double arrivalTrueAnomaly = x[1];       // Arrival true anomaly
    double firstImpulseMagnitude = x[2];    // First impulse magnitude
    double secondImpulseMagnitude = x[3];   // Second impulse magnitude
    double firstImpulseDirection = x[4];    // First impulse direction
    double transferTime = x[5];             // Transfer time
    //std::cout << "Arrival true anomaly: " << arrivalTrueAnomaly << std::endl;
    //std::cout << "First impulse magnitude: " << firstImpulseMagnitude << std::endl;
    //std::cout << "Second impulse magnitude: " << secondImpulseMagnitude << std::endl;
    //::cout << "First impulse direction: " << firstImpulseDirection << std::endl;
    //std::cout << "Departure true anomaly: " << departureTrueAnomaly << std::endl;
    //std::cout << "Transfer time: " << transferTime << std::endl;

    // Iinitial position and velocity
    if (_i1 != 0 || _i2 != 0)
    {
        // Non-coplanar case
        double r_init = calculateRadius(_R1, _e1, departureTrueAnomaly);
        double r_final = calculateRadius(_R2, _e2, arrivalTrueAnomaly);

        // Position vectors in 3D space
        Vector3 r_init_vec = calculatePosition3D(_R1, _e1, _i1, _raan1, _omega1, departureTrueAnomaly);
        Vector3 r_final_vec = calculatePosition3D(_R2, _e2, _i2, _raan2, _omega2, arrivalTrueAnomaly);

        // Initial and final velocity vectors
        Vector3 v_init_vec = calculateVelocity3D(_R1, _e1, _i1, _raan1, _omega1, departureTrueAnomaly);
        Vector3 v_final_vec = calculateVelocity3D(_R2, _e2, _i2, _raan2, _omega2, arrivalTrueAnomaly);

        // Lambert solver to find transfer orbit velocities
        std::cout << "LAMBERT TIME" << std::endl;
        std::pair<Vector3, Vector3> lambert_result = solveLambert(
            r_init_vec,
            r_final_vec,
            transferTime,
            _MU,
            false  // Optimization parameter
        );

        Vector3 v_trans_dep = lambert_result.first;
        Vector3 v_trans_arr = lambert_result.second;

        // Impulse vectors and magnitudes
        Vector3 delta_v1 = v_trans_dep - v_init_vec;
        Vector3 delta_v2 = v_final_vec - v_trans_arr;

        double deltaV1 = delta_v1.magnitude();
        double deltaV2 = delta_v2.magnitude();

        return deltaV1 + deltaV2;

    } else {
        //coplanar case

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

        // std::cout << r_init_x << ", " << r_init_y << " -> " << r_final_x << ", " << r_final_y << std::endl;
        // std::cout << "transfer time: " << transferTime << std::endl;
        // std::cout << "mu: " << _MU << std::endl;
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

        deltaV = deltaV1 + deltaV2;

        double violation = checkConstraints(x);
        if (violation > 1e-4) {
            return deltaV + 1e3 * violation;
        }

        return deltaV;
    }
}

template <typename T, typename Fun>
std::pair<Vector3, Vector3> OrbitTransferObjective<T, Fun>::solveLambert(
    const Vector3& r1, const Vector3& r2, double tof, double mu, bool isLongWay)
{
    // Magnitudes of position vectors
    double r1_mag = r1.magnitude();
    double r2_mag = r2.magnitude();

    // Unit position vectors
    Vector3 r1_hat(r1.x / r1_mag, r1.y / r1_mag, r1.z / r1_mag);
    Vector3 r2_hat(r2.x / r2_mag, r2.y / r2_mag, r2.z / r2_mag);

    //std::cout << "R1: " << r1_mag << ", R2: " << r2_mag << std::endl;

    // transfer angle
    double cos_theta = (r1.x * r2.x + r1.y * r2.y + r1.z * r2.z) / (r1_mag * r2_mag);
    cos_theta = std::min(1.0, std::max(-1.0, cos_theta));

    double theta_tilde = std::acos(cos_theta);

    // Transfer plane unit normal vector
    Vector3 l_hat(
        r1_hat.y * r2_hat.z - r1_hat.z * r2_hat.y,
        r1_hat.z * r2_hat.x - r1_hat.x * r2_hat.z,
        r1_hat.x * r2_hat.y - r1_hat.y * r2_hat.x
    );
    double l_mag = l_hat.magnitude();

    // If magnitude is close to zero, vectors are nearly parallel or anti-parallel
    if (l_mag < 1e-10) {
        // Assume coplanar - use z-axis as normal vector for XY plane
        l_hat = Vector3(0, 0, 1);
    } else {
        l_hat = Vector3(l_hat.x / l_mag, l_hat.y / l_mag, l_hat.z / l_mag);
    }

    // Transfer orbit plane unit normal vector
    Vector3 h_T = l_hat;

    // Transfer angle theta
    Vector3 h_0 =;
    Vector3 h_f =;
    double dot_h0_hf = /* dot product of h_0 and h_f */;
    double dot_r0_rf = cos_theta; // You already calculated this

    double theta;
    // Case (a)
    if (dot_h0_hf >= 0 && dot_r0_rf >= 0) {
        theta = theta_tilde;
    }
    // Case (b)
    else if (dot_h0_hf >= 0 && dot_r0_rf < 0) {
        theta = 2 * M_PI - theta_tilde;
    }

    //  chord and semi-perimeter
    double chord = std::sqrt(r1_mag*r1_mag + r2_mag*r2_mag - 2*r1_mag*r2_mag*std::cos(theta));
    double s = (r1_mag + r2_mag + chord) / 2.0;

    //  min energy ellipse semi-major axis
    double a_min = s / 2.0;

    //  minimum time of flight
    double alpha = 2.0 * std::asin(std::sqrt((s - chord) / s));
    if (theta > M_PI) {
        alpha = 2.0 * M_PI - alpha;
    }
    double t_min = std::sqrt(a_min*a_min*a_min / mu) * ((alpha - std::sin(alpha)) - (M_PI - std::sin(M_PI)));

    // If the time of flight is less than minimum, return error
    if (tof < t_min) {
        return std::make_pair(Vector3(), Vector3());
    }

    // Iteratively solve for semi-major axis that gives desired time of flight

    double a_lower = a_min;  // Lower bound for a
    double a_upper = a_min * 100.0;  // Upper bound for a (arbitrary large value)
    double a = a_min;
    double tof_current = t_min;

    int max_iter = 500;
    double tolerance = 1e-8;

    for (int i = 0; i < max_iter && std::abs(tof_current - tof) > tolerance; ++i) {
        // Bisection method
        if (tof_current < tof) {
            a_lower = a;
        } else {
            a_upper = a;
        }
        a = (a_lower + a_upper) / 2.0;

        //  time of flight with current a
        double alpha = 2.0 * std::asin(std::sqrt(s / (2.0 * a)));
        double beta = 2.0 * std::asin(std::sqrt((s - chord) / (2.0 * a)));

        if (theta > M_PI) {
            alpha = 2.0 * M_PI - alpha;
        }

        tof_current = std::sqrt(a*a*a / mu) * ((alpha - std::sin(alpha)) - (beta - std::sin(beta)));
    }

    // Unit vectors for transfer orbit plane
    Vector3 y_T0(
        h_T.y * r1_hat.z - h_T.z * r1_hat.y,
        h_T.z * r1_hat.x - h_T.x * r1_hat.z,
        h_T.x * r1_hat.y - h_T.y * r1_hat.x
    );

    Vector3 y_Tf(
        h_T.y * r2_hat.z - h_T.z * r2_hat.y,
        h_T.z * r2_hat.x - h_T.x * r2_hat.z,
        h_T.x * r2_hat.y - h_T.y * r2_hat.x
    );

    //  Lagrange coeffs
/*
    double sin_theta = std::sin(theta);
    double g;
    if (std::abs(sin_theta) < 1e-10) {
        // Special case handling when sin(theta) ≈ 0
        // Use alternative formulation or perturbation technique
        if (theta < 1e-10 || std::abs(theta - 2*M_PI) < 1e-10) {
            // For θ ≈ 0 (direct transfer)
            // ...
        } else if (std::abs(theta - M_PI) < 1e-10) {
            // For θ ≈ π (transfer through central body)
            // ...
        }
    } else {
        g = r1_mag * r2_mag * sin_theta / std::sqrt(mu * a);
    }
*/
    double f = 1.0 - r2_mag / a * (1.0 - std::cos(theta));
    double g = r1_mag * r2_mag * std::sin(theta) / std::sqrt(_MU * a);
    double g_dot = 1.0 - r1_mag / a * (1.0 - std::cos(theta));
    double f_dot = -std::sqrt(_MU / a) * std::sin(theta) / (r1_mag * r2_mag) * r2_mag;

    // Check that the input values are valid
    assert(r1_mag > 0.0 && "r1 magnitude must be positive");
    assert(r2_mag > 0.0 && "r2 magnitude must be positive");
    assert(mu > 0.0 && "Gravitational parameter must be positive");
    assert(tof > 0.0 && "Time of flight must be positive");

    // Check intermediate calculations
    assert(a > 0.0 && "Semi-major axis must be positive");
    assert(std::abs(g) > 1e-10 && "g is too close to zero for division");

    //std::cout << "F: " << f << std::endl;
    //std::cout << "G: " << g << std::endl;
    //std::cout << "G_dot: " << g_dot << std::endl;
    //std::cout << "F_dot: " << f_dot << std::endl;

    //  velocity vectors
    Vector3 v1, v2;
    v1.x = (r2.x - f * r1.x) / g;
    v1.y = (r2.y - f * r1.y) / g;
    v1.z = (r2.z - f * r1.z) / g;

    v2.x = f_dot * r1.x + g_dot * v1.x;
    v2.y = f_dot * r1.y + g_dot * v1.y;
    v2.z = f_dot * r1.z + g_dot * v1.z;

    std::cout << "v2.x: " << v2.x << std::endl;
    std::cout << "v2.y: " << v2.y << std::endl;
    std::cout << "v2.z: " << v2.z << std::endl;

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
        Vector3 r_init = calculatePosition3D(_R1, _e1, _i1, _raan1, _omega1, x[0]);
        Vector3 r_final = calculatePosition3D(_R2, _e2, _i2, _raan2, _omega2, x[1]);

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

    //std::cout << "Relaxation Factor: " << relaxationFactor << std::endl;
    //std::cout << "Total Violation: " << totalViolation << std::endl;

    return totalViolation;
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

template <typename T, typename Fun>
double OrbitTransferObjective<T, Fun>::getRelaxationFactor() const {
    return 2.0 - (static_cast<double>(PSOGlobals::currentIteration) / PSOGlobals::maxIterations);
}

template class OrbitTransferObjective<double, std::function<double(double*)>>;
