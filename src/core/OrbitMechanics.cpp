#include "core/OrbitMechanics.hpp"
#include "core/Constants.hpp"
#include <cmath>

namespace Physics {

std::pair<double, double> OrbitMechanics::calculateVelocity(double a, double e, double trueAnomaly)
{
    double r = a * (1 - e*e) / (1 + e * std::cos(trueAnomaly));
    double v_r = std::sqrt(constant::MU/a) * e * std::sin(trueAnomaly) / std::sqrt(1 - e*e);
    double v_theta = std::sqrt(constant::MU/a) * (1 + e * std::cos(trueAnomaly)) / std::sqrt(1 - e*e);
    return std::make_pair(v_r, v_theta);
}

Vector3 OrbitMechanics::calculateVelocity3D(double a, double e, double i, double raan,
                                           double omega, double true_anomaly)
{
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
    const double sqrt_mu_p = sqrt(constant::MU / p);

    // Velocity components in perifocal frame (radial and transverse)
    const double v_r = sqrt_mu_p * e * sin_nu;
    const double v_theta = sqrt_mu_p * (1 + e * cos_nu);

    // Rotations
    const double v_x_omega = v_r * cos_omega - v_theta * sin_omega;
    const double v_y_omega = v_r * sin_omega + v_theta * cos_omega;
    const double v_z_omega = 0.0;  // No z-component in perifocal frame

    const double v_x_inc = v_x_omega;
    const double v_y_inc = v_y_omega * cos_i - v_z_omega * sin_i;
    const double v_z_inc = v_y_omega * sin_i + v_z_omega * cos_i;

    const double v_x = v_x_inc * cos_raan - v_y_inc * sin_raan;
    const double v_y = v_x_inc * sin_raan + v_y_inc * cos_raan;
    const double v_z = v_z_inc;

    return Vector3(v_x, v_y, v_z);
}

Vector3 OrbitMechanics::calculatePosition3D(double a, double e, double i, double raan,
                                          double omega, double true_anomaly)
{
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

    if (std::abs(x) < 1e-14) x = 0.0;
    if (std::abs(y) < 1e-14) y = 0.0;
    if (std::abs(z) < 1e-14) z = 0.0;

    return Vector3(x, y, z);
}

double OrbitMechanics::calculateRadius(double a, double e, double trueAnomaly)
{
    return a * (1 - e*e) / (1 + e * std::cos(trueAnomaly));
}

glm::vec3 OrbitMechanics::calculateOrbitPosition(float radius, float inclination, float true_anomaly)
{
    // Position in orbital plane
    float x = radius * cos(true_anomaly);
    float y = radius * sin(true_anomaly);
    float z = 0.0f;

    // Rotate by inclination (around x-axis)
    float y_rotated = y * cos(inclination);
    float z_rotated = y * sin(inclination);

    return glm::vec3(x, y_rotated, z_rotated);
}

glm::vec3 OrbitMechanics::calculateOrbitVelocity(float radius, float inclination, float true_anomaly)
{
    float speed = sqrt(constant::MU / radius);

    // Velocity in orbital plane
    float vx = -speed * sin(true_anomaly);
    float vy = speed * cos(true_anomaly);
    float vz = 0.0f;

    // Rotate by inclination (around x-axis)
    float vy_rotated = vy * cos(inclination);
    float vz_rotated = vy * sin(inclination);

    return glm::vec3(vx, vy_rotated, vz_rotated);
}

} // namespace Physics
