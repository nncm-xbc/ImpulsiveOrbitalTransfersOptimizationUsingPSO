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
    const double r = p / (1 + e * cos_nu);
    const double sqrt_mu_p = sqrt(constant::MU / p);

    // Velocity components in perifocal frame (radial and transverse)
    const double v_r = sqrt_mu_p * e * sin_nu;
    const double v_theta = sqrt_mu_p * (1 + e * cos_nu);

    // Combined rotation matrix application (perifocal to inertial frame)
    const double v_P = v_r * cos_nu - v_theta * sin_nu;
    const double v_Q = v_r * sin_nu + v_theta * cos_nu;

    // Rotations
    double v_x = (v_P * cos_omega - v_Q * sin_omega) * cos_raan -
                 ((v_P * sin_omega + v_Q * cos_omega) * cos_i) * sin_raan;
    double v_y = (v_P * cos_omega - v_Q * sin_omega) * sin_raan +
                 ((v_P * sin_omega + v_Q * cos_omega) * cos_i) * cos_raan;
    double v_z = (v_P * sin_omega + v_Q * cos_omega) * sin_i;

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

Vector3 OrbitMechanics::calculateImpulseVector(const Vector3& pos, const Vector3& vel,
                                              double magnitude, double planeChangeAngle)
{
    // Unit vectors for orbital frame
    // Tangential = velocity direction
    double v_mag = sqrt(vel.x*vel.x + vel.y*vel.y + vel.z*vel.z);
    Vector3 t_hat = {vel.x/v_mag, vel.y/v_mag, vel.z/v_mag};

    // Radial = position direction
    double r_mag = sqrt(pos.x*pos.x + pos.y*pos.y + pos.z*pos.z);
    Vector3 r_hat = {pos.x/r_mag, pos.y/r_mag, pos.z/r_mag};

    Vector3 n_hat = {
        r_hat.y * t_hat.z - r_hat.z * t_hat.y,
        r_hat.z * t_hat.x - r_hat.x * t_hat.z,
        r_hat.x * t_hat.y - r_hat.y * t_hat.x
    };

    // Normal vector
    double n_mag = sqrt(n_hat.x*n_hat.x + n_hat.y*n_hat.y + n_hat.z*n_hat.z);
    n_hat.x /= n_mag;
    n_hat.y /= n_mag;
    n_hat.z /= n_mag;

    // Impulse components
    double tangential_component = magnitude * cos(planeChangeAngle);
    double normal_component = magnitude * sin(planeChangeAngle);

    // Impulse vector = tangential component + normal component
    Vector3 impulse = {
        t_hat.x * tangential_component + n_hat.x * normal_component,
        t_hat.y * tangential_component + n_hat.y * normal_component,
        t_hat.z * tangential_component + n_hat.z * normal_component
    };

    return impulse;
}

} // namespace Physics
