#include "core/LambertSolver.hpp"
#include "core/Constants.hpp"

#include <cmath>
#include <iostream>
#include <omp.h>

namespace Physics {

LambertSolver::LambertSolver(double mu) : mu_(mu) {}

std::pair<Physics::Vector3, Physics::Vector3> LambertSolver::solveLambert(
    const Physics::Vector3& r1_glm,
    const Physics::Vector3& r2_glm,
    double tof,
    double mu,
    bool isLongWay)
{

    Physics::Vector3 r1(r1_glm);
    Physics::Vector3 r2(r2_glm);

    double min_tof = calculateMinimumTOF(r1, r2, isLongWay);

    if (tof < min_tof * 0.9) {  // 10% tolerance
        return std::make_pair(Physics::Vector3(0, 0, 0), Physics::Vector3(0, 0, 0));
    }

    // Position magnitudes and unit vectors
    double r1_mag = r1.magnitude();
    double r2_mag = r2.magnitude();
    Physics::Vector3 r1_hat( r1.x / r1_mag, r1.y / r1_mag, r1.z / r1_mag);
    Physics::Vector3 r2_hat( r2.x / r2_mag, r2.y / r2_mag, r2.z / r2_mag);

    // Transfer angle
    double cos_theta = (r1_hat.x * r2_hat.x + r1_hat.y * r2_hat.y + r1_hat.z * r2_hat.z);
    cos_theta = std::min(1.0, std::max(-1.0, cos_theta));
    double theta_tilde = std::acos(cos_theta);

    // Transfer plane normal vector
    Physics::Vector3 h_T(r1_hat.y * r2_hat.z - r1_hat.z * r2_hat.y,
                r1_hat.z * r2_hat.x - r1_hat.x * r2_hat.z,
                r1_hat.x * r2_hat.y - r1_hat.y * r2_hat.x);
    double h_T_mag = h_T.magnitude();

    if (h_T_mag < 1e-10) {
        h_T = Physics::Vector3(0, 0, 1);
    } else {
        h_T = Physics::Vector3(h_T.x / h_T_mag, h_T.y / h_T_mag, h_T.z / h_T_mag);
    }

    double theta = isLongWay ? 2 * M_PI - theta_tilde : theta_tilde;

    // Geometric parameters
    double chord = std::sqrt(r1_mag*r1_mag + r2_mag*r2_mag - 2*r1_mag*r2_mag*std::cos(theta));
    double s = (r1_mag + r2_mag + chord) / 2.0;
    double a_min = s / 2.0; // Minimum energy ellipse semi-major axis

    // Minimum time of flight
    double alpha_min = 2.0 * std::asin(std::sqrt((s - chord) / s));
    if (theta > M_PI) {
        alpha_min = 2.0 * M_PI - alpha_min;
    }
    double t_min = std::sqrt(a_min*a_min*a_min / mu) * (alpha_min - std::sin(alpha_min) - M_PI + std::sin(M_PI));

    if (tof < t_min) {
        std::cout << "Error: Time of flight is less than minimum possible." << std::endl;
        return std::make_pair(glm::dvec3(), glm::dvec3());
    }

    // Semi-major axis using bisection method
    double a_lower = a_min;
    double a_upper = a_min * 100.0;
    double a = a_min;
    double tof_current = t_min;

    const int max_iter = 100;
    const double tolerance = 1e-8;

    for (int i = 0; i < max_iter && std::abs(tof_current - tof) > tolerance; ++i) {

        if (tof_current < tof) {
            a_lower = a;
        } else {
            a_upper = a;
        }
        a = (a_lower + a_upper) / 2.0;

        // Time of flight for current semi-major axis
        double alpha, beta, sin_alpha, sin_beta;

        #ifdef USE_OPENMP
        #pragma omp parallel sections
        {
            #pragma omp section
            {
                alpha = 2.0 * std::asin(std::sqrt(s / (2.0 * a)));

                if (theta > M_PI) {
                    alpha = 2.0 * M_PI - alpha;
                }
                sin_alpha = std::sin(alpha);
            }

            #pragma omp section
            {
                beta = 2.0 * std::asin(std::sqrt((s - chord) / (2.0 * a)));
                sin_beta = std::sin(beta);
            }
        }
        #else
        {
            alpha = 2.0 * std::asin(std::sqrt(s / (2.0 * a)));
            if (theta > M_PI) {
                alpha = 2.0 * M_PI - alpha;
            }
            sin_alpha = std::sin(alpha);
            beta = 2.0 * std::asin(std::sqrt((s - chord) / (2.0 * a)));

        }
        #endif

        tof_current = std::sqrt(a*a*a / mu) * (alpha - sin_alpha - (beta - sin_beta));
    }

    // Lagrange coefficients
    double sin_theta = std::sin(theta);
    double f, g, f_dot, g_dot;

    if (std::abs(sin_theta) < 1e-10) {
        // Special case handling for nearly 0° or 180° transfers
        double delta = 1e-10;

        if (std::abs(theta) < delta || std::abs(theta - 2*M_PI) < delta) {
            // For θ ≈ 0° (direct transfer)
            double h = r2_mag - r1_mag;

            #ifdef USE_OPENMP
            #pragma omp parallel sections
            {
                #pragma omp section
                {
                    f = 1.0 - h/a;
                    f_dot = -sqrt(mu/(a*a*a)) * h;
                }
                #pragma omp section
                {
                    g = tof - sqrt(a*a*a/mu) * (theta - sin_theta);
                    g_dot = 1.0;
                }
            }
            #else
                f = 1.0 - h/a;
                f_dot = -std::sqrt(mu/(a*a*a)) * h;
                g = tof - std::sqrt(a*a*a/mu) * (theta - sin_theta);
                g_dot = 1.0;
            #endif

        } else if (std::abs(theta - M_PI) < delta) {
            // For θ ≈ 180° (transfer through central body)
            f = -1.0;
            g = sqrt(a*a*a/mu) * (M_PI - 2.0);
            g_dot = -1.0;
            f_dot = 0.0;
        } else {
            std::cerr << "Error in transfer angle calculation" << std::endl;
            return std::make_pair(glm::dvec3(), glm::dvec3());
        }
    } else {
        // Normal case
        #ifdef USE_OPENMP
        #pragma omp parallel sections
        {
            #pragma omp section
            {
                f = 1.0 - r2_mag / a * (1.0 - std::cos(theta));
                g_dot = 1.0 - r1_mag / a * (1.0 - std::cos(theta));
            }
            #pragma omp section
            {
                g = r1_mag * r2_mag * sin_theta / std::sqrt(mu * a);
                f_dot = -std::sqrt(mu / a) * sin_theta / (r1_mag * r2_mag) * r2_mag;
            }
        }
        #else
        f = 1.0 - r2_mag / a * (1.0 - std::cos(theta));
        g = r1_mag * r2_mag * sin_theta / std::sqrt(mu * a);
        g_dot = 1.0 - r1_mag / a * (1.0 - std::cos(theta));
        f_dot = -std::sqrt(mu / a) * sin_theta / (r1_mag * r2_mag) * r2_mag;
        #endif
    }

    // Velocity vectors
    Physics::Vector3 v1((r2.x - f * r1.x) / g,
                        (r2.y - f * r1.y) / g,
                        (r2.z - f * r1.z) / g);

    Physics::Vector3 v2(f_dot * r1.x + g_dot * v1.x,
                        f_dot * r1.y + g_dot * v1.y,
                        f_dot * r1.z + g_dot * v1.z);

    return std::make_pair(v1, v2);
}

std::optional<LambertSolver::Solution> LambertSolver::solve(
    const glm::dvec3& r1,
    const glm::dvec3& r2,
    double tof,
    const Config& config) const {

    auto result = solveLambert(r1, r2, tof, mu_, config.long_way);

    // Sanity check
    if (result.first.magnitude() < 1e-10 && result.second.magnitude() < 1e-10) {
        return std::nullopt;
    }

    Solution sol;
    sol.v1 = result.first;
    sol.v2 = result.second;
    sol.is_valid = true;

    // Semi-major axis and eccentricity
    double r1_mag = glm::length(r1);
    double v1_mag = glm::length(sol.v1);
    double specific_energy = v1_mag * v1_mag / 2.0 - mu_ / r1_mag;
    sol.semi_major_axis = -mu_ / (2.0 * specific_energy);

    glm::dvec3 h = glm::cross(r1, sol.v1);
    double h_mag = glm::length(h);
    sol.eccentricity = std::sqrt(1.0 + 2.0 * specific_energy * h_mag * h_mag / (mu_ * mu_));

    return sol;
}

double LambertSolver::calculateMinimumTOF(const glm::dvec3& r1,
                                         const glm::dvec3& r2,
                                         bool long_way) {
    double r1_mag = glm::length(r1);
    double r2_mag = glm::length(r2);

    glm::dvec3 r1_hat = r1 / r1_mag;
    glm::dvec3 r2_hat = r2 / r2_mag;

    double cos_theta = glm::dot(r1_hat, r2_hat);
    double theta = std::acos(glm::clamp(cos_theta, -1.0, 1.0));

    if (long_way) {
        theta = 2 * M_PI - theta;
    }

    double chord = std::sqrt(r1_mag * r1_mag + r2_mag * r2_mag -
                            2 * r1_mag * r2_mag * std::cos(theta));
    double s = (r1_mag + r2_mag + chord) / 2.0;
    double a_min = s / 2.0;

    double alpha = 2.0 * std::asin(std::sqrt(s / (2.0 * a_min)));
    if (theta > M_PI) {
        alpha = 2 * M_PI - alpha;
    }

    return std::sqrt(a_min * a_min * a_min / constant::MU) * (alpha - std::sin(alpha));
}

bool LambertSolver::validateSolution(const Solution& solution,
                                    const glm::dvec3& r1,
                                    const glm::dvec3& r2,
                                    double expected_tof) const {
    // Propagate from r1 with v1 and check if we reach r2

    double r1_mag = glm::length(r1);
    double r2_mag = glm::length(r2);

    // Check energy consistency
    double v1_mag = glm::length(solution.v1);
    double v2_mag = glm::length(solution.v2);

    double energy1 = v1_mag * v1_mag / 2.0 - mu_ / r1_mag;
    double energy2 = v2_mag * v2_mag / 2.0 - mu_ / r2_mag;

    if (std::abs(energy1 - energy2) > 1e-6) {
        return false; // Energy not conserved
    }

    // Check angular momentum consistency
    glm::dvec3 h1 = glm::cross(r1, solution.v1);
    glm::dvec3 h2 = glm::cross(r2, solution.v2);

    if (glm::length(h1 - h2) > 1e-6) {
        return false; // Angular momentum not conserved
    }

    return true;
}

// Stumpff functions for universal variable formulation
double LambertSolver::stumpffC(double z) const {
    if (std::abs(z) < 1e-10) {
        return 0.5;
    } else if (z > 0) {
        double sqrt_z = std::sqrt(z);
        return (1.0 - std::cos(sqrt_z)) / z;
    } else {
        double sqrt_neg_z = std::sqrt(-z);
        return (std::cosh(sqrt_neg_z) - 1.0) / (-z);
    }
}

double LambertSolver::stumpffS(double z) const {
    if (std::abs(z) < 1e-10) {
        return 1.0 / 6.0;
    } else if (z > 0) {
        double sqrt_z = std::sqrt(z);
        return (sqrt_z - std::sin(sqrt_z)) / (z * sqrt_z);
    } else {
        double sqrt_neg_z = std::sqrt(-z);
        return (std::sinh(sqrt_neg_z) - sqrt_neg_z) / (-z * sqrt_neg_z);
    }
}

} // namespace Physics
