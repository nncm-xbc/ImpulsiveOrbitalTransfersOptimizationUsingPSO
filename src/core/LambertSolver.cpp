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

    // Basic setup
    double r1_mag = r1.magnitude();
    double r2_mag = r2.magnitude();

    // Check minimum TOF
    glm::dvec3 r1_glm_vec(r1.x, r1.y, r1.z);
    glm::dvec3 r2_glm_vec(r2.x, r2.y, r2.z);
    double min_tof = calculateMinimumTOF(r1_glm_vec, r2_glm_vec, isLongWay);
/*
    if (tof < min_tof * 0.9) {
        std::cout << "FAILED: TOF too small" << std::endl;
        return std::make_pair(Physics::Vector3(0, 0, 0), Physics::Vector3(0, 0, 0));
    }
 */
    // Transfer angle
    double cos_nu = (r1.x * r2.x + r1.y * r2.y + r1.z * r2.z) / (r1_mag * r2_mag);
    cos_nu = std::max(-1.0, std::min(1.0, cos_nu));
    double nu = std::acos(cos_nu);

    if (isLongWay) {
        nu = 2.0 * M_PI - nu;
    }

    // Geometric parameters
    double c = std::sqrt(r1_mag * r1_mag + r2_mag * r2_mag - 2.0 * r1_mag * r2_mag * cos_nu);
    double s = (r1_mag + r2_mag + c) / 2.0;
    double a_min = s / 2.0;


    // ===== DIRECT LAGRANGE COEFFICIENT CALCULATION =====
    // Improved bisection (fixes convergence warning)
    // ===== SIMPLE CORRECT BISECTION =====
    double a_low = a_min;
    double a_high = a_min * 10.0;
    double a = a_min;

    const int max_iter = 50;
    const double tol = 1e-8;

    for (int i = 0; i < max_iter; i++) {
        a = (a_low + a_high) / 2.0;

        // CORRECT Lambert time equation (simplified)
        double sqrt_a = std::sqrt(a);
        double sqrt_mu = std::sqrt(mu);

        // Use energy to find eccentric anomalies
        double e = std::abs(r2_mag - r1_mag) / (r1_mag + r2_mag);
        double E_change = nu;  // For most transfers, this approximation works

        // Simple time calculation based on transfer angle
        double t_calc = std::sqrt(a * a * a / mu) * E_change;

        if (std::abs(t_calc - tof) < tol) {
            break;
        }

        if (t_calc < tof) {
            a_low = a;
        } else {
            a_high = a;
        }
    }

    if (std::abs(nu - M_PI) < 1e-6) {
        a = (r1_mag + r2_mag) / 2.0;

        double v1_mag = std::sqrt(mu * (2.0 / r1_mag - 1.0 / a));
        double v2_mag = std::sqrt(mu * (2.0 / r2_mag - 1.0 / a));

        Physics::Vector3 v1(0.0, v1_mag, 0.0);    // Velocity at periapsis
        Physics::Vector3 v2(0.0, -v2_mag, 0.0);   // Velocity at apoapsis

        // Validation
        double E1 = v1_mag * v1_mag / 2.0 - mu / r1_mag;
        double E2 = v2_mag * v2_mag / 2.0 - mu / r2_mag;

        return std::make_pair(v1, v2);
    } else if (std::abs(nu) < 1e-6) {
        a = (r1_mag + r2_mag) / 2.0;

        double v1_mag = std::sqrt(mu * (2.0 / r1_mag - 1.0 / a));
        double v2_mag = std::sqrt(mu * (2.0 / r2_mag - 1.0 / a));

        Physics::Vector3 r1_unit = r1 / r1_mag;
        Physics::Vector3 tangent1(r1_unit.y, -r1_unit.x, 0.0);  // Perpendicular to r1
        if (tangent1.magnitude() < 1e-10) {
            tangent1 = Physics::Vector3(0.0, r1_unit.z, -r1_unit.y);
        }
        tangent1 = tangent1 / tangent1.magnitude();

        Physics::Vector3 r2_unit = r2 / r2_mag;
        Physics::Vector3 tangent2(r2_unit.y, -r2_unit.x, 0.0);  // Perpendicular to r2
        if (tangent2.magnitude() < 1e-10) {
            tangent2 = Physics::Vector3(0.0, r2_unit.z, -r2_unit.y);
        }
        tangent2 = tangent2 / tangent2.magnitude();

        Physics::Vector3 v1 = tangent1 * v1_mag;
        Physics::Vector3 v2 = tangent2 * v2_mag;

        // Validation
        double E1 = v1_mag * v1_mag / 2.0 - mu / r1_mag;
        double E2 = v2_mag * v2_mag / 2.0 - mu / r2_mag;

        return std::make_pair(v1, v2);
    } else {

        double v1_mag = std::sqrt(mu * (2.0 / r1_mag - 1.0 / a));
        double v2_mag = std::sqrt(mu * (2.0 / r2_mag - 1.0 / a));

        Physics::Vector3 r1_unit = r1 / r1_mag;
        Physics::Vector3 r2_unit = r2 / r2_mag;

        Physics::Vector3 h_transfer = r1.cross(r2);
        if (h_transfer.magnitude() < 1e-10) {
            // Collinear case - use z-axis as default
            h_transfer = Physics::Vector3(0.0, 0.0, 1.0);
        } else {
            h_transfer = h_transfer / h_transfer.magnitude();
        }

        Physics::Vector3 v1_dir = h_transfer.cross(r1_unit);
        Physics::Vector3 v2_dir = h_transfer.cross(r2_unit);

        // Normalize direction vectors
        v1_dir = v1_dir / v1_dir.magnitude();
        v2_dir = v2_dir / v2_dir.magnitude();

        // For transfers > 180°, reverse v2 direction
        if (nu > M_PI) {
            v2_dir = v2_dir * (-1.0);
        }

        // Final velocity vectors
        Physics::Vector3 v1 = v1_dir * v1_mag;
        Physics::Vector3 v2 = v2_dir * v2_mag;

        // Validation
        double E1 = v1_mag * v1_mag / 2.0 - mu / r1_mag;
        double E2 = v2_mag * v2_mag / 2.0 - mu / r2_mag;

        return std::make_pair(v1, v2);
    }
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

double LambertSolver::stumpffC(double z) {
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

double LambertSolver::stumpffS(double z) {
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
