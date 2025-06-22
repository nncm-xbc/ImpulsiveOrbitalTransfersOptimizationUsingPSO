#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

#include "core/Constants.hpp"
#include "core/OrbitMechanics.hpp"
#include "core/CoordinateSystem.hpp"
#include "visualization/Shader.hpp"
#include "visualization/OrbitModel.hpp"
#include "visualization/TransferModel.hpp"

TransferModel::TransferModel() : vao_(0), vbo_(0), initialized_(false) {}

TransferModel::~TransferModel() {
    if (initialized_) {
        glDeleteVertexArrays(1, &vao_);
        glDeleteBuffers(1, &vbo_);
    }
}

void TransferModel::initialize() {
    if (initialized_) return;

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);

    initialized_ = true;
}

void TransferModel::setTwoImpulseTransfer(double initialRadius, double initialInclination,
                        double targetRadius, double targetInclination,
                        double initialEccentricity, double targetEccentricity,
                        double initialTrueAnomaly, double finalTrueAnomaly,
                        std::vector<double> impulseMagnitudes, std::vector<double> planeChange) {
    initial_radius_ = initialRadius;
    target_radius_ = targetRadius;
    initial_inclination_ = initialInclination;
    target_inclination_ = targetInclination;
    initial_eccentricity_ = 0.0;
    target_eccentricity_ = 0.0;
    initial_true_anomaly_ = initialTrueAnomaly;
    final_true_anomaly_ = finalTrueAnomaly;
    impulse_magnitudes_ = impulseMagnitudes;
    plane_change_ = planeChange;

    generateTransferTrajectory();
    updateBuffers();
    debugTargetPosition();

    glm::vec3 expected_target(1.5, 0, 0);  // What we expect
    glm::vec3 actual_target = Physics::OrbitMechanics::calculateOrbitPosition(
        target_radius_, target_inclination_, final_true_anomaly_);

    if (glm::length(actual_target - expected_target) > 0.1) {
        std::cout << "WARNING: Target position mismatch!" << std::endl;
        std::cout << "  Expected: (" << expected_target.x << ", " << expected_target.y
                    << ", " << expected_target.z << ")" << std::endl;
        std::cout << "  Actual: (" << actual_target.x << ", " << actual_target.y
                    << ", " << actual_target.z << ")" << std::endl;

        // Auto-correct if needed
        if (std::abs(targetInclination) < 1e-6 && std::abs(targetEccentricity) < 1e-6) {
            // For circular equatorial orbit, calculate correct true anomaly
            final_true_anomaly_ = atan2(expected_target.y, expected_target.x);
            std::cout << "  Auto-corrected final_true_anomaly to "
                        << final_true_anomaly_ * 180/M_PI << "°" << std::endl;
        }
    } else {
        std::cout << "Target position matches expected!" << std::endl;
    }
}

const std::vector<glm::vec3>& TransferModel::getImpulsePositions() const { return impulse_positions_; }

const std::vector<glm::vec3>& TransferModel::getImpulseDirections() const { return impulse_directions_; }

const std::vector<double>& TransferModel::getImpulseMagnitudes() const { return impulse_magnitudes_; }

void TransferModel::generateTransferTrajectory() {
    transfer_points_.clear();
    complete_ellipse_points_.clear();

    // ============================================
    // COMMON CALCULATIONS FOR BOTH CASES
    // ============================================

    // Position vectors at departure and arrival
    glm::vec3 r0 = Physics::OrbitMechanics::calculateOrbitPosition(
        initial_radius_, initial_inclination_, initial_true_anomaly_);
    glm::vec3 rf = Physics::OrbitMechanics::calculateOrbitPosition(
        target_radius_, target_inclination_, final_true_anomaly_);

    printPositionVerification(r0, glm::vec3(1.0f, 0.0f, 0.0f), "Initial Position");
    printPositionVerification(rf, glm::vec3(1.5f, 0.0f, 0.0f), "Target Position");

    float r0_mag = glm::length(r0);
    float rf_mag = glm::length(rf);
    glm::vec3 r0_hat = glm::normalize(r0);
    glm::vec3 rf_hat = glm::normalize(rf);

    // Circular orbit velocities (before impulses)
    float v0_mag = sqrt(constant::MU / initial_radius_);
    float vf_mag = sqrt(constant::MU / target_radius_);

    // Check if transfer is coplanar
    bool is_coplanar = (initial_inclination_ < 1e-6f && target_inclination_ < 1e-6f);
    printTransferGenerationHeader(is_coplanar);

    // Visualization parameters
    const int resolution = 500;
    const float visualScale = 1000.0f;

    // ============================================
    // COPLANAR CASE
    // ============================================
    if (is_coplanar) {
        // Local coordinate system at departure point
        glm::vec3 normal_dir = glm::vec3(0.0f, 0.0f, 1.0f);  // z-axis for coplanar
        glm::vec3 tangential_dir = glm::normalize(glm::cross(normal_dir, r0_hat));

        // Initial velocity vector
        glm::vec3 v0_minus = v0_mag * tangential_dir;

        // First impulse (purely tangential for coplanar)
        float tangential_component = impulse_magnitudes_[0];
        glm::vec3 v0_plus = v0_minus + tangential_component * tangential_dir;

        // Transfer orbit parameters
        glm::vec3 h_transfer = glm::cross(r0, v0_plus);
        float h_transfer_mag = glm::length(h_transfer);

        // Semi-major axis from energy equation
        float v0_plus_mag = glm::length(v0_plus);
        transfer_semi_major_ = (constant::MU * r0_mag) / (2.0f * constant::MU - r0_mag * v0_plus_mag * v0_plus_mag);

        // Eccentricity from angular momentum
        transfer_eccentricity_ = sqrt(1.0f - (h_transfer_mag * h_transfer_mag) / (constant::MU * transfer_semi_major_));

        // Eccentricity vector points toward periapsis
        glm::vec3 e_vec = glm::cross(v0_plus, h_transfer) / float(constant::MU) - r0_hat;
        glm::vec3 periapsis_dir = glm::normalize(e_vec);
        glm::vec3 perpendicular = glm::normalize(glm::cross(glm::vec3(0.0f, 0.0f, 1.0f), periapsis_dir));

        // For circular-to-circular transfers
        transfer_semi_major_ = (initial_radius_ + target_radius_) / 2.0f;
        transfer_eccentricity_ = std::abs(initial_radius_ - target_radius_) / (initial_radius_ + target_radius_);

        // Periapsis direction points toward the smaller orbit
        if (initial_radius_ < target_radius_) {
            periapsis_dir = r0_hat;
        } else {
            periapsis_dir = -r0_hat;
        }

        // Perpendicular for the orbital plane (2D)
        perpendicular = glm::vec3(-periapsis_dir.y, periapsis_dir.x, 0.0f);
        perpendicular = glm::normalize(perpendicular);

        // Semi-latus rectum
        float p = transfer_semi_major_ * (1.0f - transfer_eccentricity_ * transfer_eccentricity_);

        // Generate complete ellipse points
        std::vector<glm::vec3> physics_ellipse_points;
        std::vector<glm::vec3> physics_transfer_points;

        for (int i = 0; i <= resolution; ++i) {
            float true_anomaly = 2.0f * M_PI * i / resolution;
            float r = p / (1.0f + transfer_eccentricity_ * cos(true_anomaly));

            float r_cos = r * cos(true_anomaly);
            float r_sin = r * sin(true_anomaly);

            glm::vec3 pos = glm::vec3(
                periapsis_dir.x * r_cos + perpendicular.x * r_sin,
                periapsis_dir.y * r_cos + perpendicular.y * r_sin,
                0.0f  // Z = 0 for coplanar
            );

            physics_ellipse_points.push_back(pos);
        }
        complete_ellipse_points_ = CoordinateSystem::trajectoryToVisualization(physics_ellipse_points);

        // Generate transfer trajectory points
        for (int i = 0; i <= resolution/2; ++i) {
            float t = static_cast<float>(i) / (resolution/2);
            float true_anomaly = t * M_PI;  // 0 to 180 degrees

            float r = p / (1.0f + transfer_eccentricity_ * cos(true_anomaly));

            float r_cos = r * cos(true_anomaly);
            float r_sin = r * sin(true_anomaly);

            glm::vec3 pos = glm::vec3(
                periapsis_dir.x * r_cos + perpendicular.x * r_sin,
                periapsis_dir.y * r_cos + perpendicular.y * r_sin,
                0.0f  // Z = 0 for coplanar
            );

            physics_transfer_points.push_back(pos);
        }
        transfer_points_ = CoordinateSystem::trajectoryToVisualization(physics_transfer_points);
    }

    // ============================================
    // NON-COPLANAR CASE
    // ============================================
    else {
        // Angular momentum unit vectors for initial and target orbits
        glm::vec3 h0_hat(0.0f, 0.0f, 1.0f);  // Initial orbit normal
        if (initial_inclination_ != 0.0f) {
            // Apply inclination rotation
            float cos_incl = cos(initial_inclination_);
            float sin_incl = sin(initial_inclination_);
            h0_hat = glm::vec3(0.0f, sin_incl, cos_incl);  // Rotated around x-axis
        }

        glm::vec3 hf_hat(0.0f, 0.0f, 1.0f);  // Target orbit normal
        if (target_inclination_ != 0.0f) {
            float cos_incl = cos(target_inclination_);
            float sin_incl = sin(target_inclination_);
            hf_hat = glm::vec3(0.0f, sin_incl, cos_incl);  // Rotated around x-axis
        }

        // Local coordinate system at departure point
        glm::vec3 normal_dir = glm::normalize(h0_hat);
        glm::vec3 tangential_dir = glm::normalize(glm::cross(normal_dir, r0_hat));

        // Initial and final velocity vectors
        glm::vec3 v0_minus = v0_mag * tangential_dir;
        glm::vec3 vf_plus = vf_mag * glm::normalize(glm::cross(hf_hat, rf_hat));

        // Apply first impulse with plane change component
        float plane_change1 = plane_change_[0];
        float plane_change2 = plane_change_[1];

        float dv1_mag = impulse_magnitudes_[0];
        float tangential_component = dv1_mag / sqrt(1.0f + plane_change1 * plane_change1);
        float normal_component = tangential_component * plane_change1;

        glm::vec3 impulse_vec = tangential_component * tangential_dir + normal_component * normal_dir;
        glm::vec3 v0_plus = v0_minus + impulse_vec;

        // Transfer orbit parameters
        glm::vec3 h_transfer = glm::cross(r0, v0_plus);
        float h_transfer_mag = glm::length(h_transfer);
        glm::vec3 h_transfer_hat = glm::normalize(h_transfer);

        // Calculate transfer orbit elements
        float v0_plus_mag = glm::length(v0_plus);
        float specific_energy = 0.5f * v0_plus_mag * v0_plus_mag - constant::MU / r0_mag;
        transfer_semi_major_ = -constant::MU / (2.0f * specific_energy);

        // Eccentricity vector and magnitude
        glm::vec3 e_vec = glm::cross(v0_plus, h_transfer) / float(constant::MU) - r0_hat;
        transfer_eccentricity_ = glm::length(e_vec);

        // Semi-latus rectum
        float p = h_transfer_mag * h_transfer_mag / constant::MU;

        // Orbital element angles
        float transfer_inclination = acos(h_transfer_hat.z);
        float transfer_raan = atan2(h_transfer_hat.x, -h_transfer_hat.y);

        // Direction to periapsis
        glm::vec3 periapsis_dir;
        if (transfer_eccentricity_ < 1e-6f) {
            if (std::abs(h_transfer_hat.x) < 0.9f) {
                periapsis_dir = glm::normalize(glm::cross(glm::vec3(1.0f, 0.0f, 0.0f), h_transfer_hat));
            } else {
                periapsis_dir = glm::normalize(glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), h_transfer_hat));
            }
        } else {
            periapsis_dir = glm::normalize(e_vec);
        }

        // Argument of periapsis
        glm::vec3 n_vec = glm::cross(glm::vec3(0.0f, 0.0f, 1.0f), h_transfer_hat);
        float node_mag = glm::length(n_vec);

        float transfer_arg_periapsis;
        if (node_mag < 1e-6f) {
            // Equatorial orbit - use true longitude
            transfer_arg_periapsis = atan2(periapsis_dir.y, periapsis_dir.x);
        } else {
            n_vec = glm::normalize(n_vec);
            float cos_arg_pe = glm::dot(n_vec, periapsis_dir);
            float sin_arg_pe = glm::dot(h_transfer_hat, glm::cross(n_vec, periapsis_dir));
            transfer_arg_periapsis = atan2(sin_arg_pe, cos_arg_pe);
        }

        // Project target position onto transfer orbit plane
        glm::vec3 rf_projected = rf - glm::dot(rf, h_transfer_hat) * h_transfer_hat;
        glm::vec3 rf_projected_hat = glm::normalize(rf_projected);

        // Calculate true anomalies in the transfer orbit
        glm::vec3 r0_in_transfer_plane = r0 - glm::dot(r0, h_transfer_hat) * h_transfer_hat;
                float r0_mag_in_plane = glm::length(r0_in_transfer_plane);

        // Using orbit equation: r = p / (1 + e*cos(nu))
        // Solving for nu: cos(nu) = (p/r - 1) / e
        float cos_nu0 = (p / r0_mag_in_plane - 1.0f) / transfer_eccentricity_;
        cos_nu0 = glm::clamp(cos_nu0, -1.0f, 1.0f);

        // Determine sign based on velocity direction
        glm::vec3 perpendicular = glm::normalize(glm::cross(h_transfer_hat, periapsis_dir));
        float v0_perp = glm::dot(v0_plus, perpendicular);
        float nu0 = (v0_perp >= 0) ? acos(cos_nu0) : -acos(cos_nu0);

        // For arrival point - already projected
        float rf_mag_projected = glm::length(rf_projected);

        // Find true anomaly at arrival
        float cos_nuf = (p / rf_mag_projected - 1.0f) / transfer_eccentricity_;
        cos_nuf = glm::clamp(cos_nuf, -1.0f, 1.0f);

        // We have two possible values for nuf
        float nuf_option1 = acos(cos_nuf);
        float nuf_option2 = -acos(cos_nuf);

        // Choose the one that gives a reasonable transfer angle
        float transfer_angle1 = nuf_option1 - nu0;
        float transfer_angle2 = nuf_option2 - nu0;

        // Normalize to [0, 2π]
        while (transfer_angle1 < 0) transfer_angle1 += 2 * M_PI;
        while (transfer_angle2 < 0) transfer_angle2 += 2 * M_PI;

        // Choose the appropriate angle (avoid very small or very large transfers)
        float nuf = (transfer_angle1 > 0.1 && transfer_angle1 < 2 * M_PI - 0.1) ? nuf_option1 : nuf_option2;

        // Calculate transfer angle for trajectory generation
        float nu_diff = nuf - nu0;
        if (nu_diff > M_PI) nu_diff -= 2.0f * M_PI;
        if (nu_diff < -M_PI) nu_diff += 2.0f * M_PI;

        // Generate complete ellipse points
        std::vector<glm::vec3> physics_ellipse_points;

        for (int i = 0; i < resolution; i++) {
             double true_anomaly = 2.0 * M_PI * i / (resolution - 1);
             double r = p / (1.0 + transfer_eccentricity_ * cos(true_anomaly));

             // Position in orbital plane
             glm::vec3 pos_orbital(r * cos(true_anomaly), r * sin(true_anomaly), 0.0f);

             // Transform to inertial frame using the unified transformation
             glm::mat3 R = CoordinateSystem::orbitalToInertialMatrix(
                 transfer_raan, transfer_inclination, transfer_arg_periapsis);

             glm::vec3 pos_inertial = R * pos_orbital;

             // Store in physics coordinates first
             physics_ellipse_points.push_back(pos_inertial);
        }

        // Verify the orbit is in the correct plane
        glm::vec3 expected_normal = glm::normalize(h_transfer);
        if (!CoordinateSystem::verifyOrbitalPlane(physics_ellipse_points, expected_normal)) {
            std::cerr << "\n WARNING: Transfer orbit not in expected plane!" << std::endl;
        }

        // Convert all points to visualization coordinates at once
        complete_ellipse_points_ = CoordinateSystem::trajectoryToVisualization(physics_ellipse_points);

        // Generate transfer trajectory points
        std::vector<glm::vec3> physics_transfer_points;

        for (int i = 0; i <= resolution; ++i) {
            float t = static_cast<float>(i) / resolution;

            // Interpolate true anomaly
            float true_anomaly = nu0 + t * nu_diff;

            float r = p / (1.0f + transfer_eccentricity_ * cos(true_anomaly));

            // Position in orbital plane
            glm::vec3 pos_in_plane(r * cos(true_anomaly), r * sin(true_anomaly), 0.0f);

            // Transform to inertial frame
            glm::mat3 R = CoordinateSystem::orbitalToInertialMatrix(
                transfer_raan, transfer_inclination, transfer_arg_periapsis);

            glm::vec3 pos_physics = R * pos_in_plane;

            // Store in physics coordinates
            physics_transfer_points.push_back(pos_physics);

            // Debug first and last points
            if (i == 0 || i == resolution) {
                glm::vec3 expected = (i == 0) ? r0 : rf;
                float dist = glm::length(pos_physics - expected);
                std::cout << "Point " << i << " distance to expected: " << dist << std::endl;
            }
        }

        transfer_points_ = CoordinateSystem::trajectoryToVisualization(physics_transfer_points);
    }

    printTransferCompletion(transfer_points_.size(), complete_ellipse_points_.size());
}

void TransferModel::render(const Shader& shader, const glm::mat4& view_projection,
            const glm::vec3& color, float animation_progress) {
    if (!initialized_) {
        std::cout << "Error: Attempting to render before initialization" << std::endl;
        return;
    }

    if (transfer_points_.empty()) {
        std::cout << "Error: No transfer points to render" << std::endl;
        return;
    }

    shader.use();
    shader.setMat4("viewProjection", view_projection);
    shader.setVec3("color", color);
    glBindVertexArray(vao_);
    glDrawArrays(GL_LINE_STRIP, 0, static_cast<int>(transfer_points_.size() * animation_progress));

    glBindVertexArray(0);
}

void TransferModel::renderCompleteEllipse(const Shader& shader, const glm::mat4& view_projection,
                                         const glm::vec3& color) {
    if (complete_ellipse_points_.empty()) {
        return;
    }

    // Buffers
    static GLuint ellipseVAO = 0, ellipseVBO = 0;
    if (ellipseVAO == 0) {
        glGenVertexArrays(1, &ellipseVAO);
        glGenBuffers(1, &ellipseVBO);
    }

    // Update buffer
    glBindVertexArray(ellipseVAO);
    glBindBuffer(GL_ARRAY_BUFFER, ellipseVBO);
    glBufferData(GL_ARRAY_BUFFER, complete_ellipse_points_.size() * sizeof(glm::vec3),
                complete_ellipse_points_.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);

    // Render the ellipses
    shader.use();
    shader.setMat4("viewProjection", view_projection);
    shader.setVec3("color", color);

    glm::vec3 center = calculateRenderedCenter();

    glBindVertexArray(ellipseVAO);
    glDrawArrays(GL_LINE_STRIP, 0, complete_ellipse_points_.size());
    glBindVertexArray(0);
}

glm::vec3 TransferModel::calculateRenderedCenter() {
    if (complete_ellipse_points_.empty()) {
        return glm::vec3(0.0f, 0.0f, 0.0f);
    }

    glm::vec3 center(0.0f, 0.0f, 0.0f);
    for (const auto& point : complete_ellipse_points_) {
        center += point;
    }
    center /= static_cast<float>(complete_ellipse_points_.size());

    return center;
}

void TransferModel::updateBuffers() {
    if (!initialized_) {
        std::cout << "Error: Attempting to update buffers before initialization" << std::endl;
        return;
    }

    if (transfer_points_.empty()) {
        std::cout << "Error: No transfer points to render" << std::endl;
        return;
    }
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, transfer_points_.size() * sizeof(glm::vec3),
                transfer_points_.data(), GL_STATIC_DRAW);

    // Debug
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        std::cout << "OpenGL error in updateBuffers: " << err << std::endl;
    }

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    std::cout << "Buffer update complete" << std::endl;
}

void TransferModel::debugTargetPosition() {
    std::cout << "\n╔══════════════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                        TARGET POSITION DEBUG                         ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════════════╝" << std::endl;

    double target_radius = 1.5;
    double target_inclination = 0.0;
    double final_true_anomaly = M_PI;

    std::cout << " TARGET PARAMETERS:" << std::endl;
    std::cout << "   ├─ Radius:          " << std::fixed << std::setprecision(3) << target_radius << " DU" << std::endl;
    std::cout << "   ├─ Inclination:     " << std::setprecision(1) << target_inclination * 180/M_PI << "°" << std::endl;
    std::cout << "   └─ True Anomaly:    " << final_true_anomaly * 180/M_PI << "°" << std::endl;

    glm::vec3 target_pos = Physics::OrbitMechanics::calculateOrbitPosition(
        target_radius, target_inclination, final_true_anomaly);

    std::cout << "\n CALCULATED POSITION:" << std::endl;
    std::cout << "   ├─ X: " << std::setprecision(6) << target_pos.x << " DU" << std::endl;
    std::cout << "   ├─ Y: " << target_pos.y << " DU" << std::endl;
    std::cout << "   └─ Z: " << target_pos.z << " DU" << std::endl;

    std::cout << "\n TRUE ANOMALY VERIFICATION:" << std::endl;
    struct TestPoint { double nu; std::string name; };
    std::vector<TestPoint> test_points = {
        {0.0, "0°"}, {M_PI/2, "90°"}, {M_PI, "180°"}, {3*M_PI/2, "270°"}
    };

    for (const auto& test : test_points) {
        glm::vec3 pos = Physics::OrbitMechanics::calculateOrbitPosition(
            target_radius, target_inclination, test.nu);
        std::cout << "   ν = " << std::setw(4) << test.name << ": ("
                  << std::setprecision(3) << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    }

    // Determine correct true anomaly for (1.5, 0, 0)
    glm::vec3 expected_pos(1.5, 0, 0);
    double correct_nu = atan2(expected_pos.y, expected_pos.x);

    std::cout << "\n SOLUTION:" << std::endl;
    std::cout << "   For position (1.5, 0, 0): ν = " << std::setprecision(1)
              << correct_nu * 180/M_PI << "°" << std::endl;
}

void TransferModel::printTransferGenerationHeader(bool is_coplanar) {
    std::cout << "\n╔══════════════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                      TRANSFER TRAJECTORY GENERATION                  ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════════════╝" << std::endl;

    std::cout << " Transfer Type: " << (is_coplanar ? "COPLANAR" : "NON-COPLANAR")
              << " CIRCULAR TRANSFER" << std::endl;
}

void TransferModel::printPlaneVerification(const std::vector<glm::vec3>& points, const glm::vec3& expected_normal,
                           const std::string& orbit_name) {
    if (!CoordinateSystem::verifyOrbitalPlane(points, expected_normal)) {
        std::cout << " WARNING: " << orbit_name << " orbit not in expected plane!" << std::endl;
    } else {
        std::cout << orbit_name << " orbit plane verification passed" << std::endl;
    }
}

void TransferModel::printTransferCompletion(size_t transfer_points, size_t ellipse_points) {
    std::cout << "\n TRANSFER GENERATION SUMMARY:" << std::endl;
    std::cout << "   ├─ Transfer Points:     " << transfer_points << std::endl;
    std::cout << "   ├─ Complete Ellipse:    " << ellipse_points << std::endl;
    std::cout << "   └─ Status:               COMPLETED" << std::endl;
}

void TransferModel::printPositionVerification(const glm::vec3& calculated, const glm::vec3& expected,
                              const std::string& point_name) {
    float distance = glm::length(calculated - expected);

    std::cout << "" << point_name << " VERIFICATION:" << std::endl;
    std::cout << "   ├─ Expected:  (" << std::fixed << std::setprecision(3)
              << expected.x << ", " << expected.y << ", " << expected.z << ")" << std::endl;
    std::cout << "   ├─ Actual:    (" << calculated.x << ", " << calculated.y << ", "
              << calculated.z << ")" << std::endl;
    std::cout << "   └─ Distance:  " << std::setprecision(6) << distance;

    if (distance < 0.01f) {
        std::cout << " MATCH" << std::endl;
    } else {
        std::cout << "  MISMATCH" << std::endl;
    }
}
