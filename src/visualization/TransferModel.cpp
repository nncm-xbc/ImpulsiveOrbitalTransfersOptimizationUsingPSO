#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <cmath>
#include <iostream>

#include "core/Constants.hpp"
#include "core/OrbitMechanics.hpp"
#include "core/CoordinateSystem.hpp"
#include "visualization/Shader.hpp"
#include "visualization/OrbitModel.hpp"
#include "visualization/TransferModel.hpp"

TransferModel::TransferModel() : vao_(0), vbo_(0), initialized_(false) {corrected_vao_ = 0;
corrected_vbo_ = 0;
has_corrected_transfer_ = false;}

TransferModel::~TransferModel() {
    if (initialized_) {
        glDeleteVertexArrays(1, &vao_);
        glDeleteBuffers(1, &vbo_);
    }
    if (corrected_vao_ != 0) {
        glDeleteVertexArrays(1, &corrected_vao_);
        glDeleteBuffers(1, &corrected_vbo_);
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
    correctTransferToActualIntersection();

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

    CoordinateSystem::debugTransformation("Initial position", r0);
    CoordinateSystem::debugTransformation("Target position", rf);

    float r0_mag = glm::length(r0);
    float rf_mag = glm::length(rf);
    glm::vec3 r0_hat = glm::normalize(r0);
    glm::vec3 rf_hat = glm::normalize(rf);

    // Circular orbit velocities (before impulses)
    float v0_mag = sqrt(constant::MU / initial_radius_);
    float vf_mag = sqrt(constant::MU / target_radius_);

    // Check if transfer is coplanar
    bool is_coplanar = (initial_inclination_ < 1e-6f && target_inclination_ < 1e-6f);

    // Visualization parameters
    const int resolution = 500;
    const float visualScale = 1000.0f;

    // ============================================
    // COPLANAR CASE
    // ============================================
    if (is_coplanar) {
        std::cout << "COPLANAR CIRCULAR TRANSFER" << std::endl;

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


        // Debug output for coplanar case
        std::cout << "Transfer orbit: a = " << transfer_semi_major_
                  << ", e = " << transfer_eccentricity_ << std::endl;
    }

    // ============================================
    // NON-COPLANAR CASE
    // ============================================
    else {
        std::cout << "NON-COPLANAR CIRCULAR TRANSFER" << std::endl;

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
            std::cerr << "WARNING: Transfer orbit not in expected plane!" << std::endl;
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
                CoordinateSystem::debugTransformation("Transfer point " + std::to_string(i), pos_physics);
            }
        }

        transfer_points_ = CoordinateSystem::trajectoryToVisualization(physics_transfer_points);

        // Debug
        std::cout << "Transfer orbit: a = " << transfer_semi_major_
                  << ", e = " << transfer_eccentricity_
                  << ", i = " << transfer_inclination * 180.0f / M_PI << "°"
                  << ", Ω = " << transfer_raan * 180.0f / M_PI << "°"
                  << ", ω = " << transfer_arg_periapsis * 180.0f / M_PI << "°" << std::endl;

        std::cout << "Target projected onto transfer plane:" << std::endl;
        std::cout << "  Out-of-plane component: " << glm::dot(rf, h_transfer_hat) << std::endl;
        std::cout << "  Projected radius: " << glm::length(rf_projected) << std::endl;

        // Verify intersection
        float r_at_nuf = p / (1.0f + transfer_eccentricity_ * cos(nuf));
        std::cout << "  Radius at nuf: " << r_at_nuf << std::endl;
        std::cout << "  Match? " << (std::abs(r_at_nuf - glm::length(rf_projected)) < 0.01f) << std::endl;

        // Debug output
        std::cout << "\nTrue anomaly calculation (fixed):" << std::endl;
        std::cout << "  r0 magnitude in plane: " << r0_mag_in_plane << std::endl;
        std::cout << "  rf projected magnitude: " << rf_mag_projected << std::endl;
        std::cout << "  cos(nu0): " << cos_nu0 << std::endl;
        std::cout << "  cos(nuf): " << cos_nuf << std::endl;
        std::cout << "  nu0: " << nu0 * 180/M_PI << " degrees" << std::endl;
        std::cout << "  nuf option 1: " << nuf_option1 * 180/M_PI << " degrees" << std::endl;
        std::cout << "  nuf option 2: " << nuf_option2 * 180/M_PI << " degrees" << std::endl;
        std::cout << "  nuf selected: " << nuf * 180/M_PI << " degrees" << std::endl;
        std::cout << "  Transfer angle: " << (nuf - nu0) * 180/M_PI << " degrees" << std::endl;

        // Verify
        float r_check_nu0 = p / (1.0f + transfer_eccentricity_ * cos(nu0));
        float r_check_nuf = p / (1.0f + transfer_eccentricity_ * cos(nuf));
        std::cout << "  Radius at nu0: " << r_check_nu0 << " (expected: " << r0_mag_in_plane << ")" << std::endl;
        std::cout << "  Radius at nuf: " << r_check_nuf << " (expected: " << rf_mag_projected << ")" << std::endl;
    }

    // ============================================
    // DEBUG
    // ============================================
    std::cout << "\nTransfer generation complete:" << std::endl;
    std::cout << "  Transfer points: " << transfer_points_.size() << std::endl;
    std::cout << "  Complete ellipse points: " << complete_ellipse_points_.size() << std::endl;
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
    std::cout << "\n=== TARGET POSITION DEBUG ===" << std::endl;

    // From your PSO results
    double target_radius = 1.5;
    double target_inclination = 0.0;
    double final_true_anomaly = M_PI;  // This is what PSO gives (should be 180°)

    std::cout << "Target orbit parameters:" << std::endl;
    std::cout << "  Radius: " << target_radius << std::endl;
    std::cout << "  Inclination: " << target_inclination * 180/M_PI << "°" << std::endl;
    std::cout << "  True anomaly: " << final_true_anomaly * 180/M_PI << "°" << std::endl;

    // Calculate position using OrbitMechanics
    glm::vec3 target_pos = Physics::OrbitMechanics::calculateOrbitPosition(
        target_radius, target_inclination, final_true_anomaly);

    std::cout << "Calculated target position: " << target_pos.x << ", "
              << target_pos.y << ", " << target_pos.z << std::endl;
    std::cout << "Expected: (1.5, 0, 0) for circular equatorial orbit at 180°" << std::endl;

    // Check if the issue is in the true anomaly
    // For a circular equatorial orbit:
    // - At ν = 0°: position should be (r, 0, 0)
    // - At ν = 90°: position should be (0, r, 0)
    // - At ν = 180°: position should be (-r, 0, 0)
    // - At ν = 270°: position should be (0, -r, 0)

    std::cout << "\nChecking various true anomalies for target orbit:" << std::endl;
    for (double nu : {0.0, M_PI/2, M_PI, 3*M_PI/2}) {
        glm::vec3 pos = Physics::OrbitMechanics::calculateOrbitPosition(
            target_radius, target_inclination, nu);
        std::cout << "  ν = " << nu * 180/M_PI << "°: ("
                  << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    }

    // Now check what true anomaly would give (1.5, 0, 0)
    // For circular equatorial orbit, this should be ν = 0°
    glm::vec3 expected_pos(1.5, 0, 0);
    double nu_for_expected = atan2(expected_pos.y, expected_pos.x);
    std::cout << "\nTrue anomaly for position (1.5, 0, 0): "
              << nu_for_expected * 180/M_PI << "°" << std::endl;

    // Check if OrbitMechanics gives this position at ν = 0
    glm::vec3 check_pos = Physics::OrbitMechanics::calculateOrbitPosition(
        target_radius, target_inclination, 0.0);
    std::cout << "Position at ν = 0°: (" << check_pos.x << ", "
              << check_pos.y << ", " << check_pos.z << ")" << std::endl;
}


// ============================================
// CORRECTIONS
// ============================================

std::vector<IntersectionPoint> TransferModel::findOrbitPlaneIntersections(
    const glm::vec3& target_plane_normal,
    double target_radius,
    double tolerance) const {

    std::vector<IntersectionPoint> intersections;

    if (complete_ellipse_points_.empty()) {
        std::cout << "No ellipse points to check for intersections" << std::endl;
        return intersections;
    }

    std::cout << "\n=== FINDING ORBIT PLANE INTERSECTIONS ===" << std::endl;
    std::cout << "Target plane normal: (" << target_plane_normal.x << ", "
              << target_plane_normal.y << ", " << target_plane_normal.z << ")" << std::endl;
    std::cout << "Target radius: " << target_radius << std::endl;

    // Convert visualization points back to physics coordinates for analysis
    std::vector<glm::vec3> physics_points;
    for (const auto& vis_point : complete_ellipse_points_) {
        physics_points.push_back(CoordinateSystem::visualizationToPhysics(vis_point));
    }

    // Check each segment of the transfer ellipse for plane crossings
    for (size_t i = 0; i < physics_points.size(); ++i) {
        size_t next_i = (i + 1) % physics_points.size();

        glm::vec3 p1 = physics_points[i];
        glm::vec3 p2 = physics_points[next_i];

        // Calculate distance from plane for both points
        // Distance = dot(point, normal) for plane through origin
        double d1 = glm::dot(p1, target_plane_normal);
        double d2 = glm::dot(p2, target_plane_normal);

        // Check for sign change (plane crossing)
        if (d1 * d2 <= 0 && std::abs(d1 - d2) > tolerance) {
            // Linear interpolation to find exact crossing point

            float t = static_cast<float>(std::abs(d1) / (std::abs(d1) + std::abs(d2)));
            glm::vec3 intersection_pos = p1 + t * (p2 - p1);

            // Calculate corresponding true anomaly
            double true_anomaly = 2.0 * M_PI * (i + t) / physics_points.size();

            // Calculate radius at intersection
            double radius = glm::length(intersection_pos);

            // Calculate distance to target radius
            double radius_error = std::abs(radius - target_radius);

            IntersectionPoint intersection;
            intersection.position = intersection_pos;
            intersection.true_anomaly = true_anomaly;
            intersection.radius = radius;
            intersection.distance_to_target = radius_error;
            intersection.is_valid = (radius_error < 0.3); // tolerance

            intersections.push_back(intersection);

            std::cout << "Found intersection " << intersections.size() << ":" << std::endl;
            std::cout << "  Position: (" << intersection_pos.x << ", "
                      << intersection_pos.y << ", " << intersection_pos.z << ")" << std::endl;
            std::cout << "  True anomaly: " << true_anomaly * 180/M_PI << "°" << std::endl;
            std::cout << "  Radius: " << radius << " (target: " << target_radius << ")" << std::endl;
            std::cout << "  Radius error: " << radius_error << std::endl;
            std::cout << "  Valid: " << (intersection.is_valid ? "YES" : "NO") << std::endl;
        }
    }

    // Sort intersections by distance to target radius
    std::sort(intersections.begin(), intersections.end(),
              [](const IntersectionPoint& a, const IntersectionPoint& b) {
                  return a.distance_to_target < b.distance_to_target;
              });

    std::cout << "Found " << intersections.size() << " plane intersections" << std::endl;
    if (!intersections.empty()) {
        std::cout << "Best intersection has radius error: "
                  << intersections[0].distance_to_target << std::endl;
    }

    return intersections;
}

void TransferModel::correctTransferToActualIntersection() {
    // For non-coplanar transfers, find where transfer orbit actually intersects target plane
    if (std::abs(target_inclination_) < 1e-6) {
        // Target is equatorial (z = 0 plane)
        glm::vec3 target_normal(0.0f, 0.0f, 1.0f);

        auto intersections = findOrbitPlaneIntersections(target_normal, target_radius_);

        if (!intersections.empty() && intersections[0].is_valid) {
            std::cout << "\n=== CORRECTING TRANSFER TO ACTUAL INTERSECTION ===" << std::endl;

            // Update the final true anomaly to the actual intersection
            double corrected_final_anomaly = intersections[0].true_anomaly;

            std::cout << "Original final true anomaly: " << final_true_anomaly_ * 180/M_PI << "°" << std::endl;
            std::cout << "Corrected final true anomaly: " << corrected_final_anomaly * 180/M_PI << "°" << std::endl;

            // Update internal state
            final_true_anomaly_ = corrected_final_anomaly;

            // Regenerate transfer trajectory to corrected endpoint
            generateCorrectedTransferTrajectory();
        } else {
            std::cout << "WARNING: No valid intersection found! Transfer may not reach target orbit." << std::endl;
        }
    } else {
        // For inclined target orbits, implement similar logic with target orbit normal
        std::cout << "Intersection correction for inclined target orbits not yet implemented" << std::endl;
    }
}

void TransferModel::generateCorrectedTransferTrajectory() {
    // Generate complete corrected transfer ellipse points
    std::vector<glm::vec3> physics_ellipse_points;

    std::cout << "\nGenerating corrected complete transfer ellipse" << std::endl;

    // Use the existing transfer orbit parameters to generate complete ellipse
    const int resolution = 500;

    // Generate complete ellipse (0 to 2π)
    for (int i = 0; i < resolution; ++i) {
        float true_anomaly = 2.0f * M_PI * i / resolution;

        // Calculate position on transfer ellipse
        float p = transfer_semi_major_ * (1.0f - transfer_eccentricity_ * transfer_eccentricity_);
        float r = p / (1.0f + transfer_eccentricity_ * cos(true_anomaly));

        // Position in transfer orbital plane
        glm::vec3 pos_orbital(r * cos(true_anomaly), r * sin(true_anomaly), 0.0f);

        // Transform to inertial frame (this may need adjustment based on transfer orbit orientation)
        // For now, using simplified transformation - you may need to apply proper rotation matrix
        physics_ellipse_points.push_back(pos_orbital);
    }

    // Convert to visualization coordinates and store in corrected_transfer_points_
    corrected_transfer_points_ = CoordinateSystem::trajectoryToVisualization(physics_ellipse_points);
    has_corrected_transfer_ = true;

    std::cout << "Generated complete corrected ellipse with " << corrected_transfer_points_.size() << " points" << std::endl;

    // Update OpenGL buffers for corrected transfer
    updateCorrectedBuffers();
}

void TransferModel::updateCorrectedBuffers() {
    if (!initialized_ || corrected_transfer_points_.empty()) {
        std::cout << "Error: Cannot update corrected buffers - not initialized or no points" << std::endl;
        return;
    }

    // Create buffers if they don't exist
    if (corrected_vao_ == 0) {
        glGenVertexArrays(1, &corrected_vao_);
        glGenBuffers(1, &corrected_vbo_);
    }

    glBindVertexArray(corrected_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, corrected_vbo_);
    glBufferData(GL_ARRAY_BUFFER, corrected_transfer_points_.size() * sizeof(glm::vec3),
                corrected_transfer_points_.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    std::cout << "Corrected transfer buffers updated with " << corrected_transfer_points_.size() << " points" << std::endl;
}

void TransferModel::renderCorrectedTransfer(const Shader& shader, const glm::mat4& view_projection,
                                          const glm::vec3& color) {
    if (!has_corrected_transfer_ || corrected_transfer_points_.empty() || corrected_vao_ == 0) {
        return; // No corrected transfer to render
    }

    shader.use();
    shader.setMat4("viewProjection", view_projection);
    shader.setVec3("color", color);

    glBindVertexArray(corrected_vao_);

    // Render the complete corrected ellipse (not animated)
    glLineWidth(3.0f); // Make corrected transfer more visible
    glDrawArrays(GL_LINE_LOOP, 0, static_cast<GLsizei>(corrected_transfer_points_.size()));
    glLineWidth(1.0f);

    glBindVertexArray(0);
}
