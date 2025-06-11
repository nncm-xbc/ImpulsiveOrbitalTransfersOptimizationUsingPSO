#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <cmath>
#include <iostream>

#include "Constants.hpp"
#include "Shader.hpp"
#include "OrbitModel.hpp"
#include "TransferModel.hpp"

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
}

// Placeholder for now (same as two-impulse transfer)
void TransferModel::setThreeImpulseTransfer(double initialRadius, double initialInclination,
                    double targetRadius, double targetInclination,
                    double initialTrueAnomaly, double finalTrueAnomaly,
                    double final_true_anomaly_) {
    initial_radius_ = initialRadius;
    target_radius_ = targetRadius;
    initial_inclination_ = initialInclination;
    target_inclination_ = targetInclination;
    initial_true_anomaly_ = initialTrueAnomaly;
    final_true_anomaly_ = finalTrueAnomaly;

    //generateTransferTrajectory();
    updateBuffers();
}

const std::vector<glm::vec3>& TransferModel::getImpulsePositions() const { return impulse_positions_; }

const std::vector<glm::vec3>& TransferModel::getImpulseDirections() const { return impulse_directions_; }

const std::vector<double>& TransferModel::getImpulseMagnitudes() const { return impulse_magnitudes_; }

glm::vec3 TransferModel::calculateOrbitPosition(float radius, float inclination, float true_anomaly) {
    // std::cout << "calculateOrbitPosition: radius=" << radius << ", inc=" << inclination
    //           << ", anomaly=" << true_anomaly << std::endl;

    // Position in orbital plane
    float x = radius * cos(true_anomaly);
    float y = radius * sin(true_anomaly);
    float z = 0.0f;

    // Rotate by inclination (around x-axis)
    float y_rotated = y * cos(inclination);
    float z_rotated = y * sin(inclination);

    glm::vec3 result(x, y_rotated, z_rotated);

    //std::cout << "Position: (" << result.x << ", " << result.y << ", " << result.z << ")" << std::endl;

    return result;
}

glm::vec3 TransferModel::calculateOrbitVelocity(float radius, float inclination, float true_anomaly) {
    // std::cout << "calculateOrbitVelocity: radius=" << radius << ", inc=" << inclination
    //           << ", anomaly=" << true_anomaly << std::endl;

    float speed = sqrt(constant::MU / radius);

    // Velocity in orbital plane
    float vx = -speed * sin(true_anomaly);
    float vy = speed * cos(true_anomaly);
    float vz = 0.0f;

    // Rotate by inclination (around x-axis)
    float vy_rotated = vy * cos(inclination);
    float vz_rotated = vy * sin(inclination);

    glm::vec3 result(vx, vy_rotated, vz_rotated);

    std::cout << "Velocity: (" << result.x << ", " << result.y << ", " << result.z << ")" << std::endl;

    return result;
}

void TransferModel::generateTransferTrajectory() {

    transfer_points_.clear();
    complete_ellipse_points_.clear();

    // Position vectors at departure/arrival
    glm::vec3 r0 = calculateOrbitPosition(initial_radius_, initial_inclination_, initial_true_anomaly_);
    glm::vec3 rf = calculateOrbitPosition(target_radius_, 0.0, final_true_anomaly_);

    // Velocity vectors before impulses on initial/final orbits
    float v0_mag = sqrt(constant::MU / initial_radius_);
    float vf_mag = sqrt(constant::MU / target_radius_);

    // Unit vectors aligned with r(t0) and r(tf)
    glm::vec3 r0_hat = glm::normalize(r0);
    glm::vec3 rf_hat = glm::normalize(rf);
    std::cout << "Initial position: " << r0.x << ", " << r0.y << ", " << r0.z << std::endl;
    std::cout << "Final position: " << rf.x << ", " << rf.y << ", " << rf.z << std::endl;

    // Unit vectors aligned with angular momentum
    glm::vec3 h0_hat(0.0f, 0.0f, 1.0f); // For initial orbit with inclination
    if (initial_inclination_ != 0.0f) {
        h0_hat = glm::vec3(
            -sin(0.0f),
            cos(0.0f),
            0.0f
        );

        // Rotation matrix for inclination
        float cos_incl = cos(float(initial_inclination_));
        float sin_incl = sin(float(initial_inclination_));
        glm::mat3 rotation_matrix(
            1.0f, 0.0f, 0.0f,
            0.0f, cos_incl, -sin_incl,
            0.0f, sin_incl, cos_incl
        );
        h0_hat = rotation_matrix * h0_hat;
    }
    glm::vec3 hf_hat(0.0f, 0.0f, 1.0f); // For final orbit with inclination 0

    // Local coordinate system at departure point
    glm::vec3 radial_dir = r0_hat;
    glm::vec3 normal_dir = glm::normalize(h0_hat);
    glm::vec3 tangential_dir = glm::normalize(glm::cross(normal_dir, radial_dir));
    std::cout << "Radial direction: " << radial_dir.x << ", " << radial_dir.y << ", " << radial_dir.z << std::endl;
    std::cout << "Normal direction: " << normal_dir.x << ", " << normal_dir.y << ", " << normal_dir.z << std::endl;
    std::cout << "Tangential direction: " << tangential_dir.x << ", " << tangential_dir.y << ", " << tangential_dir.z << std::endl;

    // Initial/final velocity vec
    glm::vec3 v0_minus = v0_mag * tangential_dir;
    glm::vec3 vf_plus = vf_mag * glm::normalize(glm::cross(hf_hat, rf_hat));
    std::cout << "Initial velocity1: " << v0_minus.x << ", " << v0_minus.y << ", " << v0_minus.z << std::endl;
    std::cout << "Final velocity: " << vf_plus.x << ", " << vf_plus.y << ", " << vf_plus.z << std::endl;

    float dv1_mag = impulse_magnitudes_[0];
    float dv2_mag = impulse_magnitudes_[1];
    std::cout << "Impulse magnitudes: " << dv1_mag << ", " << dv2_mag << std::endl;

    // Plane change components
    float plane_change1 = 0.0f;
    float plane_change2 = 0.0f;

    if (initial_inclination_ > 1e-6f || std::abs(initial_inclination_ - target_inclination_) > 1e-6f) {
        // Non-coplanar case, use plane change values
        plane_change1 = plane_change_[0];
        plane_change2 = plane_change_[1];
    }
    std::cout << "Plane change components: " << plane_change1 << ", " << plane_change2 << std::endl;

    // Velocity after first impulse
    float tangential_component = dv1_mag / sqrt(1.0f + plane_change1 * plane_change1);
    float normal_component = tangential_component * plane_change1;

    glm::vec3 impulse_vec = tangential_component * tangential_dir + normal_component * normal_dir;
    glm::vec3 v0_plus = v0_minus + dv1_mag * impulse_vec;

    // Debug
    std::cout << "Impulse direction: tangential=" << tangential_component << ", normal=" << normal_component << std::endl;
    std::cout << "Initial velocity2: " << glm::length(v0_minus) << std::endl;
    std::cout << "Velocity after impulse: " << glm::length(v0_plus) << std::endl;

    // Transfer orbit parameters
    glm::vec3 h_transfer = glm::cross(r0, v0_plus);
    float h_transfer_mag = glm::length(h_transfer);
    std::cout << "Angular momentum magnitude: " << h_transfer_mag << std::endl;
    std::cout << "Angular momentum vector: " << h_transfer.x << ", " << h_transfer.y << ", " << h_transfer.z << std::endl;

    // Normal to transfer orbit plane
    glm::vec3 normal;
    if (initial_inclination_ < 1e-6f && target_inclination_ < 1e-6f) {
        normal = glm::vec3(0.0f, 0.0f, 1.0f);
    } else {
        normal = h_transfer / h_transfer_mag;
    }
    std::cout << "Transfer orbit normal: " << normal.x << ", " << normal.y << ", " << normal.z << std::endl;

    if (initial_inclination_ < 1e-6f && initial_eccentricity_ < 1e-6f &&
        target_inclination_ < 1e-6f && target_eccentricity_ < 1e-6f) {
        transfer_semi_major_ = (initial_radius_ + target_radius_) / 2.0f;
        transfer_eccentricity_ = fabs(initial_radius_ - target_radius_) / (initial_radius_ + target_radius_);
    } else {
        float r0_mag = glm::length(r0);
        float v0_plus_mag = glm::length(v0_plus);
        transfer_semi_major_ = (constant::MU * r0_mag) / (2.0f * constant::MU - r0_mag * v0_plus_mag * v0_plus_mag);
        transfer_eccentricity_ = sqrt(1.0f - (h_transfer_mag * h_transfer_mag) / (constant::MU * transfer_semi_major_));
    }
    std::cout << "Transfer semi-major axis: " << transfer_semi_major_ << std::endl;
    std::cout << "Transfer eccentricity: " << transfer_eccentricity_ << std::endl;

    // Semi-latus rectum
    float p = transfer_semi_major_ * (1.0f - transfer_eccentricity_ * transfer_eccentricity_);
    std::cout << "Transfer semi-latus rectum: " << p << std::endl;

    glm::vec3 periapsis_dir;
    if (initial_radius_ < target_radius_) {
        periapsis_dir = r0_hat;
    } else {
        periapsis_dir = -r0_hat;
    }

    glm::vec3 perpendicular;
    if (initial_inclination_ < 1e-6f && initial_eccentricity_ < 1e-6f &&
        target_inclination_ < 1e-6f && target_eccentricity_ < 1e-6f)
    {
        perpendicular = glm::vec3(-periapsis_dir.y, periapsis_dir.x, 0.0f);
    } else {
        // non-circular/non-coplanar transfers
        glm::vec3 h_transfer_normalized = glm::normalize(h_transfer);
        glm::vec3 cross_product = glm::cross(v0_plus, h_transfer);
        float cross_length = glm::length(cross_product);
        glm::vec3 e_vec;

        if (cross_length > 1e-6f) {
            e_vec = (cross_product / float(constant::MU)) - r0_hat;
        } else {
            // Edge case
            e_vec = -r0_hat; // Default to pointing away from position
        }
        std::cout << "Eccentricity vector: " << e_vec.x << ", " << e_vec.y << ", " << e_vec.z << std::endl;

        glm::vec3 departure_point = r0;
        glm::vec3 arrival_point = rf;

        float distance_between = glm::length(arrival_point - departure_point);
        float sum_of_radii = glm::length(departure_point) + glm::length(arrival_point);

        // Semi-major axis
        float transfer_semi_major = (glm::length(departure_point) + glm::length(arrival_point)) / 2.0f;

        // Eccentricity for an orbit that passes through both points
        float transfer_eccentricity = distance_between / sum_of_radii;

        float e_mag = glm::length(e_vec);
        if (e_mag > 1e-6f) {
            periapsis_dir = glm::normalize(e_vec);
        }

        perpendicular = glm::normalize(glm::cross(h_transfer_normalized, periapsis_dir));
    }
    perpendicular = glm::normalize(perpendicular);

    float dot = glm::dot(perpendicular, periapsis_dir);
    if (std::abs(dot) > 1e-6f) {
        std::cout << "Warning: Perpendicular vector not orthogonal to periapsis (dot=" << dot << ")" << std::endl;
        // Force orthogonality
        perpendicular = perpendicular - dot * periapsis_dir;
        perpendicular = glm::normalize(perpendicular);
    }
    std::cout << "Perpendicular direction: " << perpendicular.x << ", " << perpendicular.y << ", " << perpendicular.z << std::endl;

    // Generate points along the transfer ellipse
    int resolution = 500;
    glm::vec3 pos;

    for (int i = 0; i <= resolution; ++i) {

        float true_anomaly = 2.0f * M_PI * i / resolution;
        float r = p / (1.0f + transfer_eccentricity_ * cos(true_anomaly));

        float r_cos = r * cos(true_anomaly);
        float r_sin = r * sin(true_anomaly);

        if (initial_inclination_ < 1e-6f && target_inclination_ < 1e-6f) {
            // coplanar
            pos = glm::vec3(
                    periapsis_dir.x * r_cos + perpendicular.x * r_sin,
                    0.0f,
                    periapsis_dir.y * r_cos + perpendicular.y * r_sin); // force z = 0
        } else {
            // non-coplanar
            glm::vec3 computational_pos = periapsis_dir * r_cos + perpendicular * r_sin;

            // y and z are swapped for OpenGL rendering
            pos = glm::vec3(
                computational_pos.x,
                computational_pos.z,
                computational_pos.y
            );        }

        // Scale for visualization
        pos *= 1000.0f;

        complete_ellipse_points_.push_back(pos);
    }

    // Generate points for the transfer trajectory
    if (initial_inclination_ < 1e-6f && initial_eccentricity_ < 1e-6f &&
        target_inclination_ < 1e-6f && target_eccentricity_ < 1e-6f) {

        for (int i = 0; i <= resolution/2; ++i) {
            float t = static_cast<float>(i) / (resolution/2);
            float true_anomaly = t * M_PI;  // 0 to 180 degrees

            float r = p / (1.0f + transfer_eccentricity_ * cos(true_anomaly));

            float r_cos = r * cos(true_anomaly);
            float r_sin = r * sin(true_anomaly);

            pos = glm::vec3(
                    periapsis_dir.x * r_cos + perpendicular.x * r_sin,
                    0.0f,
                    periapsis_dir.y * r_cos + perpendicular.y * r_sin); // force z = 0

            // Scale for visualization
            pos *= 1000.0f;

            transfer_points_.push_back(pos);
        }
    } else {
        float anomaly_diff = final_true_anomaly_ - initial_true_anomaly_;
        if (anomaly_diff > M_PI) anomaly_diff -= 2.0f * M_PI;
        if (anomaly_diff < -M_PI) anomaly_diff += 2.0f * M_PI;

        for (int i = 0; i <= resolution; ++i) {
            float t = static_cast<float>(i) / resolution;
            float true_anomaly = initial_true_anomaly_ + t * anomaly_diff;

            float r = p / (1.0f + transfer_eccentricity_ * cos(true_anomaly));

            float r_cos = r * cos(true_anomaly);
            float r_sin = r * sin(true_anomaly);

            glm::vec3 computational_pos = periapsis_dir * r_cos + perpendicular * r_sin;

            // y and z are swapped for OpenGL rendering
            pos = glm::vec3(
                computational_pos.x,
                computational_pos.z,
                computational_pos.y
            );
            // Scale for visualization
            pos *= 1000.0f;

            transfer_points_.push_back(pos);
        }
    }

    // Debug
    std::cout << "Transfer orbit: a = " << transfer_semi_major_
              << ", e = " << transfer_eccentricity_ << std::endl;
    std::cout << "Normal to transfer plane: " << normal.x << ", "
              << normal.y << ", " << normal.z << std::endl;
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

    // Render the ellipse
    shader.use();
    shader.setMat4("viewProjection", view_projection);
    shader.setVec3("color", color);

    glm::vec3 center = calculateRenderedCenter();
    //std::cout << "Rendered transfer orbit center: " << center.x << ", " << center.y << ", " << center.z << std::endl;

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

    std::cout << "Updating buffers with " << transfer_points_.size() << " points" << std::endl;
    std::cout << "First point: " << transfer_points_[0].x << ", " << transfer_points_[0].y << ", " << transfer_points_[0].z << std::endl;
    std::cout << "Last point: " << transfer_points_.back().x << ", " << transfer_points_.back().y << ", " << transfer_points_.back().z << std::endl;

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
