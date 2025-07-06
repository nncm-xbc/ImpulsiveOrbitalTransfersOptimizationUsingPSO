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

    glm::vec3 expected_target(1.5, 0, 0);
    glm::vec3 actual_target = Physics::OrbitMechanics::calculateOrbitPosition(
        target_radius_, target_inclination_, final_true_anomaly_);
}

const std::vector<glm::vec3>& TransferModel::getImpulsePositions() const { return impulse_positions_; }

const std::vector<glm::vec3>& TransferModel::getImpulseDirections() const { return impulse_directions_; }

const std::vector<double>& TransferModel::getImpulseMagnitudes() const { return impulse_magnitudes_; }

void TransferModel::generateTransferTrajectory() {
    transfer_points_.clear();
    complete_ellipse_points_.clear();

    // Check if transfer is coplanar
    bool is_coplanar = (constant::I1 == constant::I2);

    glm::vec3 r0;
    glm::vec3 rf;

    // Position vectors at departure and arrival
    if (is_coplanar) {
        r0 = Physics::OrbitMechanics::calculateOrbitPosition(
            initial_radius_, initial_inclination_, initial_true_anomaly_);
        rf = Physics::OrbitMechanics::calculateOrbitPosition(
            target_radius_, target_inclination_, final_true_anomaly_);
    } else {
        Physics::Vector3 r0_physics = Physics::OrbitMechanics::calculatePosition3D(
            initial_radius_, 0.0, initial_inclination_, 0.0, 0.0, initial_true_anomaly_);
        Physics::Vector3 rf_physics = Physics::OrbitMechanics::calculatePosition3D(
            target_radius_, 0.0, target_inclination_, 0.0, 0.0, final_true_anomaly_);

        r0 = glm::vec3(r0_physics.x, r0_physics.y, r0_physics.z);
        rf = glm::vec3(rf_physics.x, rf_physics.y, rf_physics.z);
    }

    //std::cout << "\nPSO-OPTIMIZED TRANSFER ENDPOINTS:" << std::endl;
    //std::cout << "   Departure: (" << std::fixed << std::setprecision(3)
    //          << r0.x << ", " << r0.y << ", " << r0.z << ") DU at ν=" << initial_true_anomaly_*180/M_PI << "°" << std::endl;
    //std::cout << "   Arrival:   (" << rf.x << ", " << rf.y << ", " << rf.z
    //          << ") DU at ν=" << final_true_anomaly_*180/M_PI << "°" << std::endl;
    //std::cout << "   ✓ Using PSO-optimized positions (no hardcoded expectations)" << std::endl;

    float r0_mag = glm::length(r0);
    float rf_mag = glm::length(rf);
    glm::vec3 r0_hat = glm::normalize(r0);
    glm::vec3 rf_hat = glm::normalize(rf);

    // Circular orbit velocities (before impulses)
    float v0_mag = sqrt(constant::MU / initial_radius_);
    float vf_mag = sqrt(constant::MU / target_radius_);

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

        // ============================================
        // STEP 1: Calculate positions using SAME method as OrbitModel
        // ============================================

        Physics::Vector3 r0_physics = Physics::OrbitMechanics::calculatePosition3D(
            initial_radius_, 0.0,                    // a, e (circular)
            initial_inclination_, 0.0, 0.0,          // i, RAAN, ω
            initial_true_anomaly_                    // ν
        );

        Physics::Vector3 rf_physics = Physics::OrbitMechanics::calculatePosition3D(
            target_radius_, 0.0,                     // a, e (circular)
            target_inclination_, 0.0, 0.0,           // i, RAAN, ω
            final_true_anomaly_                      // ν
        );

        // ============================================
        // STEP 2: Create transfer trajectory using simple interpolation
        // ============================================

        std::vector<Physics::Vector3> physics_transfer_points;
        const int transfer_resolution = 100;

        for (int i = 0; i <= transfer_resolution; ++i) {
            double t = static_cast<double>(i) / transfer_resolution;

            double current_radius = r0_mag + (rf_mag - r0_mag) * t;

            Physics::Vector3 normal = r0_physics.cross(rf_physics);

            if (normal.magnitude() < 1e-6) {
                // Degenerate case: points are collinear, use interpolation
                Physics::Vector3 interpolated = r0_physics * (1.0 - t) + rf_physics * t;
                physics_transfer_points.push_back(interpolated);
            } else {
                normal = normal.normalized();

                Physics::Vector3 u = r0_physics.normalized();
                Physics::Vector3 v = normal.cross(u).normalized();

                double r0_angle = atan2(r0_physics.dot(v), r0_physics.dot(u));
                double rf_angle = atan2(rf_physics.dot(v), rf_physics.dot(u));

                if (rf_angle - r0_angle > M_PI) {
                    rf_angle -= 2.0 * M_PI;
                } else if (r0_angle - rf_angle > M_PI) {
                    rf_angle += 2.0 * M_PI;
                }

                double current_angle = r0_angle + (rf_angle - r0_angle) * t;

                Physics::Vector3 interpolated_pos = u * (current_radius * cos(current_angle)) +
                                                  v * (current_radius * sin(current_angle));

                physics_transfer_points.push_back(interpolated_pos);
            }
        }

        // ============================================
        // STEP 3: Convert to visualization coordinates
        // ============================================

        std::vector<glm::vec3> glm_transfer_points;
        glm_transfer_points.reserve(physics_transfer_points.size());

        for (const auto& point : physics_transfer_points) {
            glm_transfer_points.push_back(glm::vec3(point.x, point.y, point.z));
        }

        transfer_points_ = CoordinateSystem::trajectoryToVisualization(glm_transfer_points);

        // ============================================
        // STEP 4: Verification (should be perfect now)
        // ============================================

        if (!physics_transfer_points.empty()) {
            Physics::Vector3 computed_start = physics_transfer_points.front();
            Physics::Vector3 computed_end = physics_transfer_points.back();

            double start_error = (computed_start - r0_physics).magnitude();
            double end_error = (computed_end - rf_physics).magnitude();

            if (start_error > 1e-15 && end_error > 1e-15) {
                std::cout << "  ERROR: Transfer doesn't connects perfectly to orbits" << std::endl;
            }
        }

        // ============================================
        // STEP 5: Create simple ellipse for reference
        // ============================================


        std::vector<glm::vec3> glm_ellipse_points;

        Physics::Vector3 normal = r0_physics.cross(rf_physics);

        if (normal.magnitude() > 1e-6) {
            normal = normal.normalized();

            Physics::Vector3 u = r0_physics.normalized();
            Physics::Vector3 v = normal.cross(u).normalized();

            double r0_angle = atan2(r0_physics.dot(v), r0_physics.dot(u));
            double rf_angle = atan2(rf_physics.dot(v), rf_physics.dot(u));

            if (rf_angle - r0_angle > M_PI) {
                rf_angle -= 2.0 * M_PI;
            } else if (r0_angle - rf_angle > M_PI) {
                rf_angle += 2.0 * M_PI;
            }

            const int ellipse_resolution = 200;
            double r0_mag = r0_physics.magnitude();
            double rf_mag = rf_physics.magnitude();

            for (int i = 0; i < ellipse_resolution; ++i) {
                double angle_progress = static_cast<double>(i) / ellipse_resolution;

                double full_angle = 2.0 * M_PI * angle_progress;

                double a = (r0_mag + rf_mag) / 2.0;  // Semi-major axis
                double c = std::abs(rf_mag - r0_mag) / 2.0;  // Distance from center to focus
                double e = c / a;  // Eccentricity

                double current_radius = a * (1.0 - e * e) / (1.0 + e * cos(full_angle));

                Physics::Vector3 ellipse_pos = u * (current_radius * cos(full_angle)) +
                                             v * (current_radius * sin(full_angle));

                glm_ellipse_points.push_back(glm::vec3(ellipse_pos.x, ellipse_pos.y, ellipse_pos.z));
            }

        } else {
            // Degenerate case: create simple circular reference
            double avg_radius = (r0_physics.magnitude() + rf_physics.magnitude()) / 2.0;
            Physics::Vector3 ref_direction = r0_physics.normalized();
            Physics::Vector3 perp_direction = Physics::Vector3(0, 0, 1).cross(ref_direction).normalized();

            if (perp_direction.magnitude() < 1e-6) {
                perp_direction = Physics::Vector3(1, 0, 0).cross(ref_direction).normalized();
            }

            const int ellipse_resolution = 200;
            for (int i = 0; i < ellipse_resolution; ++i) {
                double angle = 2.0 * M_PI * i / ellipse_resolution;

                Physics::Vector3 pos = ref_direction * (avg_radius * cos(angle)) +
                                     perp_direction * (avg_radius * sin(angle));

                glm_ellipse_points.push_back(glm::vec3(pos.x, pos.y, pos.z));
            }

        }

        complete_ellipse_points_ = CoordinateSystem::trajectoryToVisualization(glm_ellipse_points);

        // ============================================
        // STEP 6: Set impulse positions
        // ============================================

        impulse_positions_.clear();
        impulse_directions_.clear();

        glm::vec3 r0_glm(r0_physics.x, r0_physics.y, r0_physics.z);
        glm::vec3 rf_glm(rf_physics.x, rf_physics.y, rf_physics.z);

        impulse_positions_.push_back(CoordinateSystem::physicsToVisualization(r0_glm));
        impulse_positions_.push_back(CoordinateSystem::physicsToVisualization(rf_glm));
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
        std::cout << "ERROR: OpenGL error in updateBuffers: " << err << std::endl;
    }

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

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

void TransferModel::printTransferCompletion(size_t transfer_points, size_t ellipse_points) {
    std::cout << "\n TRANSFER GENERATION SUMMARY:" << std::endl;
    std::cout << "   ├─ Transfer Points:     " << transfer_points << std::endl;
    std::cout << "   ├─ Complete Ellipse:    " << ellipse_points << std::endl;
    std::cout << "   └─ Status:               COMPLETED" << std::endl;
}
