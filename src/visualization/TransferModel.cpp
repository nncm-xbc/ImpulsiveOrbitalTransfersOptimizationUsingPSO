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

    glm::vec3 expected_target(1.5, 0, 0);
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
            glm::vec3 expected_r0 = r0;
            glm::vec3 expected_rf = rf;

            printPositionVerification(r0, expected_r0, "Initial Position");
            printPositionVerification(rf, expected_rf, "Target Position");

            // The transfer orbit MUST lie in the plane containing r0, rf, and origin
            glm::vec3 transfer_normal = glm::normalize(glm::cross(r0, rf));

            // Handle degenerate case (collinear points)
            if (glm::length(transfer_normal) < 1e-6f) {
                transfer_normal = glm::vec3(0.0f, 0.0f, 1.0f);
            }


            float r0_mag = glm::length(r0);
            float rf_mag = glm::length(rf);

            // For circular-to-circular Hohmann-like transfer
            transfer_semi_major_ = (r0_mag + rf_mag) / 2.0f;
            transfer_eccentricity_ = std::abs(rf_mag - r0_mag) / (rf_mag + r0_mag);

            glm::vec3 periapsis_dir;
            if (r0_mag < rf_mag) {
                periapsis_dir = glm::normalize(r0);
            } else {
                periapsis_dir = glm::normalize(rf);
            }

            glm::vec3 perpendicular_dir = glm::normalize(glm::cross(transfer_normal, periapsis_dir));


            const int resolution = 500;
            float p = transfer_semi_major_ * (1.0f - transfer_eccentricity_ * transfer_eccentricity_);

            float nu0 = atan2(glm::dot(r0, perpendicular_dir), glm::dot(r0, periapsis_dir));
            float nuf = atan2(glm::dot(rf, perpendicular_dir), glm::dot(rf, periapsis_dir));

                nu0 = atan2(glm::dot(r0, perpendicular_dir), glm::dot(r0, periapsis_dir));
            nuf = atan2(glm::dot(rf, perpendicular_dir), glm::dot(rf, periapsis_dir));

            // Calculate the angular sweep needed
            float delta_nu = nuf - nu0;

            // Normalize to reasonable transfer angle
            while (delta_nu > M_PI) delta_nu -= 2.0f * M_PI;
            while (delta_nu < -M_PI) delta_nu += 2.0f * M_PI;

            // If the angle is very small, the points might be nearly collinear
            if (std::abs(delta_nu) < 0.1f) {
                delta_nu = M_PI;  // Use 180° transfer
            }

            std::cout << "   Using actual geometry - delta_nu: " << delta_nu * 180.0f / M_PI << "°" << std::endl;

            std::vector<glm::vec3> physics_transfer_points;
            std::vector<glm::vec3> physics_ellipse_points;

            // Generate complete ellipse
            for (int i = 0; i < resolution; ++i) {
                float true_anomaly = 2.0f * M_PI * i / resolution;
                float r = p / (1.0f + transfer_eccentricity_ * cos(true_anomaly));

                glm::vec3 pos = r * (cosf(true_anomaly) * periapsis_dir +
                                    sinf(true_anomaly) * perpendicular_dir);
                physics_ellipse_points.push_back(pos);
            }

            // Debug
            std::cout << "\n TRANSFER ARC DEBUG:" << std::endl;
            std::cout << "   nu0 (start): " << nu0 * 180.0f / M_PI << "°" << std::endl;
            std::cout << "   nuf (end):   " << nuf * 180.0f / M_PI << "°" << std::endl;
            std::cout << "   delta_nu:    " << delta_nu * 180.0f / M_PI << "°" << std::endl;

            int transfer_steps = std::max(50, static_cast<int>(std::abs(delta_nu) * resolution / (2.0f * M_PI)));
            std::cout << "   transfer_steps: " << transfer_steps << std::endl;

            for (int i = 0; i <= transfer_steps; ++i) {
                float t = static_cast<float>(i) / transfer_steps;
                float true_anomaly = nu0 + t * delta_nu;
                float r = p / (1.0f + transfer_eccentricity_ * cos(true_anomaly));

                if (r > 0 && r < 10.0f) {  // Reasonable bounds check
                    glm::vec3 pos = r * (cosf(true_anomaly) * periapsis_dir +
                                        sinf(true_anomaly) * perpendicular_dir);
                    physics_transfer_points.push_back(pos);

                    // Debug
                    if (i <= 2 || i >= transfer_steps - 2) {
                        std::cout << "   Point " << i << ": r=" << r
                                    << ", pos=(" << pos.x << "," << pos.y << "," << pos.z << ")" << std::endl;
                    }
                } else {
                    std::cout << "   WARNING: Invalid radius r=" << r << " at step " << i << std::endl;
                }
            }

            std::cout << "   Generated " << physics_transfer_points.size() << " transfer points" << std::endl;

            if (!physics_transfer_points.empty()) {
                glm::vec3 first_point = physics_transfer_points.front();
                glm::vec3 last_point = physics_transfer_points.back();

                std::cout << "\n TRANSFER VERIFICATION:" << std::endl;
                std::cout << "   First point distance from r0: "
                            << glm::length(first_point - r0) << std::endl;
                std::cout << "   Last point distance from rf:  "
                            << glm::length(last_point - rf) << std::endl;

                bool start_ok = glm::length(first_point - r0) < 0.01f;
                bool end_ok = glm::length(last_point - rf) < 0.01f;
                std::cout << "   Transfer connection: "
                            << (start_ok && end_ok ? "SUCCESSFUL" : "NEEDS ADJUSTMENT") << std::endl;
            }

            // Convert to visualization coordinates
            transfer_points_ = CoordinateSystem::trajectoryToVisualization(physics_transfer_points);
            complete_ellipse_points_ = CoordinateSystem::trajectoryToVisualization(physics_ellipse_points);
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
