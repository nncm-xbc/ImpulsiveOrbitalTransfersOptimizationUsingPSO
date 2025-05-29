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
                        double initialTrueAnomaly, double finalTrueAnomaly, 
                        std::vector<double> impulseMagnitudes, std::vector<double> planeChange) {
    initial_radius_ = initialRadius;
    target_radius_ = targetRadius;
    initial_inclination_ = initialInclination;
    target_inclination_ = targetInclination;
    initial_true_anomaly_ = initialTrueAnomaly;
    final_true_anomaly_ = finalTrueAnomaly;
    impulse_magnitudes_ = impulseMagnitudes;
    plane_change_ = planeChange;
    
    generateTransferTrajectory();
    generateCompleteTransferEllipse();
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
    
    //std::cout << "Rendering with color: " << color.x << ", " << color.y << ", " << color.z << std::endl;
    
    shader.setMat4("viewProjection", view_projection);
    shader.setVec3("color", color);
    
    // Debug
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        std::cout << "OpenGL error in render (uniforms): " << err << std::endl;
    }
    
    // how many points to draw based on animation progress
    int num_points = static_cast<int>(transfer_points_.size() * animation_progress);
    if (num_points < 2) num_points = 2;
    
    //std::cout << "Drawing " << num_points << " points" << std::endl;
    
    glBindVertexArray(vao_);
    glDrawArrays(GL_LINE_STRIP, 0, num_points);
    
    // Debug
    err = glGetError();
    if (err != GL_NO_ERROR) {
        std::cout << "OpenGL error in render (drawing): " << err << std::endl;
    }
    
    glBindVertexArray(0);
}

const std::vector<glm::vec3>& TransferModel::getImpulsePositions() const { return impulse_positions_; }

const std::vector<glm::vec3>& TransferModel::getImpulseDirections() const { return impulse_directions_; }

const std::vector<double>& TransferModel::getImpulseMagnitudes() const { return impulse_magnitudes_; }

// void TransferModel::generateTransferTrajectory(std::vector<double> impulseMagnitudes) {
//     std::cout << "=== GENERATING TRANSFER TRAJECTORY ===" << std::endl;
//     std::cout << "Initial radius: " << initial_radius_ << ", inclination: " << initial_inclination_ << std::endl;
//     std::cout << "Target radius: " << target_radius_ << ", inclination: " << target_inclination_ << std::endl;
//     std::cout << "Initial true anomaly: " << initial_true_anomaly_ << ", Final true anomaly: " << final_true_anomaly_ << std::endl;
//     std::cout << "Impulse magnitudes: ";
//     for (auto mag : impulseMagnitudes) std::cout << mag << " ";
//     std::cout << std::endl;
    
//     transfer_points_.clear();
//     impulse_positions_.clear();
//     impulse_directions_.clear();
//     impulse_magnitudes_.clear();
    
//     const int resolution = 100;
    
//     glm::vec3 initial_pos = calculateOrbitPosition(initial_radius_, initial_inclination_, initial_true_anomaly_);
//     glm::vec3 final_pos = calculateOrbitPosition(target_radius_, target_inclination_, final_true_anomaly_);

//     std::cout << "Initial position (orbital): " << initial_pos.x << ", " << initial_pos.y << ", " << initial_pos.z << std::endl;
//     std::cout << "Final position (orbital): " << final_pos.x << ", " << final_pos.y << ", " << final_pos.z << std::endl;

//     glm::vec3 initial_pos_gl(initial_pos.x, initial_pos.z, initial_pos.y);
//     glm::vec3 final_pos_gl(final_pos.x, final_pos.z, final_pos.y);

//     std::cout << "Initial position (OpenGL): " << initial_pos_gl.x << ", " << initial_pos_gl.y << ", " << initial_pos_gl.z << std::endl;
//     std::cout << "Final position (OpenGL): " << final_pos_gl.x << ", " << final_pos_gl.y << ", " << final_pos_gl.z << std::endl;
    
//     //impulse_positions_.push_back(glm::vec3(initial_pos.x, initial_pos.z, initial_pos.y));
//     //impulse_positions_.push_back(glm::vec3(final_pos.x, final_pos.z, final_pos.y));

//     impulse_positions_.push_back(glm::vec3(initial_pos.x, initial_pos.z, initial_pos.y) * 1000.0f);
//     impulse_positions_.push_back(glm::vec3(final_pos.x, final_pos.z, final_pos.y) * 1000.0f);
    
//     glm::vec3 initial_vel = calculateOrbitVelocity(initial_radius_, initial_inclination_, initial_true_anomaly_);
//     glm::vec3 final_vel = calculateOrbitVelocity(target_radius_, target_inclination_, final_true_anomaly_);

    
//     float dv1 = impulseMagnitudes[0];
//     float dv2 = impulseMagnitudes[1];
    
//     impulse_magnitudes_.push_back(dv1);
//     impulse_magnitudes_.push_back(dv2);

//     std::cout << "Using impulse magnitudes: " << dv1 << ", " << dv2 << std::endl;
    
//     glm::vec3 radial_dir = glm::normalize(initial_pos);
//     glm::vec3 normal_dir = glm::normalize(glm::cross(initial_vel, radial_dir));
//     glm::vec3 tangential_dir = glm::normalize(glm::cross(normal_dir, radial_dir));
    
//     float scale_factor = 0.2f;
    
//     glm::vec3 impulse1_dir = glm::normalize(tangential_dir + 0.3f * normal_dir) * scale_factor;
    
//     glm::vec3 impulse2_dir = glm::normalize(tangential_dir + 0.5f * normal_dir) * scale_factor;
    
//     glm::vec3 impulse1_dir_gl(impulse1_dir.x, impulse1_dir.z, impulse1_dir.y);
//     glm::vec3 impulse2_dir_gl(impulse2_dir.x, impulse2_dir.z, impulse2_dir.y);
    
//     impulse_directions_.push_back(impulse1_dir_gl);
//     impulse_directions_.push_back(impulse2_dir_gl);
    
//     std::cout << "Impulse 1 direction: " << impulse1_dir_gl.x << ", " << impulse1_dir_gl.y << ", " << impulse1_dir_gl.z << std::endl;
//     std::cout << "Impulse 2 direction: " << impulse2_dir_gl.x << ", " << impulse2_dir_gl.y << ", " << impulse2_dir_gl.z << std::endl;
        
//     transfer_semi_major_ = (initial_radius_ + target_radius_) / 2.0f;
//     transfer_eccentricity_ = fabs(initial_radius_ - target_radius_) / (initial_radius_ + target_radius_);
    
//     std::cout << "Transfer orbit: a = " << transfer_semi_major_ << ", e = " << transfer_eccentricity_ << std::endl;
    
//     float anomaly_diff = final_true_anomaly_ - initial_true_anomaly_;
//     if (anomaly_diff > M_PI) anomaly_diff -= 2.0f * M_PI;
//     if (anomaly_diff < -M_PI) anomaly_diff += 2.0f * M_PI;
    
//     std::cout << "Anomaly difference: " << anomaly_diff << " radians" << std::endl;
    
//     for (int i = 0; i <= resolution; ++i) {
//         float t = static_cast<float>(i) / resolution;
//         float angle = 2.0f * M_PI * i / resolution;
                
//         while (angle < 0) angle += 2.0f * M_PI;
//         while (angle >= 2.0f * M_PI) angle -= 2.0f * M_PI;
        
//         float r = transfer_semi_major_ * (1.0f - transfer_eccentricity_ * transfer_eccentricity_) / 
//                  (1.0f + transfer_eccentricity_ * cos(angle - initial_true_anomaly_));
                 
//         float inclination = initial_inclination_ * (1.0f - t) + target_inclination_ * t;
        
//         glm::vec3 pos = calculateOrbitPosition(r, inclination, angle);
        
//         glm::vec3 pos_gl(pos.x, pos.z, pos.y);
        
//         if (i == 0 || i == resolution/2 || i == resolution) {
//             std::cout << "Transfer point " << i << ": " << pos_gl.x << ", " << pos_gl.y << ", " << pos_gl.z << std::endl;
//         }
        
//         transfer_points_.push_back(pos_gl);
//     }

//     float visualScale = 1000.0f;

//     for (auto& point : transfer_points_) {
//         point *= visualScale;
//     }

//     for (auto& pos : impulse_positions_) {
//         pos *= visualScale;
//     }

//     for (auto& dir : impulse_directions_) {
//         dir = glm::normalize(dir);
//         dir *= visualScale * 0.2f;
//     }
    
//     std::cout << "Generated " << transfer_points_.size() << " transfer points" << std::endl;
//     std::cout << "=== TRANSFER TRAJECTORY GENERATION COMPLETE ===" << std::endl;
// }

void TransferModel::generateTransferTrajectory() {
    transfer_points_.clear();
    
    // Position vectors at departure/arrival
    glm::vec3 r0 = calculateOrbitPosition(initial_radius_, initial_inclination_, initial_true_anomaly_);
    glm::vec3 rf = calculateOrbitPosition(target_radius_, 0.0, final_true_anomaly_);
    
    // Velocity vectors before impulses on initial/final orbits
    float v0_mag = sqrt(constant::MU / initial_radius_);
    float vf_mag = sqrt(constant::MU / target_radius_);
    
    // Unit vectors aligned with r(t0) and r(tf)
    glm::vec3 r0_hat = glm::normalize(r0);
    glm::vec3 rf_hat = glm::normalize(rf);

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
    
    // Initial/final velocity vec
    glm::vec3 v0_minus = v0_mag * tangential_dir;
    glm::vec3 vf_plus = vf_mag * glm::normalize(glm::cross(hf_hat, rf_hat));
    
    float dv1_mag = impulse_magnitudes_[0];
    float dv2_mag = impulse_magnitudes_[1];
    
    // Plane change components
    float plane_change1 = plane_change_[0];
    float plane_change2 = plane_change_[1];  
    
    // Velocity after first impulse
    float tangential_component = dv1_mag / sqrt(1.0f + plane_change1 * plane_change1);
    float normal_component = tangential_component * plane_change1;
    
    glm::vec3 impulse_vec = tangential_component * + normal_component * normal_dir;

    glm::vec3 v0_plus = v0_minus + dv1_mag * impulse_vec;

    // Debug
    std::cout << "Impulse direction: tangential=" << tangential_component << ", normal=" << normal_component << std::endl;
    std::cout << "Initial velocity: " << glm::length(v0_minus) << std::endl;
    std::cout << "Velocity after impulse: " << glm::length(v0_plus) << std::endl;
    
    // Orbital elements of the transfer orbit
    float r0_mag = glm::length(r0);
    float v0_plus_mag = glm::length(v0_plus);
    
    float transfer_a = (constant::MU * r0_mag) / (2.0f * constant::MU - r0_mag * v0_plus_mag * v0_plus_mag);

    // Theoretical Hohmann-like values for testing
    transfer_a = (initial_radius_ + target_radius_) / 2.0f;
    
    // Angular momentum of transfer orbit
    glm::vec3 h_transfer = glm::cross(r0, v0_plus);
    float h_transfer_mag = glm::length(h_transfer);
    
    float transfer_e = sqrt(1.0f - (h_transfer_mag * h_transfer_mag) / (constant::MU * transfer_a));

    // Theoretical eccentricity for testing
    transfer_e = fabs(initial_radius_ - target_radius_) / (initial_radius_ + target_radius_);

    // Normal to transfer orbit plane
    glm::vec3 normal = glm::normalize(h_transfer);
    
    // Semi-latus rectum
    float p = transfer_a * (1.0f - transfer_e * transfer_e);
    
    // Periapsis direction in transfer plane
    glm::vec3 cross_product = glm::cross(v0_plus, h_transfer);
    glm::vec3 e_vec = (cross_product / float(constant::MU)) - r0_hat;
    glm::vec3 periapsis_dir = glm::normalize(e_vec);
    
    // Perpendicular vector in the transfer plane
    glm::vec3 perpendicular = glm::normalize(glm::cross(normal, periapsis_dir));
    
    // Generate points along the transfer ellipse
    int resolution = 100;
    for (int i = 0; i <= resolution; ++i) {
        float true_anomaly = 2.0f * M_PI * i / resolution;
        float r = p / (1.0f + transfer_e * cos(true_anomaly));
        
        float r_cos = r * cos(true_anomaly);
        float r_sin = r * sin(true_anomaly);
        glm::vec3 pos = periapsis_dir * r_cos + perpendicular * r_sin;;
        
        // Scale for visualization
        pos *= 1000.0f;
        
        transfer_points_.push_back(pos);
    }

    transfer_semi_major_ = transfer_a;
    transfer_eccentricity_ = transfer_e;
    
    // Debug
    std::cout << "Transfer orbit: a = " << transfer_a 
              << ", e = " << transfer_e << std::endl;
    std::cout << "Normal to transfer plane: " << normal.x << ", " 
              << normal.y << ", " << normal.z << std::endl;
}

void TransferModel::generateCompleteTransferEllipse() {
    complete_ellipse_points_.clear();
    
    // Transfer orbit parameters
    float p = transfer_semi_major_ * (1.0f - transfer_eccentricity_ * transfer_eccentricity_);
    
    glm::vec3 periapsis_dir = glm::normalize(glm::vec3(
        cos(initial_true_anomaly_),
        sin(initial_true_anomaly_) * cos(initial_inclination_),
        sin(initial_true_anomaly_) * sin(initial_inclination_)
    ));
    
    // Normal to the transfer plane
    glm::vec3 r0 = calculateOrbitPosition(initial_radius_, initial_inclination_, initial_true_anomaly_);
    glm::vec3 rf = calculateOrbitPosition(target_radius_, 0.0, final_true_anomaly_);
    glm::vec3 normal = glm::normalize(glm::cross(r0, rf));
    
    // Projection of periapsis_dir onto the transfer plane
    float dot_product = glm::dot(periapsis_dir, normal);
    periapsis_dir = glm::normalize(periapsis_dir - dot_product * normal);
    
    glm::vec3 perpendicular = glm::normalize(glm::cross(normal, periapsis_dir));
    
    // Points for the complete ellipse
    int resolution = 100;
    for (int i = 0; i <= resolution; ++i) {
        float true_anomaly = 2.0f * M_PI * i / resolution;
        float r = p / (1.0f + transfer_eccentricity_ * cos(true_anomaly));
        
        float r_float = static_cast<float>(r);
        float cos_val = static_cast<float>(cos(true_anomaly));
        float sin_val = static_cast<float>(sin(true_anomaly));

        glm::vec3 pos = r_float * (cos_val * periapsis_dir + sin_val * perpendicular);    

        pos *= 1000.0f;
        
        complete_ellipse_points_.push_back(pos);
    }
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
    
    glBindVertexArray(ellipseVAO);
    glDrawArrays(GL_LINE_STRIP, 0, complete_ellipse_points_.size());
    glBindVertexArray(0);
}

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