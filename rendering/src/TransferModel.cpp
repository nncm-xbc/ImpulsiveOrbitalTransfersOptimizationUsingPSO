#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <cmath>

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

// Transfer parameters for two-impulse transfer
void TransferModel::setTwoImpulseTransfer(double initialRadius, double initialInclination,
                        double targetRadius, double targetInclination,
                        double initialTrueAnomaly, double finalTrueAnomaly, 
                        std::vector<double> impulseMagnitudes) {
    initial_radius_ = initialRadius;
    target_radius_ = targetRadius;
    initial_inclination_ = initialInclination;
    target_inclination_ = targetInclination;
    initial_true_anomaly_ = initialTrueAnomaly;
    final_true_anomaly_ = finalTrueAnomaly;
    
    // Simplified transfer trajectory
    generateTransferTrajectory(impulseMagnitudes);
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

    // Simplified transfer trajectory
    //generateTransferTrajectory();
    updateBuffers();
}

void TransferModel::render(const Shader& shader, const glm::mat4& view_projection, 
            const glm::vec3& color, float animation_progress) {
    if (!initialized_ || transfer_points_.empty()) return;
    
    shader.use();
    shader.setMat4("viewProjection", view_projection);
    shader.setVec3("color", color);
    
    // how many points to draw based on animation progress
    int num_points = static_cast<int>(transfer_points_.size() * animation_progress);
    if (num_points < 2) num_points = 2;
    
    glBindVertexArray(vao_);
    glDrawArrays(GL_LINE_STRIP, 0, num_points);
    glBindVertexArray(0);
}

const std::vector<glm::vec3>& TransferModel::getImpulsePositions() const { return impulse_positions_; }

const std::vector<glm::vec3>& TransferModel::getImpulseDirections() const { return impulse_directions_; }

const std::vector<float>& TransferModel::getImpulseMagnitudes() const { return impulse_magnitudes_; }

void TransferModel::generateTransferTrajectory(std::vector<double> impulseMagnitudes) {
    transfer_points_.clear();
    impulse_positions_.clear();
    impulse_directions_.clear();
    impulse_magnitudes_.clear();
    
    const int resolution = 100;
    
    // Initial and final positions on the orbits
    glm::vec3 initial_pos = calculateOrbitPosition(initial_radius_, initial_inclination_, initial_true_anomaly_);
    glm::vec3 final_pos = calculateOrbitPosition(target_radius_, target_inclination_, final_true_anomaly_);
    
    // Impulse positions
    impulse_positions_.push_back(glm::vec3(initial_pos.x, initial_pos.z, initial_pos.y));
    impulse_positions_.push_back(glm::vec3(final_pos.x, final_pos.z, final_pos.y));

    // impulse_positions_.push_back(glm::vec3(initial_x, initial_z, initial_y)); // y and z swapped for OpenGL
    // impulse_positions_.push_back(glm::vec3(final_x, final_z, final_y));
    
    // Velocities at the initial and final positions
    glm::vec3 initial_vel = calculateOrbitVelocity(initial_radius_, initial_inclination_, initial_true_anomaly_);
    glm::vec3 final_vel = calculateOrbitVelocity(target_radius_, target_inclination_, final_true_anomaly_);

    glm::vec3 transfer_direction = glm::normalize(final_pos - initial_pos);
    
    // impulse magnitudes
    float dv1 = impulseMagnitudes[0];
    float dv2 = impulseMagnitudes[1];
    
    impulse_magnitudes_.push_back(dv1);
    impulse_magnitudes_.push_back(dv2);
    
    // Approximate impulse directions
    glm::vec3 radial_dir = glm::normalize(initial_pos);
    glm::vec3 normal_dir = glm::normalize(glm::cross(initial_vel, initial_pos));
    glm::vec3 tangential_dir = glm::normalize(glm::cross(normal_dir, radial_dir));
    
    // For non-coplanar transfers, first impulse primarily increases semi-major axis
    // with a small plane change component
    impulse_directions_.push_back(glm::normalize(tangential_dir + 0.3f * normal_dir));
    impulse_directions_.push_back(glm::normalize(glm::cross(final_vel, final_pos)));
    
    // Generate transfer path points

    // Placeholder for now - calculate the actual transfer orbit based on Lambert's problem solution
    float semi_major_axis = (initial_radius_ + target_radius_) / 2.0f;
    float ellipse_ratio = initial_radius_ / semi_major_axis;
    
    for (int i = 0; i <= resolution; ++i) {
        float t = static_cast<float>(i) / resolution;
        float angle = initial_true_anomaly_ * (1.0f - t) + final_true_anomaly_ * t;
        
        // Simple elliptical path
        float r = semi_major_axis * (1.0f - ellipse_ratio * cos(angle));
        
        // Gradually change inclination 
        float inclination = initial_inclination_ * (1.0f - t) + target_inclination_ * t;
        
        // Position on transfer ellipse
        glm::vec3 pos = calculateOrbitPosition(r, inclination, angle);
        
        // Adjust for OpenGL
        transfer_points_.push_back(glm::vec3(pos.x, pos.z, pos.y));
    }
}

glm::vec3 TransferModel::calculateOrbitPosition(float radius, float inclination, float true_anomaly) {
    // Position in orbital plane
    float x = radius * cos(true_anomaly);
    float y = radius * sin(true_anomaly);
    float z = 0.0f;
    
    // Rotate by inclination (around x-axis)
    float y_rotated = y * cos(inclination);
    float z_rotated = y * sin(inclination);
    
    return glm::vec3(x, y_rotated, z_rotated);
}

glm::vec3 TransferModel::calculateOrbitVelocity(float radius, float inclination, float true_anomaly) {
    float speed = sqrt(constant::MU / radius);
    
    // Velocity in orbital plane
    float vx = -speed * sin(true_anomaly);
    float vy = speed * cos(true_anomaly);
    float vz = 0.0f;
    
    float vy_rotated = vy * cos(inclination);
    float vz_rotated = vy * sin(inclination);
    
    return glm::vec3(vx, vy_rotated, vz_rotated);
}

void TransferModel::updateBuffers() {
    if (!initialized_ || transfer_points_.empty()) return;
    
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, transfer_points_.size() * sizeof(glm::vec3), 
                transfer_points_.data(), GL_STATIC_DRAW);
    
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);
    
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}