#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <cmath>

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

// Set transfer parameters for two-impulse transfer
void TransferModel::setTwoImpulseTransfer(double initialRadius, double initialInclination,
                            double targetRadius, double targetInclination,
                            double initialTrueAnomaly, double finalTrueAnomaly) {
    // Store parameters
    initial_radius_ = initialRadius;
    target_radius_ = targetRadius;
    initial_inclination_ = initialInclination;
    target_inclination_ = targetInclination;
    initial_true_anomaly_ = initialTrueAnomaly;
    final_true_anomaly_ = finalTrueAnomaly;
    
    // Generate a simplified transfer trajectory
    // This is a placeholder - in a real implementation, you'd use the actual
    // transfer orbit calculation based on your PSO results [[5]](https://poe.com/citation?message_id=390602460299&citation=5)
    generateSimpleTransferTrajectory();
    updateBuffers();
}

// Placeholder for now (same as two-impulse transfer)
void TransferModel::setThreeImpulseTransfer(double initialRadius, double initialInclination,
    double targetRadius, double targetInclination,
    double initialTrueAnomaly, double finalTrueAnomaly,
    double final_true_anomaly) {
// Store parameters
initial_radius_ = initialRadius;
target_radius_ = targetRadius;
initial_inclination_ = initialInclination;
target_inclination_ = targetInclination;
initial_true_anomaly_ = initialTrueAnomaly;
final_true_anomaly_ = finalTrueAnomaly;

// Generate a simplified transfer trajectory
// This is a placeholder - in a real implementation, you'd use the actual
// transfer orbit calculation based on your PSO results [[5]](https://poe.com/citation?message_id=390602460299&citation=5)
generateSimpleTransferTrajectory();
updateBuffers();
}

void TransferModel::render(const Shader& shader, const glm::mat4& view_projection, 
            const glm::vec3& color, float animation_progress) {
    if (!initialized_ || transfer_points_.empty()) return;
    
    shader.use();
    shader.setMat4("viewProjection", view_projection);
    shader.setVec3("color", color);
    
    // Calculate how many points to draw based on animation progress
    int num_points = static_cast<int>(transfer_points_.size() * animation_progress);
    if (num_points < 2) num_points = 2; // Always draw at least the first segment
    
    glBindVertexArray(vao_);
    glDrawArrays(GL_LINE_STRIP, 0, num_points);
    glBindVertexArray(0);
}

const std::vector<glm::vec3>& TransferModel::getImpulsePositions() const { return impulse_positions_; }

const std::vector<glm::vec3>& TransferModel::getImpulseDirections() const { return impulse_directions_; }

const std::vector<float>& TransferModel::getImpulseMagnitudes() const { return impulse_magnitudes_; }

void TransferModel::generateSimpleTransferTrajectory() {
    transfer_points_.clear();
    impulse_positions_.clear();
    impulse_directions_.clear();
    impulse_magnitudes_.clear();
    
    // This is a simplified model - in reality, you'd calculate the actual
    // transfer orbit based on the PSO optimization results
    // For now, we'll create a simple elliptical transfer as a placeholder
    
    const int resolution = 100;
    
    // Calculate initial and final positions on the respective orbits
    double initial_x = initial_radius_ * cos(initial_true_anomaly_);
    double initial_y = initial_radius_ * sin(initial_true_anomaly_) * cos(initial_inclination_);
    double initial_z = initial_radius_ * sin(initial_true_anomaly_) * sin(initial_inclination_);
    
    double final_x = target_radius_ * cos(final_true_anomaly_);
    double final_y = target_radius_ * sin(final_true_anomaly_) * cos(target_inclination_);
    double final_z = target_radius_ * sin(final_true_anomaly_) * sin(target_inclination_);
    
    // Store the impulse positions
    impulse_positions_.push_back(glm::vec3(initial_x, initial_z, initial_y)); // Note: y and z swapped for OpenGL
    impulse_positions_.push_back(glm::vec3(final_x, final_z, final_y));
    
    // For a real implementation, you would calculate the actual velocity vectors
    // Here we just use some placeholder values for visualization
    impulse_directions_.push_back(glm::normalize(glm::vec3(final_x - initial_x, final_z - initial_z, final_y - initial_y)));
    impulse_directions_.push_back(glm::normalize(glm::vec3(initial_x - final_x, initial_z - final_z, initial_y - final_y)));
    
    impulse_magnitudes_.push_back(500.0f); // Placeholder values
    impulse_magnitudes_.push_back(500.0f);
    
    // Generate points along the transfer path
    // For simplicity, we'll just interpolate between the initial and final positions
    // In a real implementation, you'd use the actual transfer orbit
    for (int i = 0; i <= resolution; ++i) {
        float t = static_cast<float>(i) / resolution;
        
        // Simple cubic interpolation for a more natural curve
        float h1 = 2*t*t*t - 3*t*t + 1;
        float h2 = -2*t*t*t + 3*t*t;
        float h3 = t*t*t - 2*t*t + t;
        float h4 = t*t*t - t*t;
        
        glm::vec3 initial_pos(initial_x, initial_z, initial_y);
        glm::vec3 final_pos(final_x, final_z, final_y);
        
        // Tangent vectors - for a more natural curve
        glm::vec3 initial_tangent = impulse_directions_[0] * (float)(impulse_magnitudes_[0] * 0.1);
        glm::vec3 final_tangent = -impulse_directions_[1] * (float)(impulse_magnitudes_[1] * 0.1);
        
        glm::vec3 pos = h1 * initial_pos + h2 * final_pos + h3 * initial_tangent + h4 * final_tangent;
        
        transfer_points_.push_back(pos);
    }
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