#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <cmath>

#include "Shader.hpp"
#include "OrbitModel.hpp"

OrbitModel::OrbitModel() : vao_(0), vbo_(0), initialized_(false) {}

OrbitModel::~OrbitModel() {
    if (initialized_) {
        glDeleteVertexArrays(1, &vao_);
        glDeleteBuffers(1, &vbo_);
    }
}

void OrbitModel::initialize() {
    if (initialized_) return;
    
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    
    initialized_ = true;
}

// Set orbital parameters - for circular orbits
void OrbitModel::setCircularOrbit(double radius, double inclination, double raan) {
    radius_ = radius;
    inclination_ = inclination;
    raan_ = raan;
    eccentricity_ = 0.0;
    arg_periapsis_ = 0.0;
    
    generateOrbitPoints();
    updateBuffers();
}

// Set orbital parameters - for elliptical orbits
void OrbitModel::setEllipticalOrbit(double semi_major_axis, double eccentricity, 
                        double inclination, double raan, 
                        double arg_periapsis) {
    radius_ = semi_major_axis;
    eccentricity_ = eccentricity;
    inclination_ = inclination;
    raan_ = raan;
    arg_periapsis_ = arg_periapsis;
    
    generateOrbitPoints();
    updateBuffers();
}

void OrbitModel::render(const Shader& shader, const glm::mat4& view_projection, const glm::vec3& color) {
    if (!initialized_ || orbit_points_.empty()) return;
    
    shader.use();
    shader.setMat4("viewProjection", view_projection);
    shader.setVec3("color", color);
    
    glBindVertexArray(vao_);
    glDrawArrays(GL_LINE_LOOP, 0, static_cast<GLsizei>(orbit_points_.size()));
    glBindVertexArray(0);
}

void OrbitModel::generateOrbitPoints(int resolution) {
    orbit_points_.clear();
    
    // Generate orbit in the orbital plane
    for (int i = 0; i < resolution; ++i) {
        double theta = 2.0 * M_PI * i / resolution;
        
        // For elliptical orbits, calculate radius using the orbit equation
        double r;
        if (eccentricity_ < 1e-6) { // Circular orbit
            r = radius_;
        } else { // Elliptical orbit
            r = radius_ * (1.0 - eccentricity_ * eccentricity_) / 
                (1.0 + eccentricity_ * cos(theta));
        }
        
        // Calculate position in orbital plane
        double x = r * cos(theta);
        double y = r * sin(theta);
        double z = 0.0;
        
        // Apply rotations for inclination, RAAN, and argument of periapsis
        // First, rotate by argument of periapsis around z-axis
        double x1 = x * cos(arg_periapsis_) - y * sin(arg_periapsis_);
        double y1 = x * sin(arg_periapsis_) + y * cos(arg_periapsis_);
        
        // Then, rotate by inclination around x-axis
        double y2 = y1 * cos(inclination_);
        double z2 = y1 * sin(inclination_);
        
        // Finally, rotate by RAAN around z-axis
        double x3 = x1 * cos(raan_) - y2 * sin(raan_);
        double y3 = x1 * sin(raan_) + y2 * cos(raan_);
        
        orbit_points_.push_back(glm::vec3(x3, z2, y3)); // Note: y and z swapped for OpenGL
    }
}

void OrbitModel::updateBuffers() {
    if (!initialized_ || orbit_points_.empty()) return;
    
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, orbit_points_.size() * sizeof(glm::vec3), 
                orbit_points_.data(), GL_STATIC_DRAW);
    
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);
    
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}
