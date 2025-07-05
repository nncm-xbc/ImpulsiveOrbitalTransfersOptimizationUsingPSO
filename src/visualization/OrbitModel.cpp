#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <cmath>
#include <iostream>

#include "visualization/Shader.hpp"
#include "visualization/OrbitModel.hpp"
#include "core/Constants.hpp"
#include "core/CoordinateSystem.hpp"
#include "core/OrbitMechanics.hpp"

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

void OrbitModel::setCircularOrbit(double radius, double inclination, double raan) {
    radius_ = radius;
    inclination_ = inclination;
    raan_ = raan;
    eccentricity_ = 0.0;
    arg_periapsis_ = 0.0;

    generateOrbitPoints();
    updateBuffers();
}

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

glm::vec3 OrbitModel::calculateRenderedCenter() {
    if (orbit_points_.empty()) {
        return glm::vec3(0.0f, 0.0f, 0.0f);
    }

    glm::vec3 center(0.0f, 0.0f, 0.0f);
    for (const auto& point : orbit_points_) {
        center += point;
    }
    center /= static_cast<float>(orbit_points_.size());

    return center;
}

void OrbitModel::render(const Shader& shader, const glm::mat4& view_projection, const glm::vec3& color) {
    if (!initialized_ || orbit_points_.empty()) return;

    glm::vec3 center = calculateRenderedCenter();

    shader.use();
    shader.setMat4("viewProjection", view_projection);
    shader.setVec3("color", color);

    glBindVertexArray(vao_);
    glDrawArrays(GL_LINE_LOOP, 0, static_cast<GLsizei>(orbit_points_.size()));

    GLuint centerVAO, centerVBO;
    glGenVertexArrays(1, &centerVAO);
    glGenBuffers(1, &centerVBO);

    std::vector<glm::vec3> centerMarker = {
        center + glm::vec3(-10.0f, 0.0f, 0.0f),
        center + glm::vec3(10.0f, 0.0f, 0.0f),
        center + glm::vec3(0.0f, -10.0f, 0.0f),
        center + glm::vec3(0.0f, 10.0f, 0.0f),
        center + glm::vec3(0.0f, 0.0f, -10.0f),
        center + glm::vec3(0.0f, 0.0f, 10.0f)
    };

    glBindVertexArray(centerVAO);
    glBindBuffer(GL_ARRAY_BUFFER, centerVBO);
    glBufferData(GL_ARRAY_BUFFER, centerMarker.size() * sizeof(glm::vec3), centerMarker.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);

    shader.setVec3("color", glm::vec3(1.0f, 1.0f, 1.0f));
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(centerMarker.size()));

    glDeleteVertexArrays(1, &centerVAO);
    glDeleteBuffers(1, &centerVBO);

    glBindVertexArray(0);
}

void OrbitModel::generateOrbitPoints(int resolution) {
    orbit_points_.clear();
    std::vector<glm::vec3> physics_points;

    std::cout << "OrbitModel generating points with:" << std::endl;
    std::cout << "  radius=" << radius_ << ", inclination=" << inclination_*180/M_PI
              << "°, RAAN=" << raan_*180/M_PI << "°" << std::endl;

    for (int i = 0; i < resolution; ++i) {
        double true_anomaly = 2.0 * M_PI * i / resolution;

        // USE THE SAME METHOD AS TRANSFERMODEL
        // This ensures perfect consistency between orbit rendering and transfer calculations
        Physics::Vector3 pos_physics = Physics::OrbitMechanics::calculatePosition3D(
            radius_,           // Semi-major axis (for circular: a = radius)
            eccentricity_,     // Eccentricity
            inclination_,      // Inclination
            raan_,             // Right ascension of ascending node
            arg_periapsis_,    // Argument of periapsis
            true_anomaly       // True anomaly
        );

        // Convert Physics::Vector3 to glm::vec3 for visualization pipeline
        glm::vec3 glm_pos(pos_physics.x, pos_physics.y, pos_physics.z);
        physics_points.push_back(glm_pos);
    }

    // Transform to visualization coordinates using the same pipeline as TransferModel
    orbit_points_ = CoordinateSystem::trajectoryToVisualization(physics_points);

    // Debug output for verification
    if (physics_points.size() >= 3) {
        std::cout << "  Sample points:" << std::endl;
        std::cout << "    ν=0°:   (" << physics_points[0].x << ", " << physics_points[0].y << ", " << physics_points[0].z << ")" << std::endl;
        std::cout << "    ν=90°:  (" << physics_points[resolution/4].x << ", " << physics_points[resolution/4].y << ", " << physics_points[resolution/4].z << ")" << std::endl;
        std::cout << "    ν=180°: (" << physics_points[resolution/2].x << ", " << physics_points[resolution/2].y << ", " << physics_points[resolution/2].z << ")" << std::endl;

        // Verify the orbit is circular by checking radius at different points
        double r0 = sqrt(physics_points[0].x*physics_points[0].x + physics_points[0].y*physics_points[0].y + physics_points[0].z*physics_points[0].z);
        double r90 = sqrt(physics_points[resolution/4].x*physics_points[resolution/4].x + physics_points[resolution/4].y*physics_points[resolution/4].y + physics_points[resolution/4].z*physics_points[resolution/4].z);

        std::cout << "  Radius verification: r(0°)=" << r0 << ", r(90°)=" << r90 << " (should be ≈" << radius_ << ")" << std::endl;

        if (std::abs(r0 - radius_) > 1e-6 || std::abs(r90 - radius_) > 1e-6) {
            std::cout << "  WARNING: Orbit radius mismatch!" << std::endl;
        }
    }

    std::cout << "  Generated " << orbit_points_.size() << " visualization points" << std::endl;
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
