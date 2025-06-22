#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <cmath>
#include <iostream>

#include "visualization/Shader.hpp"
#include "visualization/OrbitModel.hpp"
#include "core/Constants.hpp"
#include "core/CoordinateSystem.hpp"

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
    //std::cout << "Rendered orbit center: " << center.x << ", " << center.y << ", " << center.z << std::endl;

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

    for (int i = 0; i < resolution; ++i) {
        double true_anomaly = 2.0 * M_PI * i / (resolution - 1);

        double r;
        if (eccentricity_ < 1e-6) { // Circular orbit
            r = radius_;
        } else { // Elliptical orbit
            double p = radius_ * (1.0 - eccentricity_ * eccentricity_);
            r = p / (1.0 + eccentricity_ * cos(true_anomaly));
        }

        // Position in orbital plane
        glm::vec3 pos_orbital(
            r * cos(true_anomaly),
            r * sin(true_anomaly),
            0.0
        );

        // Transform to inertial physics coordinates
        glm::mat3 R = CoordinateSystem::orbitalToInertialMatrix(
            raan_, inclination_, arg_periapsis_
        );

        glm::vec3 pos_physics = R * pos_orbital;
        physics_points.push_back(pos_physics);

        // Debug
        if (i == 0 || i == resolution/4 || i == resolution/2 || i == 3*resolution/4) {
            std::cout << "Orbit point at nu=" << true_anomaly * 180/M_PI
                      << "°: " << pos_physics.x << ", " << pos_physics.y
                      << ", " << pos_physics.z << std::endl;
        }
    }

    orbit_points_ = CoordinateSystem::trajectoryToVisualization(physics_points);

    if (physics_points.size() >= 3) {
        glm::vec3 expected_normal(
            sin(raan_) * sin(inclination_),
            -cos(raan_) * sin(inclination_),
            cos(inclination_)
        );

        if (!CoordinateSystem::verifyOrbitalPlane(physics_points, expected_normal, 0.01f)) {
            std::cerr << "WARNING: Orbit not in expected plane!" << std::endl;
            std::cerr << "  Inclination: " << inclination_ * 180/M_PI << "°" << std::endl;
            std::cerr << "  RAAN: " << raan_ * 180/M_PI << "°" << std::endl;
        }
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
