/**
 * @file CoordinateSystem.hpp
 * @brief Unified coordinate system transformations for the entire project
 * @author PSO Orbit Transfer Team
 * @date 2025
 *
 * This file provides coordinate system transformations between physics computations
 * and OpenGL visualization, ensuring consistency across the entire project.
 */

#ifndef COORDINATE_SYSTEM_HPP
#define COORDINATE_SYSTEM_HPP

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

/**
 * @namespace CoordinateSystem
 * @brief Coordinate system transformations and utilities
 *
 * Handles transformations between two coordinate systems:
 * 1. Physics System: Standard orbital mechanics right-handed system
 * 2. Visualization System: OpenGL right-handed system for display
 */
namespace CoordinateSystem {

// ============================================
// PHYSICS COORDINATE SYSTEM (Computational)
// ============================================
// Standard orbital mechanics right-handed system:
// - X: Reference direction (e.g., vernal equinox)
// - Y: 90° from X in orbital plane
// - Z: Normal to reference plane (positive north)
// - Origin: Central body center
// - Units: DU (Distance Units), TU (Time Units)

// ============================================
// VISUALIZATION COORDINATE SYSTEM (OpenGL)
// ============================================
// OpenGL right-handed system for display:
// - X: Same as physics X
// - Y: Up on screen (corresponds to physics Z)
// - Z: Out of screen (corresponds to physics Y)
// - Origin: Same as physics
// - Units: Scaled for visibility (1000x scale factor)

/** @brief Scale factor for visualization (physics units to display units) */
constexpr float VISUAL_SCALE = 1000.0f;

// ============================================
// CONVERSION FUNCTIONS
// ============================================

/**
 * @brief Convert from physics (computational) coordinates to OpenGL visualization
 * @param physics_pos Position vector in physics coordinate system
 * @return Position vector in OpenGL coordinate system
 *
 * This handles both the axis swap and scaling transformation:
 * Physics (X,Y,Z) -> OpenGL (X,Z,Y) with scaling
 */
inline glm::vec3 physicsToVisualization(const glm::vec3& physics_pos) {
    // Apply axis transformation: Physics (X,Y,Z) -> OpenGL (X,Z,Y)
    glm::vec3 opengl_pos(
        physics_pos.x,
        physics_pos.z,
        physics_pos.y
    );

    // Apply scale for visualization
    return opengl_pos * VISUAL_SCALE;
}

/**
 * @brief Convert from physics (double precision) to visualization
 * @param physics_pos Position vector in physics coordinate system (double precision)
 * @return Position vector in OpenGL coordinate system (single precision)
 */
inline glm::vec3 physicsToVisualization(const glm::dvec3& physics_pos) {
    return physicsToVisualization(glm::vec3(physics_pos));
}

/**
 * @brief Convert from visualization back to physics coordinates
 * @param opengl_pos Position vector in OpenGL coordinate system
 * @return Position vector in physics coordinate system
 *
 * Useful for debugging or picking operations in the visualization
 */
inline glm::vec3 visualizationToPhysics(const glm::vec3& opengl_pos) {
    // Remove scale
    glm::vec3 unscaled = opengl_pos / VISUAL_SCALE;

    // Reverse axis transformation: OpenGL (X,Y,Z) -> Physics (X,Z,Y)
    return glm::vec3(
        unscaled.x,
        unscaled.z,
        unscaled.y
    );
}

/**
 * @brief Transform a whole trajectory for visualization
 * @param physics_points Vector of position points in physics coordinates
 * @return Vector of position points in OpenGL coordinates
 *
 * Efficiently transforms entire trajectories for rendering
 */
inline std::vector<glm::vec3> trajectoryToVisualization(const std::vector<glm::vec3>& physics_points) {
    std::vector<glm::vec3> visual_points;
    visual_points.reserve(physics_points.size());

    for (const auto& point : physics_points) {
        visual_points.push_back(physicsToVisualization(point));
    }

    return visual_points;
}

// ============================================
// ORBITAL PLANE TRANSFORMATIONS
// ============================================

/**
 * @brief Create rotation matrix for orbital elements in physics coordinates
 * @param raan Right ascension of ascending node (radians)
 * @param inclination Orbital inclination (radians)
 * @param arg_periapsis Argument of periapsis (radians)
 * @return 3x3 rotation matrix from orbital plane to inertial frame
 *
 * Standard orbital mechanics transformation sequence: Ω → i → ω
 */
inline glm::mat3 orbitalToInertialMatrix(float raan, float inclination, float arg_periapsis) {
    // Standard orbital mechanics rotation sequence
    glm::mat3 R_raan = glm::mat3(
        cos(raan), -sin(raan), 0.0f,
        sin(raan),  cos(raan), 0.0f,
        0.0f,       0.0f,      1.0f
    );

    glm::mat3 R_inc = glm::mat3(
        1.0f, 0.0f,            0.0f,
        0.0f, cos(inclination), -sin(inclination),
        0.0f, sin(inclination),  cos(inclination)
    );

    glm::mat3 R_omega = glm::mat3(
        cos(arg_periapsis), -sin(arg_periapsis), 0.0f,
        sin(arg_periapsis),  cos(arg_periapsis), 0.0f,
        0.0f,                0.0f,                1.0f
    );

    // Combined transformation
    return R_raan * R_inc * R_omega;
}

/**
 * @brief Verify that a set of points lie in the expected plane
 * @param points Vector of points to check
 * @param expected_normal Expected normal vector of the plane
 * @param tolerance Tolerance for planarity check (default 1e-3)
 * @return true if points lie in the expected plane within tolerance
 *
 * Useful for debugging non-coplanar transfers and validating orbital calculations
 */
inline bool verifyOrbitalPlane(const std::vector<glm::vec3>& points,
                               const glm::vec3& expected_normal,
                               float tolerance = 1e-3f) {
    if (points.size() < 3) return true;

    glm::vec3 v1 = points[1] - points[0];
    glm::vec3 v2 = points[2] - points[0];
    glm::vec3 actual_normal = glm::normalize(glm::cross(v1, v2));

    // Check if normals are aligned (or opposite)
    float dot = glm::abs(glm::dot(actual_normal, expected_normal));
    return dot > (1.0f - tolerance);
}

} // namespace CoordinateSystem

#endif // COORDINATE_SYSTEM_HPP
