// CoordinateSystem.hpp - Unified coordinate system for the entire project
#ifndef COORDINATE_SYSTEM_HPP
#define COORDINATE_SYSTEM_HPP

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

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

// Scale factor for visualization
constexpr float VISUAL_SCALE = 1000.0f;

// ============================================
// CONVERSION FUNCTIONS
// ============================================

/**
 * Convert from physics (computational) coordinates to OpenGL visualization
 * This handles both the axis swap and scaling
 */
inline glm::vec3 physicsToVisualization(const glm::vec3& physics_pos) {
    // Apply axis transformation: Physics (X,Y,Z) -> OpenGL (X,Z,Y)
    glm::vec3 opengl_pos(
        physics_pos.x,
        physics_pos.z,
        physics_pos.y
    );

    return opengl_pos * VISUAL_SCALE;
}

/**
 * Convert from physics (double precision) to visualization
 */
inline glm::vec3 physicsToVisualization(const glm::dvec3& physics_pos) {
    return physicsToVisualization(glm::vec3(physics_pos));
}

/**
 * Convert from visualization back to physics coordinates
 * (Useful for debugging or picking)
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
 * Transform a whole trajectory for visualization
 */
inline std::vector<glm::vec3> trajectoryToVisualization(const std::vector<glm::vec3>& physics_points) {
    std::vector<glm::vec3> visual_points;
    visual_points.reserve(physics_points.size());

    for (const auto& point : physics_points) {
        visual_points.push_back(physicsToVisualization(point));
    }

    return visual_points;
}

/**
 * Debug function to print coordinate transformation
 */
inline void debugTransformation(const std::string& label, const glm::vec3& physics_pos) {
    glm::vec3 visual_pos = physicsToVisualization(physics_pos);
    std::cout << label << ":" << std::endl;
    std::cout << "  Physics: (" << physics_pos.x << ", " << physics_pos.y << ", " << physics_pos.z << ")" << std::endl;
    std::cout << "  Visual:  (" << visual_pos.x << ", " << visual_pos.y << ", " << visual_pos.z << ")" << std::endl;
}

// ============================================
// ORBITAL PLANE TRANSFORMATIONS
// ============================================

/**
 * Create rotation matrix for orbital elements in physics coordinates
 * This is the standard orbital mechanics transformation
 */
inline glm::mat3 orbitalToInertialMatrix(float raan, float inclination, float arg_periapsis) {
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
 * Verify that a set of points lie in the expected plane
 * Useful for debugging non-coplanar transfers
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
