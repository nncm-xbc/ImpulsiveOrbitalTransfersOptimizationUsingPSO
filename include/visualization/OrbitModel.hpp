/**
 * @file OrbitModel.hpp
 * @brief 3D orbital trajectory rendering and visualization
 * @author PSO Orbit Transfer Team
 * @date 2025
 *
 * Provides 3D rendering capabilities for orbital trajectories including
 * circular and elliptical orbits with arbitrary orientation. Handles
 * coordinate transformations and OpenGL rendering.
 */

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <cmath>

#include "visualization/Shader.hpp"

/**
 * @class OrbitModel
 * @brief 3D orbital trajectory renderer
 *
 * Renders orbital trajectories in 3D space with support for:
 * - Circular and elliptical orbits
 * - Arbitrary orbital inclinations and orientations
 * - Real-time parameter updates
 * - Efficient OpenGL rendering with vertex buffers
 * - Coordinate system transformations
 * - Visual debugging and verification
 *
 * The model handles the complete pipeline from orbital elements to
 * rendered 3D trajectories, including coordinate transformations
 * between physics and visualization systems.
 */
class OrbitModel {
    public:

        /**
         * @brief Default constructor
         *
         * Creates an uninitialized orbit model. Must call initialize()
         * and set orbital parameters before rendering.
         */
        OrbitModel();

        /**
         * @brief Destructor - cleans up OpenGL resources
         *
         * Automatically releases vertex array and buffer objects
         * allocated for orbit rendering.
         */
        ~OrbitModel();

        /**
         * @brief Initialize OpenGL resources for rendering
         *
         * Creates vertex array object (VAO) and vertex buffer object (VBO)
         * required for efficient orbit rendering. Must be called after
         * OpenGL context creation and before any rendering operations.
         */
        void initialize();

        /**
         * @brief Configure model for circular orbit
         * @param radius Orbital radius (DU)
         * @param inclination Orbital inclination in radians (default 0.0)
         * @param raan Right ascension of ascending node in radians (default 0.0)
         *
         * Sets up a circular orbit with specified parameters. The orbit
         * lies in a plane defined by inclination and RAAN, with the
         * argument of periapsis set to zero.
         */
        void setCircularOrbit(double radius, double inclination = 0.0, double raan = 0.0);

        /**
         * @brief Configure model for elliptical orbit
         * @param semi_major_axis Semi-major axis of the ellipse (DU)
         * @param eccentricity Orbital eccentricity (0 = circular, <1 = elliptical)
         * @param inclination Orbital inclination in radians (default 0.0)
         * @param raan Right ascension of ascending node in radians (default 0.0)
         * @param arg_periapsis Argument of periapsis in radians (default 0.0)
         *
         * Sets up an elliptical orbit with full orbital element specification.
         * Supports arbitrary orientation and ellipse shape within physical
         * constraints (eccentricity < 1 for bound orbits).
         */
        void setEllipticalOrbit(double semi_major_axis, double eccentricity,
                                double inclination = 0.0, double raan = 0.0,
                                double arg_periapsis = 0.0);

        /**
         * @brief Render the orbital trajectory
         * @param shader Configured shader program for rendering
         * @param view_projection Combined view-projection matrix
         * @param color RGB color for the orbit trajectory
         *
         * Renders the orbital trajectory as a continuous line loop.
         * The orbit is drawn in the specified color using the provided
         * transformation matrices for proper 3D positioning.
         */
        void render(const Shader& shader, const glm::mat4& view_projection, const glm::vec3& color);

        /**
         * @brief Calculate the geometric center of the rendered orbit
         * @return Center position in visualization coordinates
         *
         * Computes the average position of all orbit points for debugging
         * and verification purposes. Useful for checking coordinate
         * transformations and orbit positioning.
         */
        glm::vec3 calculateRenderedCenter();

    private:
        /** @brief Orbital radius (circular) or semi-major axis (elliptical) in DU */
        double radius_;

        /** @brief Orbital eccentricity (0 = circular, <1 = elliptical) */
        double eccentricity_;

        /** @brief Orbital inclination in radians */
        double inclination_;

        /** @brief Right ascension of ascending node in radians */
        double raan_;

        /** @brief Argument of periapsis in radians */
        double arg_periapsis_;

        /** @brief OpenGL vertex array object for efficient rendering */
        GLuint vao_;

        /** @brief OpenGL vertex buffer object containing orbit points */
        GLuint vbo_;

        /** @brief Flag indicating if OpenGL resources are initialized */
        bool initialized_;

        /** @brief Vector of 3D points defining the orbital trajectory */
        std::vector<glm::vec3> orbit_points_;

        /**
         * @brief Generate 3D points along the orbital trajectory
         * @param resolution Number of points to generate (default 200)
         *
         * Calculates positions along the orbit using orbital mechanics
         * equations and transforms them to visualization coordinates.
         * Higher resolution provides smoother curves but uses more memory.
         */
        void generateOrbitPoints(int resolution = 200);

        /**
         * @brief Update OpenGL vertex buffers with current orbit data
         *
         * Uploads the generated orbit points to GPU memory for efficient
         * rendering. Called automatically when orbital parameters change.
         */
        void updateBuffers();
};
