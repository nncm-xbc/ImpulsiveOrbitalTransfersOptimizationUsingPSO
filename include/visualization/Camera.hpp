/**
 * @file Camera.hpp
 * @brief 3D camera system for orbital visualization
 * @author PSO Orbit Transfer Team
 * @date 2025
 *
 * Implements a spherical coordinate camera system optimized for viewing
 * orbital mechanics simulations. Provides smooth navigation around
 * central bodies and orbital trajectories.
 */

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "visualization/Shader.hpp"

/**
 * @class Camera
 * @brief 3D camera controller for orbital visualization
 *
 * Implements a spherical coordinate camera system that orbits around
 * a central point (typically the central body). Features include:
 * - Spherical coordinate navigation (azimuth, elevation, distance)
 * - Smooth zoom controls with distance constraints
 * - Mouse-based rotation controls
 * - View and projection matrix generation
 * - Debug coordinate axis rendering
 * - Position display utilities
 *
 * The camera is designed specifically for orbital mechanics visualization
 * where the primary interest is viewing orbits and transfers around
 * a central gravitational body.
 */
class Camera {
    public:
        /**
         * @brief Constructor with initial distance
         * @param distance Initial distance from target (default 50000.0)
         *
         * Creates a camera positioned at the specified distance from
         * the origin, looking toward the center. Initial orientation
         * provides a good overview of typical orbital scenarios.
         */
        Camera(float distance = 50000.0f);

        /**
         * @brief Rotate camera around target point
         * @param delta_azimuth Change in azimuth angle (horizontal rotation)
         * @param delta_elevation Change in elevation angle (vertical rotation)
         *
         * Updates camera orientation using spherical coordinates.
         * Elevation is clamped to prevent camera from flipping over.
         * Positive azimuth rotates counterclockwise when viewed from above.
         */
        void rotate(float delta_azimuth, float delta_elevation);

        /**
         * @brief Zoom camera in or out
         * @param delta Zoom delta (positive = zoom in, negative = zoom out)
         *
         * Adjusts camera distance from target with constraints to prevent
         * getting too close (minimum 10000) or too far (maximum 100000).
         * Distance changes are scaled for smooth user interaction.
         */
        void zoom(float delta);

        /**
         * @brief Generate view matrix for rendering
         * @return 4x4 view matrix for transforming world to camera space
         *
         * Creates a look-at view matrix based on current camera position,
         * target point, and up vector. Used by rendering pipeline to
         * transform 3D world coordinates to camera view space.
         */
        glm::mat4 getViewMatrix() const;

        /**
         * @brief Generate projection matrix for rendering
         * @param aspect_ratio Screen width/height ratio
         * @return 4x4 perspective projection matrix
         *
         * Creates a perspective projection matrix with 45-degree field of view
         * and near/far planes optimized for orbital scales (0.1 to 200000).
         */
        glm::mat4 getProjectionMatrix(float aspect_ratio) const;

        /**
         * @brief Get current camera position in world coordinates
         * @return Camera position vector
         */
        glm::vec3 getPosition() const;

        /**
         * @brief Set up shaders for coordinate axis rendering
         * @param shader Shader object to configure
         *
         * Initializes shader program for rendering debug coordinate axes.
         * Sets up vertex and fragment shaders with appropriate uniforms
         * for colored line rendering.
         */
        void setupCoordinateShaders(Shader& shader);

        /**
         * @brief Set up OpenGL buffers for coordinate axes
         * @param vao Reference to vertex array object
         * @param vbo Reference to vertex buffer object
         *
         * Creates OpenGL vertex arrays and buffers for rendering
         * X, Y, Z coordinate axes in different colors (red, green, blue).
         * Axes extend from origin to help visualize orientation.
         */
        void setupCoordinateAxes(GLuint& vao, GLuint& vbo);

        /**
         * @brief Render coordinate axes for debugging
         * @param shader Configured shader for axis rendering
         * @param viewProjection Combined view-projection matrix
         * @param vao Vertex array object containing axis geometry
         *
         * Draws colored coordinate axes to help users understand
         * spatial orientation during orbital visualization.
         */
        void renderCoordinateAxes(const Shader& shader, const glm::mat4& viewProjection, GLuint vao);

        /**
         * @brief Display current camera position in console
         * @param camera Camera object to display
         *
         * Prints current camera coordinates to console with formatting.
         * Useful for debugging camera movements and recording viewpoints.
         */
        void displayCameraPosition(const Camera& camera);

        /**
         * @brief Set camera position directly
         * @param position New camera position vector
         *
         * Directly sets camera position without updating spherical
         * coordinates. Use with caution as it may break the spherical
         * coordinate system consistency.
         */
        void setPosition(const glm::vec3& position);

        /**
         * @brief Set camera target point
         * @param target New target position vector
         *
         * Changes the point the camera looks at. Typically kept at
         * origin for orbital mechanics visualization.
         */
        void setTarget(const glm::vec3& target);

        /**
         * @brief Set zoom distance directly
         * @param zoom New distance from target
         *
         * Directly sets camera distance and updates position.
         * Applies same constraints as zoom() method.
         */
        void setZoom(float zoom);

    private:
        /** @brief Distance from camera to target point */
        float distance_;

        /** @brief Azimuth angle in spherical coordinates (radians) */
        float azimuth_;

        /** @brief Elevation angle in spherical coordinates (radians) */
        float elevation_;

        /** @brief Calculated camera position in world coordinates */
        glm::vec3 position_;

        /** @brief Target point camera looks at */
        glm::vec3 target_;

        /** @brief Camera up vector */
        glm::vec3 up_;

        /**
         * @brief Update camera position from spherical coordinates
         *
         * Converts spherical coordinates (distance, azimuth, elevation)
         * to Cartesian coordinates and updates position vector.
         * Called automatically when spherical parameters change.
         */
        void updatePosition();
    };

#endif // CAMERA_HPP
