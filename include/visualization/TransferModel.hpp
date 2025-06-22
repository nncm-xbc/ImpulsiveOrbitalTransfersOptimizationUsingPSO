/**
 * @file TransferModel.hpp
 * @brief 3D orbital transfer trajectory rendering and visualization
 * @author PSO Orbit Transfer Team
 * @date 2025
 *
 * Provides sophisticated 3D rendering capabilities for orbital transfer
 * trajectories, including two-impulse transfers between circular and
 * elliptical orbits with support for plane changes and animation.
 */

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <cmath>

#include "visualization/Shader.hpp"

/**
 * @class TransferModel
 * @brief 3D orbital transfer trajectory renderer with animation support
 *
 * Renders complex orbital transfer trajectories in 3D space with support for:
 * - Two-impulse transfers between arbitrary orbits
 * - Coplanar and non-coplanar transfer cases
 * - Complete transfer ellipse visualization
 * - Animated trajectory progression
 * - Impulse position and direction visualization
 * - Real-time parameter updates and debugging
 *
 * The model handles the complete pipeline from transfer parameters to
 * rendered 3D trajectories, including:
 * - Lambert solver integration for trajectory computation
 * - Coordinate system transformations
 * - Efficient OpenGL rendering with vertex buffers
 * - Animation support for time-based visualization
 *
 * This class is specifically designed for visualizing optimized orbital
 * transfers found by PSO algorithms, providing insight into transfer
 * mechanics and solution quality.
 */
class TransferModel {
public:
    /**
     * @brief Default constructor
     *
     * Creates an uninitialized transfer model. Must call initialize()
     * and configure transfer parameters before rendering.
     */
    TransferModel();

    /**
     * @brief Destructor - cleans up OpenGL resources
     *
     * Automatically releases all vertex array and buffer objects
     * allocated for transfer trajectory rendering.
     */
    ~TransferModel();

    /**
     * @brief Initialize OpenGL resources for rendering
     *
     * Creates vertex array objects (VAO) and vertex buffer objects (VBO)
     * required for efficient transfer trajectory rendering. Must be called
     * after OpenGL context creation and before any rendering operations.
     */
    void initialize();

    /**
     * @brief Configure model for two-impulse orbital transfer
     * @param initialRadius Initial orbit radius (DU)
     * @param initialInclination Initial orbit inclination (radians)
     * @param targetRadius Target orbit radius (DU)
     * @param targetInclination Target orbit inclination (radians)
     * @param initialEccentricity Initial orbit eccentricity (typically 0.0)
     * @param targetEccentricity Target orbit eccentricity (typically 0.0)
     * @param initialTrueAnomaly True anomaly at departure (radians)
     * @param finalTrueAnomaly True anomaly at arrival (radians)
     * @param impulseMagnitudes Vector of impulse magnitudes [dv1, dv2] (DU/TU)
     * @param planeChange Vector of plane change angles [angle1, angle2] (radians)
     *
     * Sets up a complete two-impulse transfer between specified orbits.
     * Handles both coplanar and non-coplanar cases automatically based
     * on inclination differences. Generates transfer trajectory using
     * Lambert solver and optimized parameters from PSO.
     *
     * The method performs comprehensive trajectory generation including:
     * - Impulse vector calculations
     * - Transfer orbit determination
     * - Trajectory point generation
     * - Coordinate system transformations
     * - Visual debugging and verification
     */
    void setTwoImpulseTransfer(double initialRadius, double initialInclination,
                                double targetRadius, double targetInclination,
                                double initialEccentricity, double targetEccentricity,
                                double initialTrueAnomaly, double finalTrueAnomaly,
                                std::vector<double> impulseMagnitudes, std::vector<double> planeChange);

    /**
     * @brief Render the transfer trajectory with animation support
     * @param shader Configured shader program for rendering
     * @param view_projection Combined view-projection matrix
     * @param color RGB color for the transfer trajectory
     * @param animation_progress Animation progress from 0.0 to 1.0 (default 1.0)
     *
     * Renders the transfer trajectory as a line strip with optional animation.
     * When animation_progress < 1.0, only renders a portion of the trajectory
     * to create smooth animation effects showing spacecraft progression
     * along the transfer path.
     */
    void render(const Shader& shader, const glm::mat4& view_projection,
                const glm::vec3& color, float animation_progress = 1.0f);

    /**
     * @brief Get impulse application positions
     * @return Vector of 3D positions where impulses are applied
     *
     * Returns the positions along the orbit where impulsive maneuvers
     * occur. Useful for rendering impulse markers or analyzing
     * maneuver locations.
     */
    const std::vector<glm::vec3>& getImpulsePositions() const;

    /**
     * @brief Get impulse direction vectors
     * @return Vector of 3D unit vectors indicating impulse directions
     *
     * Returns the direction vectors for each impulsive maneuver.
     * Can be used to render impulse direction indicators or analyze
     * maneuver efficiency.
     */
    const std::vector<glm::vec3>& getImpulseDirections() const;

    /**
     * @brief Get impulse magnitude values
     * @return Vector of impulse magnitudes in DU/TU
     *
     * Returns the magnitude of each impulsive maneuver for analysis
     * and display purposes.
     */
    const std::vector<double>& getImpulseMagnitudes() const;

    /**
     * @brief Render the complete transfer ellipse
     * @param shader Configured shader program for rendering
     * @param view_projection Combined view-projection matrix
     * @param color RGB color for the complete ellipse
     *
     * Renders the entire transfer orbit ellipse, showing the complete
     * trajectory that would be followed if no second impulse occurred.
     * Useful for understanding transfer mechanics and visualizing
     * the underlying orbital geometry.
     */
    void renderCompleteEllipse(const Shader& shader, const glm::mat4& view_projection,
                                         const glm::vec3& color);

    /**
     * @brief Calculate the geometric center of the rendered transfer
     * @return Center position in visualization coordinates
     *
     * Computes the average position of all transfer points for debugging
     * and verification purposes. Useful for checking coordinate
     * transformations and transfer positioning.
     */
    glm::vec3 calculateRenderedCenter();

    /**
     * @brief Debug target position calculations and transformations
     *
     * Performs comprehensive debugging of target position calculations,
     * including verification of coordinate transformations and true
     * anomaly computations. Prints detailed information to console
     * for troubleshooting transfer visualization issues.
     */
    void debugTargetPosition();

    /**
     * @brief Print header for transfer generation
     *
     * Prints a header to the console indicating the start of transfer
     * generation. Useful for tracking progress and debugging.
     */
    void printTransferGenerationHeader(bool is_coplanar);

    /**
     * @brief Print plane verification information
     *
     * Prints information about the plane verification process.
     */
    void printPlaneVerification(const std::vector<glm::vec3>& points, const glm::vec3& expected_normal,
                               const std::string& orbit_name);

    /**
     * @brief Print transfer completion information
     *
     * Prints information about the completion of the transfer process.
     */
    void printTransferCompletion(size_t transfer_points, size_t ellipse_points);

    /**
     * @brief Print position verification information
     *
     * Prints information about the position verification process.
     */
    void printPositionVerification(const glm::vec3& calculated, const glm::vec3& expected,
                                  const std::string& point_name);

private:
    // ========================================
    // TRANSFER PARAMETERS
    // ========================================

    /** @brief Initial orbit radius (DU) */
    double initial_radius_;

    /** @brief Target orbit radius (DU) */
    double target_radius_;

    /** @brief Initial orbit inclination (radians) */
    double initial_inclination_;

    /** @brief Target orbit inclination (radians) */
    double target_inclination_;

    /** @brief Initial orbit eccentricity */
    double initial_eccentricity_;

    /** @brief Target orbit eccentricity */
    double target_eccentricity_;

    /** @brief True anomaly at departure (radians) */
    double initial_true_anomaly_;

    /** @brief True anomaly at arrival (radians) */
    double final_true_anomaly_;

    /** @brief Transfer orbit semi-major axis (DU) */
    float transfer_semi_major_;

    /** @brief Transfer orbit eccentricity */
    float transfer_eccentricity_;

    // ========================================
    // OPENGL RENDERING RESOURCES
    // ========================================

    /** @brief OpenGL vertex array object for transfer trajectory */
    GLuint vao_;

    /** @brief OpenGL vertex buffer object for transfer trajectory */
    GLuint vbo_;

    /** @brief Flag indicating if OpenGL resources are initialized */
    bool initialized_;

    // ========================================
    // TRAJECTORY AND IMPULSE DATA
    // ========================================

    /** @brief Vector of 3D points defining the transfer trajectory */
    std::vector<glm::vec3> transfer_points_;

    /** @brief Positions where impulses are applied */
    std::vector<glm::vec3> impulse_positions_;

    /** @brief Direction vectors for each impulse */
    std::vector<glm::vec3> impulse_directions_;

    /** @brief Complete transfer ellipse points for visualization */
    std::vector<glm::vec3> complete_ellipse_points_;

    /** @brief Magnitude of each impulse (DU/TU) */
    std::vector<double> impulse_magnitudes_;

    /** @brief Plane change angles for each impulse (radians) */
    std::vector<double> plane_change_;

    /**
     * @brief Generate complete transfer trajectory from parameters
     *
     * Core method that computes the transfer trajectory from orbital
     * parameters. Handles both coplanar and non-coplanar cases with:
     * - Lambert solver integration for trajectory computation
     * - Transfer orbit element calculation
     * - True anomaly determination and validation
     * - Coordinate system transformations
     * - Complete ellipse generation for visualization
     *
     * This method implements the mathematical heart of transfer
     * visualization, converting optimization results into renderable
     * 3D trajectories.
     */
    void generateTransferTrajectory();

    /**
     * @brief Update OpenGL vertex buffers with current trajectory data
     *
     * Uploads the generated transfer points to GPU memory for efficient
     * rendering. Called automatically when transfer parameters change.
     * Includes error checking and debugging output.
     */
    void updateBuffers();
};
