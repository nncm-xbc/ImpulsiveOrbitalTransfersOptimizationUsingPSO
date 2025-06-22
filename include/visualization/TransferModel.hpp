#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <cmath>

#include "visualization/Shader.hpp"

struct IntersectionPoint {
    glm::vec3 position;
    double true_anomaly;
    double radius;
    double distance_to_target;
    bool is_valid;
};

class TransferModel {
public:
    TransferModel();
    ~TransferModel();

    void initialize();
    void setTwoImpulseTransfer(double initialRadius, double initialInclination,
                                double targetRadius, double targetInclination,
                                double initialEccentricity, double targetEccentricity,
                                double initialTrueAnomaly, double finalTrueAnomaly,
                                std::vector<double> impulseMagnitudes, std::vector<double> planeChange);
    void render(const Shader& shader, const glm::mat4& view_projection,
                const glm::vec3& color, float animation_progress = 1.0f);
    const std::vector<glm::vec3>& getImpulsePositions() const;
    const std::vector<glm::vec3>& getImpulseDirections() const;
    const std::vector<double>& getImpulseMagnitudes() const;
    void renderCompleteEllipse(const Shader& shader, const glm::mat4& view_projection, const glm::vec3& color);
    glm::vec3 calculateRenderedCenter();
    void debugTargetPosition();

    std::vector<IntersectionPoint> findOrbitPlaneIntersections(
        const glm::vec3& target_plane_normal,
        double target_radius,
        double tolerance = 1e-3
    ) const;
    void correctTransferToActualIntersection();
    void generateCorrectedTransferTrajectory();
    void renderCorrectedTransfer(const Shader& shader, const glm::mat4& view_projection,
                                              const glm::vec3& color);
    void updateCorrectedBuffers();

private:
    // Transfer parameters
    double initial_radius_;
    double target_radius_;
    double initial_inclination_;
    double target_inclination_;
    double initial_eccentricity_;
    double target_eccentricity_;
    double initial_true_anomaly_;
    double final_true_anomaly_;
    float transfer_semi_major_;
    float transfer_eccentricity_;

    // OpenGL objects
    GLuint vao_, vbo_;
    bool initialized_;
    GLuint corrected_vao_, corrected_vbo_;
    bool has_corrected_transfer_;
    float corrected_nu_diff_;

    // Transfer points and impulse data
    std::vector<glm::vec3> transfer_points_;
    std::vector<glm::vec3> corrected_transfer_points_;
    std::vector<glm::vec3> impulse_positions_;
    std::vector<glm::vec3> impulse_directions_;
    std::vector<glm::vec3> complete_ellipse_points_;
    std::vector<glm::vec3> corrected_ellipse_points_;
    std::vector<double> impulse_magnitudes_;
    std::vector<double> plane_change_;

    void generateTransferTrajectory();
    void updateBuffers();
};
