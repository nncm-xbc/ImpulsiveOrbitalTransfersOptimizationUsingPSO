#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <cmath>

#include "Shader.hpp"

class TransferModel {
public:
    TransferModel();
    ~TransferModel();

    void initialize();
    void setTwoImpulseTransfer(double initialRadius, double initialInclination,
                                double targetRadius, double targetInclination,
                                double initialTrueAnomaly, double finalTrueAnomaly, 
                                std::vector<double> impulseMagnitudes, std::vector<double> planeChange);
    void setThreeImpulseTransfer(double initialRadius, double initialInclination,
        double targetRadius, double targetInclination,
        double initialTrueAnomaly, double finalTrueAnomaly, double final_true_anomaly);
    void render(const Shader& shader, const glm::mat4& view_projection, 
                const glm::vec3& color, float animation_progress = 1.0f);

    const std::vector<glm::vec3>& getImpulsePositions() const;
    const std::vector<glm::vec3>& getImpulseDirections() const;
    const std::vector<double>& getImpulseMagnitudes() const;
    void generateCompleteTransferEllipse();
    void renderCompleteEllipse(const Shader& shader, const glm::mat4& view_projection, const glm::vec3& color);

private:
    // Transfer parameters
    double initial_radius_;
    double target_radius_;
    double initial_inclination_;
    double target_inclination_;
    double initial_true_anomaly_;
    double final_true_anomaly_;
    float transfer_semi_major_;
    float transfer_eccentricity_;
    
    // OpenGL objects
    GLuint vao_, vbo_;
    bool initialized_;
    
    // Transfer points and impulse data
    std::vector<glm::vec3> transfer_points_;
    std::vector<glm::vec3> impulse_positions_;
    std::vector<glm::vec3> impulse_directions_;
    std::vector<glm::vec3> complete_ellipse_points_;
    std::vector<double> impulse_magnitudes_;
    std::vector<double> plane_change_;

    void generateTransferTrajectory();
    glm::vec3 calculateOrbitVelocity(float radius, float inclination, float true_anomaly);
    glm::vec3 calculateOrbitPosition(float radius, float inclination, float true_anomaly);
    void updateBuffers();
};