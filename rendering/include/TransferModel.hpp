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
                                std::vector<double> impulseMagnitudes);
    void setThreeImpulseTransfer(double initialRadius, double initialInclination,
        double targetRadius, double targetInclination,
        double initialTrueAnomaly, double finalTrueAnomaly, double final_true_anomaly);
    void render(const Shader& shader, const glm::mat4& view_projection, 
                const glm::vec3& color, float animation_progress = 1.0f);

    const std::vector<glm::vec3>& getImpulsePositions() const;
    const std::vector<glm::vec3>& getImpulseDirections() const;
    const std::vector<float>& getImpulseMagnitudes() const;
    

private:
    // Transfer parameters
    double initial_radius_;
    double target_radius_;
    double initial_inclination_;
    double target_inclination_;
    double initial_true_anomaly_;
    double final_true_anomaly_;
    
    // OpenGL objects
    GLuint vao_, vbo_;
    bool initialized_;
    
    // Transfer points and impulse data
    std::vector<glm::vec3> transfer_points_;
    std::vector<glm::vec3> impulse_positions_;
    std::vector<glm::vec3> impulse_directions_;
    std::vector<float> impulse_magnitudes_;

    void generateTransferTrajectory(std::vector<double> impulseMagnitudes);
    glm::vec3 calculateOrbitVelocity(float radius, float inclination, float true_anomaly);
    glm::vec3 calculateOrbitPosition(float radius, float inclination, float true_anomaly);
    void updateBuffers();
};