class TransferModel {
public:
    TransferModel();
    ~TransferModel();

    void initialize();

    // Set transfer parameters for two-impulse transfer
void setTwoImpulseTransfer(const OrbitModel& initial_orbit,
                              const OrbitModel& target_orbit,
                              double initial_true_anomaly,
                              double final_true_anomaly,
                              double transfer_time);

    // Set transfer parameters for three-impulse transfer
    void setThreeImpulseTransfer(const OrbitModel& initial_orbit,
                                const OrbitModel& target_orbit,
                                double initial_true_anomaly,
                                double intermediate_radius,
                                double final_true_anomaly);

    // Generate points along the transfer trajectory
    void generateTrajectoryPoints(int resolution = 200);

    // Calculate impulse locations and vectors
    void calculateImpulses();

    // Render the transfer trajectory
    void render(const Shader& shader, const glm::mat4& view_projection,
                const glm::vec3& color, float animation_progress);

    // Get impulse data for visualization
    const std::vector<glm::vec3>& getImpulsePositions() const { return impulse_positions_; }
    const std::vector<glm::vec3>& getImpulseVectors() const { return impulse_vectors_; }
    const std::vector<float>& getImpulseMagnitudes() const { return impulse_magnitudes_; }

private:
    // Transfer parameters
    double initial_true_anomaly_;
    double final_true_anomaly_;
    double transfer_time_;
    double intermediate_radius_; // For three-impulse transfers [[3]](https://poe.com/citation?message_id=389522500747&citation=3)

    // Transfer arcs (Keplerian elliptic arcs) [[4]](https://poe.com/citation?message_id=389522500747&citation=4)[[5]](https://poe.com/citation?message_id=389522500747&citation=5)
    struct TransferArc {
        double semi_major_axis;
        double eccentricity;
        double inclination;
        double raan;
        double arg_periapsis;
        double start_true_anomaly;
        double end_true_anomaly;
        std::vector<glm::vec3> points;
    };

    std::vector<TransferArc> transfer_arcs_;

    // Impulse data
    std::vector<glm::vec3> impulse_positions_;
    std::vector<glm::vec3> impulse_vectors_;
    std::vector<float> impulse_magnitudes_;

    // OpenGL objects
    GLuint vao_, vbo_;

    // References to orbits
    const OrbitModel* initial_orbit_;
    const OrbitModel* target_orbit_;

    // Internal methods
    void calculateTransferOrbit(bool is_first_arc);
    void updateBuffers();

    // Helper methods to calculate orbital elements from position and velocity
    // Based on Appendix A referenced in [[6]](https://poe.com/citation?message_id=389522500747&citation=6)
    TransferArc calculateOrbitalElements(const glm::vec3& position, const glm::vec3& velocity);
};
