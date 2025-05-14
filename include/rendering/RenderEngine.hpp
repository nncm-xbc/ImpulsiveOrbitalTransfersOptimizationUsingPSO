class RenderEngine {
public:
    RenderEngine();
    ~RenderEngine();

    bool initialize(int window_width, int window_height);

    // Set data from PSO results
    void setInitialOrbit(double radius, double inclination, double raan = 0.0,
                         double eccentricity = 0.0, double arg_periapsis = 0.0);
    void setTargetOrbit(double radius, double inclination, double raan = 0.0,
                        double eccentricity = 0.0, double arg_periapsis = 0.0);

    void setTransferTrajectory(const double* optimal_parameters);

    // Specific setter for two-impulse transfer
    void setTwoImpulseTransfer(double initial_true_anomaly, double final_true_anomaly,
                              double transfer_time);

    void setThreeImpulseTransfer(double initial_true_anomaly, double intermediate_radius,
                                double final_true_anomaly);

    // Main rendering methods
    void update(double delta_time);
    void render();

    // Camera control
    void rotateCamera(float delta_x, float delta_y);
    void zoomCamera(float delta);
    void resetCamera();

    // Animation control
    void startAnimation();
    void pauseAnimation();
    void resetAnimation();
    void setAnimationSpeed(float speed);

private:
    // Internal models
    OrbitModel initial_orbit_;
    OrbitModel target_orbit_;
    TransferModel transfer_trajectory_;
    ImpulseVisualization impulse_visualizer_;

    // Rendering components
    Camera camera_;
    Animation animation_;

    // Shaders
    Shader orbit_shader_;
    Shader transfer_shader_;
    Shader impulse_shader_;

    // Window state
    int window_width_, window_height_;
    bool is_initialized_;

    // Helper methods
    void setupModels();
    void calculateTransferTrajectory();
    void calculateImpulses();
};
