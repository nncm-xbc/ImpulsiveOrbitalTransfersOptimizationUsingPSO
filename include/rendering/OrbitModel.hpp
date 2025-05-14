class OrbitModel {
public:
    OrbitModel();
    ~OrbitModel();

    void initialize();

    // Set orbital parameters
    void setOrbit(double radius, double inclination, double raan = 0.0,
                 double eccentricity = 0.0, double arg_periapsis = 0.0);

    // Get position and velocity at a given true anomaly (for transfer calculations)
    glm::vec3 getPosition(double true_anomaly) const;
    glm::vec3 getVelocity(double true_anomaly) const;

    void render(const Shader& shader, const glm::mat4& view_projection,
                const glm::vec3& color);

    // Accessors
    double getRadius() const { return radius_; }
    double getInclination() const { return inclination_; }
    double getRaan() const { return raan_; }
    double getEccentricity() const { return eccentricity_; }
    double getArgPeriapsis() const { return arg_periapsis_; }

private:
    // Orbital elements
    double radius_;
    double inclination_;
    double raan_;
    double eccentricity_;
    double arg_periapsis_;

    // OpenGL objects
    GLuint vao_, vbo_;

    // Orbit points for rendering
    std::vector<glm::vec3> orbit_points_;

    // Internal methods
    void generateOrbitPoints(int resolution = 200);
    void updateBuffers();
};
