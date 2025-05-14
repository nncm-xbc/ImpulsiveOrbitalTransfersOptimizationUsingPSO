class Camera {
public:
    Camera();

    // Set camera properties
    void setPosition(const glm::vec3& position);
    void setTarget(const glm::vec3& target);
    void setUpVector(const glm::vec3& up);

    // Camera manipulation
    void rotate(float delta_x, float delta_y);
    void zoom(float delta);
    void reset();

    // Get camera matrices
    glm::mat4 getViewMatrix() const;
    glm::mat4 getProjectionMatrix(float aspect_ratio) const;
    glm::mat4 getViewProjectionMatrix(float aspect_ratio) const;

private:
    glm::vec3 position_;
    glm::vec3 target_;
    glm::vec3 up_;

    // Camera parameters
    float distance_;
    float azimuth_;
    float elevation_;

    // Projection parameters
    float fov_;
    float near_plane_;
    float far_plane_;

    // Update camera position based on spherical coordinates
    void updatePosition();
};
