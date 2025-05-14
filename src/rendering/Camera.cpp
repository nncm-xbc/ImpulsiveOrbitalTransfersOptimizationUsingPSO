class Camera {
private:
    glm::vec3 _position;
    glm::vec3 _target;
    glm::vec3 _up;
    float _fov, _aspectRatio;

public:
    Camera();
    void setViewport(int width, int height);
    void zoom(float factor);
    void rotate(float yaw, float pitch);
    void pan(float x, float y);
    glm::mat4 getViewMatrix() const;
    glm::mat4 getProjectionMatrix() const;
};
