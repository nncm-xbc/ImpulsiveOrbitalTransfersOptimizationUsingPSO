class ImpulseVisualization {
public:
    ImpulseVisualization();
    ~ImpulseVisualization();

    void initialize();

    // Set impulse data
    void setImpulses(const std::vector<glm::vec3>& positions,
                    const std::vector<glm::vec3>& directions,
                    const std::vector<float>& magnitudes);

    // Render the impulses
    void render(const Shader& shader, const glm::mat4& view_projection,
                float animation_progress);

private:
    // Impulse data
    std::vector<glm::vec3> positions_;
    std::vector<glm::vec3> directions_;
    std::vector<float> magnitudes_;

    // OpenGL objects for arrow rendering
    GLuint arrow_vao_, arrow_vbo_;

    // Internal methods
    void renderArrow(const glm::vec3& position, const glm::vec3& direction,
                    float magnitude, const glm::mat4& view_projection,
                    const glm::vec3& color);

    void generateArrowGeometry();
};
