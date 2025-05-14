class Shader {
public:
    Shader();
    ~Shader();

    // Load and compile shaders
    bool loadFromFiles(const std::string& vertex_path, const std::string& fragment_path);
    bool loadFromSource(const std::string& vertex_source, const std::string& fragment_source);

    // Use this shader
    void use() const;

    // Set uniforms
    void setMat4(const std::string& name, const glm::mat4& matrix) const;
    void setVec3(const std::string& name, const glm::vec3& vector) const;
    void setVec4(const std::string& name, const glm::vec4& vector) const;
    void setFloat(const std::string& name, float value) const;
    void setInt(const std::string& name, int value) const;

private:
    GLuint program_id_;

    // Helper methods
    bool compileShader(GLuint& shader, const std::string& source, GLenum type);
    bool linkProgram(GLuint vertex_shader, GLuint fragment_shader);
};
