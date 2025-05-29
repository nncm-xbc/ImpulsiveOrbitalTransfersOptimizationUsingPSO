#ifndef SHADER_HPP
#define SHADER_HPP
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <string>

class Shader {
public:
    Shader();

    // Load and compile shaders
    bool loadFromFiles(const std::string& vertex_path, const std::string& fragment_path);
    bool loadFromSource(const std::string& vertex_source, const std::string& fragment_source);

    void use() const;

    // Set uniforms
    void setBool(const std::string &name, bool value) const;
    void setInt(const std::string &name, int value) const;
    void setFloat(const std::string &name, float value) const;
    void setVec2(const std::string &name, const glm::vec2 &value) const;
    void setVec3(const std::string &name, const glm::vec3 &value) const;
    void setVec4(const std::string &name, const glm::vec4 &value) const;
    void setMat2(const std::string &name, const glm::mat2 &mat) const;
    void setMat3(const std::string &name, const glm::mat3 &mat) const;
    void setMat4(const std::string &name, const glm::mat4 &mat) const;

private:
    unsigned int ID;

    // Helper methods
    bool checkCompileErrors(unsigned int shader, std::string type);
    bool compileShader(GLuint& shader, const std::string& source, GLenum type);
    bool linkProgram(GLuint vertex_shader, GLuint fragment_shader);
};

#endif // SHADER_HPP
