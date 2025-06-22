/**
 * @file Shader.hpp
 * @brief OpenGL shader program management and utilities
 * @author PSO Orbit Transfer Team
 * @date 2025
 *
 * Provides a comprehensive shader management system for loading,
 * compiling, and using OpenGL shader programs. Includes utilities
 * for setting uniform variables and error handling.
 */

#ifndef SHADER_HPP
#define SHADER_HPP

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <string>

/**
 * @class Shader
 * @brief OpenGL shader program wrapper with utility functions
 *
 * Encapsulates OpenGL shader program creation, compilation, and usage.
 * Features include:
 * - Loading shaders from files or source strings
 * - Automatic compilation and linking with error reporting
 * - Convenient uniform variable setting for common types
 * - Resource management and cleanup
 * - Comprehensive error checking and reporting
 *
 * The class handles the complete shader pipeline from source code
 * to executable GPU programs, making it easy to integrate custom
 * shaders into the orbital visualization system.
 */
class Shader {
public:
    /**
     * @brief Default constructor
     *
     * Creates an uninitialized shader object. Must call loadFromFiles()
     * or loadFromSource() before using the shader for rendering.
     */
    Shader();

    /**
     * @brief Load and compile shaders from files
     * @param vertex_path Path to vertex shader source file
     * @param fragment_path Path to fragment shader source file
     * @return true if compilation and linking successful, false on error
     *
     * Reads shader source code from files, compiles both vertex and
     * fragment shaders, and links them into a complete program.
     * Provides detailed error messages for compilation or linking failures.
     */
    bool loadFromFiles(const std::string& vertex_path, const std::string& fragment_path);

    /**
     * @brief Load and compile shaders from source strings
     * @param vertex_source Vertex shader source code
     * @param fragment_source Fragment shader source code
     * @return true if compilation and linking successful, false on error
     *
     * Compiles shaders directly from provided source strings without
     * file I/O. Useful for embedded shaders or dynamically generated
     * shader code.
     */
    bool loadFromSource(const std::string& vertex_source, const std::string& fragment_source);

    /**
     * @brief Activate this shader program for rendering
     *
     * Makes this shader program the active one for subsequent draw calls.
     * Must be called before setting uniforms or drawing geometry.
     */
    void use() const;

    // ========================================
    // UNIFORM VARIABLE SETTERS
    // ========================================

    /**
     * @brief Set boolean uniform variable
     * @param name Uniform variable name in shader
     * @param value Boolean value to set
     */
    void setBool(const std::string &name, bool value) const;

    /**
     * @brief Set integer uniform variable
     * @param name Uniform variable name in shader
     * @param value Integer value to set
     */
    void setInt(const std::string &name, int value) const;

    /**
     * @brief Set float uniform variable
     * @param name Uniform variable name in shader
     * @param value Float value to set
     */
    void setFloat(const std::string &name, float value) const;

    /**
     * @brief Set 2D vector uniform variable
     * @param name Uniform variable name in shader
     * @param value 2D vector value to set
     */
    void setVec2(const std::string &name, const glm::vec2 &value) const;

    /**
     * @brief Set 3D vector uniform variable
     * @param name Uniform variable name in shader
     * @param value 3D vector value to set
     *
     * Commonly used for colors, positions, and normals in orbital visualization
     */
    void setVec3(const std::string &name, const glm::vec3 &value) const;

    /**
     * @brief Set 4D vector uniform variable
     * @param name Uniform variable name in shader
     * @param value 4D vector value to set
     */
    void setVec4(const std::string &name, const glm::vec4 &value) const;

    /**
     * @brief Set 2x2 matrix uniform variable
     * @param name Uniform variable name in shader
     * @param mat 2x2 matrix value to set
     */
    void setMat2(const std::string &name, const glm::mat2 &mat) const;

    /**
     * @brief Set 3x3 matrix uniform variable
     * @param name Uniform variable name in shader
     * @param mat 3x3 matrix value to set
     */
    void setMat3(const std::string &name, const glm::mat3 &mat) const;

    /**
     * @brief Set 4x4 matrix uniform variable
     * @param name Uniform variable name in shader
     * @param mat 4x4 matrix value to set
     *
     * Most commonly used for transformation matrices (model, view, projection)
     * in 3D orbital visualization.
     */
    void setMat4(const std::string &name, const glm::mat4 &mat) const;

private:
    /** @brief OpenGL shader program ID */
    unsigned int ID;

    /**
     * @brief Check shader compilation and program linking errors
     * @param shader Shader or program object to check
     * @param type Type of object ("VERTEX", "FRAGMENT", or "PROGRAM")
     * @return true if no errors found, false if errors detected
     *
     * Comprehensive error checking for shader compilation and program
     * linking stages. Prints detailed error messages including line
     * numbers and descriptions to help with shader debugging.
     */
    bool checkCompileErrors(unsigned int shader, std::string type);

    /**
     * @brief Compile individual shader from source
     * @param shader Reference to shader object ID
     * @param source Shader source code
     * @param type OpenGL shader type (GL_VERTEX_SHADER, GL_FRAGMENT_SHADER)
     * @return true if compilation successful, false on error
     *
     * Helper method for compiling individual vertex or fragment shaders.
     * Handles OpenGL shader object creation and source code compilation.
     */
    bool compileShader(GLuint& shader, const std::string& source, GLenum type);

    /**
     * @brief Link compiled shaders into final program
     * @param vertex_shader Compiled vertex shader ID
     * @param fragment_shader Compiled fragment shader ID
     * @return true if linking successful, false on error
     *
     * Links vertex and fragment shaders into a complete executable
     * program and handles cleanup of intermediate shader objects.
     */
    bool linkProgram(GLuint vertex_shader, GLuint fragment_shader);
};

#endif // SHADER_HPP
