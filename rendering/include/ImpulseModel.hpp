#ifndef IMPULSEMODEL_HPP
#define IMPULSEMODEL_HPP

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>

#include "Shader.hpp"

class ImpulseVisualization {
    public: 
        ImpulseVisualization();
        ~ImpulseVisualization();

        void initialize();
        void setImpulses(const std::vector<glm::vec3>& positions,
                         const std::vector<glm::vec3>& directions,
                         const std::vector<double>& magnitudes);
        void render(const Shader& shader, const glm::mat4& view_projection, float animation_progress = 1.0f);

    private:
        // Impulse data
        std::vector<glm::vec3> positions_;
        std::vector<glm::vec3> directions_;
        std::vector<double> magnitudes_;

        // OpenGL objects
        GLuint vao_, vbo_, ebo_;
        bool initialized_;

        void generateArrowGeometry();
};

#endif // IMPULSEMODEL_HPP