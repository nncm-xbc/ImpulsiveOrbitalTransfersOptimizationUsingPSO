#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>

#include "Shader.hpp"
#include "ImpulseModel.hpp"


ImpulseVisualization::ImpulseVisualization() : vao_(0), vbo_(0), ebo_(0), initialized_(false) {}

ImpulseVisualization::~ImpulseVisualization() {
    if (initialized_) {
        glDeleteVertexArrays(1, &vao_);
        glDeleteBuffers(1, &vbo_);
        glDeleteBuffers(1, &ebo_);
    }
}

void ImpulseVisualization::initialize() {
    if (initialized_) return;
    
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &ebo_);
    
    // Create arrow geometry
    generateArrowGeometry();
    
    initialized_ = true;
}

void ImpulseVisualization::setImpulses(const std::vector<glm::vec3>& positions,
                const std::vector<glm::vec3>& directions,
                const std::vector<float>& magnitudes) {
    positions_ = positions;
    directions_ = directions;
    magnitudes_ = magnitudes;
}

void ImpulseVisualization::render(const Shader& shader, const glm::mat4& view_projection, float animation_progress) {
    if (!initialized_ || positions_.empty()) return;
    
    shader.use();
    
    // Only render visible impulses based on animation progress
    int visible_impulses = 1; // Always show first impulse
    if (animation_progress > 0.9 && positions_.size() > 1) {
        visible_impulses = positions_.size(); // Show all impulses near end of animation
    }
    
    // Render each impulse arrow
    for (int i = 0; i < visible_impulses && i < positions_.size(); ++i) {
        // Scale and orient the arrow based on impulse vector
        glm::vec3 direction = glm::normalize(directions_[i]);
        float magnitude = magnitudes_[i] * 0.001f; // Scale for visibility
        
        // Create model matrix for the arrow
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, positions_[i]);
        
        // Rotate to align with impulse direction
        glm::vec3 up(0.0f, 1.0f, 0.0f);
        if (std::abs(glm::dot(direction, up)) > 0.999f) {
            up = glm::vec3(1.0f, 0.0f, 0.0f); // Use different up vector if parallel
        }
        
        glm::vec3 right = glm::normalize(glm::cross(direction, up));
        up = glm::normalize(glm::cross(right, direction));
        
        // Create rotation matrix
        model[0] = glm::vec4(right, 0.0f);
        model[1] = glm::vec4(up, 0.0f);
        model[2] = glm::vec4(direction, 0.0f);
        
        // Scale arrow
        model = glm::scale(model, glm::vec3(magnitude, magnitude, magnitude));
        
        // Set uniforms
        shader.setMat4("model", model);
        shader.setMat4("viewProjection", view_projection);
        shader.setVec3("color", i == 0 ? glm::vec3(0.0f, 1.0f, 0.3f) : glm::vec3(1.0f, 0.8f, 0.0f));
        
        // Render arrow
        glBindVertexArray(vao_);
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
}

void ImpulseVisualization::generateArrowGeometry() {
    // Create a simple arrow shape (cone + cylinder)
    std::vector<glm::vec3> vertices;
    std::vector<unsigned int> indices;
    
    // Cylinder base
    const int segments = 8;
    const float radius = 0.1f;
    const float shaft_length = 0.7f;
    const float head_length = 0.3f;
    const float head_radius = 0.2f;
    
    // Base of cylinder (at origin)
    vertices.push_back(glm::vec3(0.0f, 0.0f, 0.0f)); // Center
    for (int i = 0; i < segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        vertices.push_back(glm::vec3(radius * cos(angle), radius * sin(angle), 0.0f));
    }
    
    // Top of cylinder (end of shaft)
    vertices.push_back(glm::vec3(0.0f, 0.0f, shaft_length)); // Center
    for (int i = 0; i < segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        vertices.push_back(glm::vec3(radius * cos(angle), radius * sin(angle), shaft_length));
    }
    
    // Base of cone (same as top of cylinder)
    for (int i = 0; i < segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        vertices.push_back(glm::vec3(head_radius * cos(angle), head_radius * sin(angle), shaft_length));
    }
    
    // Tip of cone (arrow tip)
    vertices.push_back(glm::vec3(0.0f, 0.0f, shaft_length + head_length));
    
    // Indices for cylinder (triangles connecting base to top)
    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        
        // Bottom circle
        indices.push_back(0); // Center
        indices.push_back(i + 1);
        indices.push_back(next + 1);
        
        // Top circle
        indices.push_back(segments + 1); // Center
        indices.push_back(segments + 1 + next);
        indices.push_back(segments + 1 + i);
        
        // Side quads (2 triangles each)
        indices.push_back(i + 1);
        indices.push_back(segments + 1 + i);
        indices.push_back(next + 1);
        
        indices.push_back(next + 1);
        indices.push_back(segments + 1 + i);
        indices.push_back(segments + 1 + next);
    }
    
    // Indices for cone
    for (int i = 0; i < segments; ++i) {
        int next = (i + 1) % segments;
        
        // Base circle
        indices.push_back(2 * segments + 2); // Center of top cylinder (reuse)
        indices.push_back(2 * segments + 2 + next);
        indices.push_back(2 * segments + 2 + i);
        
        // Side triangles
        indices.push_back(2 * segments + 2 + i);
        indices.push_back(2 * segments + 2 + next);
        indices.push_back(3 * segments + 2); // Tip
    }
    
    // Upload to GPU
    glBindVertexArray(vao_);
    
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), vertices.data(), GL_STATIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);
    
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);
    
    glBindVertexArray(0);
}