#include "glad/glad.h"
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <iomanip>
#include <sstream>

#include "Shader.hpp"
#include "Camera.hpp"

Camera::Camera(float distance) : 
    distance_(distance), 
    azimuth_(0.0f),
    elevation_(0.0f),
    target_(0.0f, 0.0f, 0.0f),
    up_(0.0f, 1.0f, 0.0f) {
    updatePosition();
}

void Camera::rotate(float delta_azimuth, float delta_elevation) {
    azimuth_ += delta_azimuth;
    elevation_ = glm::clamp(elevation_ + delta_elevation, -1.5f, 1.5f);
    updatePosition();
}

void Camera::zoom(float delta) {
    distance_ = glm::clamp(distance_ - delta * 2000.0f, 10000.0f, 100000.0f);
    updatePosition();
}

glm::mat4 Camera::getViewMatrix() const {
    return glm::lookAt(position_, target_, up_);
}

glm::mat4 Camera::getProjectionMatrix(float aspect_ratio) const {
    return glm::perspective(glm::radians(45.0f), aspect_ratio, 100.0f, 500000.0f);
}

glm::vec3 Camera::getPosition() const {
    return position_;
}

void Camera::setupCoordinateShaders(Shader& shader) {
    const char* vertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec3 aColor;
        
        uniform mat4 viewProjection;
        
        out vec3 vertexColor;
        
        void main() {
            gl_Position = viewProjection * vec4(aPos, 1.0);
            vertexColor = aColor;
        }
    )";
    
    const char* fragmentShaderSource = R"(
        #version 330 core
        in vec3 vertexColor;
        out vec4 FragColor;
        
        void main() {
            FragColor = vec4(vertexColor, 1.0);
        }
    )";
    
    shader.loadFromSource(vertexShaderSource, fragmentShaderSource);
}

void Camera::setupCoordinateAxes(GLuint& vao, GLuint& vbo) {
    // Vertex data for the three axes with colors
    float vertices[] = {
        // positions          // colors
        0.0f, 0.0f, 0.0f,     1.0f, 0.0f, 0.0f,  // X-axis start
        10000.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,  // X-axis end
        
        0.0f, 0.0f, 0.0f,     0.0f, 1.0f, 0.0f,  // Y-axis start
        0.0f, 10000.0f, 0.0f, 0.0f, 1.0f, 0.0f,  // Y-axis end
        
        0.0f, 0.0f, 0.0f,     0.0f, 0.0f, 1.0f,  // Z-axis start
        0.0f, 0.0f, 10000.0f, 0.0f, 0.0f, 1.0f   // Z-axis end
    };
    
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    
    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    // Color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void Camera::renderCoordinateAxes(const Shader& shader, const glm::mat4& viewProjection, GLuint vao) {
    shader.use();
    shader.setMat4("viewProjection", viewProjection);
    
    glBindVertexArray(vao);
    glLineWidth(3.0f);
    glDrawArrays(GL_LINES, 0, 6);
    glLineWidth(1.0f);
    glBindVertexArray(0);
}

void Camera::displayCameraPosition(const Camera& camera) {
    // Get camera position
    glm::vec3 pos = camera.getPosition();
    
    std::cout << "Camera position: " 
                << std::fixed << std::setprecision(1) 
                << pos.x << ", " << pos.y << ", " << pos.z << "\r";
    std::cout.flush();
}

float distance_;
float azimuth_;
float elevation_;
glm::vec3 position_;
glm::vec3 target_;
glm::vec3 up_;

void Camera::updatePosition() {
    float x = distance_ * cos(elevation_) * sin(azimuth_);
    float y = distance_ * sin(elevation_);
    float z = distance_ * cos(elevation_) * cos(azimuth_);
    position_ = glm::vec3(x, y, z);
}

void Camera::setPosition(const glm::vec3& position) {
    position_ = position;
}

void Camera::setTarget(const glm::vec3& target) {
    target_ = target;
}

void Camera::setZoom(float zoom) {
    distance_ = zoom;
    updatePosition();
}