#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "Shader.hpp"

class Camera {
    public:
        Camera(float distance = 50000.0f);
        
        void rotate(float delta_azimuth, float delta_elevation);
        void zoom(float delta);
        
        glm::mat4 getViewMatrix() const;
        glm::mat4 getProjectionMatrix(float aspect_ratio) const;
        glm::vec3 getPosition() const;
        
        void setupCoordinateShaders(Shader& shader);
        void setupCoordinateAxes(GLuint& vao, GLuint& vbo);
        void renderCoordinateAxes(const Shader& shader, const glm::mat4& viewProjection, GLuint vao);
        void displayCameraPosition(const Camera& camera);
        void setPosition(const glm::vec3& position);
        void setTarget(const glm::vec3& target);
        void setZoom(float zoom);
        
    private:
        float distance_;
        float azimuth_;
        float elevation_;
        glm::vec3 position_;
        glm::vec3 target_;
        glm::vec3 up_;
        
        void updatePosition();
    };

#endif // CAMERA_HPP