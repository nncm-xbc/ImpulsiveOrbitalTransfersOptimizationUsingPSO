#include "glad/glad.h"
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <iomanip>
#include <sstream>

#include "OrbitModel.hpp"
#include "Shader.hpp"
#include "Camera.hpp"
#include "TransferModel.hpp"
#include "ImpulseModel.hpp"
#include "ImpulseModel.hpp"
#include "Animation.hpp"
#include "Results.hpp"

// class DebugLine {
// private:
//     GLuint vao_, vbo_;
//     bool initialized_;
    
// public:
//     DebugLine() : vao_(0), vbo_(0), initialized_(false) {}
    
//     ~DebugLine() {
//         if (initialized_) {
//             glDeleteVertexArrays(1, &vao_);
//             glDeleteBuffers(1, &vbo_);
//         }
//     }
    
//     void initialize() {
//         if (initialized_) return;
        
//         glGenVertexArrays(1, &vao_);
//         glGenBuffers(1, &vbo_);
        
//         glm::vec3 lineVertices[2] = {
//             glm::vec3(0.0f, 0.0f, 0.0f),    // Origin
//             glm::vec3(0.0f, 0.0f, 10.0f)    // Point along Z-axis
//         };
        
//         glBindVertexArray(vao_);
//         glBindBuffer(GL_ARRAY_BUFFER, vbo_);
//         glBufferData(GL_ARRAY_BUFFER, sizeof(lineVertices), lineVertices, GL_STATIC_DRAW);
        
//         glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
//         glEnableVertexAttribArray(0);
        
//         glBindBuffer(GL_ARRAY_BUFFER, 0);
//         glBindVertexArray(0);
        
//         initialized_ = true;
//     }
    
//     void render(const Shader& shader, const glm::mat4& viewProjection, const glm::vec3& color) {
//         if (!initialized_) return;
        
//         shader.use();
//         shader.setMat4("viewProjection", viewProjection);
//         shader.setVec3("color", color);
        
//         glBindVertexArray(vao_);
//         glDrawArrays(GL_LINES, 0, 2);
//         glBindVertexArray(0);
//     }
// };

int main(int argc, char* argv[]) {
    // Init GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }
    
    // OpenGL context version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
    // Create window
    const int width = 800, height = 600;
    GLFWwindow* window = glfwCreateWindow(width, height, "Orbital Transfer Visualization", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    
    glfwMakeContextCurrent(window);
    
    // Init GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return -1;
    }
    
    glViewport(0, 0, width, height);

    PSOOrbitTransferResult psoResult;
    bool usePSOResults = false;
    
    if (argc > 1) {
        usePSOResults = loadPSOResultsFromFile(argv[1], psoResult);
        if (usePSOResults) {
            std::cout << "Loaded PSO results from " << argv[1] << std::endl;
        }
    }

    // Load shaders
    Shader orbitShader;
    Shader transferShader;
    Shader axesShader;
    Shader impulseShader;

    if (!orbitShader.loadFromFiles("../shaders/orbit.vert", "../shaders/orbit.frag") ||
        !transferShader.loadFromFiles("../shaders/transfer.vert", "../shaders/transfer.frag") ||
        !impulseShader.loadFromFiles("../shaders/impulse.vert", "../shaders/impulse.frag")
    ) {
        std::cerr << "Failed to load shaders" << std::endl;
        return -1;
    }

    Camera camera;
    camera.setZoom(14000.0f); // Initial zoom level
    camera.setPosition(glm::vec3(0.0f, 5.0f, 0.0f)); // Initial camera position
    Animation animation(5.0f);

    OrbitModel initialOrbit;
    OrbitModel targetOrbit;
    TransferModel transferModel;
    ImpulseVisualization impulseModel;

    GLuint axesVAO, axesVBO;

    initialOrbit.initialize();
    targetOrbit.initialize();
    transferModel.initialize();
    impulseModel.initialize();

    if (usePSOResults) {
        if (psoResult.initial_eccentricity < 1e-6) {
            std::cout << " Circular Init Orbit " << std::endl;
            std::cout << "Params: "<< std::endl;
            std::cout << "Init radius:" << psoResult.initial_radius << std::endl;
            std::cout << "Init incl: " << psoResult.initial_inclination << std::endl;
            std::cout << "Init raan: " << psoResult.initial_raan << std::endl;
            initialOrbit.setCircularOrbit(
                psoResult.initial_radius, 
                psoResult.initial_inclination,
                psoResult.initial_raan
            );
        } else {
            initialOrbit.setEllipticalOrbit(
                psoResult.initial_radius,
                psoResult.initial_eccentricity,
                psoResult.initial_inclination,
                psoResult.initial_raan,
                psoResult.initial_arg_periapsis
            );
        }
        
        if (psoResult.target_eccentricity < 1e-6) {
            std::cout << " Circular Target Orbit " << std::endl;
            std::cout << "Params: "<< std::endl;
            std::cout << "Target radius:" << psoResult.target_radius << std::endl;
            std::cout << "Target incl: " << psoResult.target_inclination << std::endl;
            std::cout << "Target raan: " << psoResult.target_raan << std::endl;
            targetOrbit.setCircularOrbit(
                psoResult.target_radius, 
                psoResult.target_inclination,
                psoResult.target_raan
            );
        } else {
            targetOrbit.setEllipticalOrbit(
                psoResult.target_radius,
                psoResult.target_eccentricity,
                psoResult.target_inclination,
                psoResult.target_raan,
                psoResult.target_arg_periapsis
            );
        }
        
        // Set up transfer trajectory
        if (psoResult.is_three_impulse) {
            transferModel.setThreeImpulseTransfer(
                psoResult.initial_radius, psoResult.initial_inclination,
                psoResult.target_radius, psoResult.target_inclination,
                psoResult.initial_true_anomaly, psoResult.intermediate_radius,
                psoResult.final_true_anomaly
            );
        } else {
            transferModel.setTwoImpulseTransfer(
                psoResult.initial_radius, psoResult.initial_inclination,
                psoResult.target_radius, psoResult.target_inclination,
                psoResult.initial_true_anomaly, psoResult.final_true_anomaly,
                //{0.5, 0.5} // TO REMOVE 
                psoResult.delta_v_magnitudes, psoResult.plane_change
            );
        }
    } else {
        // Default case
        initialOrbit.setCircularOrbit(6671.53, 0.0f);
        targetOrbit.setCircularOrbit(26558.56, 0.0f);
        
        transferModel.setTwoImpulseTransfer(
            6671.53, 0.0f, 
            26558.56, 0.0f,
            0.0, M_PI,
            { 0.0, 0.0 }, { 0.0, 0.0 }
        );
    }

    impulseModel.setImpulses(
        transferModel.getImpulsePositions(),
        transferModel.getImpulseDirections(),
        transferModel.getImpulseMagnitudes()
    );

    camera.setupCoordinateShaders(axesShader);
    camera.setupCoordinateAxes(axesVAO, axesVBO);
    
    // Mouse callbacks for camera control
    glfwSetWindowUserPointer(window, &camera);
    
    glfwSetCursorPosCallback(window, [](GLFWwindow* window, double x, double y) {
        static double last_x = x;
        static double last_y = y;
        
        double dx = x - last_x;
        double dy = y - last_y;
        
        last_x = x;
        last_y = y;
        
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
            Camera* camera = static_cast<Camera*>(glfwGetWindowUserPointer(window));
            camera->rotate(dx * 0.01f, dy * 0.01f);
        }
    });
    
    glfwSetScrollCallback(window, [](GLFWwindow* window, double xoffset, double yoffset) {
        Camera* camera = static_cast<Camera*>(glfwGetWindowUserPointer(window));
        camera->zoom(yoffset);
    });

    //DebugLine debugLine;
    //debugLine.initialize();

    std::cout << "Controls:\n"
    << "  Left mouse button + drag: Rotate camera\n"
    << "  Scroll wheel: Zoom in/out\n"
    << "  Space: Play/pause animation\n"
    << "  R: Reset animation\n"
    << "  +/-: Adjust animation speed\n"
    << "  Esc: Exit\n";

    float animation_speed = 1.0f;
    
    // Main rendering loop
    while (!glfwWindowShouldClose(window)) {

        float currentFrame = glfwGetTime();
        static float lastFrame = currentFrame;
        float deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        
        // Process input
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);
        
        // Animation controls
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
            static bool space_pressed = false;
            if (!space_pressed) {
                space_pressed = true;
                if (animation.isPlaying())
                    animation.pause();
                else
                    animation.start();
            }
        }
        else {
            static bool space_pressed = false;
        }
        
        if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
            animation.reset();
        
        if (glfwGetKey(window, GLFW_KEY_EQUAL) == GLFW_PRESS) {
            animation_speed += 0.1f;
            animation.setSpeed(animation_speed);
            std::cout << "Animation speed: " << animation_speed << std::endl;
        }
    
        if (glfwGetKey(window, GLFW_KEY_MINUS) == GLFW_PRESS) {
            animation_speed = std::max(0.1f, animation_speed - 0.1f);
            animation.setSpeed(animation_speed);
            std::cout << "Animation speed: " << animation_speed << std::endl;
        }
    
        // Update animation
        animation.update(deltaTime);
        
        glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        glEnable(GL_DEPTH_TEST);
        
        // View and projection matrices
        glm::mat4 view = camera.getViewMatrix();
        glm::mat4 projection = camera.getProjectionMatrix((float)width / height);
        glm::mat4 viewProjection = projection * view;
        
        // Render orbits
        initialOrbit.render(orbitShader, viewProjection, glm::vec3(0.2f, 0.6f, 1.0f)); // Blue for initial
        targetOrbit.render(orbitShader, viewProjection, glm::vec3(1.0f, 0.3f, 0.3f));  // Red for target

        transferModel.renderCompleteEllipse(orbitShader, viewProjection, glm::vec3(1.0f, 1.0f, 0.3f));

        // Render transfer trajectory with animation
        //transferModel.render(transferShader, viewProjection, glm::vec3(1.0f, 1.0f, 0.5f), animation.getProgress());
        
        // Render impulse arrows
        //impulseModel.render(impulseShader, viewProjection, animation.getProgress());

        camera.displayCameraPosition(camera);
        //camera.renderCoordinateAxes(axesShader, viewProjection, axesVAO);

        //debugLine.render(orbitShader, viewProjection, glm::vec3(1.0f, 1.0f, 1.0f));
        
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
        
    // Handle exit with escape key
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    glDeleteVertexArrays(1, &axesVAO);
    glDeleteBuffers(1, &axesVBO);
    
    glfwTerminate();
    return 0;
}