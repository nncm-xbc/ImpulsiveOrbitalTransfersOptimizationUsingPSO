#include "glad/glad.h"
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <iomanip>
#include <sstream>

#include "core/OrbitMechanics.hpp"
#include "core/Constants.hpp"
#include "visualization/OrbitModel.hpp"
#include "visualization/Shader.hpp"
#include "visualization/Camera.hpp"
#include "visualization/TransferModel.hpp"
#include "visualization/Animation.hpp"
#include "visualization/Results.hpp"

// Improved orbit information display
void printOrbitInfo(const std::string& name, double radius, double inclination, double raan, double eccentricity) {
    std::cout << "\n "<< name << " ORBIT:" << std::endl;
    std::cout << "   ├─ Type:           " << (eccentricity < 1e-6 ? "Circular" : "Elliptical") << std::endl;
    std::cout << "   ├─ Radius:         " << std::fixed << std::setprecision(3) << radius << " DU" << std::endl;
    std::cout << "   ├─ Inclination:    " << std::setprecision(1) << inclination * 180/M_PI << "°" << std::endl;
    std::cout << "   ├─ RAAN:           " << raan * 180/M_PI << "°" << std::endl;
    std::cout << "   └─ Eccentricity:   " << std::setprecision(6) << eccentricity << std::endl;
    std::cout << std::endl;
}

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
    const int width = 800, height = 700;
    GLFWwindow* window = glfwCreateWindow(width, height, "Orbital Transfer", nullptr, nullptr);
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

    double initial_radius_DU = psoResult.initial_radius / constant::DU;
    double target_radius_DU = psoResult.target_radius / constant::DU;

    psoResult.initial_radius = initial_radius_DU;
    psoResult.target_radius = target_radius_DU;

    bool usePSOResults = false;
    bool show_complete_ellipse = false;

    if (argc > 1) {
        usePSOResults = loadPSOResultsFromFile(argv[1], psoResult);
        if (usePSOResults) {
            std::cout << "\n Loaded PSO results from " << argv[1] << std::endl;
        }
    }

    // Load shaders
    Shader transferOrbitShader;
    Shader orbitShader;
    Shader transferShader;
    Shader axesShader;

    if (!orbitShader.loadFromFiles("../shaders/orbit.vert", "../shaders/orbit.frag") ||
        !transferShader.loadFromFiles("../shaders/transfer.vert", "../shaders/transfer.frag") ||
        !transferOrbitShader.loadFromFiles("../shaders/orbit_transfer.vert", "../shaders/orbit_transfer.frag")
    ) {
        std::cerr << "Failed to load shaders" << std::endl;
        return -1;
    }

    Camera camera;
    camera.setZoom(50000.0f); // Initial zoom level
    camera.setPosition(glm::vec3(0.0f, 0.0f, 0.0f)); // Initial camera position
    Animation animation(5.0f);

    OrbitModel initialOrbit;
    OrbitModel targetOrbit;
    TransferModel transferModel;

    GLuint axesVAO, axesVBO;

    initialOrbit.initialize();
    targetOrbit.initialize();
    transferModel.initialize();

    if (usePSOResults) {
        if (psoResult.initial_eccentricity < 1e-6) {

            printOrbitInfo(" INITIAL", psoResult.initial_radius, psoResult.initial_inclination, psoResult.initial_raan, psoResult.initial_eccentricity);

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

            printOrbitInfo("TARGET", psoResult.target_radius, psoResult.target_inclination, psoResult.target_raan, psoResult.target_eccentricity);

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
        transferModel.setTwoImpulseTransfer(
            psoResult.initial_radius, psoResult.initial_inclination,
            psoResult.target_radius, psoResult.target_inclination,
            psoResult.initial_eccentricity, psoResult.target_eccentricity,
            psoResult.initial_true_anomaly, psoResult.final_true_anomaly,
            psoResult.delta_v_magnitudes, psoResult.plane_change
        );


        glm::vec3 pso_target_pos = Physics::OrbitMechanics::calculateOrbitPosition(
            psoResult.target_radius,
            psoResult.target_inclination,
            psoResult.final_true_anomaly
        );

        if (std::abs(psoResult.target_inclination) < 1e-6) {  // Equatorial orbit
            // For position (1.5, 0, 0), we need ν = 0°
            psoResult.final_true_anomaly = 0.0;
        }

    } else {
        // Default case
        initialOrbit.setCircularOrbit(6671.53, 0.0f);
        targetOrbit.setCircularOrbit(26558.56, 0.0f);

        transferModel.setTwoImpulseTransfer(
            6671.53, 0.0f,
            26558.56, 0.0f,
            0.0, 0.0,
            0.0, M_PI,
            { 0.0, 0.0 }, { 0.0, 0.0 }
        );
    }

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

    std::cout << "\n Controls:\n"
    << "  Left mouse button + drag: Rotate camera\n"
    << "  Scroll wheel: Zoom in/out\n"
    << "  Space: Play/pause animation\n"
    << "  R: Reset animation\n"
    << "  +/-: Adjust animation speed\n"
    << "  L: Toggle animation loop\n"
    << "  T: Toggle transfer ellipse rendering\n"
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
        static bool space_pressed = false;
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
            if (!space_pressed) {
                space_pressed = true;
                animation.togglePlay();  // Use the new method
                std::cout << "Animation " << (animation.isPlaying() ? "playing" : "paused") << std::endl;
            }
        } else {
            space_pressed = false;
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

        static bool t_pressed = false;
        if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS) {
            if (!t_pressed) {
                t_pressed = true;
                show_complete_ellipse = !show_complete_ellipse;
                std::cout << "Complete ellipse " << (show_complete_ellipse ? "shown" : "hidden") << std::endl;
            }
        }
        else {
            t_pressed = false;
        }

        static bool l_pressed = false;
        if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS) {
            if (!l_pressed) {
                l_pressed = true;
                animation.toggleLooping();
                std::cout << "Animation looping: " << (animation.isLooping() ? "ON" : "OFF") << std::endl;
            }
        } else {
            l_pressed = false;
        }

        // Update animation
        animation.update(deltaTime);

        glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glEnable(GL_DEPTH_TEST);

        glm::mat4 view = camera.getViewMatrix();
        glm::mat4 projection = camera.getProjectionMatrix((float)width / height);
        glm::mat4 viewProjection = projection * view;

        // Render orbits
        initialOrbit.render(orbitShader, viewProjection, glm::vec3(0.2f, 0.6f, 1.0f)); // Blue for initial
        targetOrbit.render(orbitShader, viewProjection, glm::vec3(1.0f, 0.3f, 0.3f));  // Red for target

        // Render full transfer orbit
        if (show_complete_ellipse) {
            transferModel.renderCompleteEllipse(orbitShader, viewProjection, glm::vec3(0.7f, 0.7f, 0.2f));
        }

        // Render transfer with animation
        transferModel.render(transferShader, viewProjection, glm::vec3(1.0f, 1.0f, 0.0f), animation.getProgress());

        camera.displayCameraPosition(camera);
        //camera.renderCoordinateAxes(axesShader, viewProjection, axesVAO);

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
