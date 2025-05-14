#include "rendering/Animation.hpp"
#include "rendering/Camera.hpp"
#include "rendering/ImpulseVisu.hpp"
#include "rendering/OrbitModel.hpp"
#include "rendering/RenderEngine.hpp"
#include "rendering/Shader.hpp"
#include "rendering/TransferModel.hpp"

int main() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Set OpenGL context version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create window
    GLFWwindow* window = glfwCreateWindow(800, 600, "Orbital Transfer Visualization", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);

    // Initialize GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // Create render engine
    RenderEngine engine;
    if (!engine.initialize(800, 600)) {
        std::cerr << "Failed to initialize render engine" << std::endl;
        return -1;
    }

    // Set up orbits from Case 2 in Vallado [[1]](https://poe.com/citation?message_id=389522500747&citation=1)
    engine.setInitialOrbit(6671.53, glm::radians(28.5));
    engine.setTargetOrbit(26558.56, 0.0);

    // Set up transfer trajectory from PSO results
    double optimal_parameters[3] = {
        0.0,       // Initial true anomaly
        3.14159,   // Final true anomaly
        3600.0     // Transfer time
    };
    engine.setTransferTrajectory(optimal_parameters);

    // Set up mouse callback for camera control
    glfwSetWindowUserPointer(window, &engine);
    glfwSetCursorPosCallback(window, [](GLFWwindow* window, double x, double y) {
        static double last_x = x;
        static double last_y = y;

        double dx = x - last_x;
        double dy = y - last_y;

        last_x = x;
        last_y = y;

        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
            RenderEngine* engine = static_cast<RenderEngine*>(glfwGetWindowUserPointer(window));
            engine->rotateCamera(dx * 0.01f, dy * 0.01f);
        }
    });

    glfwSetScrollCallback(window, [](GLFWwindow* window, double xoffset, double yoffset) {
        RenderEngine* engine = static_cast<RenderEngine*>(glfwGetWindowUserPointer(window));
        engine->zoomCamera(yoffset * 0.1f);
    });

    // Start animation
    engine.startAnimation();

    // Main loop
    double last_time = glfwGetTime();
    while (!glfwWindowShouldClose(window)) {
        double current_time = glfwGetTime();
        double delta_time = current_time - last_time;
        last_time = current_time;

        // Handle keyboard input
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
            static bool space_pressed = false;
            if (!space_pressed) {
                space_pressed = true;
                if (engine.isAnimationPlaying()) {
                    engine.pauseAnimation();
                } else {
                    engine.startAnimation();
                }
            }
        } else {
            space_pressed = false;
        }

        if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
            engine.resetAnimation();
        }

        // Update and render
        engine.update(delta_time);
        engine.render();

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
