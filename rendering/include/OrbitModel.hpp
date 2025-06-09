#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <cmath>

#include "Shader.hpp"

class OrbitModel {
    public:

        OrbitModel();
        ~OrbitModel();

        void initialize();
        void setCircularOrbit(double radius, double inclination = 0.0, double raan = 0.0);
        void setEllipticalOrbit(double semi_major_axis, double eccentricity,
                                double inclination = 0.0, double raan = 0.0,
                                double arg_periapsis = 0.0);
        void render(const Shader& shader, const glm::mat4& view_projection, const glm::vec3& color);
        glm::vec3 calculateRenderedCenter();

    private:
        double radius_;          // Radius for circular orbits, semi-major axis for elliptical
        double eccentricity_;    // Orbital eccentricity
        double inclination_;     // Orbit inclination in radians
        double raan_;            // Right ascension of ascending node in radians
        double arg_periapsis_;   // Argument of periapsis in radians

        GLuint vao_, vbo_;
        bool initialized_;

        std::vector<glm::vec3> orbit_points_;

        void generateOrbitPoints(int resolution = 200);
        void updateBuffers();
};
