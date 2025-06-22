#ifndef ORBIT_MECHANICS_HPP
#define ORBIT_MECHANICS_HPP

#include <glm/glm.hpp>
#include <vector>
#include <utility>

namespace Physics {

    struct Vector3 {
        double x, y, z;

        Vector3() : x(0), y(0), z(0) {}
        Vector3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

        // Conversion from glm::dvec3
        Vector3(const glm::dvec3& v) : x(v.x), y(v.y), z(v.z) {}

        // Conversion to glm::dvec3
        operator glm::dvec3() const {
            return glm::dvec3(x, y, z);
        }

        Vector3 operator-(const Vector3& v) const {
            return Vector3(x - v.x, y - v.y, z - v.z);
        }

        Vector3 operator+(const Vector3& v) const {
            return Vector3(x + v.x, y + v.y, z + v.z);
        }

        double magnitude() const {
            return std::sqrt(x*x + y*y + z*z);
        }
    };


class OrbitMechanics {
public:
    static std::pair<double, double> calculateVelocity(double a, double e, double trueAnomaly);
    static Vector3 calculateVelocity3D(double a, double e, double i, double raan,
                                       double omega, double true_anomaly);

    static Vector3 calculatePosition3D(double a, double e, double i, double raan,
                                      double omega, double true_anomaly);
    static double calculateRadius(double a, double e, double trueAnomaly);

    static glm::vec3 calculateOrbitPosition(float radius, float inclination, float true_anomaly);
    static glm::vec3 calculateOrbitVelocity(float radius, float inclination, float true_anomaly);

    static Vector3 calculateImpulseVector(const Vector3& pos, const Vector3& vel,
                                         double magnitude, double planeChangeAngle);
};

} // namespace Physics

#endif
