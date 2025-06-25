/**
 * @file OrbitMechanics.hpp
 * @brief Core orbital mechanics calculations and utilities
 * @author PSO Orbit Transfer Team
 * @date 2025
 *
 * Provides fundamental orbital mechanics calculations including position,
 * velocity, and impulse computations for various orbital configurations.
 */

#ifndef ORBIT_MECHANICS_HPP
#define ORBIT_MECHANICS_HPP

#include <glm/glm.hpp>
#include <vector>
#include <utility>
#include <stdexcept>

namespace Physics {

    /**
     * @struct Vector3
     * @brief 3D vector class with orbital mechanics operations
     *
     * Custom vector class that provides seamless conversion between
     * double precision calculations and GLM vectors for visualization.
     * Includes common vector operations needed for orbital mechanics.
     */
    struct Vector3 {
        /** @brief X component */
        double x;

        /** @brief Y component */
        double y;

        /** @brief Z component */
        double z;

        /** @brief Default constructor - initializes to zero vector */
        Vector3() : x(0), y(0), z(0) {}

        /**
         * @brief Component constructor
         * @param _x X component
         * @param _y Y component
         * @param _z Z component
         */
        Vector3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

        /**
         * @brief Conversion constructor from glm::dvec3
         * @param v GLM double precision vector
         */
        Vector3(const glm::dvec3& v) : x(v.x), y(v.y), z(v.z) {}

        /**
         * @brief Conversion operator to glm::dvec3
         * @return GLM double precision vector
         */
        operator glm::dvec3() const {
            return glm::dvec3(x, y, z);
        }

        /**
         * @brief Vector subtraction operator
         * @param v Vector to subtract
         * @return Result of subtraction
         */
        Vector3 operator-(const Vector3& v) const {
            return Vector3(x - v.x, y - v.y, z - v.z);
        }

        /**
         * @brief Vector addition operator
         * @param v Vector to add
         * @return Result of addition
         */
        Vector3 operator+(const Vector3& v) const {
            return Vector3(x + v.x, y + v.y, z + v.z);
        }

        /**
         * @brief Calculate vector magnitude
         * @return Euclidean norm of the vector
         */
        double magnitude() const {
            return std::sqrt(x*x + y*y + z*z);
        }

        /**
         * @brief Calculate dot product with another vector
         * @param v Vector to compute dot product with
         * @return Scalar dot product result
         */
        double dot(const Vector3& v) const {
            return x * v.x + y * v.y + z * v.z;
        }

        /**
         * @brief Calculate cross product with another vector
         * @param v Vector to compute cross product with
         * @return Vector perpendicular to both input vectors
         */
        Vector3 cross(const Vector3& v) const {
            return Vector3(
                y * v.z - z * v.y,
                z * v.x - x * v.z,
                x * v.y - y * v.x
            );
        }

        /**
         * @brief Return normalized (unit) vector
         * @return Unit vector in the same direction as this vector
         * @throws std::runtime_error if vector is zero (cannot normalize)
         */
        Vector3 normalized() const {
            double mag = magnitude();
            if (mag < 1e-15) {
                throw std::runtime_error("Vector3: Cannot normalize zero vector");
            }
            return Vector3(x / mag, y / mag, z / mag);
        }
    };

/**
 * @class OrbitMechanics
 * @brief Static class providing orbital mechanics calculations
 *
 * Contains fundamental orbital mechanics algorithms for computing
 * positions, velocities, and orbital maneuvers. All methods are
 * static to allow easy access without instantiation.
 */
class OrbitMechanics {
public:
    /**
     * @brief Calculate velocity components in orbital plane
     * @param a Semi-major axis (DU)
     * @param e Eccentricity (dimensionless)
     * @param trueAnomaly True anomaly (radians)
     * @return Pair of (radial velocity, tangential velocity) in DU/TU
     *
     * Computes velocity components in the orbital plane using the
     * vis-viva equation and orbital mechanics relationships.
     */
    static std::pair<double, double> calculateVelocity(double a, double e, double trueAnomaly);

    /**
     * @brief Calculate 3D velocity vector in inertial frame
     * @param a Semi-major axis (DU)
     * @param e Eccentricity (dimensionless)
     * @param i Inclination (radians)
     * @param raan Right ascension of ascending node (radians)
     * @param omega Argument of periapsis (radians)
     * @param true_anomaly True anomaly (radians)
     * @return 3D velocity vector in inertial coordinates (DU/TU)
     *
     * Transforms orbital plane velocity to inertial frame using
     * standard orbital element rotations: Ω → i → ω
     */
    static Vector3 calculateVelocity3D(double a, double e, double i, double raan,
                                       double omega, double true_anomaly);

    /**
     * @brief Calculate 3D position vector in inertial frame
     * @param a Semi-major axis (DU)
     * @param e Eccentricity (dimensionless)
     * @param i Inclination (radians)
     * @param raan Right ascension of ascending node (radians)
     * @param omega Argument of periapsis (radians)
     * @param true_anomaly True anomaly (radians)
     * @return 3D position vector in inertial coordinates (DU)
     *
     * Calculates position using the orbit equation and transforms
     * from orbital plane to inertial frame.
     */
    static Vector3 calculatePosition3D(double a, double e, double i, double raan,
                                      double omega, double true_anomaly);

    /**
     * @brief Calculate orbital radius at given true anomaly
     * @param a Semi-major axis (DU)
     * @param e Eccentricity (dimensionless)
     * @param trueAnomaly True anomaly (radians)
     * @return Orbital radius (DU)
     *
     * Uses the fundamental orbit equation: r = a(1-e²)/(1+e·cos(ν))
     */
    static double calculateRadius(double a, double e, double trueAnomaly);

    /**
     * @brief Calculate orbit position for visualization (simplified circular case)
     * @param radius Orbital radius (DU)
     * @param inclination Orbital inclination (radians)
     * @param true_anomaly True anomaly (radians)
     * @return 3D position vector for visualization
     *
     * Simplified calculation for circular orbits used by visualization
     * components. Assumes argument of periapsis = 0 and RAAN = 0.
     */
    static glm::vec3 calculateOrbitPosition(float radius, float inclination, float true_anomaly);

    /**
     * @brief Calculate orbit velocity for visualization (simplified circular case)
     * @param radius Orbital radius (DU)
     * @param inclination Orbital inclination (radians)
     * @param true_anomaly True anomaly (radians)
     * @return 3D velocity vector for visualization
     *
     * Simplified calculation for circular orbit velocities used by
     * visualization components.
     */
    static glm::vec3 calculateOrbitVelocity(float radius, float inclination, float true_anomaly);
};

} // namespace Physics

#endif
