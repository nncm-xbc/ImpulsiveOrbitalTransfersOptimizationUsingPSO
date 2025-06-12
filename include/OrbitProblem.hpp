#ifndef ORBIT_HPP
#define ORBIT_HPP

#include <cmath>
#include <vector>
#include <map>
#include <string>
#include "Constants.hpp"

struct Vector3 {
    double x, y, z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

    Vector3 operator-(const Vector3& v) const {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }

    double magnitude() const {
        return std::sqrt(x*x + y*y + z*z);
    }
};

template <typename T, typename Fun>
class PSO;  // Forward declaration

template<typename T, typename Fun>
class OrbitTransferObjective
{
    private:
        const double _MU = constant::MU;
        double _R1, _R2, _Rmax; // orbit radii
        double _e1, _e2; // Eccentricities
        double _i1, _i2; // Inclination
        double _raan1, _raan2; // Right Ascension of Ascending Node
        double _omega1, _omega2; // Argument of Perigee

        int _currentIteration = 0;
        int _maxIterations = 1;

        std::pair<T, T> calculateVelocity(double a, double e, double trueAnomaly);
        Vector3 calculateVelocity3D(double a, double e, double i, double raan, double omega, double true_anomaly);
        Vector3 calculatePosition3D(double a, double e, double i, double raan, double omega, double true_anomaly);
        double calculateRadius(double a, double e, double trueAnomaly);
        double calculateDeltaV(const std::vector<double>& x);
        std::pair<Vector3, Vector3> solveLambert(const Vector3& r1,const Vector3& r2, double tof, double mu, bool isLongWay);
        double computePeriapsis(const std::vector<T>& x);
        double computeApoapsis(const std::vector<T>& x);
        double checkConstraints(const std::vector<double>& x);

        //PSO class is a friend :)
        template<typename Y, typename Fin>
        friend class PSO;

    public:
        OrbitTransferObjective(double R1, double R2, double Rmax,
                    double E1 = 0.0, double E2 = 0.0,
                    double I1 = 0.0, double I2 = 0.0,
                    double RAAN1 = 0.0, double RAAN2 = 0.0,
                    double OMEGA1 = 0.0, double OMEGA2 = 0.0);
        double operator()(double* x);

        double getE1() const;
        double getE2() const;
        double getI1() const;
        double getI2() const;
        std::map<std::string, double> getTransferDetails(const std::vector<double>& x);
        double getRelaxationFactor() const;
};

#endif
