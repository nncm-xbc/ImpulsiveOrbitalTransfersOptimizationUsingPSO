#include "optimization/PSO.hpp"
#include "core/OrbitProblem.hpp"

#include <iostream>
#include <functional>
#include <math.h>

int main()
{
    // Define PSO parameters
    size_t numParticles = 1500;
    size_t dimension = 3;
        // x[0]: departure true anomaly (0 to 2π)
        // x[1]: arrival true anomaly (0 to 2π)
        // x[5]: time of flight (0 to 1)
    size_t maxIterations = 10000;
    double tolerance = 1e-2;
    double inertiaWeight = 0.9;
    double cognitiveWeight = 2.0;
    double socialWeight = 2.0;

    OrbitTransferObjective<double, std::function<double(double*)>> objectiveFunction(
        constant::R1,
        constant::R2,
        constant::Rmax,
        constant::E1,constant::E2,
        constant::I1, constant::I2,
        constant::RAAN1, constant::RAAN2,
        constant::OMEGA1, constant::OMEGA2);

    std::vector<double> lowerBounds(dimension);
    std::vector<double> upperBounds(dimension);

    // Angle bounds 1
    lowerBounds[0] = 0.0;
    upperBounds[0] = 2*M_PI;
    // Angle bounds 2
    lowerBounds[1] = 0.0;
    upperBounds[1] = 2*M_PI;

    // Transfer time bounds (min -> 1/4 of orbital period at avg radius)
    double avg_orbit_period = 2*M_PI * std::sqrt(pow((constant::R1 + constant::R2)/2, 3)/constant::MU);
    double minTransferTime = 0.25 * avg_orbit_period;
    double maxTransferTime = 2*M_PI * std::sqrt(pow((constant::R1 + constant::R2)/2, 3)/constant::MU);

    lowerBounds[2] = minTransferTime;
    upperBounds[2] = maxTransferTime;

    // Create PSO instance
    PSO<double, std::function<double(double*)>> pso(
        numParticles,
        dimension,
        maxIterations,
        tolerance,
        inertiaWeight,
        cognitiveWeight,
        socialWeight,
        objectiveFunction,
        lowerBounds,
        upperBounds);

    // Main loop
    pso.solve();

    // Results
    pso.printResults();
    pso.saveResults("../ressources/results.txt", objectiveFunction);

    return 0;
}
