#include "PSO.hpp"
#include "OrbitProblem.hpp"

#include <iostream>
#include <functional>
#include <math.h>

int main()
{
    // Define problem parameters
    size_t numParticles = 100;
    size_t dimension = 6;
        // x[0]: departure true anomaly (0 to 2π)
        // x[1]: arrival true anomaly (0 to 2π)
        // x[2]: first impulse direction (-π/2 to π/2)
        // x[3]: second impulse direction (-π/2 to π/2)
        // x[4]: First impulse direction (0 to 2π)
        // x[5]: time of flight (0 to 1)
    size_t maxIterations = 10000;
    double tolerance = 1e-2;
    double inertiaWeight = 0.5;
    double cognitiveWeight = 2.0;
    double socialWeight = 2.0;

    OrbitTransferObjective<double, std::function<double(double*)>> objectiveFunction(
        constant::R1, constant::R2, constant::Rmax,
        0.0, 0.0, // e1, e2 - For circular orbits
        28.5 * M_PI / 180.0, 0.0); // i1, i2 - Inclination and RAAN

    std::vector<double> lowerBounds(dimension);
    std::vector<double> upperBounds(dimension);

    // Angle bounds 1
    lowerBounds[0] = 0.0;
    upperBounds[0] = 2*M_PI;
    // Angle bounds 2
    lowerBounds[1] = 0.0;
    upperBounds[1] = 2*M_PI;
    // Impulse magnitude bounds 1
    lowerBounds[2] = 0.0;
    upperBounds[2] = 5.0;
    // Impulse magnitude bounds 2
    lowerBounds[3] = 0.0;
    upperBounds[3] = 5.0;
    // Impulse direction bounds
    lowerBounds[4] = -M_PI;
    upperBounds[4] = M_PI;
    // Transfer time bounds
    double minTransferTime = 0.0;
    double maxTransferTime = 2*M_PI * std::sqrt(pow((constant::R1 + constant::R2)/2, 3)/constant::MU);
    lowerBounds[5] = minTransferTime;
    upperBounds[5] = maxTransferTime;

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
