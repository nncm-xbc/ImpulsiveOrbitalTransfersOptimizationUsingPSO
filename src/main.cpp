#include "PSO.hpp"
#include "Functions.hpp"
#include "OrbitProblem.hpp"

#include <iostream>
#include <functional>
#include <math.h>

int main()
{
    // Define problem parameters
    size_t numParticles = 500;
    size_t impulses = 2;
    size_t dimension = 4;
    size_t maxIterations = 10000;
    double tolerance = 1e-12;
    double inertiaWeight = 0.5;
    double cognitiveWeight = 2.0;
    double socialWeight = 2.0;

    // Init orbit transfer problem
    //Distance Unit : 1DU = 6378.165 km
    double R1 = 16678.0/6378.165;
    double R2 = 42164.0/6378.165;
    double Rmax = 42164.0/6378.165;
    OrbitTransferObjective<double, std::function<double(double*)>> objectiveFunction(R1, R2, Rmax);

    std::vector<double> lowerBounds(dimension);
    std::vector<double> upperBounds(dimension);

    // Angle bounds 1
    lowerBounds[0] = 0.0;
    upperBounds[0] = 2*M_PI;
    // Angle bounds 2
    lowerBounds[1] = 0.0;
    upperBounds[1] = 2*M_PI;
    // Impulse direction bounds 1
    lowerBounds[2] = -M_PI/2;
    upperBounds[2] = M_PI/2;
    // Impulse direction bounds 2
    lowerBounds[3] = -M_PI/2;
    upperBounds[3] = M_PI/2;

    // x[0]: departure true anomaly (0 to 2π)
    // x[1]: arrival true anomaly (0 to 2π)
    // x[2]: first impulse direction (-π/2 to π/2)
    // x[3]: second impulse direction (-π/2 to π/2)

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

    // Print results
    pso.printResults();

    return 0;
}
