#include "PSO.hpp"
#include "Functions.hpp"
#include "OrbitProblem.hpp"

#include <iostream>
#include <functional>
#include <math.h>

int main()
{
    // Define problem parameters
    size_t numParticles = 100;
    // dim 4 = 2 angles + 2 impulse directions
    size_t dimension = 4;
    size_t maxIterations = 10000;
    double tolerance = 1e-2;
    double inertiaWeight = 0.5;
    double cognitiveWeight = 2.0;
    double socialWeight = 2.0;

    OrbitTransferObjective<double, std::function<double(double*)>> objectiveFunction(constant::R1, constant::R2, constant::Rmax);

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
