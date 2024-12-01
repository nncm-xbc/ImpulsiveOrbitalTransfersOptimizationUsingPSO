#include "PSO.hpp"
#include "Functions.hpp"
#include "OrbitProblem.hpp"

#include <iostream>
#include <functional>
#include <math.h>

int main() {
    // Define problem parameters
    size_t numParticles = 150;
    size_t dimension = 2;
    size_t maxIterations = 500;
    double tolerance = 1e-12;
    double inertiaWeight = 0.5;
    double cognitiveWeight = 2.0;
    double socialWeight = 2.0;

    // Init orbit transfer problem
    double R1 = 6678.0;
    double R2 = 42164.0;
    double Rmax = 42164.0;
    OrbitTransferObjective<double, std::function<double(double*, size_t)>> objectiveFunction(R1, R2, Rmax);

    std::vector<double> lowerBounds(dimension);
    std::vector<double> upperBounds(dimension);

    lowerBounds[0] = R1;
    upperBounds[0] = R2;
    lowerBounds[1] = 0.0;
    upperBounds[1] = 2*M_PI;

    // Create PSO instance
    PSO<double, std::function<double(double*, size_t)>> pso(
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
