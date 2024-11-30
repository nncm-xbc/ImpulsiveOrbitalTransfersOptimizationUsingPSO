#include "PSO.hpp"
#include "Functions.hpp"
#include "OrbitProblem.hpp"

#include <iostream>
#include <functional>

int main() {
    // Define problem parameters
    size_t numParticles = 100;
    size_t dimension = 20;
    size_t maxIterations = 100000;
    double tolerance = 1e-12;
    double inertiaWeight = 0.5;
    double cognitiveWeight = 2.0;
    double socialWeight = 2.0;

    // Init orbit transfer problem
    double R1 = 6678.0;
    double R2 = 42164.0;
    double Rmax = 42164.0;
    OrbitTransferObjective<double, std::function<double(double*, size_t)>> objectiveFunction(R1, R2, Rmax);

    // Create PSO instance
    PSO<double, std::function<double(double*, size_t)>> pso(
        numParticles,
        dimension,
        maxIterations,
        tolerance,
        inertiaWeight,
        cognitiveWeight,
        socialWeight,
        objectiveFunction);

    // Main loop
    pso.solve();

    // Print results
    pso.printResults();

    return 0;
}
