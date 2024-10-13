#include "PSO.hpp"
#include "Functions.hpp"

#include <iostream>
#include <functional>

int main() {
    // Define problem parameters
    size_t numParticles = 100;
    size_t dimension = 10;
    size_t maxIterations = 100000;
    double tolerance = 1e-12;
    double inertiaWeight = 0.5;
    double cognitiveWeight = 2.0;
    double socialWeight = 2.0;

    // Choose the objective function
    std::function<double(double*, size_t)> objectiveFunction = Function::Sphere<double>;

    // Create PSO instance
    PSO<double, std::function<double(double*, size_t)>> pso(
        numParticles, dimension, maxIterations, tolerance,
        inertiaWeight, cognitiveWeight, socialWeight, objectiveFunction);

    // Main loop
    pso.solve();

    // Print results
    pso.printResults();

    return 0;
}
