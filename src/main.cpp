#include <iostream>
#include <functional>
#include "PSO.hpp"
#include "Functions.hpp"

int main() {
    // Define problem parameters
    size_t numParticles = 30;
    size_t dimension = 2;
    size_t maxIterations = 1000;
    double tolerance = 1e-6;
    double inertiaWeight = 0.7;
    double cognitiveWeight = 1.5;
    double socialWeight = 1.5;

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
