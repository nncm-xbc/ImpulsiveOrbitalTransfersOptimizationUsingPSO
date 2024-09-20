#include <iostream>
#include <functional>
#include "PSO.hpp"
#include "Functions.hpp"

int main() {
    // Define problem parameters
    const size_t numParticles = 30;
    const size_t dimension = 2;
    const size_t maxIterations = 1000;
    const double tolerance = 1e-6;
    const double inertiaWeight = 0.7;
    const double cognitiveWeight = 1.5;
    const double socialWeight = 1.5;

    // Choose the objective function (Sphere function in this case)
    auto objectiveFunction = Function::Sphere<const std::vector<T>>;

    // Create PSO instance
    PSO<double, std::function<double(double*)>> pso(
        numParticles, dimension, maxIterations, tolerance,
        inertiaWeight, cognitiveWeight, socialWeight, objectiveFunction);

//PSO(size_t numParticles, size_t dimension, size_t maxIterations, 
//    T tolerance, T inertiaWeight, T cognitiveWeight, T socialWeight, const Fun& objectiveFunction)


    // Solve the optimization problem
    pso.solve();

    // Print results
    pso.printResults();

    return 0;
}