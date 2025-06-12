#include "PSO.hpp"
#include "OrbitProblem.hpp"

#include <iostream>
#include <functional>
#include <math.h>

int main()
{
    // Define PSO parameters
    size_t numParticles = 500;
    size_t dimension = 6;
        // x[0]: departure true anomaly (0 to 2π)
        // x[1]: arrival true anomaly (0 to 2π)
        // x[2]: first impulse direction (-π/2 to π/2)
        // x[3]: second impulse direction (-π/2 to π/2)
        // x[4]: First impulse direction (0 to 2π)
        // x[5]: time of flight (0 to 1)
    size_t maxIterations = 15000;
    double tolerance = 1e-2;
    double inertiaWeight = 0.5;
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

    // Impulse magnitude bounds 1
    double v_circ1 = sqrt(constant::MU / constant::R1);
    double v_circ2 = sqrt(constant::MU / constant::R2);
    double max_impulse = 1.5 * std::max(v_circ1, v_circ2);

    lowerBounds[2] = 1e-4 * std::min(v_circ1, v_circ2);
    upperBounds[2] = max_impulse;

    // Impulse magnitude bounds 2
    lowerBounds[3] = 1e-4 * std::min(v_circ1, v_circ2);
    upperBounds[3] = max_impulse;

    // Impulse direction bounds
    lowerBounds[4] = -M_PI;
    upperBounds[4] = M_PI;

    // Transfer time bounds (min -> 1/4 of orbital period at avg radius)
    double avg_orbit_period = 2*M_PI * std::sqrt(pow((constant::R1 + constant::R2)/2, 3)/constant::MU);
    double minTransferTime = 0.25 * avg_orbit_period;
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
