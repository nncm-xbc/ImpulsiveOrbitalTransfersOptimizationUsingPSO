#include "optimization/PSO.hpp"
#include "core/OrbitProblem.hpp"
#include "core/ExactSolution.hpp"

#include "core/OrbitMechanics.hpp"
#include "core/LambertSolver.hpp"

#include <iostream>
#include <functional>
#include <math.h>

int main()
{
    // Define PSO parameters
    size_t numParticles = 1000;
    size_t dimension = 3;
        // x[0]: departure true anomaly (0 to 2π)
        // x[1]: arrival true anomaly (0 to 2π)
        // x[2]: time of flight (0 to 1)
    size_t maxIterations = 1000;
    double tolerance = 1e-2;
    double inertiaWeight = 0.6;
    double cognitiveWeight = 1.8;
    double socialWeight = 1.8;

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

    // Transfer time bounds
    double avg_orbit_period = 2*M_PI * std::sqrt(pow((constant::R1 + constant::R2)/2, 3)/constant::MU);
    double minTransferTime = 0.3 * avg_orbit_period;
    double maxTransferTime = 3.0 * avg_orbit_period;

    lowerBounds[2] = 0.1;
    upperBounds[2] = 10.0;

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

    std::vector<double> bestSolution = pso.getBestPosition();
     double bestDeltaV = pso.getBestValue();

     // Extract parameters
     double theta0 = bestSolution[0];
     double thetaF = bestSolution[1];
     double transferTime = bestSolution[2];

    // Results
    //pso.printResults();
    pso.saveResults("../ressources/results.txt", objectiveFunction);

    // For coplanar
    if (constant::I1 == 0.0 && constant::I2 == 0.0) {
        std::cout << "\n=== ANALYTICAL VALIDATION ===" << std::endl;

        HohmannSolution<double> hohmann(constant::R1, constant::R2, constant::MU);

        hohmann.printValidationReport(bestDeltaV, transferTime, theta0, thetaF);
    } else {
        std::cout << "\n Non-coplanar transfer - No analytical validation" << std::endl;
        std::cout << "   Expected ΔV range for plane change: " << std::fixed << std::setprecision(1)
                    << "2.0-4.0 km/s" << std::endl;
    }

    return 0;
}
