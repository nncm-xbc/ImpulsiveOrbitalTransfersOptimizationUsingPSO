#include "optimization/PSO.hpp"
#include "optimization/Logger.hpp"
#include "core/ExactSolution.hpp"
#include "optimization/PSOGlobals.hpp"
#include "core/OrbitProblem.hpp"

#include <iomanip>
#include <vector>
#include <omp.h>

template <typename T, typename Fun>
PSO<T, Fun>::PSO(size_t numParticles,
                size_t dimension,
                size_t maxIterations,
                T tolerance,
                T inertiaWeight,
                T cognitiveWeight,
                T socialWeight,
                const Fun &objectiveFunction,
                const std::vector<T> lowerBounds,
                const std::vector<T> upperBounds):
                    swarm(numParticles,
                    dimension,
                    objectiveFunction,
                    inertiaWeight,
                    cognitiveWeight,
                    socialWeight,
                    lowerBounds,
                    upperBounds),
                _maxIterations(maxIterations),
                _tolerance(tolerance){}

template <typename T, typename Fun>
void PSO<T, Fun>::solve()
{
    swarm.init();
    swarm.info();

    Logger conv_logger("../ressources/convergence_log.csv");

    PSOGlobals::maxIterations = _maxIterations;

    const size_t progress_interval = _maxIterations / 20; // Report every 5%
    auto start_time = std::chrono::high_resolution_clock::now();

    std::cout << "\n Progress: [";
    for (size_t i = 0; i < 20; ++i) std::cout << " ";
    std::cout << "] 0%" << std::flush;

    for (size_t iter = 0; iter < _maxIterations; ++iter)
    {
        PSOGlobals::currentIteration = iter;

        std::vector<T> GBPos_previous = swarm.getGlobalBestPosition();

        #pragma omp parallel for schedule(static)
        for (size_t i = 0; i < swarm.getNumParticles(); ++i)
        {
            auto &particle = swarm.particles[i];

            swarm.updateVelocity(particle);
            swarm.updatePosition(particle);
            swarm.updatePBestPos(particle);
     }
        swarm.updateGBestPos();
        updateWC(GBPos_previous, iter);

        // Progress
         if (iter % progress_interval == 0 || iter == _maxIterations - 1) {
             auto current_time = std::chrono::high_resolution_clock::now();
             auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);

             size_t progress_bars = (iter * 20) / _maxIterations;
             std::cout << "\r Progress: [";
             for (size_t i = 0; i < progress_bars; ++i) std::cout << "█";
             for (size_t i = progress_bars; i < 20; ++i) std::cout << " ";
             std::cout << "] " << std::setw(3) << (iter * 100) / _maxIterations << "% | "
                       << "Best ΔV: " << std::fixed << std::setprecision(6) << swarm.getGlobalBestValue()
                       << " km/s | Time: " << elapsed.count() << "s" << std::flush;
         }

        if(iter%100 == 0)
        {
            conv_logger.log(iter, swarm.getGlobalBestValue(), swarm.getInertiaWeight(), swarm.getSocialWeight(), swarm.getCognitiveWeight());
        }
    }
    conv_logger.flushBuffer();
}

template <typename T, typename Fun>
void PSO<T, Fun>::updateWC(std::vector<T> GBPos_previous, size_t iter)
{
    // Linear decrease of inertia weight with iteration count
    T currentIteration = static_cast<T>(iter);
    T maxWeight = 0.9;
    T minWeight = 0.5;
    T inertiaWeight = maxWeight - minWeight * (currentIteration / _maxIterations);
    double c1 = 2.5 - 1.0 * (currentIteration / _maxIterations);  // Decrease cognitive
    double c2 = 0.5 + 1.5 * (currentIteration / _maxIterations);  // Increase social

    swarm.setInertiaWeight(inertiaWeight);
    swarm.setCognitiveWeight(c1);
    swarm.setSocialWeight(c2);
}

template <typename T, typename Fun>
void PSO<T, Fun>::printResults() const {
    std::vector<T> bestSolution = swarm.getGlobalBestPosition();
    double bestDeltaV = swarm.getGlobalBestValue();

    std::cout << "\n╔══════════════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                     PSO ORBITAL TRANSFER RESULTS                     ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ OPTIMAL TRANSFER PARAMETERS:                                         ║" << std::endl;
    std::cout << "║   • Departure True Anomaly:    " << std::setw(12) << bestSolution[0] * 180.0/M_PI << " degrees" << std::setw(21) << " ║" << std::endl;
    std::cout << "║   • Arrival True Anomaly:      " << std::setw(12) << bestSolution[1] * 180.0/M_PI << " degrees" << std::setw(21) << " ║" << std::endl;
    std::cout << "║   • Transfer Time:             " << std::setw(12) << bestSolution[2] * constant::TU / 3600. << " hours" << std::setw(23) << " ║" << std::endl;
    std::cout << "║   • Total ΔV Required:         " << std::setw(12) << bestDeltaV << " km/s" << std::setw(24) << " ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ OPTIMIZATION VARIABLES:                                              ║" << std::endl;
    std::cout << "║   • Variable 1 (θ₀):           " << std::setw(12) << bestSolution[0] << " radians" << std::setw(21) << " ║" << std::endl;
    std::cout << "║   • Variable 2 (θf):           " << std::setw(12) << bestSolution[1] << " radians" << std::setw(21) << " ║" << std::endl;
    std::cout << "║   • Variable 3 (Δt):           " << std::setw(12) << bestSolution[2] << " TU" << std::setw(26) << " ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ ALGORITHM PERFORMANCE:                                               ║" << std::endl;
    std::cout << "║   • Iterations Completed:      " << std::setw(12) << _maxIterations << std::setw(29) << " ║" << std::endl;
    std::cout << "║   • Max Iterations:            " << std::setw(12) << _maxIterations << std::setw(29) << " ║" << std::endl;
    std::cout << "║   • Convergence Status:        " << std::setw(20) << "COMPLETED" << std::setw(21) << " ║" << std::endl;
    std::cout << "║   • Final Inertia Weight:      " << std::setw(12) << swarm.getInertiaWeight() << std::setw(29) << " ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════════════╝" << std::endl;

    std::cout << "\n MISSION SUMMARY: Optimal orbital transfer found with "
              << bestDeltaV << " km/s total ΔV requirement" << std::endl;

    std::cout << std::scientific;
}

template <typename T, typename Fun>
void PSO<T, Fun>::saveResults(const std::string& filename, OrbitTransferObjective<double, std::function<double(double*)>> orbitProblem) {
    std::vector<T> bestSolution = swarm.getGlobalBestPosition();

    double R1 = constant::R1;
    double R2 = constant::R2;
    double Rmax = constant::Rmax;
    double e1 = orbitProblem.getE1();
    double e2 = orbitProblem.getE2();
    double i1 = orbitProblem.getI1();
    double i2 = orbitProblem.getI2();

    // Get transfer details
    auto details = orbitProblem.getTransferDetails(bestSolution);

    std::ofstream outFile(filename);
    outFile << "# PSO Optimization Results for Orbital Transfer" << std::endl;

    // Case type determination
    int caseType = 1;
    if (i1 != 0.0 || i2 != 0.0) caseType = 2;
    else if (e1 != 0.0 || e2 != 0.0) caseType = 3;

    outFile << "# Case " << caseType << ": Transfer between ";
    switch(caseType) {
        case 1: outFile << "coplanar circular orbits"; break;
        case 2: outFile << "non-coplanar circular orbits"; break;
    }
    outFile << std::endl << std::endl;

    // Orbit parameters
    outFile << "[InitialOrbit]" << std::endl;
    outFile << "radius = " << R1 << std::endl;
    outFile << "inclination = " << i1 << std::endl;
    outFile << "raan = 0.0" << std::endl;
    outFile << "eccentricity = " << e1 << std::endl;
    outFile << "arg_periapsis = 0.0" << std::endl;

    outFile << std::endl << "[TargetOrbit]" << std::endl;
    outFile << "radius = " << R2 << std::endl;
    outFile << "inclination = " << i2 << std::endl;
    outFile << "raan = 0.0" << std::endl;
    outFile << "eccentricity = " << e2 << std::endl;
    outFile << "arg_periapsis = 0.0" << std::endl;

    // Transfer parameters
    outFile << std::endl << "[OptimalTransfer]" << std::endl;
    outFile << "initial_true_anomaly = " << details["initial_true_anomaly"] << std::endl;
    outFile << "final_true_anomaly = " << details["final_true_anomaly"] << std::endl;
    outFile << "transfer_time = " << details["transfer_time"] << std::endl;
    outFile << "is_three_impulse = " << (details["is_three_impulse"] > 0.5 ? "true" : "false") << std::endl;

    // Delta-V information
    outFile << std::endl << "[DeltaV]" << std::endl;
    outFile << "magnitude = " << details["impulse_mag_1"] << "," << details["impulse_mag_2"] << std::endl;

    if (caseType == 2) {
        outFile << "plane_change = " << details["plane_change_1"] << "," << details["plane_change_2"] << std::endl;
    }

    outFile.close();
}

template<typename T, typename F>
std::vector<T> PSO<T, F>::getBestPosition() const {
    return swarm.getGlobalBestPosition();
}

template<typename T, typename F>
T PSO<T, F>::getBestValue() const {
    return swarm.getGlobalBestValue();
}

template class PSO<double, std::function<double(double*)>>;
