#include "optimization/PSO.hpp"
#include "optimization/Logger.hpp"
#include "core/ExactSolution.hpp"
#include "optimization/PSOGlobals.hpp"
#include "core/OrbitProblem.hpp"

#include <iomanip>
#include <vector>

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

    //HohmannSolution<T> exactSolution = HohmannSolution<T>(constant::R1,constant::R2,constant::MU);

    Logger conv_logger("../ressources/convergence_log.csv");

    PSOGlobals::maxIterations = _maxIterations;

    for (size_t iter = 0; iter < _maxIterations; ++iter)
    {
        PSOGlobals::currentIteration = iter;

        //std::cout << "Iteration: " << iter << " START ----------"<< std::endl;
        std::vector<T> GBPos_previous = swarm.getGlobalBestPosition();
        //std::cout << "Global Best Pos_previous: " << GBPos_previous[0] << std::endl;


        for (size_t i = 0; i < swarm.getNumParticles(); ++i)
        {
            auto &particle = swarm.particles[i];
/*
            std::cout << "PARTICLE : " << i << " START --- Position: " << particle.getPosition()[0]
                << " Velocity: " << particle.getVelocity()[0]
                << " PBest: " << particle.getBestPosition()[0]
                << " GBest: " << swarm.getGlobalBestPosition()[0]
            << std::endl;
*/

            swarm.updateVelocity(particle);
            swarm.updatePosition(particle);
            swarm.updatePBestPos(particle);
/*
            std::cout << "PARTICLE : " << i << " END --- Position: " << particle.getPosition()[0]
                << " Velocity: " << particle.getVelocity()[0]
                << " PBest: " << particle.getBestPosition()[0]
                << " GBest: " << swarm.getGlobalBestPosition()[0]
            << std::endl;
*/
        }
        swarm.updateGBestPos();
        updateWC(GBPos_previous, iter);

/*
        T error = exactSolution.getError(swarm.getGlobalBestValue());
        std::cout << "Error: " << error << std::endl;

        if (error < _tolerance)
        {
            std::cout << "Convergence reached !!" << std::endl;
            break;
        }
*/
        if(iter%100 == 0)
        {
            conv_logger.log(iter, swarm.getGlobalBestValue(), swarm.getInertiaWeight(), swarm.getSocialWeight(), swarm.getCognitiveWeight());
        }
        //std::cout << "Iteration: " << iter << " END ----------"<< std::endl;
    }
    conv_logger.flushBuffer();
}

template <typename T, typename Fun>
void PSO<T, Fun>::updateWC(std::vector<T> GBPos_previous, size_t iter)
{
    // Linear decrease of inertia weight with iteration count
    T currentIteration = static_cast<T>(iter);
    T maxWeight = 0.9;
    T minWeight = 0.4;
    T inertiaWeight = maxWeight - ((maxWeight - minWeight) * currentIteration / _maxIterations);

    swarm.setInertiaWeight(inertiaWeight);
}

template <typename T, typename Fun>
void PSO<T, Fun>::printResults() const
{
    std::cout.setf(std::ios::scientific);
    std::cout << "\n╔═════════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║           Optimization Results                                  ║" << std::endl;
    std::cout << "╠═════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Best solution found:                                            ║" << std::endl;
    for (size_t i = 0; i < swarm.getGlobalBestPosition().size(); ++i)
    {
        std::cout << "║ " << std::setw(63) << swarm.getGlobalBestPosition()[i] << " ║" << std::endl;
    }
    std::cout << "╠═════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Best fitness value: " << std::setw(43) << swarm.getGlobalBestValue() << " ║" << std::endl;
    std::cout << "║ Max iterations: " << std::setw(47) << _maxIterations << " ║" << std::endl;
    std::cout << "║ Iterations performed: " << std::setw(41) << _maxIterations << " ║" << std::endl;
    std::cout << "╚═════════════════════════════════════════════════════════════════╝" << std::endl;

    std::cout << "\nTotal ΔV for transfer: " << swarm.getGlobalBestValue() << " km/s" << std::endl;
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

    // Add plane change info for non-coplanar transfers
    if (caseType == 2) {
        outFile << "plane_change = " << details["plane_change_1"] << "," << details["plane_change_2"] << std::endl;
    }

    outFile.close();
}

template class PSO<double, std::function<double(double*)>>;
