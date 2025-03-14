#include "PSO.hpp"
#include "Logger.hpp"
#include "ExactSolution.hpp"
#include "OrbitProblem.hpp"

#include <omp.h>
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

    HohmannSolution<T> exactSolution = HohmannSolution<T>(constant::R1,constant::R2,constant::MU);

    Logger conv_logger("../ressources/convergence_log.csv");

    for (size_t iter = 0; iter < _maxIterations; ++iter)
    {
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

        T error = exactSolution.getError(swarm.getGlobalBestValue());
        std::cout << "Error: " << error << std::endl;

        if (error < _tolerance)
        {
            std::cout << "Convergence reached !!" << std::endl;
            break;
        }

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

    // Additional information for orbital transfer problems
    std::cout << "\nTotal ΔV for transfer: " << swarm.getGlobalBestValue() << " km/s" << std::endl;
}

// Explicit instantiation
template class PSO<double, std::function<double(double*)>>;
