#include "PSO.hpp"
#include "Logger.hpp"

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
                _tolerance(tolerance),
                _velMax(0.1),
                _posMin(0.0),
                _posMax(100.0) {}

template <typename T, typename Fun>
void PSO<T, Fun>::solve() {
    swarm.init();
    swarm.info();

    Logger conv_logger("../ressources/convergence_log.csv");

    for (size_t iter = 0; iter < _maxIterations; ++iter) {
        std::vector<T> GBPos_previous = swarm.getGlobalBestPosition();

        for (size_t i = 0; i < swarm.getNumParticles(); ++i) {
            auto &particle = swarm.particles[i];

            swarm.updateVelocity(particle);
            swarm.updatePosition(particle);
            swarm.updatePBestPos(particle);
        }
        swarm.updateGBestPos();
        updateWC(GBPos_previous, iter);

        if (swarm.getGlobalBestValue() < _tolerance) {
            break;
        }

        if(iter%100 == 0){
            conv_logger.log(iter, swarm.getGlobalBestValue(), swarm.getInertiaWeight(), swarm.getSocialWeight(), swarm.getCognitiveWeight());
        }
    }

    conv_logger.flushBuffer();
}

template <typename T, typename Fun>
void PSO<T, Fun>::updateWC(std::vector<T> GBPos_previous, size_t iter) {
    // Linear decrease of inertia weight with iteration count
    T currentIteration = static_cast<T>(iter);
    T maxWeight = 0.9;
    T minWeight = 0.4;

    T inertiaWeight = maxWeight - ((maxWeight - minWeight) * currentIteration / _maxIterations);

    swarm.setInertiaWeight(inertiaWeight);
}

template <typename T, typename Fun>
void PSO<T, Fun>::printResults() const {
    std::cout.setf(std::ios::scientific);
    std::cout << "\n╔═════════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║           Optimization Results                                  ║" << std::endl;
    std::cout << "╠═════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Best solution found:                                            ║" << std::endl;
    for (size_t i = 0; i < swarm.getGlobalBestPosition().size(); ++i) {
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
template class PSO<double, std::function<double(double*, size_t)>>;
