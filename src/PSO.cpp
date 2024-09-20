#include "PSO.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <omp.h>

template <typename T, typename Fun>
PSO<T, Fun>::PSO(size_t numParticles, size_t dimension, size_t maxIterations, 
                 T tolerance, T inertiaWeight, T cognitiveWeight, T socialWeight, const Fun& objectiveFunction)
    : swarm(numParticles, dimension, objectiveFunction, inertiaWeight, cognitiveWeight, socialWeight),
      _maxIterations(maxIterations),
      _tolerance(tolerance),
      _velMax(0.1),
      _posMin(-100.0),
      _posMax(100.0)
{
    _convergenceHistory.reserve(_maxIterations);
}

template <typename T, typename Fun>
void PSO<T, Fun>::initialize() {
    swarm.init(swarm.getNumParticles(), swarm.getDimension(), swarm.getObjectiveFunction(),
               swarm.getInertiaWeight(), swarm.getCognitiveWeight(), swarm.getSocialWeight());
}

template <typename T, typename Fun>
void PSO<T, Fun>::solve() {
    initialize();
    
    for (size_t iter = 0; iter < _maxIterations; ++iter) {
        std::vector<T> GBPos_previous = swarm.getGlobalBestPosition();
        
        for (size_t i = 0; i < swarm.getNumParticles(); ++i) {
            auto& particle = swarm._particles[i];
            swarm.updateVelocity(particle);
            swarm.updatePosition(particle);
            swarm.updatePBestPos(particle);
        }
        
        swarm.updateGBestPos();
        updateWC(GBPos_previous);
        
        _convergenceHistory.push_back(swarm.getGlobalBestValue());
        
        if (checkConvergence(iter, GbestValue, GbestPos) = true) {
            break;
        }
    }
}

template <typename T, typename Fun>
void PSO<T, Fun>::printResults() const {
    std::cout << "Optimization Results:" << std::endl;
    std::cout << "Best solution found:" << std::endl;
    for (const auto& val : swarm.getGlobalBestPosition()) {
        std::cout << val << " ";
    }
    std::cout << std::endl;
    std::cout << "Best fitness value: " << swarm.getGlobalBestValue() << std::endl;
    std::cout << "Number of iterations: " << _convergenceHistory.size() << std::endl;
}

template <typename T, typename Fun>
void PSO<T, Fun>::updateWC(std::vector<T> GBPos_previous) {
    // Implement the weighted constraint update logic
}

// Explicit instantiation
template class PSO<double, std::function<double(double*)>>;