#include "PSO.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <omp.h>

template <typename T, typename Fun>
PSO<T, Fun>::PSO(size_t numParticles, size_t dimension, size_t maxIterations, 
                 T tolerance, T inertiaWeight, T cognitiveWeight, T socialWeight, 
                 const Fun& objectiveFunction)
    : swarm(numParticles, dimension, objectiveFunction),
      maxIterations(maxIterations),
      tolerance(tolerance),
      inertiaWeight(inertiaWeight),
      cognitiveWeight(cognitiveWeight),
      socialWeight(socialWeight),
      _velMax(1.0),
      _posMin(-100.0),
      _posMax(100.0),
      _rng(std::random_device{}()),
      _dis(_posMin, _posMax)
{
    initialize();
}

template <typename T, typename Fun>
void PSO<T, Fun>::initialize() {
    swarm.init(swarm.getNumParticles(), swarm.getDimension(), swarm.getObjectiveFunction());
    
    #pragma omp parallel for
    for (size_t i = 0; i < swarm.getNumParticles(); ++i) {
        auto& particle = swarm._particles[i];
        std::vector<T> position(swarm.getDimension());
        std::vector<T> velocity(swarm.getDimension());
        
        for (size_t j = 0; j < swarm.getDimension(); ++j) {
            position[j] = _dis(_rng);
            velocity[j] = _dis(_rng) * 0.1; // Initialize with small velocities
        }
        
        particle.setPosition(position);
        particle.setVelocity(velocity);
        swarm.initPBestPos(particle);
    }
    
    swarm.updateGBestPos();
}

template <typename T, typename Fun>
void PSO<T, Fun>::solve() {
    auto start = std::chrono::high_resolution_clock::now();
    
    for (size_t iter = 0; iter < maxIterations; ++iter) {
        #pragma omp parallel for
        for (size_t i = 0; i < swarm.getNumParticles(); ++i) {
            auto& particle = swarm._particles[i];
            swarm.updateVelocity(particle);
            swarm.updatePosition(particle);
            swarm.updatePBestPos(particle);
        }
        
        std::vector<T> previousGBest = swarm.getGlobalBestPosition();
        swarm.updateGBestPos();
        updateWC(previousGBest);
        
        T currentBestValue = swarm.getGlobalBestValue();
        convergenceHistory.push_back(currentBestValue);
        
        if (currentBestValue < tolerance) {
            break;
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Optimization completed in " << elapsed.count() << " seconds." << std::endl;
}

template <typename T, typename Fun>
void PSO<T, Fun>::printResults() const {
    std::cout << "Best solution found:" << std::endl;
    const auto& bestPosition = swarm.getGlobalBestPosition();
    for (size_t i = 0; i < bestPosition.size(); ++i) {
        std::cout << "x[" << i << "] = " << bestPosition[i] << std::endl;
    }
    std::cout << "Objective function value: " << swarm.getGlobalBestValue() << std::endl;
    
    // Print convergence history
    std::cout << "Convergence history:" << std::endl;
    for (size_t i = 0; i < convergenceHistory.size(); ++i) {
        std::cout << "Iteration " << i << ": " << convergenceHistory[i] << std::endl;
    }
}

template <typename T, typename Fun>
void PSO<T, Fun>::updateWC(std::vector<T> GBPos_previous) {
    if (swarm.getGlobalBestValue() < swarm.getObjectiveFunction()(GBPos_previous.data())) {
        inertiaWeight = std::min(inertiaWeight * 1.2, 0.9);
    } else {
        inertiaWeight = std::max(inertiaWeight * 0.9, 0.1);
    }
}

// Explicit instantiation
template class PSO<double, std::function<double(double*)>>;