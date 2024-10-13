#include "PSO.hpp"
#include "Logger.hpp"

#include <omp.h>
#include <iomanip>


template <typename T, typename Fun>
PSO<T, Fun>::PSO(size_t numParticles, size_t dimension, size_t maxIterations,
                 T tolerance, T inertiaWeight, T cognitiveWeight,
                 T socialWeight, Fun &objectiveFunction)
    : swarm(numParticles, dimension, objectiveFunction, inertiaWeight,
            cognitiveWeight, socialWeight),
      _maxIterations(maxIterations), _tolerance(tolerance), _velMax(0.1),
      _posMin(-100.0), _posMax(100.0) {}

template <typename T, typename Fun> void PSO<T, Fun>::initialize() {
  swarm.init(swarm.getNumParticles(), swarm.getDimension(),
             swarm.getObjectiveFunction(), swarm.getInertiaWeight(),
             swarm.getCognitiveWeight(), swarm.getSocialWeight());
  swarm.info();
 }

template <typename T, typename Fun> void PSO<T, Fun>::solve() {
  initialize();
  Logger logger("../ressources/convergence_log.csv");
  for (size_t iter = 0; iter < _maxIterations; ++iter) {
    std::vector<T> GBPos_previous = swarm.getGlobalBestPosition();

    for (size_t i = 0; i < swarm.getNumParticles(); ++i) {
      auto &particle = swarm.particles[i];
      swarm.updateVelocity(particle);
      swarm.updatePosition(particle);
      swarm.updatePBestPos(particle);
    }
    swarm.updateGBestPos();

    updateWC(GBPos_previous);

    /*if (checkConvergence(iter, GbestValue, GbestPos) = true) {
        break;
    }*/
   
    if(iter%100 == 0){
      logger.log(iter, swarm.getGlobalBestValue(), swarm.getInertiaWeight(), swarm.getSocialWeight(), swarm.getCognitiveWeight());
    }
  }
  logger.flushBuffer();
}

template <typename T, typename Fun>
void PSO<T, Fun>::printResults() const {
    std::cout.setf(std::ios::scientific);
    std::cout << "\n╔════════════════════════════════════════════╗" << std::endl;
    std::cout << "║           Optimization Results             ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Best solution found:                       ║" << std::endl;
    std::cout << "║ ";
    for (const auto &val : swarm.getGlobalBestPosition()) {
        std::cout << std::setw(20) << val << " ";
    }
    std::cout << " ║" << std::endl;
    std::cout << "╠════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Best fitness value: " << std::setw(22) << swarm.getGlobalBestValue() << " ║" << std::endl;
    std::cout << "║ Number of iterations: " << std::setw(20) << _maxIterations << " ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════╝" << std::endl;

    // Additional information for orbital transfer problems
    std::cout << "\nTotal ΔV for transfer: " << swarm.getGlobalBestValue() << " km/s" << std::endl;
}

template <typename T, typename Fun>
void PSO<T, Fun>::updateWC(std::vector<T> GBPos_previous) {
  // The weight is updated based on the distance between the previous and
  // current global best positions
  T distance = 0.0;
  for (size_t i = 0; i < GBPos_previous.size(); ++i) {
    distance +=
        std::pow(GBPos_previous[i] - swarm.getGlobalBestPosition()[i], 2);
  }
  distance = std::sqrt(distance);

  T maxWeight = 0.9; 
  T minWeight = 0.4;
  T maxDistance = 1.0;
  T weightRange = maxWeight - minWeight; 
  
  T newWeight = maxWeight - (weightRange * (distance /maxDistance));
  newWeight = std::max(minWeight, std::min(maxWeight, newWeight));

  swarm.setInertiaWeight(newWeight);
}

// Explicit instantiation
template class PSO<double, std::function<double(double *, size_t)>>;
