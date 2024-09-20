#include "Swarm.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <omp.h>

template <typename T, typename Fun>
Swarm<T, Fun>::Swarm(const size_t &numParticles,
                      const size_t &dimension,
                      const Fun &objectiveFunction,
                      const T inertiaWeight, 
                      const T cognitiveWeight, 
                      const T socialWeight):
                       _numParticles(numParticles), 
                       _Dimension(dimension), 
                       _objectiveFunction(objectiveFunction),
                       _inertiaWeight(inertiaWeight),
                       _cognitiveWeight(cognitiveWeight),
                       _socialWeight(socialWeight)
                       {
    allocateMemory();
    std::random_device rd;
    _rng = std:: mt19937(rd());
    _dis = std::uniform_real_distribution<T>(0.0, 1.0);
}

template <typename T, typename Fun>
Swarm<T, Fun>::~Swarm() {
    deallocateMemory();
}

template <typename T, typename Fun>
void Swarm<T, Fun>::init(const size_t &numParticles,
                      const size_t &dimension,
                      const Fun &objectiveFunction,
                      const T &inertiaWeight, 
                      const T &cognitiveWeight, 
                      const T &socialWeight) {
    _numParticles = numParticles;
    _Dimension = dimension;
    _objectiveFunction = objectiveFunction;
    _inertiaWeight = inertiaWeight;
    _cognitiveWeight = cognitiveWeight;
    _socialWeight = socialWeight;
    _particles.resize(numParticles, Particle<T, Fun>(objectiveFunction, dimension, _rng, _dis));
    _gBestPos.resize(dimension);
    _gBestVal = std::numeric_limits<T>::max();
}

template <typename T, typename Fun>
void Swarm<T, Fun>::info() const {
    std::cout << "Swarm Information:" << std::endl;
    std::cout << "Number of Particles: " << _numParticles << std::endl;
    std::cout << "Dimension: " << _Dimension << std::endl;
    std::cout << "Hyper-parameters:" << std::endl;
    std::cout << "Intertia weight: " << _inertiaWeight << std::endl;
    std::cout << "Cognitive weight: " << _cognitiveWeight << std::endl;
    std::cout << "Social weight: " << _socialWeight << std::endl;
    std::cout << "Global Best Value: " << _gBestVal << std::endl;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::initPBestPos(Particle<T, Fun> &particle) {
    particle.setBestPosition(particle.getPosition());
    particle.setBestValue(particle.getValue());
}

template <typename T, typename Fun>
void Swarm<T, Fun>::updatePosition(Particle<T, Fun> &particle) {
    std::vector<T> newPosition = particle.getPosition();
    const std::vector<T>& velocity = particle.getVelocity();
    
    for (size_t i = 0; i < _Dimension; ++i) {
        newPosition[i] += velocity[i];
    }
    
    particle.setPosition(newPosition);
    particle.setValue(_objectiveFunction(newPosition.data()));
}

template <typename T, typename Fun>
void Swarm<T, Fun>::updateVelocity(Particle<T, Fun> &particle) {
    std::vector<T> newVelocity(particle.getVelocity());
    const std::vector<T>& position = particle.getPosition();
    const std::vector<T>& pBest = particle.getBestPosition();
    
    for (size_t i = 0; i < _Dimension; ++i) {
        T r1 = _dis(_rng);
        T r2 = _dis(_rng);
        newVelocity[i] = _inertiaWeight * newVelocity[i] +
                          _cognitiveWeight * r1 * (pBest[i] - position[i]) +
                          _socialWeight * r2 * (_gBestPos[i] - position[i]);
    }
    particle.setVelocity(newVelocity);
}

template <typename T, typename Fun>
void Swarm<T, Fun>::updatePBestPos(Particle<T, Fun> &particle) {
    if (particle.getValue() < particle.getBestValue()) {
        particle.setBestPosition(particle.getPosition());
        particle.setBestValue(particle.getValue());
    }
}

template <typename T, typename Fun>
void Swarm<T, Fun>::updatePBestVal(Particle<T, Fun> &particle) {
    particle.setBestValue(_objectiveFunction(particle.getBestPosition().data()));
}

template <typename T, typename Fun>
void Swarm<T, Fun>::updateGBestPos() {
    for (size_t i = 0; i < _numParticles; ++i) {
      T newBestValue = _particles[i].getBestValue();
        if (newBestValue < _gBestVal) {
          if (newBestValue < _gBestVal) {
            _gBestVal = newBestValue;
            _gBestPos = _particles[i].getBestPosition();
          }
        }
    }
}

// TODO: setters, getters, memory management

// Explicit instantiation
template class Swarm<double, std::function<double(double*)>>;