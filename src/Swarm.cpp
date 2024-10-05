#include "Swarm.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <omp.h>

template <typename T, typename Fun>
Swarm<T, Fun>::Swarm(const size_t &numParticles, const size_t &dimension,
                     const Fun &objectiveFunction, const T inertiaWeight,
                     const T cognitiveWeight, const T socialWeight)
    : _numParticles(numParticles), _dimension(dimension),
      _objectiveFunction(objectiveFunction), _inertiaWeight(inertiaWeight),
      _cognitiveWeight(cognitiveWeight), _socialWeight(socialWeight) {
  std::random_device rd;
  _rng = std::mt19937(rd());
  _dis = std::uniform_real_distribution<T>(0.0, 1.0);
}

template <typename T, typename Fun> Swarm<T, Fun>::~Swarm() {
  deallocateMemory();
}

template <typename T, typename Fun>
void Swarm<T, Fun>::init(const size_t &numParticles, const size_t &dimension,
                         const Fun &objectiveFunction, const T &inertiaWeight,
                         const T &cognitiveWeight, const T &socialWeight) {
  _numParticles = numParticles;
  _dimension = dimension;
  _objectiveFunction = objectiveFunction;
  _inertiaWeight = inertiaWeight;
  _cognitiveWeight = cognitiveWeight;
  _socialWeight = socialWeight;
  _gBestPos.resize(dimension);
  _gBestVal = std::numeric_limits<T>::max();
  particles.resize(numParticles,
                   Particle<T, Fun>(objectiveFunction, dimension, _rng, _dis));
}

template <typename T, typename Fun> void Swarm<T, Fun>::info() const {
  std::cout << "Swarm Information:" << std::endl;
  std::cout << "Number of Particles: " << _numParticles << std::endl;
  std::cout << "Dimension: " << _dimension << std::endl;
  std::cout << "Hyper-parameters:" << std::endl;
  std::cout << "Intertia weight: " << _inertiaWeight << std::endl;
  std::cout << "Cognitive weight: " << _cognitiveWeight << std::endl;
  std::cout << "Social weight: " << _socialWeight << std::endl;
  std::cout << "Global Best Value: " << _gBestVal << std::endl;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::updatePosition(Particle<T, Fun> &particle) {
  std::vector<T> newPosition = particle.getPosition();
  const std::vector<T> &velocity = particle.getVelocity();

  for (size_t i = 0; i < _dimension; ++i) {
    newPosition[i] += velocity[i];
  }

  particle.setPosition(newPosition, _objectiveFunction);
}

template <typename T, typename Fun>
void Swarm<T, Fun>::updateVelocity(Particle<T, Fun> &particle) {
  std::vector<T> newVelocity(particle.getVelocity());
  const std::vector<T> &position = particle.getPosition();
  const std::vector<T> &pBest = particle.getBestPosition();

  for (size_t i = 0; i < _dimension; ++i) {
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
  particle.setBestValue(
      _objectiveFunction(particle.getBestPosition().data(), _dimension));
}

template <typename T, typename Fun> void Swarm<T, Fun>::updateGBestPos() {
  for (size_t i = 0; i < _numParticles; ++i) {
    T newBestValue = particles[i].getBestValue();
    if (newBestValue < _gBestVal) {
      if (newBestValue < _gBestVal) {
        _gBestVal = newBestValue;
        _gBestPos = particles[i].getBestPosition();
      }
    }
  }
}

template <typename T, typename Fun>
size_t Swarm<T, Fun>::getNumParticles() const {
  return _numParticles;
}

template <typename T, typename Fun> size_t Swarm<T, Fun>::getDimension() const {
  return _dimension;
}

template <typename T, typename Fun>
Fun Swarm<T, Fun>::getObjectiveFunction() const {
  return _objectiveFunction;
}

template <typename T, typename Fun> T Swarm<T, Fun>::getInertiaWeight() const {
  return _inertiaWeight;
}

template <typename T, typename Fun>
T Swarm<T, Fun>::getCognitiveWeight() const {
  return _cognitiveWeight;
}

template <typename T, typename Fun> T Swarm<T, Fun>::getSocialWeight() const {
  return _socialWeight;
}

template <typename T, typename Fun>
std::vector<T> Swarm<T, Fun>::getPosition(Particle<T, Fun> &particle) const {
  return particle.getPosition();
}

template <typename T, typename Fun>
std::vector<T> Swarm<T, Fun>::getVelocity(Particle<T, Fun> &particle) const {
  return particle.getVelocity();
}

template <typename T, typename Fun>
std::vector<T> Swarm<T, Fun>::getGlobalBestPosition() const {
  return _gBestPos;
}

template <typename T, typename Fun>
double Swarm<T, Fun>::getGlobalBestValue() const {
  return _gBestVal;
}

// Setters
template <typename T, typename Fun>
void Swarm<T, Fun>::setNumParticles(const size_t &numParticles) {
  _numParticles = numParticles;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::setDimension(const size_t &dimension) {
  _dimension = dimension;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::setObjectiveFunction(const Fun &objectiveFunction) {
  _objectiveFunction = objectiveFunction;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::setInertiaWeight(const T &inertiaWeight) {
  _inertiaWeight = inertiaWeight;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::setCognitiveWeight(const T &cognitiveWeight) {
  _cognitiveWeight = cognitiveWeight;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::setSocialWeight(const T &socialWeight) {
  _socialWeight = socialWeight;
}

// Memory management
template <typename T, typename Fun> void Swarm<T, Fun>::deallocateMemory() {
  particles.clear();
}

// Explicit instantiation
template class Swarm<double, std::function<double(double*, size_t)>>;
