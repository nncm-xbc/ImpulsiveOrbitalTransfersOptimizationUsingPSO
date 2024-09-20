#ifndef SWARM_HPP
#define SWARM_HPP

#include "Particle.hpp"
#include <vector>

template <typename T, typename Fun>
class Swarm {
public:
  // Constructor
  Swarm(const size_t &numParticles,
        const size_t &dimension,
        const Fun &objectiveFunction,
        const T inertiaWeight,
        const T cognitiveWeight,
        const T socialWeight);

  Swarm() = default;

  // Destructor
  ~Swarm();

  // Public interfaces
  void init(const size_t &numParticles,
            const size_t &dimension, 
            const Fun &objectiveFunction,
            const T &inertiaWeight,
            const T &cognitiveWeight,
            const T &socialWeight);
  
  void info() const;

  //Update logic
  void initPBestPos(Particle<T, Fun> &particle);
  void updatePosition(Particle<T, Fun> &particle);
  void updateVelocity(Particle<T, Fun> &particle);
  void updatePBestPos(Particle<T, Fun> &particle);
  void updatePBestVal(Particle<T, Fun> &particle);
  void updateGBestPos();

  // Setters
  void setNumParticles(const size_t &numParticles);
  void setDimension(const size_t &dimension);
  void setObjectiveFunction(const Fun &objectiveFunction);
  void setInertiaWeight(const T &inertiaWeight);
  void setCognitiveWeight(const T &cognitiveWeight);
  void setSocialWeight(const T &socialWeight);

  // Getters
  size_t getNumParticles() const;
  size_t getDimension() const;
  Fun getObjectiveFunction() const;
  T getInertiaWeight() const;
  T getCognitiveWeight() const;
  T getSocialWeight() const;
  std::vector<T> getPosition(Particle<T, Fun> &particle) const;
  std::vector<T> getVelocity(Particle<T, Fun> &particle) const;
  std::vector<T> getGlobalBestPosition() const;
  double getGlobalBestValue() const;

  // Memory management
  void allocateMemory();
  void deallocateMemory();

private:
  // Private variables
  size_t _numParticles;
  size_t _Dimension;
  Fun _objectiveFunction;
  T _inertiaWeight;
  T _cognitiveWeight;
  T _socialWeight;
  std::vector<Particle<T, Fun>> _particles;
  std::vector<T> _gBestPos;
  T _gBestVal;
  std::mt19937 _rng; 
  std::uniform_real_distribution<double> _dis;
};

#endif
