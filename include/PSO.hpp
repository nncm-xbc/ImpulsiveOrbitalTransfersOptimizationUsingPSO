#ifndef PSO_HPP
#define PSO_HPP

#include "Swarm.hpp"

template <typename T, typename Fun>
class PSO {
public:
  PSO(size_t numParticles, size_t dimension, size_t maxIterations, T tolerance,
      T inertiaWeight, T cognitiveWeight, T socialWeight,
      Fun &objectiveFunction);

  void solve();
  // bool checkConvergence(int iter, T globalBestValue, std::vector<T>
  // globalBestPosition);
  void printResults() const;

private:
  Swarm<T, Fun> swarm;
  size_t _maxIterations;
  T _tolerance;

  double _velMax;
  double _posMin;
  double _posMax;

  void updateWC(std::vector<T> GBPos_previous, size_t iter);
};

#endif
