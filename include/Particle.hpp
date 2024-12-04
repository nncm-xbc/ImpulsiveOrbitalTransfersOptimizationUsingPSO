#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <cstddef>
#include <functional>
#include <iostream>
#include <random>
#include <vector>

template <typename T, typename Fun> class Particle {
public:
  Particle(const Fun objectiveFunction,
            const size_t &dimension,
            std::mt19937 &rng,
            std::vector<std::uniform_real_distribution<T>> &dis,
            std::vector<T> adjustedRange);
  Particle() = default;

  // Setters
  void setPosition(const std::vector<T> &position,const Fun &objectiveFunction);
  void setVelocity(const std::vector<T> &velocity);
  void setBestPosition(const std::vector<T> &best_position);
  void setValue(const T &value);
  void setBestValue(const T &best_value);

  // Getters
  std::vector<T> &getPosition();
  std::vector<T> &getVelocity();
  std::vector<T> &getBestPosition();
  T &getValue();
  T &getBestValue();

private:
  size_t _dimension; // Problem dimension

  std::vector<T> _position;
  std::vector<T> _velocity;
  std::vector<T> _bestPosition;
  T _value;
  T _bestValue;
};
// Constructor
template <typename T, typename Fun>
Particle<T, Fun>::Particle(const Fun objectiveFunction,
                            const size_t &dimension,
                            std::mt19937 &rng,
                            std::vector<std::uniform_real_distribution<T>> &dis,
                            std::vector<T> adjustedRange):
                        _dimension(dimension),
                        _position(dimension),
                        _velocity(dimension),
                        _bestPosition(dimension) {
    double velocityScale = 0.1; // 10% of position range
    for (size_t i = 0; i < _dimension; ++i) {
        _position[i] = dis[i](rng);
        _velocity[i] = velocityScale * adjustedRange[i] * (dis[i](rng) - 0.5);  }

    _bestPosition = _position;
    _value = objectiveFunction(_position.data(), _dimension);
    _bestValue = _value;
}

// Setters
template <typename T, typename Fun>
void Particle<T, Fun>::setPosition(const std::vector<T> &position, const Fun &objectiveFunction) {
  if (position.size() != _dimension) {
    throw std::invalid_argument("Position dimension mismatch");
  }
  _position = position;
  _value = objectiveFunction(_position.data(), _dimension);
}

template <typename T, typename Fun>
void Particle<T, Fun>::setVelocity(const std::vector<T> &velocity) {
  if (velocity.size() != _dimension) {
    throw std::invalid_argument("Velocity dimension mismatch");
  }
  _velocity = velocity;
}

template <typename T, typename Fun>
void Particle<T, Fun>::setBestPosition(const std::vector<T> &best_position) {
  if (best_position.size() != _dimension) {
    throw std::invalid_argument("Position dimension mismatch");
  }
  _bestPosition = best_position;
}

template <typename T, typename Fun>
void Particle<T, Fun>::setValue(const T &value) {
  _value = value;
}

template <typename T, typename Fun>
void Particle<T, Fun>::setBestValue(const T &best_value) {
  _bestValue = best_value;
}

// Getters
template <typename T, typename Fun>
std::vector<T> &Particle<T, Fun>::getPosition() {
  return _position;
}

template <typename T, typename Fun>
std::vector<T> &Particle<T, Fun>::getVelocity() {
  return _velocity;
}

template <typename T, typename Fun>
std::vector<T> &Particle<T, Fun>::getBestPosition() {
  return _bestPosition;
}

template <typename T, typename Fun> T &Particle<T, Fun>::getValue() {
  return _value;
}

template <typename T, typename Fun> T &Particle<T, Fun>::getBestValue() {
  return _bestValue;
}

#endif // !PARTICLE_HPP
