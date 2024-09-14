#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <iostream>
#include <vector>
#include <functional>
#include <random>
#include <cstddef>

template <typename T, typename Fun>
class Particle {
public:

    // Constructor
    Particle(const Fun &objectiveFunction, const size_t& dimension);
    Particle() = default;

    // Setters
    void setPosition(const std::vector<T>& position);
    void setVelocity(const std::vector<T>& velocity);
    void setBestPosition(const std::vector<T>& best_position);
    void setValue(const T& value);
    void setBestValue(const T& best_value);

    // Getters
    std::vector<T>& getPosition();
    std::vector<T>& getVelocity();
    const std::vector<T>& getBestPosition() const;
    const T& getValue() const;
    const T& getBestValue() const;

private:
    size_t _D; // Problem dimension

    std::vector<T> _position;
    std::vector<T> _velocity;
    std::vector<T> _best_position;
    T _value;
    T _best_value;

};

#endif // !PARTICLE_HPP