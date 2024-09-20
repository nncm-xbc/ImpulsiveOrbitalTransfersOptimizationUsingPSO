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
        Particle(const Fun& objectiveFunction, const size_t& dimension, std::mt19937& rng, std::uniform_real_distribution<>& dis);
        Particle() = default;

        //Setters
        void setPosition(const std::vector<T>& position);
        void setVelocity(const std::vector<T>& velocity);
        void setBestPosition(const std::vector<T>& best_position);
        void setValue(const T& value);
        void setBestValue(const T& best_value);

        //Getters
        std::vector<T>& getPosition();
        std::vector<T>& getVelocity();
        const std::vector<T>& getBestPosition() const;
        const T& getValue() const;
        const T& getBestValue() const;

    private:
        size_t _D; // Problem dimension

        std::vector<T> _position;
        std::vector<T> _velocity;
        std::vector<T> _bestPosition;
        T _value;
        T _bestValue;
};
// Constructor
template <typename T, typename Fun>
Particle<T, Fun>::Particle(const Fun& objectiveFunction, const size_t& dimension, std::mt19937& rng, std::uniform_real_distribution<>& dis) 
: _D(dimension), _position(dimension), _velocity(dimension), _bestPosition(dimension) {

        for (size_t i = 0; i < _D; ++i) {
            _position[i] = dis(rng);
            _velocity[i] = dis(rng);
        }

        _bestPosition = _position;
        _value = objectiveFunction(_position);
        _bestValue = _value;
    }    

    //Setters
    template <typename T, typename Fun>
    void Particle<T, Fun>::setPosition(const std::vector<T>& position) {
        _position = position;
    }

template <typename T, typename Fun>
    void Particle<T, Fun>::setVelocity(const std::vector<T>& velocity) {
        _velocity = velocity;
    }

template <typename T, typename Fun>
    void Particle<T, Fun>::setBestPosition(const std::vector<T>& best_position) {
        _bestPosition = best_position;
    }

template <typename T, typename Fun>
    void Particle<T, Fun>::setValue(const T& value) {
        _value = value;
    }

template <typename T, typename Fun>
    void Particle<T, Fun>::setBestValue(const T& best_value) {
        _bestValue = best_value;
    }

    //Getters
    template <typename T, typename Fun>
    std::vector<T>& Particle<T, Fun>::getPosition() {
        return _position;
    }

template <typename T, typename Fun>
    std::vector<T>& Particle<T, Fun>::getVelocity() {
        return _velocity;
    }

template <typename T, typename Fun>
    const std::vector<T>& Particle<T, Fun>::getBestPosition() const {
        return _bestPosition;
    }

template <typename T, typename Fun>
    const T& Particle<T, Fun>::getValue() const {
        return _value;
    }

template <typename T, typename Fun>
    const T& Particle<T, Fun>::getBestValue() const {
        return _bestValue;
    }

#endif // !PARTICLE_HPP