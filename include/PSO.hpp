#ifndef PSO_HPP
#define PSO_HPP

#include "swarm.hpp"

template <typename T, typename Fun>
class PSO {
public:
    PSO(size_t numParticles, size_t dimension, size_t maxIterations, 
        T tolerance, T inertiaWeight, T cognitiveWeight, T socialWeight, 
        const Fun& objectiveFunction);

    void initialize();
    void solve();
    void printResults() const;

private:
    Swarm<T, Fun> swarm;
    size_t maxIterations;
    T tolerance;
    T inertiaWeight;
    T cognitiveWeight;
    T socialWeight;
    std::vector<T> convergenceHistory;

    double _velMax;
    double _posMin;
    double _posMax

    void updateWC(std::vector<T> GBPos_previous);

    //TODO
    mt19937 _rng; // To be addressed
    uniform_real_distribution<T> _dis; // To be addressed
    mt19937_64 _rng{random_device{}()};
    uniform_real_distribution<T> _dist_position{-32.0, 32.0}; // to be addressed
    uniform_real_distribution<T> _dist_velocity{-1.0, 1.0}; // to be addressed
};

#endif