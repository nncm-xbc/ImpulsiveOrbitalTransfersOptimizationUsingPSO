#ifndef PSO_HPP
#define PSO_HPP

#include "OrbitProblem.hpp"
#include "Swarm.hpp"

template <typename T, typename Fun>
class PSO
{
    public:
        PSO(size_t numParticles,
            size_t dimension,
            size_t maxIterations,
            T tolerance,
            T inertiaWeight,
            T cognitiveWeight,
            T socialWeight,
            const Fun &objectiveFunction,
            const std::vector<T> lowerBounds,
            const std::vector<T> upperBounds);

        void solve();
        void printResults() const;
        void saveResults(const std::string& filename, OrbitTransferObjective<double, std::function<double(double*)>> orbitProblem);

    private:
        Swarm<T, Fun> swarm;
        size_t _maxIterations;
        T _tolerance;

        void updateWC(std::vector<T> GBPos_previous, size_t iter);
};
#endif
