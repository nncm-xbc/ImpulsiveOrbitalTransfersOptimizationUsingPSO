#include "core/OrbitProblem.hpp"

template<typename T>
class HohmannSolution {
private:
    bool _isValid;

public:
    T _R1, _R2, _MU;
    HohmannSolution(T R1, T R2, T MU = 1.0): _R1(R1), _R2(R2), _MU(MU) {
        T ratio = R2/R1;
        _isValid = (ratio >= 0.08376 && ratio <= 15.582);
    }

    struct Solution {
        T deltaV;          // Total ΔV
        T deltaV1;         // First impulse
        T deltaV2;         // Second impulse
        bool isOptimal;    // Whether solution is globally optimal
    };

    Solution compute() const {
        Solution sol;
        sol.isOptimal = _isValid;

        // Circular orbit velocities
        T v1 = std::sqrt(_MU/_R1);
        T v2 = std::sqrt(_MU/_R2);

        // Hohmann transfer orbit semi-major axis
        T a = (_R1 + _R2)/2.0;

        sol.deltaV1 = std::sqrt(_MU*(2.0/_R1 - 1.0/a)) - v1;
        sol.deltaV2 = v2 - std::sqrt(_MU*(2.0/_R2 - 1.0/a));
        sol.deltaV = std::abs(sol.deltaV1) + std::abs(sol.deltaV2);

        return sol;
    }

    T getError(T numericalDeltaV) const {
        if (!_isValid) return -1.0;
        Solution exactSol = compute();
        return std::abs(numericalDeltaV - exactSol.deltaV)/exactSol.deltaV;
    }
};

//explicit instantiation
template class HohmannSolution<double>;
