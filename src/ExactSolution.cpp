#include <vector>

template<typename T>
class OrbitTransferExactSolution : public AbstractExactSolution<T>
{
    private:
        std::vector<T> _initialOrbit;
        std::vector<T> _finalOrbit;
        T _tolerance;

    public:
        OrbitTransferExactSolution(const std::vector<T>& initial,
                                const std::vector<T>& final,
                                T tol = 1e-6);
        T evaluate(const std::vector<T>& parameters) const override;
        bool isValid(const std::vector<T>& solution) const override;
        T getError(const std::vector<T>& numericalSolution) const override;
};
