#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP

#include <cmath>
#include <vector>
#include <functional>
#include "Swarm.hpp"

using namespace std;

namespace Function {
// Rosenbrock function parameter
const double aR = 1.0;
const double bR = 100.0;
// Ackley function parameters
const double aA = 20.0;
const double bA = 0.2;
const    double cA = 2 * M_PI;

template <typename T> double Rosenbrock(double *x, size_t sizeX) {
  double result = 0;
  for (size_t i = 0; i < sizeX - 1; ++i) {
    result += (aR - x[i]) * (aR - x[i]) +
              bR * (x[i + 1] - x[i] * x[i]) * (x[i + 1] - x[i] * x[i]);
  }
  return result;
}

template <typename T> double Sphere(const Swarm<T, std::function<double(double *)>>& swarm) {
    size_t dim = swarm.getD();
    size_t numP = swarm.getNumP();
    for ( int i=0; i<numP; i++) {
        double *x = swarm.getPositions(i);
        if (dim != 2) {

            cout << "Dimensions of X : " << dim << endl;
            cout << "x[0]: " << x[0] << endl;
            cout << "x[1]: " << x[1] << endl;
            // Handle the error or throw an exception
            throw std::out_of_range("Dim exceeds the size of the x array");
        }

        double result = 0;
        for (size_t i = 0; i < dim; ++i) {
            result += x[i] * x[i];
        }
        return result;
    }
}


template <typename T> T Ackley(double *x, size_t sizeX) {
  T sum1 = 0;
  T sum2 = 0;

  for (size_t i = 0; i < sizeX; ++i) {
    sum1 += x[i] * x[i];
    sum2 += cos(cA * x[i]);
  }

  T term1 = -aA * exp(-bA * sqrt(sum1 / sizeX));
  T term2 = -exp(sum2 / sizeX);

  return term1 + term2 + aA + exp(1.0);
}

template <typename T> T Griewank(double *x, size_t sizeX) {
  T sum = 0;
  T prod = 1;

  for (size_t i = 0; i < sizeX; ++i) {
    sum += x[i] * x[i] / 4000.0;
    prod *= cos(x[i] / sqrt(i + 1));
  }

  return 1.0 + sum - prod;
}

template <typename T> T Rastrigin(double *x, size_t sizeX) {
  T result = 0;
  for (size_t i = 0; i < sizeX; ++i) {
    result += x[i] * x[i] - 10 * cos(2 * M_PI * x[i]) + 10;
  }
  return result;
}

template <typename T> T Shaffer(double *x, size_t sizeX) {
  T term = 0;
  for (size_t i = 0; i < sizeX; ++i) {
    term += x[i] * x[i];
  }
  T result =
      0.5 + (std::sin(std::sqrt(term)) * std::sin(std::sqrt(term)) - 0.5) /
                ((1 + 0.001 * term) * (1 + 0.001 * term));
  return result;
}
} // namespace Function
#endif
