#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP

#include <cmath>
#include <vector>

using namespace std;

namespace Function {
// Rosenbrock function parameter
constexpr double aR = 1.0;
constexpr double bR = 100.0;
// Ackley function parameters
constexpr double aA = 20.0;
constexpr double bA = 0.2;
constexpr double cA = 2 * M_PI;

template <typename T> T Rosenbrock(double *x) {
  T result = 0;
  for (size_t i = 0; i < sizeof(x) - 1; ++i) {
    result += (aR - x[i]) * (aR - x[i]) +
              bR * (x[i + 1] - x[i] * x[i]) * (x[i + 1] - x[i] * x[i]);
  }
  return result;
}

template <typename T> T Sphere(double *x) {
  T result = 0;
  for (size_t i = 0; i < sizeof(x); ++i) {
    result += x[i] * x[i];
  }
  return result;
}

template <typename T> T Ackley(double *x) {
  T sum1 = 0;
  T sum2 = 0;

  for (size_t i = 0; i < sizeof(x); ++i) {
    sum1 += x[i] * x[i];
    sum2 += cos(cA * x[i]);
  }

  T term1 = -aA * exp(-bA * sqrt(sum1 / sizeof(x)));
  T term2 = -exp(sum2 / sizeof(x));

  return term1 + term2 + aA + exp(1.0);
}

template <typename T> T Griewank(double *x) {
  T sum = 0;
  T prod = 1;

  for (size_t i = 0; i < sizeof(x); ++i) {
    sum += x[i] * x[i] / 4000.0;
    prod *= cos(x[i] / sqrt(i + 1));
  }

  return 1.0 + sum - prod;
}

template <typename T> T Rastrigin(double *x) {
  T result = 0;
  for (size_t i = 0; i < sizeof(x); ++i) {
    result += x[i] * x[i] - 10 * cos(2 * M_PI * x[i]) + 10;
  }
  return result;
}

template <typename T> T Shaffer(double *x) {
  T term = 0;
  for (size_t i = 0; i < sizeof(x); ++i) {
    term += x[i] * x[i];
  }
  T result =
      0.5 + (std::sin(std::sqrt(term)) * std::sin(std::sqrt(term)) - 0.5) /
                ((1 + 0.001 * term) * (1 + 0.001 * term));
  return result;
}
} // namespace Function

#endif
