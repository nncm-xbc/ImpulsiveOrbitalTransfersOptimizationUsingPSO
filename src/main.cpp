#if defined(_OPENMP)

#include <functional>
#include <iostream>
#include <omp.h>
#include <vector>

#include "Functions.hpp"
#include "Swarm.hpp"

using namespace std;
using namespace Function;

int main() {

  size_t D = 2;
  size_t max_iter = 1000000;
  double tol = 1e-6;
  size_t numP = 10;
  size_t num_sswarms = 1;
  double w = 0.5;
  double c1 = 1.5, c2 = 1.5;
  double posMin = -5, posMax = 5;

  using SwarmType = Swarm<double, decltype(fun)>;

  vector<SwarmType> master;
  master.reserve(num_sswarms);

  // #pragma omp paralle  l for num_threads(num_sswarms) shared(master)
  for (size_t sub_swarm_id = 0; sub_swarm_id < num_sswarms; ++sub_swarm_id) {
    SwarmType sub_swarm(numP, D, max_iter, tol, w, c1, c2, posMin, posMax, fun);
    auto fun = [&SwarmType] {
      return Function::Sphere<double>(Swarm);
    };
    // #pragma omp critical
    master.emplace_back(sub_swarm);
  }
  // #pragma omp barrier
  // #pragma omp parallel for num_threads(num_sswarms) shared(master)
  for (SwarmType &sub_swarm : master) {
    sub_swarm.solve();
  }
  // #pragma omp barrier
  for (SwarmType &sub_swarm : master) {
    sub_swarm.info();
  }
  // #pragma omp barrier
  return 0;
}
#endif
