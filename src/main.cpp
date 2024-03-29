#if defined(_OPENMP)

#include <chrono>
#include <functional>
#include <iostream>
#include <omp.h>
#include <vector>

#include "Functions.hpp"
#include "Swarm.hpp"

using namespace std;
using namespace chrono;
using namespace Function;

int main() {
  size_t D = 2;
  size_t max_iter = 10000;
  double tol = 1e-6;
  size_t numP = 10;
  size_t num_sswarms = 1;
  double w = 0.5;
  double c1 = 1.5, c2 = 1.5;
  double posMin = -5, posMax = 5;
  string functionName = "2";

  function<double(double *)> fun;
  using SwarmType = Swarm<double, decltype(fun)>;

  if (functionName == "1") {
    fun = Function::Rosenbrock<double>;
  } else if (functionName == "2") {
    fun = Function::Sphere<double>;
  } else if (functionName == "3") {
    fun = Function::Ackley<double>;
  } else if (functionName == "4") {
    fun = Function::Griewank<double>;
  } else if (functionName == "5") {
    fun = Function::Rastrigin<double>;
  } else if (functionName == "6") {
    fun = Function::Shaffer<double>;
  } else {
    cerr << "Invalid function name. Exiting." << endl;
    return 1;
  }

  vector<SwarmType> master;

#pragma omp parallel for
  for (size_t sub_swarm_id = 0; sub_swarm_id < num_sswarms; ++sub_swarm_id) {
    SwarmType subSwarm(numP, D);
    subSwarm.init(max_iter, tol, w, c1, c2, posMin, posMax, fun);
#pragma omp critical
    { master.emplace_back(subSwarm); }
  }
  master[0].info();
#pragma omp parallel for
  for (SwarmType &sub_swarm : master) {
    auto start = high_resolution_clock::now();
    sub_swarm.solve();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    // #pragma omp critical
    cout << "\nSwarm" < < < < "Elapsed time : " << duration.count() << " ms"
                                                << endl;
  }
  return 0;
}
#endif
