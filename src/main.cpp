/*
#if defined (_OPENMP)

#include <chrono>
#include <functional>
#include <iostream>
#include <omp.h>
#include <vector>

#include "Functions.hpp"
#include "PSO.hpp"
#include "Particle.hpp"

    using namespace std;
using namespace chrono;
using namespace Function;

int main() {
  size_t D;
  cout << "\nEnter the problem dimension:\n\n ";
  cin >> D;

  string functionName;

  cout << "\nEnter the function name:\n 1-Rosenbrock (HARD, flat global "
          "minimun region) \n 2-Sphere (EASY) \n 3-Ackley (MEDIUM, many local "
          "minima)\n 4-Griewank (VERY HARD, many local minima) \n 5-Rastrigin "
          "(VERY HARD, many local minima)\n 6-Shaffer [Original-problem] (VERY "
          "VERY VERY HARD, many local minima)\n\n ";
  cin >> functionName;

  function<double(const vector<double> &)> fun;
  using SwarmType = Swarm<T, fun>;

  vector<double> exact_solution; // exact solution of the problem

  if (functionName == "1") {
    fun = Function::Rosenbrock<double>;
    for (size_t i = 0; i < D; ++i)
      exact_solution.emplace_back(1.0);
  } else if (functionName == "2") {
    fun = Function::Sphere<double>;
    for (size_t i = 0; i < D; ++i)
      exact_solution.emplace_back(0.0);
  } else if (functionName == "3") {
    fun = Function::Ackley<double>;
    for (size_t i = 0; i < D; ++i)
      exact_solution.emplace_back(0.0);
  } else if (functionName == "4") {
    fun = Function::Griewank<double>;
    for (size_t i = 0; i < D; ++i)
      exact_solution.emplace_back(0.0);
  } else if (functionName == "5") {
    fun = Function::Rastrigin<double>;
    for (size_t i = 0; i < D; ++i)
      exact_solution.emplace_back(0.0);
  } else if (functionName == "6") {
    fun = Function::Shaffer<double>;
    for (size_t i = 0; i < D; ++i)
      exact_solution.emplace_back(0.0);
  } else {
    cerr << "Invalid function name. Exiting." << endl;
    return 1;
  }

  size_t max_iter, num_particles, num_sswarms;
  double tol;
  cout << "\nEnter the maximum number of iterations:\n\n ";
  cin >> max_iter;
  cout << "\nEnter the tolerance: \n\n ";
  cin >> tol;
  cout << "\nEnter the number of particles: \n\n ";
  cin >> num_particles;
  cout << "\nEnter the number of sub-swarms: \n\n ";
  cin >> num_sswarms;

  vector<SwarmType> master;

#pragma omp parallel for
  for (size_t sub_swarm_id = 0; sub_swarm_id < num_sswarms; ++sub_swarm_id) {
    SwarmType Swarm;
    pso.init(sub_swarm_id, max_iter, tol, 0.5, 2.0, 2.0, num_particles, fun, D,
             exact_solution);
#pragma omp critical
    { master.emplace_back(pso); }
  }
#pragma omp barrier

  master[0].info(functionName);

#pragma omp parallel for num_threads(num_sswarms)
  for (size_t i = 0; i < master.size(); ++i) {
    SwarmType &sub_swarm = master[i];
    auto start = high_resolution_clock::now();
    sub_swarm.solve();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    cout << "\nElapsed time : " << duration.count() << " ms" << endl;
  }
#pragma omp barrier
  return 0;
}
#endif
*/

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
  size_t D;
  cout << "\nEnter the problem dimension:\n\n ";
  cin >> D;

  string functionName;
  cout << "\nEnter the function name:\n ... [Function Options] ...\n\n ";
  cin >> functionName;

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

  size_t max_iter, numP, num_sswarms;
  double tol;
  cout << "\nEnter the maximum number of iterations:\n\n ";
  cin >> max_iter;
  cout << "\nEnter the tolerance: \n\n ";
  cin >> tol;
  cout << "\nEnter the number of particles: \n\n ";
  cin >> numP;
  cout << "\nEnter the number of sub-swarms: \n\n ";
  cin >> num_sswarms;

  vector<SwarmType> master;

#pragma omp parallel for
  for (size_t sub_swarm_id = 0; sub_swarm_id < num_sswarms; ++sub_swarm_id) {
    SwarmType subSwarm(numP, D);
#pragma omp critical
    { master.emplace_back(subSwarm); }
  }

  // No need for an explicit barrier here; the parallel region will join
  // at the end

  // Assuming you have an info or similar method to display swarm
  // information master[0].info(functionName);

#pragma omp parallel for
  for (SwarmType &sub_swarm : master) {
    auto start = high_resolution_clock::now();
    sub_swarm.solve(); // Ensure you have a solve method in your Swarm class
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
#pragma omp critical
    cout << "\nElapsed time : " << duration.count() << " ms" << endl;
  }
  return 0;
}
#endif
