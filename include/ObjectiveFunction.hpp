#ifndef OBJECTIVEFUNCTION_HPP
#define OBJECTIVEFUNCTION_HPP

#include <string>
#include <vector>
#include <functional>

using namespace std;
namespace ObjectiveFunction {

    double Ackley(double* position, int dim);
    double Rosenbrock(double* position, int dim);
    double Rastrigin(double* position, int dim);
    double Quadratic_function(double* position, int dim);
    double SphereOne(double* position, int dim);

    pair<double, double>* get_bounds(string objFunction,int dim);
    function<double(double*, int)> get_objective_function(string objective_function_name);
}
#endif