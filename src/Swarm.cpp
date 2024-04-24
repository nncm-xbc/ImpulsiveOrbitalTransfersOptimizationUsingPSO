#include "Swarm.hpp"
#include <chrono>
#include <iostream>
#include <omp.h>
#include <random>

using namespace std;
using namespace chrono;

double eps = numeric_limits<double>::epsilon();

// Constructor
template <typename T, typename Fun>
Swarm<T, Fun>::Swarm(const size_t &numP, const size_t &D, const size_t &max_iter, const T &tol, const T &w,
                         const T &c1, const T &c2, const double &posMin,
                         const double &posMax, const Fun &fun)
    : _numP(numP), _D(D), _max_iter(max_iter), _tol(tol), _w(w), _c1(c1), _c2(c2), _posMin(posMin), _posMax(posMax), _fun(fun) {
      setRng();
      allocateMemory();

      uniform_real_distribution<T> _dis(_posMin, _posMax);
      for (int p = 0; p < _numP; ++p) {
        for (int d = 0; d < _D; ++d) {
          _positions[p][d] = _dis(_rng);
          _velocities[p][d] = _dis(_rng);
        }
      }
      for (size_t p = 0; p < _numP; ++p) {
        if (p < _numP) {
          initPBestPos(p);
        } else {
          throw std::out_of_range("Particle index is out of bounds");
        }
      }
      _gBestPos = _pBestPos[0];
      cout << "Dim " << _D << endl;
      size_t dim = _D;
      _gBestVal = _fun(_gBestPos);
    #pragma omp critical
      { updateGBestPos(); }

       // checks :
       // Check size of _positions
       cout << "_numP \n" << _numP << endl;
       cout << "Size of _pos \n" << sizeof(_positions) << endl;
       // Check size of elements of _positions
       cout << "_D \n" << _D << endl;
       for (int p = 0; p < _numP; ++p) {
         for (int d = 0; d < _D; ++d) {
           cout << "_positions complete p|d :" << p << " | " << d
                << "| vla = " << _positions[p][d] << endl;
         }
       }
  }

// Destructor
template <typename T, typename Fun> Swarm<T, Fun>::~Swarm() {
  deallocateMemory();
}

//Public methods
template <typename T, typename Fun> void Swarm<T, Fun>::update() {
  // updateGBestPos();
  // #pragma omp parallel for num_threads(_numP)
  for (int p = 0; p < _numP; ++p) {
    // random factors
    vector<T> r1, r2;
    for (int d = 0; d < _D; ++d) {
      r1.emplace_back(_dis(_rng));
      r2.emplace_back(_dis(_rng));
    }
    // update velocity and position
    for (int d = 0; d < _D; ++d) {
      _velocities[p][d] = _w * _velocities[p][d] +
                          _c1 * r1[d] * (_pBestPos[p][d] - _positions[p][d]) +
                          _c2 * r2[d] * (_gBestPos[d] - _positions[p][d]);
    }
    for (int d = 0; d < _D; ++d) {
      _positions[p][d] = _positions[p][d] + _velocities[p][d];
    }
  }
}


template <typename T, typename Fun> void Swarm<T, Fun>::solve() {
  auto start = high_resolution_clock::now();
  for (int i = 0; i < _max_iter; ++i) {
    update();
    updatePBestPos();
    updateGBestPos();
    T *GBPos_previous = _gBestPos;
    // updateWC(GBPos_previous);
    if (i == _max_iter - 1 or _gBestVal < _tol) {
      auto stop = high_resolution_clock::now();
      auto duration = duration_cast<milliseconds>(stop - start);
      cout << " --> Elapsed time : " << duration.count() << " ms" << endl;
      return void();
    }
  }
}

template <typename T, typename Fun>
void Swarm<T, Fun>::updatePosition(T **&positions) {}

template <typename T, typename Fun>
void Swarm<T, Fun>::updateVelocity(const T **&velocities) {}

template <typename T, typename Fun> void Swarm<T, Fun>::updatePBestPos() {
  for (int p = 0; p < _numP; ++p) {
    if (_fun(_positions[p]) < _fun(_pBestPos[p])) {
      _pBestPos[p] = _positions[p];
    }
  }
}

template <typename T, typename Fun>
void Swarm<T, Fun>::updatePBestVal(const double &pBestVal) {}

template <typename T, typename Fun> void Swarm<T, Fun>::updateGBestPos() {
  size_t id_best = 0;
  // #pragma omp parallel for shared(id_best)
  for (size_t id = 0; id < _numP; ++id) {
    double valfun_Id = _fun(_positions[id]);
    if (valfun_Id < _fun(_positions[id_best]) && valfun_Id < _gBestVal) {
      _gBestVal = valfun_Id;
      id_best = id;
      cout << "New Global best : " << _gBestVal << endl;
    }
  }
}

template <typename T, typename Fun>
void Swarm<T, Fun>::updateWC(T *GBPos_previous) {
  if (_gBestVal < _fun(GBPos_previous)) {
    _w = min(_w * 1.2, 0.9);
    if (abs(_w - 0.9) < eps) {
      _w *= 0.95;
    }
  } else {
    _w = max(_w * 0.9, 0.1);
    if (abs(_w - 0.1) < eps) {
      _w *= 2;
    }
  }
}

template <typename T, typename Fun>
void Swarm<T, Fun>::printResults(const std::vector<double> &convergenceHistory,
                                 double computationTime,
                                 int numFuncEvaluations) const {
  std::cout << "\n=== Optimization Results ===" << std::endl;
  std::cout << "Final Global Best Position: ";
  for (int i = 0; i < _D; ++i) {
    std::cout << _gBestPos[i] << " ";
  }
  std::cout << std::endl;
  std::cout << "Final Global Best Fitness: " << _gBestVal << std::endl;
  std::cout << "Computation Time: " << computationTime << " seconds"
            << std::endl;
  std::cout << "Number of Function Evaluations: " << numFuncEvaluations
            << std::endl;
  std::cout << "Convergence History: ";
  for (const auto &fitness : convergenceHistory) {
    std::cout << fitness << " ";
  }
  std::cout << std::endl;
  std::cout << "================================" << std::endl;
}

template <typename T, typename Fun> void Swarm<T, Fun>::info() const {
  cout << "\n============================================="
       << "\n               PSO algorithm                 "
       << "\n=============================================" << endl;
  cout << " Problem Dimension   : " << _D << endl;
  cout << " Max Iter            : " << _max_iter << endl;
  cout << " Tolerance           : " << _tol << endl;
  cout << " Number of Particles : " << _numP << endl;
  cout << " Inertia Weight      : " << _w << endl;
  cout << " Cognitive Parameter : " << _c1 << endl;
  cout << " Social Parameter    : " << _c2 << endl;
  cout << " Global best positions: " << endl;
  for (int i = 0; i < _D; ++i) {
    cout << _gBestPos[i] << " ";
  }
  cout << endl;
  if (_gBestVal < _tol) {
    cout << "CONVERGENCE: Global best value    : " << _gBestVal << endl;
  } else {
    cout << "DIVERGENCE: Global best value    : " << _gBestVal << endl;
  }
  cout << "=============================================" << endl;
}

// Setters
template <typename T, typename Fun>
void Swarm<T, Fun>::setNumP(const size_t &N) {
  _numP = N;
}

template <typename T, typename Fun> void Swarm<T, Fun>::setD(const size_t &D) {
  _D = D;
}

template <typename T, typename Fun> void Swarm<T, Fun>::setW(const double &w) {
  _w = w;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::setMaxIter(const size_t &max_iter) {
  _max_iter = max_iter;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::setC1(const double &c1) {
  _c1 = c1;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::setC2(const double &c2) {
  _c2 = c2;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::setVelMax(const double &velMax) {
  _velMax = velMax;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::setPosMin(const double &posMin) {
  _posMin = posMin;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::setPosMax(const double &posMax) {
  _posMax = posMax;
}

template <typename T, typename Fun> void Swarm<T, Fun>::setFun(const Fun &fun) {
  _fun = fun;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::setTol(const double &tol) {
  _tol = tol;
}

template <typename T, typename Fun> void Swarm<T, Fun>::setRng() {
  random_device rd;
  _rng = mt19937(rd());
}

template <typename T, typename Fun>
void Swarm<T, Fun>::initPBestPos(size_t &id) {
  _pBestPos[id] = _positions[id];
}

// Getters
template <typename T, typename Fun> size_t Swarm<T, Fun>::getNumP() const {
  return _numP;
}

template <typename T, typename Fun> size_t Swarm<T, Fun>::getD() const {
  return _D;
}

template <typename T, typename Fun> double Swarm<T, Fun>::getW() const {
  return _w;
}

template <typename T, typename Fun> double Swarm<T, Fun>::getC1() const {
  return _c1;
}

template <typename T, typename Fun> double Swarm<T, Fun>::getC2() const {
  return _c2;
}

template <typename T, typename Fun> double Swarm<T, Fun>::getVelMax() const {
  return _velMax;
}

template <typename T, typename Fun> double Swarm<T, Fun>::getPosMin() const {
  return _posMin;
}

template <typename T, typename Fun> double Swarm<T, Fun>::getPosMax() const {
  return _posMax;
}

template <typename T, typename Fun>
T *Swarm<T, Fun>::getPBestPos(size_t &id) const {
  return _pBestPos[id];
}

template <typename T, typename Fun> T *Swarm<T, Fun>::getGBestPos() const {
  return _gBestPos;
}

template <typename T, typename Fun> Fun Swarm<T, Fun>::getFun() const {
  return _fun;
}

template <typename T, typename Fun> double Swarm<T, Fun>::getTol() const {
  return _tol;
}

// Memory Management
template <typename T, typename Fun> void Swarm<T, Fun>::allocateMemory() {
  _positions = new double *[_numP];
  _velocities = new double *[_numP];
  _pBestPos = new double *[_numP];
  _gBestPos = new double[_D];

  for (size_t i = 0; i < _numP; ++i) {
    _positions[i] = new double[_D];
    _velocities[i] = new double[_D];
    _pBestPos[i] = new double[_D];
  }
}
template <typename T, typename Fun> void Swarm<T, Fun>::deallocateMemory() {
  delete[] _positions;
  delete[] _velocities;
  delete[] _pBestPos;
  delete[] _gBestPos;
}

template class Swarm<double, std::function<double(double *)>>;
