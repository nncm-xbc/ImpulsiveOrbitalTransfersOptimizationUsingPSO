#include "Swarm.hpp"
#include <iostream>
#include <random>

using namespace std;

constexpr double eps = numeric_limits<double>::epsilon();

// Constructor
template <typename T, typename Fun>
Swarm<T, Fun>::Swarm(const size_t &numP, const size_t &D) : _numP(numP), _D(D) {
  allocateMemory();
}

// Destructor
template <typename T, typename Fun> Swarm<T, Fun>::~Swarm() {
  deallocateMemory();
}

// Public Interfaces
template <typename T, typename Fun>
void Swarm<T, Fun>::init(const T &tol, const T &w, const T &c1, const T &c2,
                         const double &posMin, const double &posMax) {
  setW(w);
  setC1(c1);
  setC2(c2);
  setPosMin(posMin);
  setPosMax(posMax);
  setTol(tol);

  setRng();
  uniform_real_distribution<T> _dis(_posMin, _posMax);

  for (int p = 0; p < _numP; ++p) {
    T *pos = new T[_D];
    T *vel = new T[_D];
    for (int d = 0; d < _D; ++d) {
      pos[d] = _dis(_rng);
      vel[d] = _dis(_rng);
    }
    _positions[p] = pos;
    _velocities[p] = vel;
    delete[] pos;
    delete[] vel;
  }
  for (size_t p = 0; p < _numP; ++p) {
    if (p < _numP) {
      for (size_t i = 0; i < _D; ++i) {
        _pBestPos[p][i] = getPBestPos()[i];
      }
    } else {
      // Handle the case where p is out of bounds, e.g., log an error or throw
      // an exception
      throw std::out_of_range("Particle index is out of bounds");
    }
  }
  _gBestPos = getGBestPos();
}

template <typename T, typename Fun> void Swarm<T, Fun>::update() {
  _gBestPos = getGBestPos();
  // #pragma omp parallel for
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
                          _c1 * r1[d] *
                              (_pBestPos[p][d] -
                               _positions[p][d]) /* or get positions (decide how
                                                    to populate _positions) */
                          + _c2 * r2[d] * (_gBestPos[d] - _positions[p][d]);
    }
    for (int d = 0; d < _D; ++d) {
      _positions[p][d] = _positions[p][d] + _velocities[p][d];
    }
    _pBestPos[p] = getPBestPos();
  }
  T *gBestPos_new = getGBestPos();

  if (_fun(gBestPos_new) < _fun(_gBestPos)) {
    updateGBestPos(gBestPos_new);
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

template <typename T, typename Fun> void Swarm<T, Fun>::solve() {
  for (int i = 0; i < _max_iter; ++i) {
    update();
    updatePBestPos();
    getGBestPos();
    if (i == _max_iter - 1) {
      printResults();
      return void();
    }
  }
}

template <typename T, typename Fun> void Swarm<T, Fun>::printResults() const {
  // cout << "\n Tolerance achieved: " << errorNorm(_gBestPos) << endl;
  cout << "\n Global Best Position: ";
  for (int i = 0; i < _D; ++i) {
    cout << _gBestPos[i] << " ";
  }
  cout << endl;
  cout << "\n Global Best Value: " << _fun(_gBestPos) << endl;
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

template <typename T, typename Fun> T *Swarm<T, Fun>::getPBestPos() const {
  for (size_t i = 0; i < _numP; ++i) {
    _pBestPos[i] = _positions[i];
  }
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
  _positions = new T *[_numP];
  _velocities = new T *[_numP];
  _pBestPos = new T *[_numP];
  _gBestPos = new T[_D];

  for (size_t i = 0; i < _numP; ++i) {
    _positions[i] = new T[_D];
    _velocities[i] = new T[_D];
    _pBestPos[i] = new T[_D];
  }
}
template <typename T, typename Fun> void Swarm<T, Fun>::deallocateMemory() {
  for (size_t i = 0; i < _numP; ++i) {
    delete[] _positions[i];
    delete[] _velocities[i];
    delete[] _pBestPos[i];
  }
  delete[] _positions;
  delete[] _velocities;
  delete[] _pBestPos;
  delete[] _gBestPos;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::updatePosition(T **&positions) {}

template <typename T, typename Fun>
void Swarm<T, Fun>::updateVelocity(const T **&velocities) {}

template <typename T, typename Fun> void Swarm<T, Fun>::updatePBestPos() {
  for (int p = 0; p < _numP; ++p) {
    _pBestPos[p] = getPBestPos();
  }
}

template <typename T, typename Fun>
void Swarm<T, Fun>::updatePBestVal(const double &pBestVal) {}

template <typename T, typename Fun>
void Swarm<T, Fun>::updateGBestPos(const T *gBestPos) {
  size_t id_best = 0;
  for (size_t id = 0; id < _numP; ++id) {
    if (_fun(_positions[id]) < _fun(_positions[id_best]) ||
        (_fun(_positions[id]) < _gBestVal)) {
      _gBestVal = _fun(_positions[id]);
      id_best = id;
    }
  }
  _gBestPos = _positions[id_best];
}

template class Swarm<double, std::function<double(double *)>>;
