#ifndef SWARM_HPP
#define SWARM_HPP

#include <functional>
#include <random>
#include <vector>

using namespace std;

template <typename T, typename Fun> class Swarm {
public:
  // Constructor
  Swarm(const size_t &numP, const size_t &D);
  Swarm() = default;
  // Destructor
  ~Swarm();

  // Public interfaces
  void init(const T &tol, const T &w, const T &c1, const T &c2,
            const double &posMin, const double &posMax);
  void update();
  void solve();
  void printResults() const; // print or visualization or record metrics method
  void info() const;

  // Setters
  void setNumP(const size_t &N);
  void setD(const size_t &D);
  void setW(const double &w);
  void setC1(const double &c1);
  void setC2(const double &c2);
  void setVelMax(const double &velMax);
  void setPosMin(const double &posMin);
  void setPosMax(const double &posMax);
  void setFun(const Fun &fun);
  void setTol(const double &tol);
  void setRng();

  // Getters
  size_t getNumP() const;
  size_t getD() const;
  double getW() const;
  double getC1() const;
  double getC2() const;
  double getVelMax() const;
  double getPosMin() const;
  double getPosMax() const;
  T *getPosition(size_t &id) const;
  T *getVelocity(size_t &id) const;
  T *getPBestPos() const;
  T *getGBestPos() const;
  double getPBestVal() const;
  double getGBestVal() const;
  Fun getFun() const;
  double getTol() const;

private:
  // Private variables
  T **_positions;
  T **_velocities;
  T **_pBestPos;
  T *_gBestPos;
  double _pBestVal;
  double _gBestVal;
  size_t _numP;
  size_t _D;
  double _w;
  double _c1;
  double _c2;
  double _velMax;
  double _posMin;
  double _posMax;
  Fun _fun;
  double _tol;
  mt19937 _rng;
  uniform_real_distribution<T> _dis;
  double _max_iter;

  // Memory management
  void allocateMemory();
  void deallocateMemory();

  // Private setters
  void updatePosition(T **&positions);
  void updateVelocity(const T **&velocities);
  void updatePBestPos();
  void updatePBestVal(const double &pBestVal);
  void updateGBestPos(const T *gBestPos);
};

#endif
