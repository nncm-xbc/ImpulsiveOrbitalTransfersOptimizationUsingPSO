#ifndef SWARM_HPP
#define SWARM_HPP

#include <functional>
#include <random>
#include <vector>

using namespace std;

template <typename T, typename Fun> class Swarm {
public:
  // Constructor
  Swarm(const size_t &numP, const size_t &D, const size_t &max_iter, const T &tol, const T &w,
                           const T &c1, const T &c2, const double &posMin,
                           const double &posMax, const Fun &fun);
  Swarm() = default;
  // Destructor
  ~Swarm();

  // Public interfaces
  void init(const size_t &numP, const size_t &D, const size_t &max_iter, const T &tol, const T &w, const T &c1,
            const T &c2, const double &posMin, const double &posMax,
            const Fun &fun);
  void update();
  void solve();
  void printResults(const std::vector<double> &convergenceHistory,
                    double computationTime, int numFuncEvaluations) const;
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
  void setMaxIter(const size_t &max_iter);
  void setRng();
  void setPBestPos(size_t &id);

  // Getters
  size_t getNumP() const;
  size_t getD() const;
  double getW() const;
  double getC1() const;
  double getC2() const;
  double getVelMax() const;
  double getPosMin() const;
  double getPosMax() const;
  double *getPosition(size_t &id) const;
  double *getVelocity(size_t &id) const;
  T *getPBestPos(size_t &id) const;
  T *getGBestPos() const;
  double getPBestVal() const;
  double getGBestVal() const;
  Fun getFun() const;
  double getTol() const;

  // Memory management
  void allocateMemory();
  void deallocateMemory();

private:
  // Private variables
  double **_positions;
  double **_velocities;
  double **_pBestPos;
  double *_gBestPos;
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
  size_t _max_iter;

  // Private setters
  void initPBestPos(size_t &id);
  void updatePosition(T **&positions);
  void updateVelocity(const T **&velocities);
  void updatePBestPos();
  void updatePBestVal(const double &pBestVal);
  void updateGBestPos();
  void updateWC(T *GBPos_previous);
};

#endif
