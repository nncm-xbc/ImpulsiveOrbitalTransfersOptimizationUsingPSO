#ifndef LOG_HPP
#define LOG_HPP 

#include <fstream>
#include <mutex>
#include <vector>

struct LogEntry {
  double iter;
  double value;
  double inertiaWeight;
  double socialWeight;
  double cognitiveWeight;
};

class Logger {
private:
  std::ofstream logFile;
  std::mutex fileMutex;
  std::vector<LogEntry> buffer;
  size_t batchSize = 10;

public: 
  Logger(const std::string& filename);

  void log(double iter, double value, double inertiaWeight, double socialWeight, double cognitiveWeight); 
  void flushBuffer();

  ~Logger();
};

#endif
