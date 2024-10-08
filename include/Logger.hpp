#ifndef LOG_HPP
#define LOG_HPP 

#include <thread>
#include <fstream>
#include <atomic>
#include <queue>

struct LogEntry {
  double iter;
  double value;
  double inertiaWeight;
  double socialWeight;
  double cognitiveWeight;
};

template<typename T>
class ThreadSafeQueue{
private:
  std::queue<LogEntry> queue;
  std::mutex mutex;

public:
  void push(const LogEntry& entry);
  bool pop(LogEntry& entry);
};

class Logger {
private:
  ThreadSafeQueue<LogEntry> logQueue;
  std::thread loggingThread;
  std::atomic<bool> running{true};
  std::ofstream logFile;

  void logginFunction();

public: 
  Logger(const std::string& filename): logFile(filename){}

  void log(double iter, double value, double inertiaWeight, double socialWeight, double cognitiveWeight);

  ~Logger();
};

#endif
