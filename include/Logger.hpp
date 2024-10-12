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

class ThreadSafeQueue{
private:
  std::queue<std::vector<LogEntry>> queue;
  std::mutex mutex;

public:
  void push(const std::vector<LogEntry>& entries);
  bool pop(std::vector<LogEntry>& entries);
  bool empty() const;
};

class Logger {
private:
  ThreadSafeQueue logQueue;
  std::thread loggingThread;
  std::atomic<bool> running{true};
  std::ofstream logFile;
  std::vector<LogEntry> buffer;
  size_t batchSize = 100;

  void loggingFunction();

public: 
  Logger(const std::string& filename);

  void log(double iter, double value, double inertiaWeight, double socialWeight, double cognitiveWeight); 
  void flushBuffer();

  ~Logger();
};

#endif
