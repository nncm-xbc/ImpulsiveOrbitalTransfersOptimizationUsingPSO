#include "Logger.hpp"
#include <thread>
#include <fstream>
#include <atomic>
#include <mutex>
#include <queue>

Logger(const std::string& filename): logFile(filename){
  loggingThread = std::thread(&Logger::loggingFunction, this);
}

void Logger::logginFunction(){
  while (running) {
    LogEntry entry;
    if (logQueue.pop(entry)) {
      logFile << entry.iter << "," << entry.value << "," << entry.inertiaWeight << "," << entry.socialWeight << "," << entry.cognitiveWeight << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void Logger::log(double iter, double value, double inertiaWeight, double socialWeight, double cognitiveWeight){
  logQueue.push({iter, value, inertiaWeight, socialWeight, cognitiveWeight});
}

Logger::~Logger(){
    running=false; 
    if (loggingThread.joinable()){
      loggingThread.join();
    }
  }
};

template<typename T>
void ThreadSafeQueue::push(const LogEntry& entry) {
    std::lock_guard<std::mutex> lock(mutex);
    queue.push(entry);
}

template<typename T>
bool ThreadSafeQueue::pop(LogEntry& entry){
    std::lock_guard<std::mutex> lock(mutex);
    if(queue.empty()) return false;
    entry = queue.front();
    queue.pop();
    return true; 
  }
};


