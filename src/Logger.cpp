#include "Logger.hpp"
#include <thread>
#include <fstream>
#include <atomic>
#include <mutex>
#include <queue>

Logger::Logger(const std::string& filename): logFile(filename){
  loggingThread = std::thread(&Logger::loggingFunction, this);
}

void Logger::loggingFunction(){
  while (running) {
    std::vector<LogEntry> entries;
    if (logQueue.pop(entries)) {
      for(const auto& entry : entries){
        logFile << entry.iter << "," << entry.value << "," << entry.inertiaWeight << "," << entry.socialWeight << "," << entry.cognitiveWeight << std::endl;
      }
      logFile.flush();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  flushBuffer();
}

void Logger::log(double iter, double value, double inertiaWeight, double socialWeight, double cognitiveWeight){
  buffer.push_back({iter, value, inertiaWeight, socialWeight, cognitiveWeight});
  if(buffer.size() >= batchSize) {
    flushBuffer();
  }
}

void Logger::flushBuffer(){
  if(!buffer.empty()){
    logQueue.push(buffer);
    buffer.clear();
  }
}

Logger::~Logger(){
    flushBuffer();
    running=false; 
    if (loggingThread.joinable()){
      loggingThread.join();
    }
}

void ThreadSafeQueue::push(const std::vector<LogEntry>& entries) {
    std::lock_guard<std::mutex> lock(mutex);
    queue.push(entries);
}

bool ThreadSafeQueue::pop(std::vector<LogEntry>& entries){
    std::lock_guard<std::mutex> lock(mutex);
    if(this->queue.empty()) return false;
    entries = std::move(this->queue.front());
    this->queue.pop();
    return true; 
  }
