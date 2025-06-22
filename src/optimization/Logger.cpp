#include "optimization/Logger.hpp"
#include <iostream>

Logger::Logger(const std::string& filename): logFile(filename){
  if(!logFile.is_open()){
    std::cout << "Failed to open log file." << std::endl;
  }
}

void Logger::log(double iter, double value, double inertiaWeight, double socialWeight, double cognitiveWeight){
  LogEntry entry = {iter, value, inertiaWeight, socialWeight, cognitiveWeight};
  buffer.push_back(entry);

  if(buffer.size() >= batchSize){
    flushBuffer();
  }
}

void Logger::flushBuffer(){
  std::lock_guard<std::mutex> lock(fileMutex);
  for (const auto& entry : buffer){
    logFile << entry.iter <<","<< entry.value <<","<< entry.inertiaWeight <<","<< entry.socialWeight <<","<< entry.cognitiveWeight << std::endl;
  }

  buffer.clear();
  logFile.flush();
}

Logger::~Logger(){
    flushBuffer();
    logFile.close();
}
