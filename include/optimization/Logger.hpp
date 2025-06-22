/**
 * @file Logger.hpp
 * @brief Thread-safe logging system for PSO optimization convergence tracking
 * @author PSO Orbit Transfer Team
 * @date 2025
 *
 * Provides efficient logging capabilities for tracking PSO optimization
 * progress, including convergence metrics and parameter evolution.
 */

#ifndef LOG_HPP
#define LOG_HPP

#include <fstream>
#include <mutex>
#include <vector>

/**
 * @struct LogEntry
 * @brief Single log entry containing PSO state information
 *
 * Captures a snapshot of PSO optimization state at a specific iteration,
 * including objective function value and algorithm parameters.
 */
struct LogEntry {
  /** @brief Iteration number */
  double iter;

  /** @brief Best objective function value at this iteration */
  double value;

  /** @brief Current inertia weight parameter */
  double inertiaWeight;

  /** @brief Current social weight parameter */
  double socialWeight;

  /** @brief Current cognitive weight parameter */
  double cognitiveWeight;
};

/**
 * @class Logger
 * @brief Thread-safe logger for PSO optimization monitoring
 *
 * Implements a buffered logging system that collects PSO convergence data
 * and writes it to CSV files for analysis. Features:
 * - Thread-safe operation for parallel PSO implementations
 * - Buffered writes for improved performance
 * - Automatic flushing when buffer reaches capacity
 * - CSV format for easy data analysis
 *
 * The logger tracks key PSO parameters over time to enable:
 * - Convergence analysis
 * - Parameter tuning studies
 * - Algorithm performance evaluation
 */
class Logger {
private:
  /** @brief Output file stream for log data */
  std::ofstream logFile;

  /** @brief Mutex for thread-safe file operations */
  std::mutex fileMutex;

  /** @brief Buffer to collect log entries before writing */
  std::vector<LogEntry> buffer;

  /** @brief Number of entries to buffer before flushing */
  size_t batchSize = 10;

public:
  /**
   * @brief Constructor - opens log file for writing
   * @param filename Path to output log file
   *
   * Creates a new log file or overwrites existing file.
   * Prints error message if file cannot be opened.
   */
  Logger(const std::string& filename);

  /**
   * @brief Log a single PSO iteration state
   * @param iter Current iteration number
   * @param value Current best objective function value
   * @param inertiaWeight Current inertia weight parameter
   * @param socialWeight Current social weight parameter
   * @param cognitiveWeight Current cognitive weight parameter
   *
   * Adds an entry to the buffer. Automatically flushes to file
   * when buffer reaches batchSize capacity.
   */
  void log(double iter, double value, double inertiaWeight, double socialWeight, double cognitiveWeight);

  /**
   * @brief Force write all buffered entries to file
   *
   * Thread-safe operation that writes all pending log entries
   * to the output file and clears the buffer. Called automatically
   * when buffer is full or during destruction.
   */
  void flushBuffer();

  /**
   * @brief Destructor - ensures all data is written before closing
   *
   * Flushes any remaining buffered entries and closes the log file.
   * Guarantees no data loss even if program terminates unexpectedly.
   */
  ~Logger();
};

#endif
