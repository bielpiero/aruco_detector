#ifndef TIMING_LOGGER_HPP
#define TIMING_LOGGER_HPP

#include <chrono>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>

class TimingLogger {
public:
  static double nowMs() {
    using clock = std::chrono::steady_clock;

    return std::chrono::duration<double, std::milli>(
               clock::now().time_since_epoch())
        .count();
  }

  static void log(const std::string &filename, const std::string &operation,
                  double duration_ms) {

    static std::mutex mutex;

    std::lock_guard<std::mutex> lock(mutex);

    std::ofstream file(filename, std::ios::app);

    file << operation << "," << std::fixed << std::setprecision(9)
         << duration_ms << "\n";
  }
};

#endif