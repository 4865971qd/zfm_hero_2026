#ifndef RM_UTILS_LOGGER_LOGGER_IMPL_HPP_
#define RM_UTILS_LOGGER_LOGGER_IMPL_HPP_

// std
#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

// fmt
#include <fmt/color.h>
#include <fmt/core.h>

// project
#include "rm_utils/logger/impl/writer.hpp"
#include "rm_utils/logger/types.hpp"

namespace zfm::logger {

namespace internal {
class Logger {
public:
  Logger(std::string name, std::string path, LogLevel level, LogOptions ops);

  template <typename... Args>
  void log(LogLevel level, const std::string &format, Args... args) {
    std::string log_info = fmt::format(format, args...);
    std::string level_prefix = fmt::format("[{}] ", LogNameTable[static_cast<std::uint8_t>(level)]);
    std::string name_prefix = fmt::format("[{}] ", name_);
    std::string log_time = fmt::format("[{}] ", getLocalTime());
    std::string message =
      fmt::format("{} {} {}: {}", level_prefix, name_prefix, log_time, log_info);

    if (level >= this->level_) {
      std::string colored_message =
        fmt::format(LogColorTable[static_cast<std::uint8_t>(level)], message);
      writer_->write(colored_message);
    }
    std::lock_guard<std::mutex> lock(consle_mutex_);
    fmt::print(fg(LogFmtColorTable[static_cast<std::uint8_t>(level)]), "{}\n", message);
  }

  template <typename... Args>
  void debug(const std::string &format, Args... args) {
    log(LogLevel::DEBUG, format, args...);
  }

  template <typename... Args>
  void info(const std::string &format, Args... args) {
    log(LogLevel::INFO, format, args...);
  }

  template <typename... Args>
  void warn(const std::string &format, Args... args) {
    log(LogLevel::WARN, format, args...);
  }

  template <typename... Args>
  void error(const std::string &format, Args... args) {
    log(LogLevel::ERROR, format, args...);
  }

  template <typename... Args>
  void fatal(const std::string &format, Args... args) {
    log(LogLevel::FATAL, format, args...);
  }

  template <typename... Args>
  void print(const std::string &format, Args... args) {
    std::lock_guard<std::mutex> lock(consle_mutex_);
    fmt::print(format, args...);
  }

  void setLevel(LogLevel level);

  void flush();

private:
  std::string getLocalTime();

  std::mutex &consle_mutex_;
  std::string name_;
  LogLevel level_;
  std::unique_ptr<Writer> writer_;
};
}  // namespace internal

}  // namespace zfm::logger
#endif  // RM_UTILS_LOGGER_LOGGER_IMPL_HPP_ 
