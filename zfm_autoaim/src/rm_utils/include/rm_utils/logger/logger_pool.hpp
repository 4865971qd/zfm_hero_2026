#ifndef RM_UTILS_LOGGER_POOL_HPP_
#define RM_UTILS_LOGGER_POOL_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "rm_utils/logger/impl/logger_impl.hpp"

namespace zfm::logger {
class LoggerPool {
public:
  static internal::Logger &getLogger(const std::string &name);

  static void registerLogger(const std::string &name,
                             const std::string &path,
                             LogLevel level,
                             LogOptions = DEFAULT_OPTIONS);

private:
  LoggerPool() = default;
  ~LoggerPool() = default;
  LoggerPool(const LoggerPool &) = delete;
  LoggerPool &operator=(const LoggerPool &) = delete;
  LoggerPool(LoggerPool &&) = delete;

  static std::mutex l_mutex_;
  static std::unordered_map<std::string, std::shared_ptr<internal::Logger>> loggers_;
};
}  // namespace zfm::logger
#endif // RM_UTILS?LOGGER_POOL_HPP_
