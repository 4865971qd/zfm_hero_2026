#include "rm_utils/logger/logger_pool.hpp"

#include "rm_utils/logger/exception.hpp"

namespace zfm::logger {
internal::Logger &LoggerPool::getLogger(const std::string &name) {
  if (auto iter = loggers_.find(name); iter != loggers_.end()) {
    return *iter->second;
  } else {
    throw LoggerNotFoundError(name);
  }
}

void LoggerPool::registerLogger(const std::string &name,
                                const std::string &path,
                                LogLevel level,
                                LogOptions ops) {
  std::lock_guard<std::mutex> lock(l_mutex_);
  if (auto iter = loggers_.find(name); iter == loggers_.end()) {
    loggers_[name] = std::make_shared<internal::Logger>(name, path, level, ops);
  }
}

std::mutex LoggerPool::l_mutex_;
std::unordered_map<std::string, std::shared_ptr<internal::Logger>> LoggerPool::loggers_;
}  // namespace zfm::logger
