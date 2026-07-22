#ifndef RM_UTILS_LOGGER_LOGGER_HPP_
#define RM_UTILS_LOGGER_LOGGER_HPP_

#include <rm_utils/logger/logger_pool.hpp>
#include <rm_utils/logger/types.hpp>

#define ZFM_REGISTER_LOGGER(name, path, level)                           \
  do {                                                                   \
    zfm::logger::LoggerPool::registerLogger(                             \
      name, path, zfm::logger::LogLevel::level, DATE_DIR | DATE_SUFFIX); \
  } while (0)

#define ZFM_LOG(name, level, ...)                                     \
  do {                                                                \
    zfm::logger::LoggerPool::getLogger(name).log(level, __VA_ARGS__); \
  } while (0)

#define ZFM_DEBUG(name, ...) ZFM_LOG(name, zfm::logger::LogLevel::DEBUG, __VA_ARGS__)

#define ZFM_INFO(name, ...) ZFM_LOG(name, zfm::logger::LogLevel::INFO, __VA_ARGS__)

#define ZFM_WARN(name, ...) ZFM_LOG(name, zfm::logger::LogLevel::WARN, __VA_ARGS__)

#define ZFM_ERROR(name, ...) ZFM_LOG(name, zfm::logger::LogLevel::ERROR, __VA_ARGS__)

#define ZFM_FATAL(name, ...) ZFM_LOG(name, zfm::logger::LogLevel::FATAL, __VA_ARGS__)

#endif  // RM_UTILS_LOGGER_LOGGER_HPP_
