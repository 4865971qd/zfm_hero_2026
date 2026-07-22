#ifndef RM_UTILS_LOGGER_EXCEPTION_HPP_
#define RM_UTILS_LOGGER_EXCEPTION_HPP_

#include <fmt/format.h>

namespace zfm::logger {
class LoggerNotFoundError : public std::exception {
public:
  explicit LoggerNotFoundError(std::string_view name) {
    msg = fmt::format("Logger {} Not Found", name);
  }
  const char *what() const noexcept override { return msg.data(); }

private:
  std::string_view msg;
};

class WriteError : public std::exception {
public:
  explicit WriteError(std::string_view name) { msg = fmt::format("Write to {} Error", name); }
  const char *what() const noexcept override { return msg.data(); }

private:
  std::string_view msg;
};

}  // namespace zfm::logger
#endif // RM_UTILS_LOGGER_EXCEPTION_HPP_
