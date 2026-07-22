#ifndef RM_UTILS_LOGGER_WRITER_HPP_
#define RM_UTILS_LOGGER_WRITER_HPP_

// std
#include <fstream>
#include <mutex>

namespace zfm::logger {

class Writer {
public:
  explicit Writer(const std::string &filename);

  ~Writer();

  void write(const std::string &message);

  void flush();

private:
  std::ofstream file_;
  std::mutex &r_mutex_;
};
}  // namespace zfm::logger
#endif  // RM_UTILS_LOGGER_WRITER_HPP_ 
