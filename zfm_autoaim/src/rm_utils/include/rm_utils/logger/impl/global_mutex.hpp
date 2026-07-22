#ifndef RM_UTILS_LOGGER_GLOBAL_MUTEX_HPP_
#define RM_UTILS_LOGGER_GLOBAL_MUTEX_HPP_

#include <mutex>
#include <unordered_map>

namespace zfm::logger {

static std::mutex g_mutex_;

class GlobalMutex {
public:
  inline static std::mutex &getConsoleMutex() {
    static std::mutex s_mutex;
    return s_mutex;
  }

  inline static std::mutex &getFileMutex(const std::string &filename) {
    static std::unordered_map<std::string, std::mutex> file_mutex_map;
    std::lock_guard<std::mutex> lock(g_mutex_);
    return file_mutex_map[filename];
  }

private:
  GlobalMutex() = default;
  ~GlobalMutex() = default;
};
}  // namespace zfm::logger
#endif
