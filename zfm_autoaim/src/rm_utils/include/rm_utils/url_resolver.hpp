#ifndef RM_UTILS_URL_RESOLVER_HPP_
#define RM_UTILS_URL_RESOLVER_HPP_

#include <filesystem>
#include <string>

namespace zfm::utils {
class URLResolver {
public:
  static std::filesystem::path getResolvedPath(const std::string &url);

private:
  static std::string resolveUrl(const std::string &url);

  URLResolver() = delete;

  enum class UrlType {
    EMPTY = 0,  // empty string
    FILE,       // file
    PACKAGE,    // package
    INVALID,    // anything >= is invalid
  };
  static UrlType parseUrl(const std::string &url);

  static std::string getPackageFileName(const std::string &url);
};
}  // namespace zfm::utils

#endif // RM_UTILS_URL_RESOLVER_HPP_
