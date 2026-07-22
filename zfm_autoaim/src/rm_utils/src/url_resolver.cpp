#include "rm_utils/url_resolver.hpp"

#include <filesystem>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rcpputils/env.hpp"
#include "rcpputils/filesystem_helper.hpp"

namespace zfm::utils {
std::filesystem::path URLResolver::getResolvedPath(const std::string &url) {
  const std::string resolved_url = resolveUrl(url);
  UrlType url_type = parseUrl(url);

  std::string res;

  switch (url_type) {
    case UrlType::EMPTY: {
      break;
    }
    case UrlType::FILE: {
      res = resolved_url.substr(7);
      break;
    }
    case UrlType::PACKAGE: {
      res = getPackageFileName(resolved_url);
      break;
    }
    default: {
      break;
    }
  }

  return std::filesystem::path(res);
}

std::string URLResolver::resolveUrl(const std::string &url) {
  std::string resolved;
  size_t rest = 0;

  while (true) {
    // find the next '$' in the URL string
    size_t dollar = url.find('$', rest);

    if (dollar >= url.length()) {
      // no more variables left in the URL
      resolved += url.substr(rest);
      break;
    }

    // copy characters up to the next '$'
    resolved += url.substr(rest, dollar - rest);

    if (url.substr(dollar + 1, 1) != "{") {
      // no '{' follows, so keep the '$'
      resolved += "$";
    } else if (url.substr(dollar + 1, 10) == "{ROS_HOME}") {
      // substitute $ROS_HOME
      std::string ros_home;
      std::string ros_home_env = rcpputils::get_env_var("ROS_HOME");
      std::string home_env = rcpputils::get_env_var("HOME");
      if (!ros_home_env.empty()) {
        // use environment variable
        ros_home = ros_home_env;
      } else if (!home_env.empty()) {
        // use "$HOME/.ros"
        ros_home = home_env + "/.ros";
      }
      resolved += ros_home;
      dollar += 10;
    } else {
      // not a valid substitution variable
      resolved += "$";  // keep the bogus '$'
    }

    // look for next '$'
    rest = dollar + 1;
  }

  return resolved;
}

URLResolver::UrlType URLResolver::parseUrl(const std::string &url) {
  if (url == "") {
    return UrlType::EMPTY;
  }

  // Easy C++14 replacement for boost::iequals from :
  // https://stackoverflow.com/a/4119881
  auto iequals = [](const std::string &a, const std::string &b) {
    return std::equal(a.begin(), a.end(), b.begin(), b.end(), [](char a, char b) {
      return tolower(a) == tolower(b);
    });
  };

  if (iequals(url.substr(0, 8), "file:///")) {
    return UrlType::FILE;
  }
  if (iequals(url.substr(0, 10), "package://")) {
    // look for a '/' following the package name, make sure it is
    // there, the name is not empty, and something follows it
    size_t rest = url.find('/', 10);
    if (rest < url.length() - 1 && rest > 10) {
      return UrlType::PACKAGE;
    }
  }
  return UrlType::INVALID;
}

std::string URLResolver::getPackageFileName(const std::string &url) {
  // Scan URL from after "package://" until next '/' and extract
  // package name.  The parseURL() already checked that it's present.
  size_t prefix_len = std::string("package://").length();
  size_t rest = url.find('/', prefix_len);
  std::string package(url.substr(prefix_len, rest - prefix_len));

  // Look up the ROS package path name.
  std::string pkg_path = ament_index_cpp::get_package_share_directory(package);
  if (pkg_path.empty()) {  // package not found?
    return pkg_path;
  } else {
    // Construct file name from package location and remainder of URL.
    return pkg_path + url.substr(rest);
  }
}

}  // namespace zfm::utils
