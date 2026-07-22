#ifndef RM_UTILS_ASSERT_HPP_
#define RM_UTILS_ASSERT_HPP_

#include <iostream>
#include <sstream>

#define ZFM_ASSERT(condition)                            \
  do {                                                   \
    if (!(condition)) {                                  \
      std::ostringstream oss;                            \
      oss << "Assertion failed: (" << #condition << ")"; \
      std::cerr << oss.str() << std::endl;               \
      std::abort();                                      \
    }                                                    \
  } while (0)

#define ZFM_ASSERT_MSG(condition, msg)                           \
  do {                                                           \
    if (!(condition)) {                                          \
      std::ostringstream oss;                                    \
      oss << "Assertion failed: (" << #condition << ") " << msg; \
      std::cerr << oss.str() << std::endl;                       \
      std::abort();                                              \
    }                                                            \
  } while (0)

#endif