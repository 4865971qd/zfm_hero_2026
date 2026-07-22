#ifndef RM_UTILS_COMMON_HPP_
#define RM_UTILS_COMMON_HPP_

#include <string>

namespace zfm {

enum class EnemyColor {
  RED = 0,
  BLUE = 1,
  WHITE = 2,
};

inline std::string enemyColorToString(EnemyColor color) {
  switch (color) {
    case EnemyColor::RED:
      return "RED";
    case EnemyColor::BLUE:
      return "BLUE";
    case EnemyColor::WHITE:
      return "WHITE";
    default:
      return "UNKNOWN";
  }
}

enum VisionMode {
  AUTO_AIM_RED = 0,
  AUTO_AIM_BLUE = 1,
  SMALL_RUNE_RED = 2,
  SMALL_RUNE_BLUE = 3,
  BIG_RUNE_RED = 4,
  BIG_RUNE_BLUE = 5,
};

inline std::string visionModeToString(VisionMode mode) {
  switch (mode) {
    case VisionMode::AUTO_AIM_RED:
      return "AUTO_AIM_RED";
    case VisionMode::AUTO_AIM_BLUE:
      return "AUTO_AIM_BLUE";
    case VisionMode::SMALL_RUNE_RED:
      return "SMALL_RUNE_RED";
    case VisionMode::SMALL_RUNE_BLUE:
      return "SMALL_RUNE_BLUE";
    case VisionMode::BIG_RUNE_RED:
      return "BIG_RUNE_RED";
    case VisionMode::BIG_RUNE_BLUE:
      return "BIG_RUNE_BLUE";
    default:
      return "UNKNOWN";
  }
}

}  // namespace zfm
#endif
