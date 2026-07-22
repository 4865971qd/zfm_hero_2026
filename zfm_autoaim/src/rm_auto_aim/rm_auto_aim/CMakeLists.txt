cmake_minimum_required(VERSION 3.10)
project(rm_auto_aim)

# 处理CMP0148政策警告
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 NEW)
endif()

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
