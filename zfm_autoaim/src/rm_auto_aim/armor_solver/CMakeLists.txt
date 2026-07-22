cmake_minimum_required(VERSION 3.10)
project(armor_solver)

# 处理CMP0148政策警告
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 NEW)
endif()

## Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -O3)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################
find_package(OpenCV 4.0 REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 显式添加 tf2 相关依赖
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

# 添加目标依赖
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
  tf2
  tf2_geometry_msgs
  tf2_ros
  geometry_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN zfm::auto_aim::ArmorSolverNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)

endif()

#############
## Install ##
#############

ament_auto_package()
