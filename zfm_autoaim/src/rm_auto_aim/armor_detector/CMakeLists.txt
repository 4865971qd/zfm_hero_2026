cmake_minimum_required(VERSION 3.10)
project(armor_detector)

# 处理CMP0148政策警告
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 NEW)
endif()

## Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(USE_SCOPED_HEADER_INSTALL_DIR OFF)
#set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)
add_definitions(-O3)


## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXTERNAL_INCLUDE_DIRS ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${G2O_LIBRARIES})

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV 4.1.2 REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb)
find_package(G2O REQUIRED)
find_package(fmt REQUIRED)
find_package(Sophus REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC
${OpenCV_INCLUDE_DIRS}
${G2O_INCLUDE_DIRS}
${Sophus_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
g2o_core
g2o_stuff
g2o_solver_csparse
g2o_types_sba
g2o_types_slam3d
g2o_solver_dense
fmt::fmt
tbb
)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN zfm::auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
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
    ament_cmake_lint_cmake
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)

endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  docs
  model
)
