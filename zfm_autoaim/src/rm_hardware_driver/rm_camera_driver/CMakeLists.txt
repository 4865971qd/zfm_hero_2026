cmake_minimum_required(VERSION 3.10)
project(rm_camera_driver)

# 处理CMP0148政策警告
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 NEW)
endif()

## 使用C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")

# 【修改】在使用ament_auto_package之前设置这个变量
set(USE_SCOPED_HEADER_INSTALL_DIR ON)

## 编译选项
add_definitions(-Wall -Werror -O3)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## 依赖查找
find_package(ament_cmake_auto REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_components REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV 4.0 REQUIRED)
find_package(rm_utils REQUIRED)

# 查找海康MVS SDK核心库
find_library(MV_CAMERA_LIBRARY
    NAMES MvCameraControl
    PATHS /opt/MVS/lib/64
    NO_DEFAULT_PATH
)

# 检查是否找到MVS库
if(NOT MV_CAMERA_LIBRARY)
    message(FATAL_ERROR "海康MVS SDK库libMvCameraControl.so未找到！请确认路径：/opt/MVS/lib/64")
endif()

# 【重要】先添加头文件路径，让编译能找到
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    /opt/MVS/include
)

# 创建库
ament_auto_add_library(${PROJECT_NAME} SHARED
    src/hik_camera.cpp 
    src/recorder.cpp
    src/video_player.cpp
)

# 【修改】使用target_include_directories指定公开头文件目录
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# 架构适配
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    install(
        DIRECTORY /opt/MVS/lib/64/
        DESTINATION lib
        FILES_MATCHING PATTERN "*.so*"
    )
else()
    message(FATAL_ERROR "不支持的架构：${CMAKE_HOST_SYSTEM_PROCESSOR}")
endif()

# 链接库
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${MV_CAMERA_LIBRARY}
    rm_utils::rm_utils
)

# 插件描述
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

# 注册节点
rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "hik_camera::HikCameraNode"
    EXECUTABLE camera_driver_node
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "zfm::camera_driver::VideoPlayerNode"
    EXECUTABLE video_player_node
)

# 让ament_auto_package自动处理头文件安装

# 安装配置文件
install(
    FILES plugins.xml
    DESTINATION share/${PROJECT_NAME}
)

# 测试部分
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
        ament_cmake_copyright
        ament_cmake_cpplint
        ament_cmake_uncrustify
    )
    ament_lint_auto_find_test_dependencies()
endif()

# 【重要】确保头文件被正确安装
ament_auto_package(
    INSTALL_TO_SHARE
)
