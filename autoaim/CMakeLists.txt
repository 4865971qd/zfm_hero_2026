cmake_minimum_required(VERSION 3.16.3)

project(autoaim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

# Required dependencies.
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

# Optional dependencies.

# ONNX Runtime is used by the number classifier when available.
find_library(ONNXRUNTIME_LIB onnxruntime PATHS /usr/lib /usr/local/lib)
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h PATHS /usr/include/onnxruntime /usr/local/include/onnxruntime)
if(ONNXRUNTIME_LIB AND ONNXRUNTIME_INCLUDE_DIR)
    message(STATUS "ONNX Runtime found")
    set(HAS_ONNX TRUE)
else()
    message(WARNING "ONNX Runtime not found; number classifier will use OpenCV DNN instead")
    set(HAS_ONNX FALSE)
endif()

# Ceres is used by EKF autodiff when available.
find_package(Ceres QUIET)
if(Ceres_FOUND)
    message(STATUS "Ceres found")
    set(HAS_CERES TRUE)
else()
    message(WARNING "Ceres not found; EKF will use analytic Jacobians")
    set(HAS_CERES FALSE)
endif()

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(utils)
add_subdirectory(hardware)
add_subdirectory(src)

add_executable(autoaim src/main.cpp)
target_link_libraries(autoaim
    utils
    hardware
    src_core
    ${OpenCV_LIBS}
    fmt::fmt
    spdlog::spdlog
    yaml-cpp
)

if(HAS_ONNX)
    target_include_directories(autoaim PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
    target_link_libraries(autoaim ${ONNXRUNTIME_LIB})
endif()
if(HAS_CERES)
    target_link_libraries(autoaim ${CERES_LIBRARIES})
endif()

add_executable(offline_test tests/offline_test.cpp)
target_link_libraries(offline_test
    utils
    hardware
    src_core
    ${OpenCV_LIBS}
    fmt::fmt
    spdlog::spdlog
    yaml-cpp
)

if(HAS_ONNX)
    target_include_directories(offline_test PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
    target_link_libraries(offline_test ${ONNXRUNTIME_LIB})
endif()

if(WIN32)
    set(GESTALT_BRIDGE_ROOT "D:/reasonix_ws/autoaim_bridge" CACHE PATH
        "Path to the working Gestalt bridge source tree")
    set(EASYWS_DIR "${GESTALT_BRIDGE_ROOT}/third_party/easywsclient")

    add_executable(autoaim_gestalt
        src/gestalt_main.cpp
        "${GESTALT_BRIDGE_ROOT}/io/gestalt/game_link.cpp"
        "${GESTALT_BRIDGE_ROOT}/io/gestalt/shared_frame_capture.cpp"
        "${EASYWS_DIR}/easywsclient.cpp"
    )

    target_include_directories(autoaim_gestalt PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${GESTALT_BRIDGE_ROOT}
        ${EASYWS_DIR}
    )

    target_link_libraries(autoaim_gestalt
        utils
        hardware
        src_core
        ${OpenCV_LIBS}
        fmt::fmt
        spdlog::spdlog
        yaml-cpp
        ws2_32
        iphlpapi
    )

    if(HAS_ONNX)
        target_include_directories(autoaim_gestalt PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
        target_link_libraries(autoaim_gestalt ${ONNXRUNTIME_LIB})
    endif()
    if(HAS_CERES)
        target_link_libraries(autoaim_gestalt ${CERES_LIBRARIES})
    endif()

    target_compile_options(autoaim_gestalt PRIVATE /utf-8 /EHa)
    target_compile_definitions(autoaim_gestalt PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN _USE_MATH_DEFINES)
endif()
