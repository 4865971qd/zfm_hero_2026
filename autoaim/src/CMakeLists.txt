add_library(src_core STATIC
    armor.cpp
    classifier.cpp
    detector.cpp
    solver.cpp
    tracker.cpp
    target.cpp
)

target_include_directories(src_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(src_core
    utils
    ${OpenCV_LIBS}
    Eigen3::Eigen
    fmt::fmt
    spdlog::spdlog
    yaml-cpp
)

if(HAS_ONNX)
    target_include_directories(src_core PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
    target_link_libraries(src_core ${ONNXRUNTIME_LIB})
endif()
if(HAS_CERES)
    target_link_libraries(src_core ${CERES_LIBRARIES})
endif()
