add_library(utils STATIC
    math.cpp
    pnp.cpp
    ekf.cpp
    trajectory.cpp
    manual_compensator.cpp
    plotter.cpp
    recorder.cpp
    logger.cpp
    exiter.cpp
)

target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(utils
    ${OpenCV_LIBS}
    Eigen3::Eigen
    fmt::fmt
    spdlog::spdlog
)

if(HAS_CERES)
    target_link_libraries(utils ${CERES_LIBRARIES})
endif()
