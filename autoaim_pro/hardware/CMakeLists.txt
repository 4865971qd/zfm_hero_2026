add_library(hardware STATIC
    camera.cpp
    video_player.cpp
    serial.cpp
)

target_include_directories(hardware PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} /opt/MVS/include)

target_link_libraries(hardware
    ${OpenCV_LIBS}
    fmt::fmt
    spdlog::spdlog
    yaml-cpp
    /opt/MVS/lib/64/libMvCameraControl.so
)
