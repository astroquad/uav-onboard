cmake_minimum_required(VERSION 3.16)

project(uav_onboard LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(onboard_core STATIC
    src/common/NetworkConfig.cpp
    src/common/Time.cpp
    src/network/UdpTelemetrySender.cpp
    src/protocol/TelemetryMessage.cpp
)

target_include_directories(onboard_core PUBLIC
    src
)

if(WIN32)
    target_link_libraries(onboard_core PUBLIC ws2_32)
endif()

add_executable(uav_onboard
    src/main.cpp
)

target_link_libraries(uav_onboard PRIVATE onboard_core)

option(BUILD_TOOLS "Build onboard development tools" ON)
if(BUILD_TOOLS)
    find_package(OpenCV QUIET COMPONENTS core imgcodecs videoio)

    if(OpenCV_FOUND)
        add_executable(camera_preview
            tools/camera_preview.cpp
            src/vision/OpenCvCameraSource.cpp
        )
        target_include_directories(camera_preview PRIVATE src)
        target_link_libraries(camera_preview PRIVATE ${OpenCV_LIBS})
    else()
        message(WARNING "OpenCV was not found; camera_preview will not be built.")
    endif()

    foreach(tool
        line_detector_tuner
        aruco_detector_tester
        replay_vision
        mock_gcs_command
        video_streamer
    )
        add_executable(${tool} tools/${tool}.cpp)
        target_include_directories(${tool} PRIVATE src)
    endforeach()
endif()

option(BUILD_TESTS "Build onboard tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
