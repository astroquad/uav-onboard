cmake_minimum_required(VERSION 3.16)

project(uav_onboard LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(uav_onboard
    src/main.cpp
)

target_include_directories(uav_onboard PRIVATE
    src
)

option(BUILD_TOOLS "Build onboard development tools" ON)
if(BUILD_TOOLS)
    foreach(tool
        camera_preview
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
