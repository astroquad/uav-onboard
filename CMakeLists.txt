cmake_minimum_required(VERSION 3.16)

project(uav_onboard LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

function(ensure_header_dependency dep_name dep_url dep_include_subdir dep_header out_var)
    set(include_var "${dep_name}_INCLUDE_DIR")
    find_path(${include_var} ${dep_header})
    if(${include_var})
        set(${out_var} "${${include_var}}" PARENT_SCOPE)
        return()
    endif()

    FetchContent_Declare(
        ${dep_name}
        URL ${dep_url}
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        TLS_VERIFY FALSE
    )
    FetchContent_MakeAvailable(${dep_name})
    FetchContent_GetProperties(${dep_name} SOURCE_DIR dep_source_dir)
    set(${out_var} "${dep_source_dir}/${dep_include_subdir}" PARENT_SCOPE)
endfunction()

ensure_header_dependency(
    nlohmann_json
    "https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz"
    "include"
    "nlohmann/json.hpp"
    NLOHMANN_JSON_INCLUDE_DIR
)

ensure_header_dependency(
    tomlplusplus
    "https://github.com/marzer/tomlplusplus/archive/refs/tags/v3.4.0.tar.gz"
    "include"
    "toml++/toml.hpp"
    TOMLPLUSPLUS_INCLUDE_DIR
)

add_library(onboard_dependencies INTERFACE)
target_include_directories(onboard_dependencies INTERFACE
    ${NLOHMANN_JSON_INCLUDE_DIR}
    ${TOMLPLUSPLUS_INCLUDE_DIR}
)

add_library(onboard_core STATIC
    src/common/NetworkConfig.cpp
    src/common/Time.cpp
    src/common/VisionConfig.cpp
    src/network/UdpTelemetrySender.cpp
    src/protocol/TelemetryMessage.cpp
)

target_include_directories(onboard_core PUBLIC
    src
)

target_link_libraries(onboard_core PUBLIC onboard_dependencies)

if(WIN32)
    target_link_libraries(onboard_core PUBLIC ws2_32)
endif()

add_library(onboard_video STATIC
    src/camera/RpicamMjpegSource.cpp
    src/video/UdpMjpegStreamer.cpp
    src/video/VideoPacket.cpp
)
target_include_directories(onboard_video PUBLIC src)
target_link_libraries(onboard_video PUBLIC onboard_core)

add_executable(uav_onboard
    src/main.cpp
)

target_link_libraries(uav_onboard PRIVATE onboard_core)

option(BUILD_TOOLS "Build onboard development tools" ON)
if(BUILD_TOOLS)
    find_package(OpenCV QUIET COMPONENTS core imgcodecs imgproc videoio aruco)

    if(OpenCV_FOUND)
        add_library(onboard_vision STATIC
            src/vision/ArucoDetector.cpp
            src/vision/OpenCvCameraSource.cpp
        )
        target_include_directories(onboard_vision PUBLIC src)
        target_link_libraries(onboard_vision PUBLIC onboard_core ${OpenCV_LIBS})

        add_executable(camera_preview
            tools/camera_preview.cpp
        )
        target_include_directories(camera_preview PRIVATE src)
        target_link_libraries(camera_preview PRIVATE onboard_vision)

        add_executable(aruco_detector_tester tools/aruco_detector_tester.cpp)
        target_include_directories(aruco_detector_tester PRIVATE src)
        target_link_libraries(aruco_detector_tester PRIVATE onboard_vision)

        add_executable(vision_debug_node tools/vision_debug_node.cpp)
        target_include_directories(vision_debug_node PRIVATE src)
        target_link_libraries(vision_debug_node PRIVATE onboard_core onboard_video onboard_vision)
    else()
        message(WARNING "OpenCV with aruco was not found; camera_preview, aruco_detector_tester, and vision_debug_node will not be built.")
    endif()

    foreach(tool
        line_detector_tuner
        replay_vision
        mock_gcs_command
    )
        add_executable(${tool} tools/${tool}.cpp)
        target_include_directories(${tool} PRIVATE src)
    endforeach()

    add_executable(video_streamer tools/video_streamer.cpp)
    target_include_directories(video_streamer PRIVATE src)
    target_link_libraries(video_streamer PRIVATE onboard_core onboard_video)
endif()

option(BUILD_TESTS "Build onboard tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
