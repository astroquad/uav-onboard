cmake_minimum_required(VERSION 3.16)

project(astroquad_onboard LANGUAGES CXX)

if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

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
    src/control/GridControlMapper.cpp
    src/control/GuidedVelocityController.cpp
    src/mission/AltitudePolicy.cpp
    src/mission/GridCoordinateTracker.cpp
    src/mission/GridMission.cpp
    src/mission/IntersectionDecision.cpp
    src/mission/LineFollowMission.cpp
    src/mission/MarkerRevisitPlanner.cpp
    src/mission/MarkerRegistry.cpp
    src/mission/SnakePlanner.cpp
    src/network/TelemetryChunker.cpp
    src/network/UdpTelemetrySender.cpp
    src/protocol/TelemetryMessage.cpp
    src/safety/SafetyMonitor.cpp
    src/video/VideoPacket.cpp
)

target_include_directories(onboard_core PUBLIC
    src
)

target_link_libraries(onboard_core PUBLIC onboard_dependencies)

if(WIN32)
    target_link_libraries(onboard_core PUBLIC ws2_32)
endif()

option(ONBOARD_FETCH_MAVLINK "Download MAVLink C headers when they are not installed locally" ON)

set(MAVLINK_INCLUDE_DIR "" CACHE PATH "Path containing mavlink/v2.0/ardupilotmega/mavlink.h")
if(NOT MAVLINK_INCLUDE_DIR)
    find_path(DETECTED_MAVLINK_INCLUDE_DIR
        mavlink/v2.0/ardupilotmega/mavlink.h
        HINTS
            "$ENV{HOME}/ardupilot/build/sitl/libraries/GCS_MAVLink/include"
            "$ENV{HOME}/ardupilot/libraries/GCS_MAVLink/include"
            "/usr/local/include"
            "/usr/include"
    )
    if(DETECTED_MAVLINK_INCLUDE_DIR)
        set(MAVLINK_INCLUDE_DIR "${DETECTED_MAVLINK_INCLUDE_DIR}" CACHE PATH
            "Path containing mavlink/v2.0/ardupilotmega/mavlink.h" FORCE)
    endif()
endif()

if(NOT MAVLINK_INCLUDE_DIR AND ONBOARD_FETCH_MAVLINK)
    FetchContent_Declare(
        mavlink_c_library_v2
        URL https://github.com/mavlink/c_library_v2/archive/refs/heads/master.tar.gz
    )
    FetchContent_MakeAvailable(mavlink_c_library_v2)
    FetchContent_GetProperties(mavlink_c_library_v2 SOURCE_DIR MAVLINK_C_LIBRARY_V2_SOURCE_DIR)

    set(MAVLINK_COMPAT_INCLUDE_DIR "${CMAKE_BINARY_DIR}/_deps/mavlink_include")
    file(REMOVE_RECURSE "${MAVLINK_COMPAT_INCLUDE_DIR}/mavlink")
    file(MAKE_DIRECTORY "${MAVLINK_COMPAT_INCLUDE_DIR}/mavlink")
    file(CREATE_LINK
        "${MAVLINK_C_LIBRARY_V2_SOURCE_DIR}"
        "${MAVLINK_COMPAT_INCLUDE_DIR}/mavlink/v2.0"
        SYMBOLIC
        COPY_ON_ERROR)
    set(MAVLINK_INCLUDE_DIR "${MAVLINK_COMPAT_INCLUDE_DIR}" CACHE PATH
        "Path containing mavlink/v2.0/ardupilotmega/mavlink.h" FORCE)
endif()

if(MAVLINK_INCLUDE_DIR AND NOT WIN32)
    add_library(onboard_autopilot STATIC
        src/autopilot/AutopilotMavlinkAdapter.cpp
        src/autopilot/SerialMavlinkTransport.cpp
        src/autopilot/UdpMavlinkTransport.cpp
    )
    target_include_directories(onboard_autopilot PUBLIC src)
    target_include_directories(onboard_autopilot SYSTEM PUBLIC
        ${MAVLINK_INCLUDE_DIR}
    )
    target_link_libraries(onboard_autopilot PUBLIC onboard_core)
else()
    message(WARNING "MAVLink headers were not found or platform is unsupported; line_follow_node will not be built.")
endif()

add_library(onboard_video STATIC
    src/camera/RpicamMjpegSource.cpp
    src/video/UdpMjpegStreamer.cpp
)
target_include_directories(onboard_video PUBLIC src)
target_link_libraries(onboard_video PUBLIC onboard_core)

add_executable(uav-onboard-telem
    src/telemetry_main.cpp
)

target_link_libraries(uav-onboard-telem PRIVATE onboard_core)

if(TARGET onboard_autopilot)
    add_executable(line_follow_node tools/line_follow_node.cpp)
    target_include_directories(line_follow_node PRIVATE src)
    target_link_libraries(line_follow_node PRIVATE onboard_core onboard_autopilot)

    add_executable(mavlink_probe tools/mavlink_probe.cpp)
    target_include_directories(mavlink_probe PRIVATE src)
    target_link_libraries(mavlink_probe PRIVATE onboard_core onboard_autopilot)

    add_executable(mavlink_motor_test tools/mavlink_motor_test.cpp)
    target_include_directories(mavlink_motor_test PRIVATE src)
    target_link_libraries(mavlink_motor_test PRIVATE onboard_core onboard_autopilot)
endif()

option(BUILD_TOOLS "Build onboard development tools" ON)
if(BUILD_TOOLS)
    find_package(OpenCV QUIET COMPONENTS core imgcodecs imgproc videoio aruco)
    find_package(gz-msgs10 QUIET CONFIG)
    find_package(gz-transport13 QUIET CONFIG)

    if(OpenCV_FOUND)
        set(ONBOARD_GAZEBO_LIBS "")
        if(gz-msgs10_FOUND AND gz-transport13_FOUND)
            set(ONBOARD_GAZEBO_LIBS
                gz-msgs10::gz-msgs10
                gz-transport13::gz-transport13
            )
        endif()

        add_library(onboard_vision STATIC
            src/vision/ArucoDetector.cpp
            src/vision/FakeFrameSource.cpp
            src/vision/GazeboCameraSource.cpp
            src/vision/IntersectionDetector.cpp
            src/vision/IntersectionStabilizer.cpp
            src/vision/LineDetector.cpp
            src/vision/LineMaskBuilder.cpp
            src/vision/LineStabilizer.cpp
            src/vision/MarkerStabilizer.cpp
            src/vision/OpenCvCameraSource.cpp
            src/vision/RpicamFrameSource.cpp
            src/vision/VisionProcessor.cpp
        )
        target_include_directories(onboard_vision PUBLIC src)
        target_link_libraries(onboard_vision PUBLIC
            onboard_core
            onboard_video
            ${OpenCV_LIBS}
            ${ONBOARD_GAZEBO_LIBS}
        )
        if(ONBOARD_GAZEBO_LIBS)
            target_compile_definitions(onboard_vision PUBLIC ONBOARD_HAVE_GAZEBO_TRANSPORT=1)
        endif()

        if(TARGET line_follow_node)
            target_sources(line_follow_node PRIVATE
                src/app/GcsTelemetryPublisher.cpp
                src/app/LatestVideoSender.cpp
            )
            target_link_libraries(line_follow_node PRIVATE onboard_vision onboard_video)
            target_compile_definitions(line_follow_node PRIVATE ONBOARD_LINE_FOLLOW_HAS_VISION=1)
        endif()

        add_executable(camera_preview
            tools/camera_preview.cpp
        )
        target_include_directories(camera_preview PRIVATE src)
        target_link_libraries(camera_preview PRIVATE onboard_vision)

        add_executable(aruco_detector_tester tools/aruco_detector_tester.cpp)
        target_include_directories(aruco_detector_tester PRIVATE src)
        target_link_libraries(aruco_detector_tester PRIVATE onboard_vision)

        add_executable(line_detector_tuner tools/line_detector_tuner.cpp)
        target_include_directories(line_detector_tuner PRIVATE src)
        target_link_libraries(line_detector_tuner PRIVATE onboard_vision)

        add_executable(grid_image_smoke tools/grid_image_smoke.cpp)
        target_include_directories(grid_image_smoke PRIVATE src)
        target_link_libraries(grid_image_smoke PRIVATE onboard_vision onboard_core)

        add_executable(marker_grid_replay tools/marker_grid_replay.cpp)
        target_include_directories(marker_grid_replay PRIVATE src)
        target_link_libraries(marker_grid_replay PRIVATE onboard_vision onboard_core)

        add_executable(vision_debug_node
            tools/vision_debug_node.cpp
            src/app/VisionDebugPipeline.cpp
            src/app/GcsTelemetryPublisher.cpp
            src/app/LatestVideoSender.cpp
        )
        target_include_directories(vision_debug_node PRIVATE src)
        target_link_libraries(vision_debug_node PRIVATE onboard_core onboard_video onboard_vision)

        if(TARGET onboard_autopilot)
            # grid_mission_node (byte-identical compatibility alias) was
            # removed 2026-07; astroquad-onboard is the single mission runtime.
            add_executable(astroquad-onboard
                src/main.cpp
                src/app/AstroquadOnboardApp.cpp
                src/app/GcsTelemetryPublisher.cpp
                src/app/LatestVideoSender.cpp
            )
            target_include_directories(astroquad-onboard PRIVATE src)
            target_link_libraries(astroquad-onboard PRIVATE
                onboard_core onboard_autopilot onboard_video onboard_vision)
            target_compile_definitions(astroquad-onboard PRIVATE
                ONBOARD_PROGRAM_NAME="astroquad-onboard")
        endif()
    else()
        message(WARNING "OpenCV with aruco was not found; camera_preview, aruco_detector_tester, and vision_debug_node will not be built.")
    endif()

    foreach(tool
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
