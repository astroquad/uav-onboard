add_executable(test_telemetry_line_json
    test_telemetry_line_json.cpp
)
target_include_directories(test_telemetry_line_json PRIVATE ../src)
target_link_libraries(test_telemetry_line_json PRIVATE onboard_core)
add_test(NAME telemetry_line_json COMMAND test_telemetry_line_json)

add_executable(test_telemetry_chunker
    test_telemetry_chunker.cpp
)
target_include_directories(test_telemetry_chunker PRIVATE ../src)
target_link_libraries(test_telemetry_chunker PRIVATE onboard_core)
add_test(NAME telemetry_chunker COMMAND test_telemetry_chunker)

add_executable(test_line_stabilizer
    test_line_stabilizer.cpp
    ../src/vision/LineStabilizer.cpp
)
target_include_directories(test_line_stabilizer PRIVATE ../src)
target_link_libraries(test_line_stabilizer PRIVATE onboard_core)
add_test(NAME line_stabilizer COMMAND test_line_stabilizer)

add_executable(test_marker_stabilizer
    test_marker_stabilizer.cpp
    ../src/vision/MarkerStabilizer.cpp
)
target_include_directories(test_marker_stabilizer PRIVATE ../src)
target_link_libraries(test_marker_stabilizer PRIVATE onboard_core)
add_test(NAME marker_stabilizer COMMAND test_marker_stabilizer)

add_executable(test_intersection_stabilizer
    test_intersection_stabilizer.cpp
    ../src/vision/IntersectionStabilizer.cpp
)
target_include_directories(test_intersection_stabilizer PRIVATE ../src)
target_link_libraries(test_intersection_stabilizer PRIVATE onboard_core)
add_test(NAME intersection_stabilizer COMMAND test_intersection_stabilizer)

add_executable(test_intersection_decision
    test_intersection_decision.cpp
)
target_include_directories(test_intersection_decision PRIVATE ../src)
target_link_libraries(test_intersection_decision PRIVATE onboard_core)
add_test(NAME intersection_decision COMMAND test_intersection_decision)

add_executable(test_guided_velocity_controller
    test_guided_velocity_controller.cpp
)
target_include_directories(test_guided_velocity_controller PRIVATE ../src)
target_link_libraries(test_guided_velocity_controller PRIVATE onboard_core)
add_test(NAME guided_velocity_controller COMMAND test_guided_velocity_controller)

add_executable(test_grid_control_mapper_pi
    test_grid_control_mapper_pi.cpp
)
target_include_directories(test_grid_control_mapper_pi PRIVATE ../src)
target_link_libraries(test_grid_control_mapper_pi PRIVATE onboard_core)
add_test(NAME grid_control_mapper_pi COMMAND test_grid_control_mapper_pi)

add_executable(test_line_follow_mission
    test_line_follow_mission.cpp
)
target_include_directories(test_line_follow_mission PRIVATE ../src)
target_link_libraries(test_line_follow_mission PRIVATE onboard_core)
add_test(NAME line_follow_mission COMMAND test_line_follow_mission)

add_executable(test_grid_mission_entry_origin
    test_grid_mission_entry_origin.cpp
)
target_include_directories(test_grid_mission_entry_origin PRIVATE ../src)
target_link_libraries(test_grid_mission_entry_origin PRIVATE onboard_core)
add_test(NAME grid_mission_entry_origin COMMAND test_grid_mission_entry_origin)

add_executable(test_safety_monitor
    test_safety_monitor.cpp
)
target_include_directories(test_safety_monitor PRIVATE ../src)
target_link_libraries(test_safety_monitor PRIVATE onboard_core)
add_test(NAME safety_monitor COMMAND test_safety_monitor)

if(TARGET onboard_vision)
    add_executable(test_intersection_detector
        test_intersection_detector.cpp
    )
    target_include_directories(test_intersection_detector PRIVATE ../src)
    target_link_libraries(test_intersection_detector PRIVATE onboard_vision onboard_core)
    add_test(NAME intersection_detector COMMAND test_intersection_detector)

    add_executable(test_vision_processor
        test_vision_processor.cpp
    )
    target_include_directories(test_vision_processor PRIVATE ../src)
    target_link_libraries(test_vision_processor PRIVATE onboard_vision onboard_core)
    add_test(NAME vision_processor COMMAND test_vision_processor)

    add_executable(test_fake_frame_source
        test_fake_frame_source.cpp
    )
    target_include_directories(test_fake_frame_source PRIVATE ../src)
    target_link_libraries(test_fake_frame_source PRIVATE onboard_vision onboard_core)
    add_test(NAME fake_frame_source COMMAND test_fake_frame_source)
endif()

if(TARGET onboard_autopilot)
    add_executable(test_autopilot_poll_drain
        test_autopilot_poll_drain.cpp
    )
    target_include_directories(test_autopilot_poll_drain PRIVATE ../src)
    target_link_libraries(test_autopilot_poll_drain PRIVATE onboard_autopilot onboard_core)
    add_test(NAME autopilot_poll_drain COMMAND test_autopilot_poll_drain)

    add_executable(test_autopilot_setpoints
        test_autopilot_setpoints.cpp
    )
    target_include_directories(test_autopilot_setpoints PRIVATE ../src)
    target_link_libraries(test_autopilot_setpoints PRIVATE onboard_autopilot onboard_core)
    add_test(NAME autopilot_setpoints COMMAND test_autopilot_setpoints)

    add_executable(test_udp_mavlink_transport
        test_udp_mavlink_transport.cpp
    )
    target_include_directories(test_udp_mavlink_transport PRIVATE ../src)
    target_link_libraries(test_udp_mavlink_transport PRIVATE onboard_autopilot onboard_core)
    add_test(NAME udp_mavlink_transport COMMAND test_udp_mavlink_transport)

    add_executable(test_serial_mavlink_transport
        test_serial_mavlink_transport.cpp
    )
    target_include_directories(test_serial_mavlink_transport PRIVATE ../src)
    target_link_libraries(test_serial_mavlink_transport PRIVATE onboard_autopilot onboard_core)
    add_test(NAME serial_mavlink_transport COMMAND test_serial_mavlink_transport)
endif()

if(TARGET onboard_vision)
    add_executable(test_line_mask_strategies
        test_line_mask_strategies.cpp
    )
    target_include_directories(test_line_mask_strategies PRIVATE ../src)
    target_link_libraries(test_line_mask_strategies PRIVATE onboard_vision onboard_core)
    add_test(NAME line_mask_strategies COMMAND test_line_mask_strategies)
endif()
