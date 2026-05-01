add_executable(test_telemetry_line_json
    test_telemetry_line_json.cpp
)
target_include_directories(test_telemetry_line_json PRIVATE ../src)
target_link_libraries(test_telemetry_line_json PRIVATE onboard_core)
add_test(NAME telemetry_line_json COMMAND test_telemetry_line_json)

add_executable(test_line_stabilizer
    test_line_stabilizer.cpp
    ../src/vision/LineStabilizer.cpp
)
target_include_directories(test_line_stabilizer PRIVATE ../src)
target_link_libraries(test_line_stabilizer PRIVATE onboard_core)
add_test(NAME line_stabilizer COMMAND test_line_stabilizer)

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

if(TARGET onboard_vision)
    add_executable(test_intersection_detector
        test_intersection_detector.cpp
    )
    target_include_directories(test_intersection_detector PRIVATE ../src)
    target_link_libraries(test_intersection_detector PRIVATE onboard_vision onboard_core)
    add_test(NAME intersection_detector COMMAND test_intersection_detector)
endif()
