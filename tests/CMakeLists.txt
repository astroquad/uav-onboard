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
