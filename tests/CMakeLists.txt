add_executable(test_telemetry_line_json
    test_telemetry_line_json.cpp
)
target_include_directories(test_telemetry_line_json PRIVATE ../src)
target_link_libraries(test_telemetry_line_json PRIVATE onboard_core)
add_test(NAME telemetry_line_json COMMAND test_telemetry_line_json)
