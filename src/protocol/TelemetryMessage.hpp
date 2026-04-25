#pragma once

#include <cstdint>
#include <string>

namespace onboard::protocol {

struct BringupTelemetry {
    std::uint32_t seq = 0;
    std::int64_t timestamp_ms = 0;
    std::string camera_status = "not_checked";
    int frame_width = 0;
    int frame_height = 0;
    double measured_fps = 0.0;
    std::string note;
};

std::string buildTelemetryJson(const BringupTelemetry& telemetry);

} // namespace onboard::protocol
