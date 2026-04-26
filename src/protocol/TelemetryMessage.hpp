#pragma once

#include <cstdint>
#include <string>

namespace onboard::protocol {

struct CameraTelemetry {
    std::string status = "not_checked";
    int width = 0;
    int height = 0;
    double fps = 0.0;
    std::uint32_t frame_seq = 0;
};

struct VisionTelemetry {
    bool line_detected = false;
    double line_offset = 0.0;
    double line_angle = 0.0;
    bool intersection_detected = false;
    double intersection_score = 0.0;
    bool marker_detected = false;
    int marker_id = -1;
};

struct GridTelemetry {
    int row = -1;
    int col = -1;
    double heading_deg = 0.0;
};

struct DebugTelemetry {
    double processing_latency_ms = 0.0;
};

struct BringupTelemetry {
    std::uint32_t seq = 0;
    std::int64_t timestamp_ms = 0;
    CameraTelemetry camera;
    VisionTelemetry vision;
    GridTelemetry grid;
    DebugTelemetry debug;
    std::string note;
};

std::string buildTelemetryJson(const BringupTelemetry& telemetry);

} // namespace onboard::protocol
