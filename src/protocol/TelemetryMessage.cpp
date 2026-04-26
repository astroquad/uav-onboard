#include "protocol/TelemetryMessage.hpp"

#include <nlohmann/json.hpp>

namespace onboard::protocol {

std::string buildTelemetryJson(const BringupTelemetry& telemetry)
{
    nlohmann::json message;
    message["protocol_version"] = 1;
    message["type"] = "TELEMETRY";
    message["seq"] = telemetry.seq;
    message["timestamp_ms"] = telemetry.timestamp_ms;
    message["mission"] = {
        {"state", "IDLE"},
        {"elapsed_ms", 0},
    };
    message["camera"] = {
        {"status", telemetry.camera.status},
        {"width", telemetry.camera.width},
        {"height", telemetry.camera.height},
        {"fps", telemetry.camera.fps},
        {"frame_seq", telemetry.camera.frame_seq},
    };
    message["vision"] = {
        {"line_detected", telemetry.vision.line_detected},
        {"line_offset", telemetry.vision.line_offset},
        {"line_angle", telemetry.vision.line_angle},
        {"intersection_detected", telemetry.vision.intersection_detected},
        {"intersection_score", telemetry.vision.intersection_score},
        {"marker_detected", telemetry.vision.marker_detected},
        {"marker_id", telemetry.vision.marker_id},
    };
    message["grid"] = {
        {"row", telemetry.grid.row},
        {"col", telemetry.grid.col},
        {"heading_deg", telemetry.grid.heading_deg},
    };
    message["debug"] = {
        {"processing_latency_ms", telemetry.debug.processing_latency_ms},
        {"note", telemetry.note},
    };

    return message.dump();
}

} // namespace onboard::protocol
