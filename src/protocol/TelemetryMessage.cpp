#include "protocol/TelemetryMessage.hpp"

#include <nlohmann/json.hpp>

namespace onboard::protocol {
namespace {

nlohmann::json pointToJson(const Point2f& point)
{
    return {
        {"x", point.x},
        {"y", point.y},
    };
}

} // namespace

std::string buildTelemetryJson(const BringupTelemetry& telemetry)
{
    nlohmann::json markers = nlohmann::json::array();
    for (const auto& marker : telemetry.vision.markers) {
        nlohmann::json corners = nlohmann::json::array();
        for (const auto& corner : marker.corners_px) {
            corners.push_back(pointToJson(corner));
        }
        markers.push_back({
            {"id", marker.id},
            {"center_px", pointToJson(marker.center_px)},
            {"corners_px", corners},
            {"orientation_deg", marker.orientation_deg},
        });
    }

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
        {"marker_count", telemetry.vision.markers.size()},
        {"markers", markers},
    };
    message["grid"] = {
        {"row", telemetry.grid.row},
        {"col", telemetry.grid.col},
        {"heading_deg", telemetry.grid.heading_deg},
    };
    message["debug"] = {
        {"processing_latency_ms", telemetry.debug.processing_latency_ms},
        {"aruco_latency_ms", telemetry.debug.aruco_latency_ms},
        {"note", telemetry.note},
    };

    return message.dump();
}

} // namespace onboard::protocol
