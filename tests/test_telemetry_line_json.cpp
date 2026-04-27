#include "protocol/TelemetryMessage.hpp"

#include <nlohmann/json.hpp>

#include <cassert>
#include <cstdint>
#include <string>

int main()
{
    onboard::protocol::BringupTelemetry telemetry;
    telemetry.seq = 7;
    telemetry.timestamp_ms = 123456;
    telemetry.camera.frame_seq = 42;
    telemetry.camera.width = 640;
    telemetry.camera.height = 480;
    telemetry.vision.line_detected = true;
    telemetry.vision.line_offset = -12.5;
    telemetry.vision.line_angle = 3.0;
    telemetry.vision.line.detected = true;
    telemetry.vision.line.raw_detected = true;
    telemetry.vision.line.filtered = true;
    telemetry.vision.line.tracking_point_px = {307.5, 336.0};
    telemetry.vision.line.raw_tracking_point_px = {300.0, 336.0};
    telemetry.vision.line.centroid_px = {310.0, 260.0};
    telemetry.vision.line.center_offset_px = -12.5;
    telemetry.vision.line.raw_center_offset_px = -20.0;
    telemetry.vision.line.angle_deg = 3.0;
    telemetry.vision.line.raw_angle_deg = 6.0;
    telemetry.vision.line.confidence = 0.8;
    telemetry.vision.line.contour_px.push_back({280.0, 120.0});
    telemetry.vision.line.contour_px.push_back({350.0, 120.0});
    telemetry.debug.line_contours_found = 3;
    telemetry.debug.video_jpeg_bytes = 12345;

    const auto json = nlohmann::json::parse(onboard::protocol::buildTelemetryJson(telemetry));
    assert(json["vision"]["line_detected"].get<bool>());
    assert(json["vision"]["line"]["detected"].get<bool>());
    assert(json["vision"]["line"]["raw_detected"].get<bool>());
    assert(json["vision"]["line"]["filtered"].get<bool>());
    assert(json["vision"]["line"]["raw_center_offset_px"].get<double>() == -20.0);
    assert(json["vision"]["line"]["contour_px"].size() == 2);
    assert(json["debug"].contains("line_latency_ms"));
    assert(json["debug"]["line_contours_found"].get<int>() == 3);
    assert(json["debug"]["video_jpeg_bytes"].get<std::uint64_t>() == 12345);
    return 0;
}
