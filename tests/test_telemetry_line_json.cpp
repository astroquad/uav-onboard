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
    telemetry.system.board_model = "Raspberry Pi 4 Model B";
    telemetry.system.throttled_raw = "throttled=0x0";
    telemetry.system.cpu_load_1m = 0.5;
    telemetry.system.mem_available_kb = 123456;
    telemetry.camera.sensor_model = "imx519";
    telemetry.camera.camera_index = 0;
    telemetry.camera.frame_seq = 42;
    telemetry.camera.width = 960;
    telemetry.camera.height = 720;
    telemetry.camera.configured_fps = 12.0;
    telemetry.camera.measured_capture_fps = 11.8;
    telemetry.camera.autofocus_mode = "manual";
    telemetry.camera.lens_position = 0.67;
    telemetry.camera.exposure_mode = "sport";
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
    telemetry.debug.video_send_ms = 4.5;
    telemetry.debug.capture_fps = 11.8;
    telemetry.debug.processing_fps = 11.7;
    telemetry.debug.debug_video_send_fps = 8.0;
    telemetry.debug.video_chunk_pacing_us = 150;
    telemetry.debug.cpu_temp_c = 62.5;
    telemetry.debug.video_chunk_count = 12;
    telemetry.debug.video_chunks_sent = 120;
    telemetry.debug.video_send_failures = 1;
    telemetry.debug.video_skipped_frames = 2;

    const auto json = nlohmann::json::parse(onboard::protocol::buildTelemetryJson(telemetry));
    assert(json["system"]["board_model"].get<std::string>() == "Raspberry Pi 4 Model B");
    assert(json["system"]["throttled_raw"].get<std::string>() == "throttled=0x0");
    assert(json["camera"]["sensor_model"].get<std::string>() == "imx519");
    assert(json["camera"]["width"].get<int>() == 960);
    assert(json["camera"]["measured_capture_fps"].get<double>() == 11.8);
    assert(json["camera"]["autofocus_mode"].get<std::string>() == "manual");
    assert(json["camera"]["lens_position"].get<double>() == 0.67);
    assert(json["vision"]["line_detected"].get<bool>());
    assert(json["vision"]["line"]["detected"].get<bool>());
    assert(json["vision"]["line"]["raw_detected"].get<bool>());
    assert(json["vision"]["line"]["filtered"].get<bool>());
    assert(json["vision"]["line"]["raw_center_offset_px"].get<double>() == -20.0);
    assert(json["vision"]["line"]["contour_px"].size() == 2);
    assert(json["debug"].contains("line_latency_ms"));
    assert(json["debug"]["video_send_ms"].get<double>() == 4.5);
    assert(json["debug"]["capture_fps"].get<double>() == 11.8);
    assert(json["debug"]["processing_fps"].get<double>() == 11.7);
    assert(json["debug"]["debug_video_send_fps"].get<double>() == 8.0);
    assert(json["debug"]["video_chunk_pacing_us"].get<int>() == 150);
    assert(json["debug"]["cpu_temp_c"].get<double>() == 62.5);
    assert(json["debug"]["video_chunk_count"].get<int>() == 12);
    assert(json["debug"]["video_chunks_sent"].get<std::uint64_t>() == 120);
    assert(json["debug"]["video_send_failures"].get<std::uint64_t>() == 1);
    assert(json["debug"]["video_skipped_frames"].get<std::uint64_t>() == 2);
    assert(json["debug"]["line_contours_found"].get<int>() == 3);
    assert(json["debug"]["video_jpeg_bytes"].get<std::uint64_t>() == 12345);
    return 0;
}
