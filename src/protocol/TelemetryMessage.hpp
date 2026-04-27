#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace onboard::protocol {

struct Point2f {
    double x = 0.0;
    double y = 0.0;
};

struct MarkerTelemetry {
    int id = -1;
    Point2f center_px;
    std::array<Point2f, 4> corners_px {};
    double orientation_deg = 0.0;
};

struct LineTelemetry {
    bool detected = false;
    bool raw_detected = false;
    bool filtered = false;
    bool held = false;
    bool rejected_jump = false;
    Point2f tracking_point_px;
    Point2f raw_tracking_point_px;
    Point2f centroid_px;
    double center_offset_px = 0.0;
    double raw_center_offset_px = 0.0;
    double angle_deg = 0.0;
    double raw_angle_deg = 0.0;
    double confidence = 0.0;
    std::vector<Point2f> contour_px;
};

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
    LineTelemetry line;
    bool intersection_detected = false;
    double intersection_score = 0.0;
    bool marker_detected = false;
    int marker_id = -1;
    std::vector<MarkerTelemetry> markers;
};

struct GridTelemetry {
    int row = -1;
    int col = -1;
    double heading_deg = 0.0;
};

struct DebugTelemetry {
    double processing_latency_ms = 0.0;
    double read_frame_ms = 0.0;
    double jpeg_decode_ms = 0.0;
    double aruco_latency_ms = 0.0;
    double line_latency_ms = 0.0;
    double telemetry_build_ms = 0.0;
    double telemetry_send_ms = 0.0;
    double video_submit_ms = 0.0;
    std::uint64_t telemetry_bytes = 0;
    std::uint64_t video_jpeg_bytes = 0;
    std::uint64_t video_sent_frames = 0;
    std::uint64_t video_dropped_frames = 0;
    int line_mask_count = 0;
    int line_contours_found = 0;
    int line_candidates_evaluated = 0;
    int line_roi_pixels = 0;
    int line_selected_contour_points = 0;
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
