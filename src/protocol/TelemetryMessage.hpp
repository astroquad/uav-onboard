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

struct BranchTelemetry {
    std::string direction;
    bool present = false;
    double score = 0.0;
    Point2f endpoint_px;
    double angle_deg = 0.0;
};

struct IntersectionTelemetry {
    bool valid = false;
    bool detected = false;
    std::string type = "none";
    std::string raw_type = "none";
    bool stable = false;
    bool held = false;
    Point2f center_px;
    Point2f raw_center_px;
    double score = 0.0;
    double raw_score = 0.0;
    int branch_mask = 0;
    int branch_count = 0;
    int stable_frames = 0;
    double radius_px = 0.0;
    int selected_mask_index = -1;
    std::vector<BranchTelemetry> branches;
};

struct BranchEvidenceTelemetry {
    std::string direction;
    int present_frames = 0;
    double max_score = 0.0;
    double average_score = 0.0;
};

struct GridNodeTelemetry {
    bool valid = false;
    std::uint32_t id = 0;
    int x = 0;
    int y = 0;
    std::string topology = "unknown";
    std::string arrival_heading = "unknown";
    int camera_branch_mask = 0;
    int grid_branch_mask = 0;
    bool first_node = false;
    bool origin_local_only = true;
};

// Cycle 13: fractional drone position relative to the last committed grid
// node, used by the GCS to render the heading arrow at a sub-cell position
// so the operator can see progress along the line in real time rather than
// only at intersection commits.
struct DronePositionTelemetry {
    bool   valid = false;
    double cell_progress = 0.0;  // 0.0 (at last node) ~ 1.0 (next node)
    double grid_offset_x = 0.0;  // fractional grid-frame X offset from last node
    double grid_offset_y = 0.0;  // fractional grid-frame Y offset from last node
};

struct IntersectionDecisionTelemetry {
    std::string state = "cruise";
    std::string action = "continue";
    std::string accepted_type = "none";
    std::string best_observed_type = "none";
    bool event_ready = false;
    bool turn_candidate = false;
    bool required_turn = false;
    bool front_available = false;
    bool node_recorded = false;
    bool cooldown_active = false;
    int accepted_branch_mask = 0;
    int window_frames = 0;
    int age_ms = 0;
    double confidence = 0.0;
    Point2f center_px;
    double center_y_norm = 0.0;
    std::string approach_phase = "far";
    bool overshoot_risk = false;
    bool too_late_to_turn = false;
    std::vector<BranchEvidenceTelemetry> branches;
    GridNodeTelemetry node;
};

struct CameraTelemetry {
    std::string status = "not_checked";
    std::string sensor_model;
    int camera_index = 0;
    int width = 0;
    int height = 0;
    double fps = 0.0;
    double configured_fps = 0.0;
    double measured_capture_fps = 0.0;
    std::uint32_t frame_seq = 0;
    std::string autofocus_mode;
    double lens_position = 0.0;
    std::string exposure_mode;
    int shutter_us = 0;
    double gain = 0.0;
    std::string awb;
};

struct VisionTelemetry {
    bool line_detected = false;
    double line_offset = 0.0;
    double line_angle = 0.0;
    LineTelemetry line;
    bool intersection_detected = false;
    double intersection_score = 0.0;
    IntersectionTelemetry intersection;
    IntersectionDecisionTelemetry intersection_decision;
    GridNodeTelemetry grid_node;
    // Cycle 13: drone fractional position from the last committed grid node.
    DronePositionTelemetry drone_position;
    bool marker_detected = false;
    int marker_id = -1;
    std::vector<MarkerTelemetry> markers;
};

struct GridTelemetry {
    int row = -1;
    int col = -1;
    double heading_deg = 0.0;
};

struct MissionGridTelemetry {
    int x = 0;
    int y = 0;
    std::string heading = "unknown";
    std::string snake_dir = "unknown";
    bool valid = false;
};

struct MissionVertiportTelemetry {
    bool verified = false;
    int marker_id = -1;
};

struct MissionMarkerEntry {
    int id = -1;
    int grid_x = 0;
    int grid_y = 0;
    bool grid_valid = false;
    double orientation_deg = 0.0;
    // Cycle 23: onboard wall-clock seconds when this marker was first
    // committed to the MarkerRegistry. Surfaced on the GCS markers panel.
    double first_seen_s = 0.0;
};

struct MissionTelemetry {
    bool present = false;
    std::string state = "idle";
    std::string control_intent = "idle";
    std::int64_t phase_elapsed_ms = 0;
    double target_altitude_m = 0.0;
    bool altitude_off_pad_confirmed = false;
    MissionGridTelemetry grid;
    MissionVertiportTelemetry vertiport;
    std::vector<MissionMarkerEntry> markers_found;
    int markers_expected = 0;
    bool snake_complete = false;
    std::string last_safety_event;
};

struct DebugTelemetry {
    double processing_latency_ms = 0.0;
    double read_frame_ms = 0.0;
    double jpeg_decode_ms = 0.0;
    double aruco_latency_ms = 0.0;
    double line_latency_ms = 0.0;
    double intersection_latency_ms = 0.0;
    double intersection_decision_latency_ms = 0.0;
    double telemetry_build_ms = 0.0;
    double telemetry_send_ms = 0.0;
    double video_submit_ms = 0.0;
    double video_send_ms = 0.0;
    double capture_fps = 0.0;
    double processing_fps = 0.0;
    double debug_video_send_fps = 0.0;
    int video_chunk_pacing_us = 0;
    double cpu_temp_c = 0.0;
    std::uint64_t telemetry_bytes = 0;
    std::uint64_t video_jpeg_bytes = 0;
    std::uint64_t video_sent_frames = 0;
    std::uint64_t video_dropped_frames = 0;
    std::uint64_t video_skipped_frames = 0;
    std::uint64_t video_chunks_sent = 0;
    std::uint64_t video_send_failures = 0;
    int video_chunk_count = 0;
    int line_mask_count = 0;
    int line_contours_found = 0;
    int line_candidates_evaluated = 0;
    int line_roi_pixels = 0;
    int line_selected_contour_points = 0;
};

struct SystemTelemetry {
    std::string board_model;
    std::string os_release;
    double uptime_s = 0.0;
    double cpu_temp_c = 0.0;
    std::string throttled_raw;
    double cpu_load_1m = 0.0;
    std::uint64_t mem_available_kb = 0;
    double wifi_signal_dbm = 0.0;
    double wifi_tx_bitrate_mbps = 0.0;
};

struct BringupTelemetry {
    std::uint32_t seq = 0;
    std::int64_t timestamp_ms = 0;
    SystemTelemetry system;
    CameraTelemetry camera;
    VisionTelemetry vision;
    GridTelemetry grid;
    DebugTelemetry debug;
    MissionTelemetry mission;
    std::string note;
};

std::string buildTelemetryJson(const BringupTelemetry& telemetry);

} // namespace onboard::protocol
