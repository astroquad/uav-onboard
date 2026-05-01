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
    nlohmann::json contour = nlohmann::json::array();
    for (const auto& point : telemetry.vision.line.contour_px) {
        contour.push_back(pointToJson(point));
    }

    nlohmann::json branches = nlohmann::json::array();
    for (const auto& branch : telemetry.vision.intersection.branches) {
        branches.push_back({
            {"direction", branch.direction},
            {"present", branch.present},
            {"score", branch.score},
            {"endpoint_px", pointToJson(branch.endpoint_px)},
            {"angle_deg", branch.angle_deg},
        });
    }

    nlohmann::json decision_branches = nlohmann::json::array();
    for (const auto& branch : telemetry.vision.intersection_decision.branches) {
        decision_branches.push_back({
            {"direction", branch.direction},
            {"present_frames", branch.present_frames},
            {"max_score", branch.max_score},
            {"average_score", branch.average_score},
        });
    }

    const auto nodeToJson = [](const GridNodeTelemetry& node) {
        return nlohmann::json {
            {"valid", node.valid},
            {"id", node.id},
            {"local_coord", {
                {"x", node.x},
                {"y", node.y},
            }},
            {"topology", node.topology},
            {"arrival_heading", node.arrival_heading},
            {"camera_branch_mask", node.camera_branch_mask},
            {"grid_branch_mask", node.grid_branch_mask},
            {"first_node", node.first_node},
            {"origin_local_only", node.origin_local_only},
        };
    };

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
    message["system"] = {
        {"board_model", telemetry.system.board_model},
        {"os_release", telemetry.system.os_release},
        {"uptime_s", telemetry.system.uptime_s},
        {"cpu_temp_c", telemetry.system.cpu_temp_c},
        {"throttled_raw", telemetry.system.throttled_raw},
        {"cpu_load_1m", telemetry.system.cpu_load_1m},
        {"mem_available_kb", telemetry.system.mem_available_kb},
        {"wifi_signal_dbm", telemetry.system.wifi_signal_dbm},
        {"wifi_tx_bitrate_mbps", telemetry.system.wifi_tx_bitrate_mbps},
    };
    message["camera"] = {
        {"status", telemetry.camera.status},
        {"sensor_model", telemetry.camera.sensor_model},
        {"camera_index", telemetry.camera.camera_index},
        {"width", telemetry.camera.width},
        {"height", telemetry.camera.height},
        {"fps", telemetry.camera.fps},
        {"configured_fps", telemetry.camera.configured_fps},
        {"measured_capture_fps", telemetry.camera.measured_capture_fps},
        {"frame_seq", telemetry.camera.frame_seq},
        {"autofocus_mode", telemetry.camera.autofocus_mode},
        {"lens_position", telemetry.camera.lens_position},
        {"exposure_mode", telemetry.camera.exposure_mode},
        {"shutter_us", telemetry.camera.shutter_us},
        {"gain", telemetry.camera.gain},
        {"awb", telemetry.camera.awb},
    };
    message["vision"] = {
        {"line_detected", telemetry.vision.line_detected},
        {"line_offset", telemetry.vision.line_offset},
        {"line_angle", telemetry.vision.line_angle},
        {"line", {
            {"detected", telemetry.vision.line.detected},
            {"raw_detected", telemetry.vision.line.raw_detected},
            {"filtered", telemetry.vision.line.filtered},
            {"held", telemetry.vision.line.held},
            {"rejected_jump", telemetry.vision.line.rejected_jump},
            {"tracking_point_px", pointToJson(telemetry.vision.line.tracking_point_px)},
            {"raw_tracking_point_px", pointToJson(telemetry.vision.line.raw_tracking_point_px)},
            {"centroid_px", pointToJson(telemetry.vision.line.centroid_px)},
            {"center_offset_px", telemetry.vision.line.center_offset_px},
            {"raw_center_offset_px", telemetry.vision.line.raw_center_offset_px},
            {"angle_deg", telemetry.vision.line.angle_deg},
            {"raw_angle_deg", telemetry.vision.line.raw_angle_deg},
            {"confidence", telemetry.vision.line.confidence},
            {"contour_px", contour},
        }},
        {"intersection_detected", telemetry.vision.intersection_detected},
        {"intersection_score", telemetry.vision.intersection_score},
        {"intersection", {
            {"valid", telemetry.vision.intersection.valid},
            {"detected", telemetry.vision.intersection.detected},
            {"type", telemetry.vision.intersection.type},
            {"raw_type", telemetry.vision.intersection.raw_type},
            {"stable", telemetry.vision.intersection.stable},
            {"held", telemetry.vision.intersection.held},
            {"center_px", pointToJson(telemetry.vision.intersection.center_px)},
            {"raw_center_px", pointToJson(telemetry.vision.intersection.raw_center_px)},
            {"score", telemetry.vision.intersection.score},
            {"raw_score", telemetry.vision.intersection.raw_score},
            {"branch_mask", telemetry.vision.intersection.branch_mask},
            {"branch_count", telemetry.vision.intersection.branch_count},
            {"stable_frames", telemetry.vision.intersection.stable_frames},
            {"radius_px", telemetry.vision.intersection.radius_px},
            {"selected_mask_index", telemetry.vision.intersection.selected_mask_index},
            {"branches", branches},
        }},
        {"intersection_decision", {
            {"state", telemetry.vision.intersection_decision.state},
            {"action", telemetry.vision.intersection_decision.action},
            {"accepted_type", telemetry.vision.intersection_decision.accepted_type},
            {"best_observed_type", telemetry.vision.intersection_decision.best_observed_type},
            {"event_ready", telemetry.vision.intersection_decision.event_ready},
            {"turn_candidate", telemetry.vision.intersection_decision.turn_candidate},
            {"required_turn", telemetry.vision.intersection_decision.required_turn},
            {"front_available", telemetry.vision.intersection_decision.front_available},
            {"node_recorded", telemetry.vision.intersection_decision.node_recorded},
            {"cooldown_active", telemetry.vision.intersection_decision.cooldown_active},
            {"accepted_branch_mask", telemetry.vision.intersection_decision.accepted_branch_mask},
            {"window_frames", telemetry.vision.intersection_decision.window_frames},
            {"age_ms", telemetry.vision.intersection_decision.age_ms},
            {"confidence", telemetry.vision.intersection_decision.confidence},
            {"center_px", pointToJson(telemetry.vision.intersection_decision.center_px)},
            {"center_y_norm", telemetry.vision.intersection_decision.center_y_norm},
            {"approach_phase", telemetry.vision.intersection_decision.approach_phase},
            {"overshoot_risk", telemetry.vision.intersection_decision.overshoot_risk},
            {"too_late_to_turn", telemetry.vision.intersection_decision.too_late_to_turn},
            {"branches", decision_branches},
            {"node", nodeToJson(telemetry.vision.intersection_decision.node)},
        }},
        {"grid_node", nodeToJson(telemetry.vision.grid_node)},
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
        {"read_frame_ms", telemetry.debug.read_frame_ms},
        {"jpeg_decode_ms", telemetry.debug.jpeg_decode_ms},
        {"aruco_latency_ms", telemetry.debug.aruco_latency_ms},
        {"line_latency_ms", telemetry.debug.line_latency_ms},
        {"intersection_latency_ms", telemetry.debug.intersection_latency_ms},
        {"intersection_decision_latency_ms", telemetry.debug.intersection_decision_latency_ms},
        {"telemetry_build_ms", telemetry.debug.telemetry_build_ms},
        {"telemetry_send_ms", telemetry.debug.telemetry_send_ms},
        {"video_submit_ms", telemetry.debug.video_submit_ms},
        {"video_send_ms", telemetry.debug.video_send_ms},
        {"capture_fps", telemetry.debug.capture_fps},
        {"processing_fps", telemetry.debug.processing_fps},
        {"debug_video_send_fps", telemetry.debug.debug_video_send_fps},
        {"video_chunk_pacing_us", telemetry.debug.video_chunk_pacing_us},
        {"cpu_temp_c", telemetry.debug.cpu_temp_c},
        {"telemetry_bytes", telemetry.debug.telemetry_bytes},
        {"video_jpeg_bytes", telemetry.debug.video_jpeg_bytes},
        {"video_sent_frames", telemetry.debug.video_sent_frames},
        {"video_dropped_frames", telemetry.debug.video_dropped_frames},
        {"video_skipped_frames", telemetry.debug.video_skipped_frames},
        {"video_chunks_sent", telemetry.debug.video_chunks_sent},
        {"video_send_failures", telemetry.debug.video_send_failures},
        {"video_chunk_count", telemetry.debug.video_chunk_count},
        {"line_mask_count", telemetry.debug.line_mask_count},
        {"line_contours_found", telemetry.debug.line_contours_found},
        {"line_candidates_evaluated", telemetry.debug.line_candidates_evaluated},
        {"line_roi_pixels", telemetry.debug.line_roi_pixels},
        {"line_selected_contour_points", telemetry.debug.line_selected_contour_points},
        {"note", telemetry.note},
    };

    return message.dump();
}

} // namespace onboard::protocol
