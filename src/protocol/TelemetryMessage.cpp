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
        {"telemetry_build_ms", telemetry.debug.telemetry_build_ms},
        {"telemetry_send_ms", telemetry.debug.telemetry_send_ms},
        {"video_submit_ms", telemetry.debug.video_submit_ms},
        {"video_send_ms", telemetry.debug.video_send_ms},
        {"cpu_temp_c", telemetry.debug.cpu_temp_c},
        {"telemetry_bytes", telemetry.debug.telemetry_bytes},
        {"video_jpeg_bytes", telemetry.debug.video_jpeg_bytes},
        {"video_sent_frames", telemetry.debug.video_sent_frames},
        {"video_dropped_frames", telemetry.debug.video_dropped_frames},
        {"video_skipped_frames", telemetry.debug.video_skipped_frames},
        {"video_chunks_sent", telemetry.debug.video_chunks_sent},
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
