#pragma once

#include "common/NetworkConfig.hpp"
#include "common/VisionConfig.hpp"
#include "mission/GridCoordinateTracker.hpp"
#include "mission/IntersectionDecision.hpp"
#include "protocol/TelemetryMessage.hpp"
#include "vision/FrameSource.hpp"
#include "vision/VisionProcessor.hpp"

#include <opencv2/core.hpp>

#include <cstdint>
#include <string>

namespace onboard::app {

struct GcsTelemetryPublisherOptions {
    common::NetworkConfig network;
    common::VisionConfig vision;
    bool send_video = false;
    bool send_telemetry = true;
    std::string note = "vision_debug";
    std::string camera_sensor_model;
    int camera_index = 0;
};

struct GcsTelemetryPublishInput {
    vision::Frame frame;
    cv::Mat image_bgr;
    vision::VisionProcessingOutput vision_output;
    mission::IntersectionDecision intersection_decision;
    mission::GridNodeEvent grid_node;
    // Drone fractional position from the last committed grid node,
    // used by the GCS GridMapTracker to render the heading arrow at a sub-cell
    // position so the operator can see the drone moving along the line.
    bool   drone_position_valid = false;
    double drone_cell_progress = 0.0;
    double drone_grid_offset_x = 0.0;
    double drone_grid_offset_y = 0.0;
    // High-level mission telemetry (state, found-marker registry,
    // grid coord/heading). Populated by the composition root from the
    // GridMission output + MarkerRegistry; forwarded verbatim to the GCS.
    protocol::MissionTelemetry mission;
    double read_frame_ms = 0.0;
    double jpeg_decode_ms = 0.0;
    double processing_latency_ms = 0.0;
    double intersection_decision_latency_ms = 0.0;
    double capture_fps = 0.0;
    double processing_fps = 0.0;
    std::string camera_status = "streaming";
};

struct GcsTelemetryPublishStats {
    double telemetry_build_ms = 0.0;
    double telemetry_send_ms = 0.0;
    double video_submit_ms = 0.0;
    double video_send_ms = 0.0;
    std::uint64_t telemetry_bytes = 0;
    std::uint64_t video_jpeg_bytes = 0;
    std::uint64_t video_sent_frames = 0;
    std::uint64_t video_dropped_frames = 0;
    std::uint64_t video_skipped_frames = 0;
    std::uint64_t video_chunks_sent = 0;
    std::uint64_t video_send_failures = 0;
    int video_chunk_count = 0;
};

class GcsTelemetryPublisher {
public:
    GcsTelemetryPublisher();
    ~GcsTelemetryPublisher();

    GcsTelemetryPublisher(const GcsTelemetryPublisher&) = delete;
    GcsTelemetryPublisher& operator=(const GcsTelemetryPublisher&) = delete;

    bool open(const GcsTelemetryPublisherOptions& options);
    GcsTelemetryPublishStats publish(GcsTelemetryPublishInput input);
    void close();
    // Error from open(); after open() succeeded, prefer takeLastError().
    std::string lastError() const;
    // Consumes the most recent publish/send error. The GCS link is
    // best-effort: a failure never stops future publishes, so callers must
    // not treat a sticky error as a reason to skip publishing.
    std::string takeLastError();

private:
    struct Impl;
    Impl* impl_;
};

} // namespace onboard::app
