#include "vision/VisionProcessor.hpp"

#include <algorithm>
#include <chrono>
#include <utility>
#include <vector>

namespace onboard::vision {
namespace {

using Clock = std::chrono::steady_clock;

double elapsedMs(Clock::time_point started, Clock::time_point finished)
{
    return std::chrono::duration<double, std::milli>(finished - started).count();
}

} // namespace

VisionProcessor::VisionProcessor(VisionProcessorOptions options)
    : options_(std::move(options))
    , aruco_detector_(options_.vision.aruco)
    , line_mask_builder_(options_.vision.line)
    , line_detector_(options_.vision.line)
    , line_stabilizer_(options_.vision.line)
    , intersection_detector_(options_.vision.line)
    , intersection_stabilizer_(options_.vision.line)
    , marker_stabilizer_(options_.vision.aruco)
{
}

VisionProcessingOutput VisionProcessor::process(
    const cv::Mat& image,
    const VisionFrameMetadata& metadata)
{
    VisionProcessingOutput output;
    auto& result = output.result;
    result.frame_seq = metadata.frame_seq;
    result.timestamp_ms = metadata.timestamp_ms;
    result.width = metadata.width > 0 ? metadata.width : image.cols;
    result.height = metadata.height > 0 ? metadata.height : image.rows;

    if (image.empty()) {
        ++processed_count_;
        return output;
    }

    std::vector<MarkerObservation> marker_mask_input;
    std::vector<MarkerObservation> marker_tracking_input;

    if (options_.enable_aruco) {
        const int aruco_interval = std::max(1, options_.vision.aruco.detect_interval_frames);
        const bool run_aruco = (processed_count_ % aruco_interval) == 0;
        std::vector<MarkerObservation> fresh_markers;
        if (run_aruco) {
            const auto aruco_started = Clock::now();
            const auto aruco_result = aruco_detector_.detect(
                image,
                result.frame_seq,
                result.timestamp_ms);
            const auto aruco_finished = Clock::now();
            fresh_markers = aruco_result.markers;
            marker_mask_input = fresh_markers;
            output.metrics.aruco_latency_ms = elapsedMs(aruco_started, aruco_finished);
        }
        result.markers = marker_stabilizer_.update(fresh_markers);
        if (marker_stabilizer_.ageFrames() <= aruco_interval) {
            marker_tracking_input = result.markers;
        }
    }

    if (options_.enable_line && options_.vision.line.enabled) {
        const auto line_started = Clock::now();
        const auto line_masks = line_mask_builder_.build(
            image,
            marker_mask_input,
            options_.vision.line.marker_mask_detect_candidates);
        auto raw_line = line_detector_.detect(line_masks);
        raw_line = applyMarkerCenterTracking(
            raw_line,
            marker_tracking_input,
            result.width,
            result.height);
        const auto line_finished = Clock::now();
        result.line = line_stabilizer_.update(raw_line, result.width);
        result.line = applyMarkerCenterTracking(
            result.line,
            marker_tracking_input,
            result.width,
            result.height);
        result.line_detected = result.line.detected;
        output.metrics.line_latency_ms = elapsedMs(line_started, line_finished);

        const auto intersection_started = Clock::now();
        const auto raw_intersection = intersection_detector_.detect(line_masks, raw_line);
        result.intersection = intersection_stabilizer_.update(raw_intersection);
        result.intersection_detected = result.intersection.intersection_detected;
        const auto intersection_finished = Clock::now();
        output.metrics.intersection_latency_ms =
            elapsedMs(intersection_started, intersection_finished);
    }

    ++processed_count_;
    return output;
}

std::string VisionProcessor::arucoDictionaryName() const
{
    return aruco_detector_.dictionaryName();
}

} // namespace onboard::vision
