#pragma once

#include "common/VisionConfig.hpp"
#include "vision/ArucoDetector.hpp"
#include "vision/IntersectionDetector.hpp"
#include "vision/IntersectionStabilizer.hpp"
#include "vision/LineDetector.hpp"
#include "vision/LineMaskBuilder.hpp"
#include "vision/LineStabilizer.hpp"
#include "vision/MarkerStabilizer.hpp"
#include "vision/VisionTypes.hpp"

#include <opencv2/core.hpp>

#include <cstdint>
#include <string>

namespace onboard::vision {

struct VisionFrameMetadata {
    std::uint32_t frame_seq = 0;
    std::int64_t timestamp_ms = 0;
    int width = 0;
    int height = 0;
};

struct VisionProcessorOptions {
    common::VisionConfig vision;
    bool enable_aruco = true;
    bool enable_line = true;
};

struct VisionProcessingMetrics {
    double aruco_latency_ms = 0.0;
    double line_latency_ms = 0.0;
    double intersection_latency_ms = 0.0;
};

struct VisionProcessingOutput {
    VisionResult result;
    VisionProcessingMetrics metrics;
};

class VisionProcessor {
public:
    explicit VisionProcessor(VisionProcessorOptions options);

    VisionProcessingOutput process(const cv::Mat& image, const VisionFrameMetadata& metadata);
    std::string arucoDictionaryName() const;

private:
    VisionProcessorOptions options_;
    ArucoDetector aruco_detector_;
    LineMaskBuilder line_mask_builder_;
    LineDetector line_detector_;
    LineStabilizer line_stabilizer_;
    IntersectionDetector intersection_detector_;
    IntersectionStabilizer intersection_stabilizer_;
    MarkerStabilizer marker_stabilizer_;
    int processed_count_ = 0;
};

} // namespace onboard::vision
