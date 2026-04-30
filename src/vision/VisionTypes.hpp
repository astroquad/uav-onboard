#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace onboard::vision {

struct Point2f {
    float x = 0.0f;
    float y = 0.0f;
};

struct MarkerObservation {
    int id = -1;
    Point2f center_px;
    std::array<Point2f, 4> corners_px {};
    float orientation_deg = 0.0f;
};

struct LineDetection {
    bool detected = false;
    bool raw_detected = false;
    bool filtered = false;
    bool held = false;
    bool rejected_jump = false;
    Point2f tracking_point_px;
    Point2f raw_tracking_point_px;
    Point2f centroid_px;
    float center_offset_px = 0.0f;
    float raw_center_offset_px = 0.0f;
    float angle_deg = 0.0f;
    float raw_angle_deg = 0.0f;
    float confidence = 0.0f;
    std::vector<Point2f> contour_px;
    int mask_count = 0;
    int contours_found = 0;
    int candidates_evaluated = 0;
    int roi_pixels = 0;
    int selected_contour_points = 0;
};

enum class IntersectionType {
    None,
    Unknown,
    Straight,
    L,
    T,
    Cross,
};

enum class BranchDirection {
    Front,
    Right,
    Back,
    Left,
};

struct BranchObservation {
    BranchDirection direction = BranchDirection::Front;
    bool present = false;
    float score = 0.0f;
    Point2f endpoint_px;
    float angle_deg = 0.0f;
};

struct IntersectionDetection {
    bool valid = false;
    bool intersection_detected = false;
    IntersectionType type = IntersectionType::None;
    IntersectionType raw_type = IntersectionType::None;
    bool stable = false;
    bool held = false;
    Point2f center_px;
    Point2f raw_center_px;
    float score = 0.0f;
    float raw_score = 0.0f;
    std::array<BranchObservation, 4> branches {};
    std::uint8_t branch_mask = 0;
    int branch_count = 0;
    int stable_frames = 0;
    float radius_px = 0.0f;
    int selected_mask_index = -1;
};

struct VisionResult {
    std::uint32_t frame_seq = 0;
    std::int64_t timestamp_ms = 0;
    int width = 0;
    int height = 0;
    std::vector<MarkerObservation> markers;
    LineDetection line;
    IntersectionDetection intersection;

    bool line_detected = false;
    bool intersection_detected = false;
};

} // namespace onboard::vision
