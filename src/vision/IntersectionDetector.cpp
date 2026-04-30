#include "vision/IntersectionDetector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace onboard::vision {
namespace {

struct RayScore {
    bool present = false;
    float score = 0.0f;
    cv::Point endpoint;
    float angle_deg = 0.0f;
};

struct CenterCandidate {
    cv::Point point;
    double seed_score = 0.0;
};

struct EvaluatedCenter {
    IntersectionDetection detection;
    double score = 0.0;
};

double clampRatio(double value, double fallback)
{
    if (!std::isfinite(value)) {
        return fallback;
    }
    return std::clamp(value, 0.0, 1.0);
}

int oddKernelSize(int value)
{
    if (value <= 1) {
        return 1;
    }
    return value % 2 == 0 ? value + 1 : value;
}

bool validGeometry(const LineMaskGeometry& geometry)
{
    return geometry.source_width > 0 &&
           geometry.source_height > 0 &&
           geometry.work_width > 0 &&
           geometry.work_height > 0;
}

float angleDegForDirection(BranchDirection direction)
{
    switch (direction) {
    case BranchDirection::Front:
        return -90.0f;
    case BranchDirection::Right:
        return 0.0f;
    case BranchDirection::Back:
        return 90.0f;
    case BranchDirection::Left:
        return 180.0f;
    }
    return 0.0f;
}

std::array<BranchDirection, 4> branchDirections()
{
    return {
        BranchDirection::Front,
        BranchDirection::Right,
        BranchDirection::Back,
        BranchDirection::Left,
    };
}

cv::Point directionVector(BranchDirection direction)
{
    switch (direction) {
    case BranchDirection::Front:
        return {0, -1};
    case BranchDirection::Right:
        return {1, 0};
    case BranchDirection::Back:
        return {0, 1};
    case BranchDirection::Left:
        return {-1, 0};
    }
    return {0, 0};
}

int branchBit(BranchDirection direction)
{
    switch (direction) {
    case BranchDirection::Front:
        return 1 << 0;
    case BranchDirection::Right:
        return 1 << 1;
    case BranchDirection::Back:
        return 1 << 2;
    case BranchDirection::Left:
        return 1 << 3;
    }
    return 0;
}

double angleDeltaDeg(double lhs, double rhs)
{
    double delta = std::fmod(lhs - rhs + 180.0, 360.0);
    if (delta < 0.0) {
        delta += 360.0;
    }
    return std::abs(delta - 180.0);
}

cv::Point toWorkPoint(const cv::Point2d& point, const LineMaskGeometry& geometry)
{
    return {
        std::clamp(static_cast<int>(std::lround(point.x)), 0, geometry.work_width - 1),
        std::clamp(static_cast<int>(std::lround(point.y)), 0, geometry.work_height - 1),
    };
}

Point2f toTelemetryPoint(const cv::Point& point)
{
    return {
        static_cast<float>(point.x),
        static_cast<float>(point.y),
    };
}

cv::Rect clampRect(const cv::Rect& input, const LineMaskGeometry& geometry)
{
    const cv::Rect frame(0, 0, geometry.work_width, geometry.work_height);
    return input & frame;
}

bool rectIntersects(const cv::Rect& lhs, const cv::Rect& rhs)
{
    return (lhs & rhs).area() > 0;
}

int scaledMinArea(const common::LineConfig& config, const LineMaskGeometry& geometry)
{
    const double scaled_area =
        config.min_area_px / std::max(1.0, geometry.scale_x * geometry.scale_y);
    return std::max(20, static_cast<int>(std::lround(scaled_area)));
}

cv::Mat bridgeMask(const cv::Mat& mask, const common::LineConfig& config, const LineMaskGeometry& geometry)
{
    cv::Mat output = mask.clone();
    int close_kernel = scaledLineKernelSize(
        config.morph_close_kernel > 0 ? config.morph_close_kernel : config.morph_kernel,
        geometry);
    close_kernel = std::clamp(oddKernelSize(close_kernel * 2 - 1), 3, 25);
    const cv::Mat close = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(close_kernel, close_kernel));
    cv::morphologyEx(output, output, cv::MORPH_CLOSE, close);

    const int dilate_kernel = std::clamp(oddKernelSize(close_kernel / 2), 1, 9);
    if (dilate_kernel > 1) {
        const cv::Mat dilate = cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(dilate_kernel, dilate_kernel));
        cv::dilate(output, output, dilate);
    }
    return output;
}

double foregroundDensity(const cv::Mat& mask, cv::Point center, int radius)
{
    const cv::Rect bounds = cv::Rect(
        center.x - radius,
        center.y - radius,
        radius * 2 + 1,
        radius * 2 + 1) & cv::Rect(0, 0, mask.cols, mask.rows);
    if (bounds.empty()) {
        return 0.0;
    }
    return cv::countNonZero(mask(bounds)) / static_cast<double>(bounds.area());
}

cv::Rect centerSearchRect(const LineMaskGeometry& geometry)
{
    const int width = std::max(1, static_cast<int>(std::lround(geometry.work_width * 0.45)));
    const int height = std::max(1, static_cast<int>(std::lround(geometry.work_height * 0.55)));
    return clampRect(
        cv::Rect(
            geometry.work_width / 2 - width / 2,
            geometry.work_height / 2 - height / 2,
            width,
            height),
        geometry);
}

cv::Rect rawPointSearchRect(const LineDetection& raw_line, const LineMaskGeometry& geometry)
{
    if (!raw_line.raw_detected && !raw_line.detected) {
        return {};
    }
    const auto raw = raw_line.raw_detected ? raw_line.raw_tracking_point_px : raw_line.tracking_point_px;
    const cv::Point work = onboard::vision::toWorkPoint(raw.x, raw.y, geometry);
    const int width = std::max(12, geometry.work_width / 5);
    const int height = std::max(12, geometry.work_height / 5);
    return clampRect(cv::Rect(work.x - width / 2, work.y - height / 2, width, height), geometry);
}

void addUniqueCandidate(std::vector<CenterCandidate>& candidates, cv::Point point, double seed_score)
{
    for (const auto& candidate : candidates) {
        if (std::abs(candidate.point.x - point.x) <= 2 &&
            std::abs(candidate.point.y - point.y) <= 2) {
            return;
        }
    }
    candidates.push_back({point, seed_score});
}

std::vector<CenterCandidate> centerCandidates(
    const cv::Mat& mask,
    const common::LineConfig& config,
    const LineMaskGeometry& geometry,
    const LineDetection& raw_line)
{
    std::vector<CenterCandidate> candidates;
    const cv::Rect center_rect = centerSearchRect(geometry);
    const cv::Rect raw_rect = rawPointSearchRect(raw_line, geometry);
    addUniqueCandidate(candidates, {geometry.work_width / 2, geometry.work_height / 2}, 0.2);

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int components = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8);
    const int min_area = scaledMinArea(config, geometry);
    int best_label = -1;
    int best_area = 0;
    for (int label = 1; label < components; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < min_area) {
            continue;
        }
        const cv::Rect bounds(
            stats.at<int>(label, cv::CC_STAT_LEFT),
            stats.at<int>(label, cv::CC_STAT_TOP),
            stats.at<int>(label, cv::CC_STAT_WIDTH),
            stats.at<int>(label, cv::CC_STAT_HEIGHT));
        if (!rectIntersects(bounds, center_rect) &&
            (raw_rect.empty() || !rectIntersects(bounds, raw_rect))) {
            continue;
        }
        if (area > best_area) {
            best_area = area;
            best_label = label;
        }
    }
    if (best_label >= 0) {
        addUniqueCandidate(
            candidates,
            toWorkPoint(
                {centroids.at<double>(best_label, 0), centroids.at<double>(best_label, 1)},
                geometry),
            0.8);
    }

    const int stride = std::max(6, std::min(geometry.work_width, geometry.work_height) / 28);
    for (int y = center_rect.y; y < center_rect.y + center_rect.height; y += stride) {
        for (int x = center_rect.x; x < center_rect.x + center_rect.width; x += stride) {
            const int radius = std::max(2, stride / 2);
            if (foregroundDensity(mask, {x, y}, radius) > 0.10) {
                addUniqueCandidate(candidates, {x, y}, 0.4);
            }
        }
    }
    if (!raw_rect.empty()) {
        for (int y = raw_rect.y; y < raw_rect.y + raw_rect.height; y += stride) {
            for (int x = raw_rect.x; x < raw_rect.x + raw_rect.width; x += stride) {
                const int radius = std::max(2, stride / 2);
                if (foregroundDensity(mask, {x, y}, radius) > 0.10) {
                    addUniqueCandidate(candidates, {x, y}, 0.3);
                }
            }
        }
    }

    return candidates;
}

RayScore scoreBranch(
    const cv::Mat& mask,
    cv::Point center,
    BranchDirection direction,
    double present_threshold)
{
    const cv::Point vector = directionVector(direction);
    const int line_width = std::max(3, std::min(mask.cols, mask.rows) / 55);
    const int strip = std::max(1, line_width / 2);
    const int inner_radius = std::max(5, std::min(mask.cols, mask.rows) / 45);
    const int max_distance = [&]() {
        switch (direction) {
        case BranchDirection::Front:
            return center.y;
        case BranchDirection::Right:
            return mask.cols - 1 - center.x;
        case BranchDirection::Back:
            return mask.rows - 1 - center.y;
        case BranchDirection::Left:
            return center.x;
        }
        return 0;
    }();
    const int outer_radius = std::min(
        max_distance,
        std::max(inner_radius + 1, static_cast<int>(std::lround(std::max(mask.cols, mask.rows) * 0.38))));

    int samples = 0;
    int hits = 0;
    int run = 0;
    int best_run = 0;
    cv::Point endpoint = center;
    for (int distance = inner_radius; distance <= outer_radius; distance += 2) {
        const cv::Point base = center + vector * distance;
        int strip_hits = 0;
        int strip_samples = 0;
        for (int offset = -strip; offset <= strip; ++offset) {
            cv::Point point = base;
            if (direction == BranchDirection::Front || direction == BranchDirection::Back) {
                point.x += offset;
            } else {
                point.y += offset;
            }
            if (point.x < 0 || point.x >= mask.cols || point.y < 0 || point.y >= mask.rows) {
                continue;
            }
            ++strip_samples;
            if (mask.at<std::uint8_t>(point) != 0) {
                ++strip_hits;
            }
        }
        if (strip_samples == 0) {
            continue;
        }
        ++samples;
        const bool occupied = strip_hits >= std::max(1, strip_samples / 3);
        if (occupied) {
            ++hits;
            ++run;
            endpoint = base;
            best_run = std::max(best_run, run);
        } else {
            run = 0;
        }
    }

    const double occupancy = samples > 0 ? hits / static_cast<double>(samples) : 0.0;
    const double continuity = samples > 0 ? best_run / static_cast<double>(samples) : 0.0;
    const double score = 0.65 * occupancy + 0.35 * continuity;

    RayScore output;
    output.present = score >= present_threshold;
    output.score = static_cast<float>(std::clamp(score, 0.0, 1.0));
    output.endpoint = endpoint;
    output.angle_deg = angleDegForDirection(direction);
    return output;
}

IntersectionType classifyTwoBranches(const std::array<BranchObservation, 4>& branches)
{
    std::vector<float> angles;
    for (const auto& branch : branches) {
        if (branch.present) {
            angles.push_back(branch.angle_deg);
        }
    }
    if (angles.size() != 2) {
        return IntersectionType::Unknown;
    }
    const double delta = angleDeltaDeg(angles[0], angles[1]);
    if (delta >= 150.0) {
        return IntersectionType::Straight;
    }
    if (delta >= 60.0 && delta <= 120.0) {
        return IntersectionType::L;
    }
    return IntersectionType::Unknown;
}

IntersectionType classifyBranches(int branch_count, const std::array<BranchObservation, 4>& branches)
{
    if (branch_count >= 4) {
        return IntersectionType::Cross;
    }
    if (branch_count == 3) {
        return IntersectionType::T;
    }
    if (branch_count == 2) {
        return classifyTwoBranches(branches);
    }
    return branch_count > 0 ? IntersectionType::Unknown : IntersectionType::None;
}

bool isIntersectionType(IntersectionType type)
{
    return type == IntersectionType::Cross ||
           type == IntersectionType::T ||
           type == IntersectionType::L;
}

std::optional<EvaluatedCenter> evaluateCenter(
    const cv::Mat& mask,
    const LineMaskGeometry& geometry,
    cv::Point center,
    double seed_score,
    int selected_mask_index,
    const common::LineConfig& config)
{
    const double present_threshold = std::clamp(
        config.intersection_threshold > 0.0 ? config.intersection_threshold : 0.8,
        0.25,
        0.95);
    const int center_radius = std::max(3, std::min(mask.cols, mask.rows) / 48);
    const double center_density = foregroundDensity(mask, center, center_radius);

    IntersectionDetection detection;
    detection.valid = center_density > 0.04;
    detection.center_px = toTelemetryPoint(toSourcePoint(center, geometry));
    detection.raw_center_px = detection.center_px;
    detection.selected_mask_index = selected_mask_index;
    detection.radius_px = static_cast<float>(center_radius * std::max(geometry.scale_x, geometry.scale_y));

    int branch_count = 0;
    double present_score_sum = 0.0;
    std::uint8_t branch_mask = 0;
    const auto directions = branchDirections();
    for (std::size_t index = 0; index < directions.size(); ++index) {
        const auto direction = directions[index];
        const auto ray = scoreBranch(mask, center, direction, present_threshold);
        auto& branch = detection.branches[index];
        branch.direction = direction;
        branch.present = ray.present;
        branch.score = ray.score;
        branch.endpoint_px = toTelemetryPoint(toSourcePoint(ray.endpoint, geometry));
        branch.angle_deg = ray.angle_deg;
        if (ray.present) {
            ++branch_count;
            present_score_sum += ray.score;
            branch_mask = static_cast<std::uint8_t>(branch_mask | branchBit(direction));
        }
    }

    detection.branch_count = branch_count;
    detection.branch_mask = branch_mask;
    detection.type = classifyBranches(branch_count, detection.branches);
    detection.raw_type = detection.type;
    detection.intersection_detected = isIntersectionType(detection.type);
    const double branch_score = branch_count > 0 ? present_score_sum / branch_count : 0.0;
    const double branch_count_score = std::clamp(branch_count / 4.0, 0.0, 1.0);
    const double center_penalty = std::clamp(
        std::hypot(center.x - mask.cols / 2.0, center.y - mask.rows / 2.0) /
            std::max(1.0, std::hypot(mask.cols / 2.0, mask.rows / 2.0)),
        0.0,
        1.0);
    detection.score = static_cast<float>(std::clamp(
        0.45 * branch_score +
            0.25 * branch_count_score +
            0.20 * std::clamp(center_density * 2.0, 0.0, 1.0) +
            0.10 * seed_score -
            0.08 * center_penalty,
        0.0,
        1.0));
    detection.raw_score = detection.score;
    detection.valid = detection.valid && detection.type != IntersectionType::None && detection.score >= 0.20f;

    if (!detection.valid) {
        return std::nullopt;
    }

    double type_bonus = 0.0;
    if (detection.type == IntersectionType::Cross) {
        type_bonus = 0.12;
    } else if (detection.type == IntersectionType::T) {
        type_bonus = 0.09;
    } else if (detection.type == IntersectionType::L) {
        type_bonus = 0.06;
    }

    EvaluatedCenter evaluated;
    evaluated.detection = detection;
    evaluated.score = detection.score + type_bonus;
    return evaluated;
}

} // namespace

IntersectionDetector::IntersectionDetector(const common::LineConfig& config)
    : config_(config)
{
}

IntersectionDetection IntersectionDetector::detect(
    const LineMaskFrame& masks,
    const LineDetection& raw_line) const
{
    if (!config_.enabled || !validGeometry(masks.geometry) || masks.masks.empty()) {
        return {};
    }

    std::optional<EvaluatedCenter> best;
    for (std::size_t mask_index = 0; mask_index < masks.masks.size(); ++mask_index) {
        const auto& candidate_mask = masks.masks[mask_index].mask;
        if (candidate_mask.empty()) {
            continue;
        }
        const cv::Mat mask = bridgeMask(candidate_mask, config_, masks.geometry);
        const auto candidates = centerCandidates(mask, config_, masks.geometry, raw_line);
        for (const auto& candidate : candidates) {
            auto evaluated = evaluateCenter(
                mask,
                masks.geometry,
                candidate.point,
                candidate.seed_score,
                static_cast<int>(mask_index),
                config_);
            if (evaluated && (!best || evaluated->score > best->score)) {
                best = std::move(evaluated);
            }
        }
    }

    if (!best) {
        return {};
    }
    return best->detection;
}

const char* intersectionTypeName(IntersectionType type)
{
    switch (type) {
    case IntersectionType::None:
        return "none";
    case IntersectionType::Unknown:
        return "unknown";
    case IntersectionType::Straight:
        return "straight";
    case IntersectionType::L:
        return "L";
    case IntersectionType::T:
        return "T";
    case IntersectionType::Cross:
        return "+";
    }
    return "unknown";
}

const char* branchDirectionName(BranchDirection direction)
{
    switch (direction) {
    case BranchDirection::Front:
        return "front";
    case BranchDirection::Right:
        return "right";
    case BranchDirection::Back:
        return "back";
    case BranchDirection::Left:
        return "left";
    }
    return "unknown";
}

} // namespace onboard::vision
