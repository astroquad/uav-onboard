#include "vision/LineDetector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace onboard::vision {
namespace {

struct Candidate {
    LineDetection line;
    double score = 0.0;
};

struct WorkGeometry {
    int source_width = 0;
    int source_height = 0;
    int roi_top = 0;
    int roi_height = 0;
    int work_width = 0;
    int work_height = 0;
    double scale_x = 1.0;
    double scale_y = 1.0;
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

cv::Point toSourcePoint(const cv::Point& point, const WorkGeometry& geometry)
{
    return {
        static_cast<int>(std::lround(point.x * geometry.scale_x)),
        geometry.roi_top + static_cast<int>(std::lround(point.y * geometry.scale_y)),
    };
}

Point2f toTelemetryPoint(const cv::Point& point)
{
    return {
        static_cast<float>(point.x),
        static_cast<float>(point.y),
    };
}

std::vector<cv::Point> toSourceContour(
    const std::vector<cv::Point>& contour,
    const WorkGeometry& geometry)
{
    std::vector<cv::Point> output;
    output.reserve(contour.size());
    for (const auto& point : contour) {
        output.push_back(toSourcePoint(point, geometry));
    }
    return output;
}

std::vector<Point2f> simplifyContour(
    const std::vector<cv::Point>& contour,
    int max_points)
{
    std::vector<cv::Point> simplified;
    cv::approxPolyDP(contour, simplified, 2.0, true);
    if (simplified.empty()) {
        simplified = contour;
    }

    if (max_points > 0 && static_cast<int>(simplified.size()) > max_points) {
        std::vector<cv::Point> sampled;
        sampled.reserve(static_cast<std::size_t>(max_points));
        for (int index = 0; index < max_points; ++index) {
            const std::size_t source_index =
                static_cast<std::size_t>(index) * simplified.size() /
                static_cast<std::size_t>(max_points);
            sampled.push_back(simplified[source_index]);
        }
        simplified = std::move(sampled);
    }

    std::vector<Point2f> output;
    output.reserve(simplified.size());
    for (const auto& point : simplified) {
        output.push_back(toTelemetryPoint(point));
    }
    return output;
}

float lineAngleDeg(const std::vector<cv::Point>& contour)
{
    if (contour.size() < 2) {
        return 0.0f;
    }

    cv::Vec4f line;
    cv::fitLine(contour, line, cv::DIST_L2, 0.0, 0.01, 0.01);
    return static_cast<float>(std::atan2(line[1], line[0]) * 180.0 / CV_PI);
}

cv::Mat thresholdMask(const cv::Mat& gray, bool light_line, int configured_threshold)
{
    cv::Mat mask;
    const int base_type = light_line ? cv::THRESH_BINARY : cv::THRESH_BINARY_INV;
    if (configured_threshold <= 0) {
        cv::threshold(gray, mask, 0, 255, base_type | cv::THRESH_OTSU);
    } else {
        cv::threshold(
            gray,
            mask,
            std::clamp(configured_threshold, 0, 255),
            255,
            base_type);
    }
    return mask;
}

void cleanMask(cv::Mat& mask, int morph_kernel)
{
    const int kernel_size = oddKernelSize(morph_kernel);
    if (kernel_size <= 1) {
        return;
    }

    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(kernel_size, kernel_size));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
}

int scaledMorphKernel(const common::LineConfig& config, const WorkGeometry& geometry)
{
    const double scale = std::max(1.0, std::max(geometry.scale_x, geometry.scale_y));
    return oddKernelSize(static_cast<int>(std::lround(config.morph_kernel / scale)));
}

std::vector<cv::Mat> buildMasks(
    const cv::Mat& gray,
    const common::LineConfig& config,
    const WorkGeometry& geometry)
{
    std::vector<cv::Mat> masks;
    const std::string mode = config.mode;
    if (mode == "auto") {
        masks.push_back(thresholdMask(gray, true, 0));
        masks.push_back(thresholdMask(gray, false, 0));
    } else if (mode == "light_on_dark") {
        masks.push_back(thresholdMask(gray, true, config.threshold));
    } else {
        masks.push_back(thresholdMask(gray, false, config.threshold));
    }

    for (auto& mask : masks) {
        cleanMask(mask, scaledMorphKernel(config, geometry));
    }
    return masks;
}

int scaledMinArea(const common::LineConfig& config, const WorkGeometry& geometry)
{
    const double scaled_area =
        config.min_area_px / std::max(1.0, geometry.scale_x * geometry.scale_y);
    return std::max(20, static_cast<int>(std::lround(scaled_area)));
}

int scaledMinLineWidth(const common::LineConfig& config, const WorkGeometry& geometry)
{
    const double scaled_width = config.min_line_width_px / std::max(1.0, geometry.scale_x);
    return std::max(2, static_cast<int>(std::lround(scaled_width)));
}

std::vector<std::size_t> sortedCandidateIndexes(
    const std::vector<std::vector<cv::Point>>& contours,
    int min_area,
    int max_candidates)
{
    std::vector<std::pair<double, std::size_t>> ranked;
    ranked.reserve(contours.size());
    for (std::size_t index = 0; index < contours.size(); ++index) {
        const double area = cv::contourArea(contours[index]);
        if (area >= min_area) {
            ranked.push_back({area, index});
        }
    }

    std::sort(
        ranked.begin(),
        ranked.end(),
        [](const auto& lhs, const auto& rhs) {
            return lhs.first > rhs.first;
        });

    if (max_candidates > 0 && static_cast<int>(ranked.size()) > max_candidates) {
        ranked.resize(static_cast<std::size_t>(max_candidates));
    }

    std::vector<std::size_t> indexes;
    indexes.reserve(ranked.size());
    for (const auto& entry : ranked) {
        indexes.push_back(entry.second);
    }
    return indexes;
}

std::optional<Candidate> evaluateContour(
    const std::vector<cv::Point>& work_contour,
    const cv::Mat& mask,
    const common::LineConfig& config,
    const WorkGeometry& geometry,
    int work_lookahead_y)
{
    const double work_area = cv::contourArea(work_contour);
    if (work_area < scaledMinArea(config, geometry)) {
        return std::nullopt;
    }

    const cv::Rect bounds = cv::boundingRect(work_contour);
    if (work_lookahead_y < bounds.y || work_lookahead_y >= bounds.y + bounds.height) {
        return std::nullopt;
    }

    const auto* row = mask.ptr<std::uint8_t>(work_lookahead_y);
    int min_x = bounds.x + bounds.width;
    int max_x = -1;
    for (int x = bounds.x; x < bounds.x + bounds.width; ++x) {
        if (row[x] != 0) {
            min_x = std::min(min_x, x);
            max_x = std::max(max_x, x);
        }
    }

    if (max_x < min_x) {
        return std::nullopt;
    }

    const int work_line_width = max_x - min_x + 1;
    const int min_line_width = scaledMinLineWidth(config, geometry);
    if (work_line_width < min_line_width) {
        return std::nullopt;
    }

    const int max_line_width = std::max(
        min_line_width + 1,
        static_cast<int>(std::lround(
            geometry.work_width * clampRatio(config.max_line_width_ratio, 0.22))));
    const double work_tracking_x = (min_x + max_x) / 2.0;
    const double source_tracking_x = work_tracking_x * geometry.scale_x;
    const double source_lookahead_y =
        geometry.roi_top + work_lookahead_y * geometry.scale_y;

    const auto source_contour = toSourceContour(work_contour, geometry);
    const cv::Rect source_bounds = cv::boundingRect(source_contour);
    const int long_span = std::max(source_bounds.width, source_bounds.height);
    const double span_score = std::clamp(
        long_span / static_cast<double>(std::max(1, geometry.roi_height)),
        0.0,
        1.0);

    const double source_area = work_area * geometry.scale_x * geometry.scale_y;
    const double roi_area = static_cast<double>(geometry.source_width) * geometry.roi_height;
    const double area_score = roi_area > 0.0
        ? std::clamp(source_area / (roi_area * 0.08), 0.0, 1.0)
        : 0.0;

    const double width_score = 1.0 - std::clamp(
        (work_line_width - static_cast<double>(min_line_width)) /
            std::max(1.0, max_line_width - static_cast<double>(min_line_width)),
        0.0,
        1.0);

    const double center_score = 1.0 - std::clamp(
        std::abs(source_tracking_x - geometry.source_width / 2.0) /
            std::max(1.0, geometry.source_width / 2.0),
        0.0,
        1.0);

    const bool touches_side =
        source_bounds.x <= 2 ||
        (source_bounds.x + source_bounds.width) >= geometry.source_width - 2;
    const double edge_penalty = touches_side ? 0.20 : 0.0;

    const double score =
        0.30 * span_score +
        0.25 * area_score +
        0.25 * width_score +
        0.20 * center_score -
        edge_penalty;
    if (score < config.confidence_min) {
        return std::nullopt;
    }

    LineDetection line;
    line.detected = true;
    line.raw_detected = true;
    line.tracking_point_px.x = static_cast<float>(source_tracking_x);
    line.tracking_point_px.y = static_cast<float>(source_lookahead_y);
    line.raw_tracking_point_px = line.tracking_point_px;
    line.center_offset_px =
        line.tracking_point_px.x - static_cast<float>(geometry.source_width) / 2.0f;
    line.raw_center_offset_px = line.center_offset_px;
    line.angle_deg = lineAngleDeg(source_contour);
    line.raw_angle_deg = line.angle_deg;
    line.confidence = static_cast<float>(std::clamp(score, 0.0, 1.0));
    line.contour_px = simplifyContour(source_contour, config.max_contour_points);
    line.selected_contour_points = static_cast<int>(line.contour_px.size());

    cv::Moments moments = cv::moments(source_contour);
    if (std::abs(moments.m00) > std::numeric_limits<double>::epsilon()) {
        line.centroid_px.x = static_cast<float>(moments.m10 / moments.m00);
        line.centroid_px.y = static_cast<float>(moments.m01 / moments.m00);
    } else {
        line.centroid_px.x = static_cast<float>(source_bounds.x + source_bounds.width / 2.0);
        line.centroid_px.y = static_cast<float>(source_bounds.y + source_bounds.height / 2.0);
    }

    Candidate candidate;
    candidate.line = std::move(line);
    candidate.score = score;
    return candidate;
}

} // namespace

LineDetector::LineDetector(const common::LineConfig& config)
    : config_(config)
{
}

LineDetection LineDetector::detect(const cv::Mat& image) const
{
    if (!config_.enabled || image.empty() || image.cols <= 0 || image.rows <= 0) {
        return {};
    }

    const int width = image.cols;
    const int height = image.rows;
    const int roi_top = std::clamp(
        static_cast<int>(std::lround(height * clampRatio(config_.roi_top_ratio, 0.08))),
        0,
        std::max(0, height - 1));
    const cv::Rect source_roi(0, roi_top, width, height - roi_top);
    if (source_roi.width <= 0 || source_roi.height <= 0) {
        return {};
    }

    WorkGeometry geometry;
    geometry.source_width = width;
    geometry.source_height = height;
    geometry.roi_top = roi_top;
    geometry.roi_height = source_roi.height;

    const int requested_width = config_.process_width > 0 ? config_.process_width : width;
    geometry.work_width = std::clamp(requested_width, 1, width);
    if (geometry.work_width == width) {
        geometry.work_height = source_roi.height;
    } else {
        const double scale = geometry.work_width / static_cast<double>(width);
        geometry.work_height =
            std::max(1, static_cast<int>(std::lround(source_roi.height * scale)));
    }
    geometry.scale_x = width / static_cast<double>(geometry.work_width);
    geometry.scale_y = source_roi.height / static_cast<double>(geometry.work_height);

    cv::Mat roi_image = image(source_roi);
    cv::Mat work_image;
    if (geometry.work_width == source_roi.width && geometry.work_height == source_roi.height) {
        work_image = roi_image;
    } else {
        cv::resize(
            roi_image,
            work_image,
            cv::Size(geometry.work_width, geometry.work_height),
            0.0,
            0.0,
            cv::INTER_AREA);
    }

    cv::Mat gray;
    if (work_image.channels() == 1) {
        gray = work_image;
    } else {
        cv::cvtColor(work_image, gray, cv::COLOR_BGR2GRAY);
    }

    const int source_lookahead_y = std::clamp(
        static_cast<int>(std::lround(height * clampRatio(config_.lookahead_y_ratio, 0.55))),
        roi_top,
        height - 1);
    const int work_lookahead_y = std::clamp(
        static_cast<int>(std::lround((source_lookahead_y - roi_top) / geometry.scale_y)),
        0,
        geometry.work_height - 1);

    int total_contours = 0;
    int candidates_evaluated = 0;
    std::optional<Candidate> best;
    auto masks = buildMasks(gray, config_, geometry);
    for (const auto& mask : masks) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        total_contours += static_cast<int>(contours.size());

        const auto candidate_indexes = sortedCandidateIndexes(
            contours,
            scaledMinArea(config_, geometry),
            config_.max_candidates);
        for (const auto index : candidate_indexes) {
            ++candidates_evaluated;
            auto candidate = evaluateContour(
                contours[index],
                mask,
                config_,
                geometry,
                work_lookahead_y);
            if (candidate && (!best || candidate->score > best->score)) {
                best = std::move(candidate);
            }
        }
    }

    LineDetection output = best ? best->line : LineDetection {};
    output.mask_count = static_cast<int>(masks.size());
    output.contours_found = total_contours;
    output.candidates_evaluated = candidates_evaluated;
    output.roi_pixels = geometry.work_width * geometry.work_height;
    output.selected_contour_points = static_cast<int>(output.contour_px.size());
    return output;
}

} // namespace onboard::vision
