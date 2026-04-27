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

struct Run {
    int start = 0;
    int end = 0;

    int width() const
    {
        return end - start + 1;
    }

    double center() const
    {
        return (start + end) / 2.0;
    }
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

Point2f toPoint(const cv::Point& point)
{
    return {
        static_cast<float>(point.x),
        static_cast<float>(point.y),
    };
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
        output.push_back(toPoint(point));
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

std::vector<Run> findLineWidthRuns(
    const std::uint8_t* row,
    int width,
    int min_line_width,
    int max_line_width)
{
    std::vector<Run> runs;
    int x = 0;
    while (x < width) {
        while (x < width && row[x] == 0) {
            ++x;
        }

        const int start = x;
        while (x < width && row[x] != 0) {
            ++x;
        }

        if (start >= width) {
            break;
        }

        const Run run {start, x - 1};
        if (run.width() >= min_line_width && run.width() <= max_line_width) {
            runs.push_back(run);
        }
    }
    return runs;
}

std::optional<Run> nearestRun(
    const std::uint8_t* row,
    int width,
    double expected_x,
    int min_line_width,
    int max_line_width)
{
    const auto runs = findLineWidthRuns(row, width, min_line_width, max_line_width);
    if (runs.empty()) {
        return std::nullopt;
    }

    const double max_jump = std::max(12.0, max_line_width * 1.25);
    std::optional<Run> best;
    double best_distance = std::numeric_limits<double>::max();
    for (const auto& run : runs) {
        const double distance = std::abs(run.center() - expected_x);
        if (distance < best_distance) {
            best = run;
            best_distance = distance;
        }
    }

    if (!best || best_distance > max_jump) {
        return std::nullopt;
    }
    return best;
}

void drawRun(cv::Mat& mask, int y, const Run& run)
{
    auto* row = mask.ptr<std::uint8_t>(y);
    std::fill(row + run.start, row + run.end + 1, static_cast<std::uint8_t>(255));
}

void traceLineBranch(
    const cv::Mat& contour_mask,
    cv::Mat& branch_mask,
    int start_y,
    int end_y,
    int step,
    double initial_x,
    int min_line_width,
    int max_line_width)
{
    double expected_x = initial_x;
    for (int y = start_y;
         (step < 0 && y >= end_y) || (step > 0 && y <= end_y);
         y += step) {
        const auto* row = contour_mask.ptr<std::uint8_t>(y);
        auto run = nearestRun(row, contour_mask.cols, expected_x, min_line_width, max_line_width);
        if (!run) {
            continue;
        }

        drawRun(branch_mask, y, *run);
        expected_x = run->center();
    }
}

std::optional<std::vector<cv::Point>> extractLineBranchContour(
    const cv::Mat& contour_mask,
    int lookahead_y,
    double tracking_x,
    int min_line_width,
    int max_line_width,
    int morph_kernel)
{
    // Keep the line-width branch around the tracking row and drop wide glare or
    // intersection spans that are attached to the same threshold component.
    const int mask_y = std::clamp(lookahead_y, 0, contour_mask.rows - 1);
    cv::Mat branch_mask = cv::Mat::zeros(contour_mask.size(), CV_8UC1);

    traceLineBranch(
        contour_mask,
        branch_mask,
        mask_y,
        0,
        -1,
        tracking_x,
        min_line_width,
        max_line_width);
    if (mask_y + 1 < contour_mask.rows) {
        traceLineBranch(
            contour_mask,
            branch_mask,
            mask_y + 1,
            contour_mask.rows - 1,
            1,
            tracking_x,
            min_line_width,
            max_line_width);
    }

    if (cv::countNonZero(branch_mask) < min_line_width) {
        return std::nullopt;
    }

    const int kernel_size = oddKernelSize(morph_kernel);
    if (kernel_size > 1) {
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(3, kernel_size));
        cv::morphologyEx(branch_mask, branch_mask, cv::MORPH_CLOSE, kernel);
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(branch_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        return std::nullopt;
    }

    std::optional<std::size_t> best_index;
    double best_score = -1.0;
    for (std::size_t index = 0; index < contours.size(); ++index) {
        const auto bounds = cv::boundingRect(contours[index]);
        if (lookahead_y < bounds.y || lookahead_y >= bounds.y + bounds.height) {
            continue;
        }

        const double center_distance =
            std::abs((bounds.x + bounds.width / 2.0) - tracking_x);
        const double score = cv::contourArea(contours[index]) - center_distance;
        if (score > best_score) {
            best_score = score;
            best_index = index;
        }
    }

    if (!best_index) {
        return std::nullopt;
    }
    return contours[*best_index];
}

std::vector<cv::Mat> buildMasks(const cv::Mat& gray, const common::LineConfig& config)
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
        cleanMask(mask, config.morph_kernel);
    }
    return masks;
}

std::optional<Candidate> evaluateContour(
    const std::vector<cv::Point>& roi_contour,
    const cv::Mat& mask,
    const common::LineConfig& config,
    const cv::Rect& roi,
    int image_width,
    int lookahead_y)
{
    const double area = cv::contourArea(roi_contour);
    if (area < config.min_area_px) {
        return std::nullopt;
    }

    cv::Mat contour_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> draw_contours {roi_contour};
    cv::drawContours(contour_mask, draw_contours, 0, cv::Scalar(255), cv::FILLED);

    const int mask_y = std::clamp(lookahead_y - roi.y, 0, contour_mask.rows - 1);
    const auto* row = contour_mask.ptr<std::uint8_t>(mask_y);
    const int max_line_width = std::max(
        config.min_line_width_px + 1,
        static_cast<int>(std::lround(image_width * clampRatio(config.max_line_width_ratio, 0.22))));
    const auto row_runs = findLineWidthRuns(
        row,
        image_width,
        config.min_line_width_px,
        max_line_width);
    if (row_runs.empty()) {
        return std::nullopt;
    }

    Run selected_run = row_runs.front();
    double best_run_distance =
        std::abs(selected_run.center() - image_width / 2.0);
    for (const auto& run : row_runs) {
        const double distance = std::abs(run.center() - image_width / 2.0);
        if (distance < best_run_distance) {
            selected_run = run;
            best_run_distance = distance;
        }
    }

    const int line_width = selected_run.width();
    const double tracking_x = selected_run.center();
    auto line_branch = extractLineBranchContour(
        contour_mask,
        mask_y,
        tracking_x,
        config.min_line_width_px,
        max_line_width,
        config.morph_kernel);

    std::vector<cv::Point> contour = line_branch.value_or(roi_contour);
    for (auto& point : contour) {
        point.y += roi.y;
    }

    const double selected_area = cv::contourArea(contour);
    const cv::Rect bounds = cv::boundingRect(contour);
    const int long_span = std::max(bounds.width, bounds.height);
    const double span_score = std::clamp(
        long_span / static_cast<double>(std::max(1, roi.height)),
        0.0,
        1.0);

    const double roi_area = static_cast<double>(roi.width) * roi.height;
    const double area_score = roi_area > 0.0
        ? std::clamp(selected_area / (roi_area * 0.08), 0.0, 1.0)
        : 0.0;

    const double width_score = 1.0 - std::clamp(
        (line_width - static_cast<double>(config.min_line_width_px)) /
            std::max(1.0, max_line_width - static_cast<double>(config.min_line_width_px)),
        0.0,
        1.0);

    const double center_score = 1.0 - std::clamp(
        std::abs(tracking_x - image_width / 2.0) / std::max(1.0, image_width / 2.0),
        0.0,
        1.0);

    const bool touches_side =
        bounds.x <= 2 || (bounds.x + bounds.width) >= image_width - 2;
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
    line.tracking_point_px.x = static_cast<float>(tracking_x);
    line.tracking_point_px.y = static_cast<float>(lookahead_y);
    line.center_offset_px = line.tracking_point_px.x - static_cast<float>(image_width) / 2.0f;
    line.angle_deg = lineAngleDeg(contour);
    line.confidence = static_cast<float>(std::clamp(score, 0.0, 1.0));
    line.contour_px = simplifyContour(contour, config.max_contour_points);

    cv::Moments moments = cv::moments(contour);
    if (std::abs(moments.m00) > std::numeric_limits<double>::epsilon()) {
        line.centroid_px.x = static_cast<float>(moments.m10 / moments.m00);
        line.centroid_px.y = static_cast<float>(moments.m01 / moments.m00);
    } else {
        line.centroid_px.x = static_cast<float>(bounds.x + bounds.width / 2.0);
        line.centroid_px.y = static_cast<float>(bounds.y + bounds.height / 2.0);
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
    const cv::Rect roi(0, roi_top, width, height - roi_top);
    if (roi.width <= 0 || roi.height <= 0) {
        return {};
    }

    cv::Mat gray;
    const cv::Mat roi_image = image(roi);
    if (roi_image.channels() == 1) {
        gray = roi_image;
    } else {
        cv::cvtColor(roi_image, gray, cv::COLOR_BGR2GRAY);
    }

    const int lookahead_y = std::clamp(
        static_cast<int>(std::lround(height * clampRatio(config_.lookahead_y_ratio, 0.55))),
        roi_top,
        height - 1);

    std::optional<Candidate> best;
    auto masks = buildMasks(gray, config_);
    for (const auto& mask : masks) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (std::size_t index = 0; index < contours.size(); ++index) {
            auto candidate = evaluateContour(
                contours[index],
                mask,
                config_,
                roi,
                width,
                lookahead_y);
            if (candidate && (!best || candidate->score > best->score)) {
                best = std::move(candidate);
            }
        }
    }

    return best ? best->line : LineDetection {};
}

} // namespace onboard::vision
