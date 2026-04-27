#include "vision/LineDetector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <vector>

namespace onboard::vision {
namespace {

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

} // namespace

LineDetector::LineDetector(const common::LineConfig& config)
    : config_(config)
{
}

LineDetection LineDetector::detect(const cv::Mat& image) const
{
    LineDetection output;
    if (!config_.enabled || image.empty() || image.cols <= 0 || image.rows <= 0) {
        return output;
    }

    const int width = image.cols;
    const int height = image.rows;
    const int roi_top = std::clamp(
        static_cast<int>(std::lround(height * clampRatio(config_.roi_top_ratio, 0.35))),
        0,
        std::max(0, height - 1));
    const cv::Rect roi(0, roi_top, width, height - roi_top);
    if (roi.width <= 0 || roi.height <= 0) {
        return output;
    }

    cv::Mat gray;
    const cv::Mat roi_image = image(roi);
    if (roi_image.channels() == 1) {
        gray = roi_image;
    } else {
        cv::cvtColor(roi_image, gray, cv::COLOR_BGR2GRAY);
    }

    cv::Mat mask;
    const int threshold = std::clamp(config_.threshold, 0, 255);
    const int threshold_type =
        config_.mode == "light_on_dark" ? cv::THRESH_BINARY : cv::THRESH_BINARY_INV;
    cv::threshold(gray, mask, threshold, 255, threshold_type);

    const int kernel_size = oddKernelSize(config_.morph_kernel);
    if (kernel_size > 1) {
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(kernel_size, kernel_size));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        return output;
    }

    int best_index = -1;
    double best_area = 0.0;
    for (std::size_t index = 0; index < contours.size(); ++index) {
        const double area = cv::contourArea(contours[index]);
        if (area > best_area) {
            best_area = area;
            best_index = static_cast<int>(index);
        }
    }

    if (best_index < 0 || best_area < config_.min_area_px) {
        return output;
    }

    std::vector<cv::Point> contour = contours[static_cast<std::size_t>(best_index)];
    for (auto& point : contour) {
        point.y += roi_top;
    }

    cv::Moments moments = cv::moments(contour);
    if (std::abs(moments.m00) > std::numeric_limits<double>::epsilon()) {
        output.centroid_px.x = static_cast<float>(moments.m10 / moments.m00);
        output.centroid_px.y = static_cast<float>(moments.m01 / moments.m00);
    } else {
        const cv::Rect bounds = cv::boundingRect(contour);
        output.centroid_px.x = static_cast<float>(bounds.x + bounds.width / 2.0);
        output.centroid_px.y = static_cast<float>(bounds.y + bounds.height / 2.0);
    }

    cv::Mat contour_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> roi_contour = contours;
    cv::drawContours(contour_mask, roi_contour, best_index, cv::Scalar(255), cv::FILLED);

    const int lookahead_y = std::clamp(
        static_cast<int>(std::lround(height * clampRatio(config_.lookahead_y_ratio, 0.70))),
        roi_top,
        height - 1);
    const int mask_y = std::clamp(lookahead_y - roi_top, 0, contour_mask.rows - 1);
    const auto* row = contour_mask.ptr<std::uint8_t>(mask_y);

    int min_x = width;
    int max_x = -1;
    for (int x = 0; x < width; ++x) {
        if (row[x] != 0) {
            min_x = std::min(min_x, x);
            max_x = std::max(max_x, x);
        }
    }

    if (max_x >= min_x) {
        output.tracking_point_px.x = static_cast<float>((min_x + max_x) / 2.0);
        output.tracking_point_px.y = static_cast<float>(lookahead_y);
    } else {
        output.tracking_point_px = output.centroid_px;
    }

    const double roi_area = static_cast<double>(roi.width) * roi.height;
    const double normalized_area = roi_area > 0.0 ? best_area / (roi_area * 0.25) : 0.0;
    output.confidence = static_cast<float>(std::clamp(normalized_area, 0.0, 1.0));
    if (output.confidence < config_.confidence_min) {
        return LineDetection {};
    }

    output.detected = true;
    output.center_offset_px = output.tracking_point_px.x - static_cast<float>(width) / 2.0f;
    output.angle_deg = lineAngleDeg(contour);
    output.contour_px = simplifyContour(contour, config_.max_contour_points);
    return output;
}

} // namespace onboard::vision
