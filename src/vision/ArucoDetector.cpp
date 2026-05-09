#include "vision/ArucoDetector.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <vector>

namespace onboard::vision {
namespace {

cv::aruco::PredefinedDictionaryType dictionaryFromName(const std::string& name)
{
    static const std::map<std::string, cv::aruco::PredefinedDictionaryType> dictionaries = {
        {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
        {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
        {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
        {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
        {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
        {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
        {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
        {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
        {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
        {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
        {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
        {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
        {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
        {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
        {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
        {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
        {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
    };

    const auto it = dictionaries.find(name);
    if (it == dictionaries.end()) {
        return cv::aruco::DICT_4X4_50;
    }
    return it->second;
}

Point2f toPoint(const cv::Point2f& point)
{
    return {point.x, point.y};
}

float orientationDeg(const std::array<Point2f, 4>& corners)
{
    const float dx = corners[1].x - corners[0].x;
    const float dy = corners[1].y - corners[0].y;
    return static_cast<float>(std::atan2(dy, dx) * 180.0 / CV_PI);
}

float distancePx(Point2f lhs, Point2f rhs)
{
    const float dx = lhs.x - rhs.x;
    const float dy = lhs.y - rhs.y;
    return std::sqrt(dx * dx + dy * dy);
}

float markerApproxSidePx(const MarkerObservation& marker)
{
    float sum = 0.0f;
    for (std::size_t index = 0; index < marker.corners_px.size(); ++index) {
        sum += distancePx(
            marker.corners_px[index],
            marker.corners_px[(index + 1) % marker.corners_px.size()]);
    }
    return sum / static_cast<float>(marker.corners_px.size());
}

bool isLiveFrameSize(const cv::Size& size)
{
    return std::min(size.width, size.height) <= 1000;
}

bool isLikelyPartialLiveMarker(const MarkerObservation& marker, const cv::Size& frame_size)
{
    if (!isLiveFrameSize(frame_size)) {
        return false;
    }
    const float frame_min = static_cast<float>(std::max(1, std::min(frame_size.width, frame_size.height)));
    return markerApproxSidePx(marker) < frame_min * 0.22f;
}

MarkerObservation makeMarkerObservation(int id, const std::vector<cv::Point2f>& corners)
{
    MarkerObservation marker;
    marker.id = id;
    for (std::size_t corner_index = 0; corner_index < marker.corners_px.size(); ++corner_index) {
        marker.corners_px[corner_index] = toPoint(corners[corner_index]);
    }

    for (const auto& corner : marker.corners_px) {
        marker.center_px.x += corner.x;
        marker.center_px.y += corner.y;
    }
    marker.center_px.x /= static_cast<float>(marker.corners_px.size());
    marker.center_px.y /= static_cast<float>(marker.corners_px.size());
    marker.orientation_deg = orientationDeg(marker.corners_px);
    return marker;
}

void appendMarkerIfUnique(VisionResult& result, const MarkerObservation& marker)
{
    if (marker.id < 0) {
        return;
    }
    const float marker_side = markerApproxSidePx(marker);
    for (auto& existing : result.markers) {
        const float existing_side = markerApproxSidePx(existing);
        const float near_threshold = std::max(12.0f, std::max(marker_side, existing_side) * 0.55f);
        if (existing.id == marker.id || distancePx(existing.center_px, marker.center_px) < near_threshold) {
            if (marker_side > existing_side * 1.25f) {
                existing = marker;
            }
            return;
        }
    }
    result.markers.push_back(marker);
}

void appendDetectedMarkers(
    const std::vector<std::vector<cv::Point2f>>& marker_corners,
    const std::vector<int>& ids,
    const cv::Size& frame_size,
    std::vector<MarkerObservation>& partial_markers,
    VisionResult& result)
{
    for (std::size_t marker_index = 0; marker_index < ids.size(); ++marker_index) {
        if (marker_index >= marker_corners.size() || marker_corners[marker_index].size() != 4) {
            continue;
        }
        const auto marker = makeMarkerObservation(ids[marker_index], marker_corners[marker_index]);
        if (isLikelyPartialLiveMarker(marker, frame_size)) {
            partial_markers.push_back(marker);
            continue;
        }
        appendMarkerIfUnique(result, marker);
    }
}

cv::Rect2f boundingRectForCorners(const std::vector<cv::Point2f>& corners)
{
    float min_x = corners.front().x;
    float min_y = corners.front().y;
    float max_x = corners.front().x;
    float max_y = corners.front().y;
    for (const auto& corner : corners) {
        min_x = std::min(min_x, corner.x);
        min_y = std::min(min_y, corner.y);
        max_x = std::max(max_x, corner.x);
        max_y = std::max(max_y, corner.y);
    }
    return {min_x, min_y, max_x - min_x, max_y - min_y};
}

double darkRatioInRect(const cv::Mat& gray, const cv::Rect& rect)
{
    const cv::Rect clipped = rect & cv::Rect(0, 0, gray.cols, gray.rows);
    if (clipped.empty()) {
        return 0.0;
    }
    cv::Mat dark;
    cv::threshold(gray(clipped), dark, 95, 255, cv::THRESH_BINARY_INV);
    return cv::countNonZero(dark) / static_cast<double>(std::max(1, clipped.area()));
}

bool markerShapeLooksPlausible(
    const cv::Mat& gray,
    const std::vector<cv::Point2f>& source_corners)
{
    const cv::Rect2f bounds = boundingRectForCorners(source_corners);
    if (bounds.width <= 0.0f || bounds.height <= 0.0f) {
        return false;
    }

    const float side_ratio = bounds.width / bounds.height;
    const float min_side = std::min(bounds.width, bounds.height);
    const float max_side = std::max(bounds.width, bounds.height);
    const float frame_min = static_cast<float>(std::max(1, std::min(gray.cols, gray.rows)));
    const bool live_frame = isLiveFrameSize(gray.size());
    const float min_side_ratio = live_frame ? 0.20f : 0.030f;
    const float max_side_ratio = live_frame ? 0.42f : 0.36f;
    if (side_ratio < 0.65f || side_ratio > 1.55f ||
        min_side < std::max(16.0f, frame_min * min_side_ratio) ||
        max_side > frame_min * max_side_ratio) {
        return false;
    }

    const int pad = std::max(1, static_cast<int>(std::lround(min_side * 0.08f)));
    const cv::Rect rect(
        static_cast<int>(std::floor(bounds.x)) - pad,
        static_cast<int>(std::floor(bounds.y)) - pad,
        static_cast<int>(std::ceil(bounds.width)) + pad * 2,
        static_cast<int>(std::ceil(bounds.height)) + pad * 2);
    return darkRatioInRect(gray, rect) >= 0.35;
}

bool shouldRunFullFallback(
    const common::ArucoConfig& config,
    std::uint32_t frame_seq,
    const cv::Size& frame_size)
{
    if (!isLiveFrameSize(frame_size)) {
        return true;
    }

    const int interval = std::max(1, config.full_fallback_interval_frames);
    const int phase_window = std::clamp(
        std::max(1, config.detect_interval_frames),
        1,
        interval);
    return static_cast<int>(frame_seq % static_cast<std::uint32_t>(interval)) < phase_window;
}

cv::Mat brightComponentMask(const cv::Mat& image, const cv::Mat& gray)
{
    cv::Mat mask;
    if (image.channels() >= 3) {
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 0, 170), cv::Scalar(180, 190, 255), mask);
    } else {
        cv::threshold(gray, mask, 220, 255, cv::THRESH_BINARY);
    }

    const cv::Mat close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, close);
    return mask;
}

std::vector<int> fallbackRoiSizes(int frame_min, int component_width, int component_height)
{
    const int min_size = std::max(44, static_cast<int>(std::lround(frame_min * 0.035)));
    const int max_size = std::min(
        std::max(min_size, static_cast<int>(std::lround(frame_min * 0.38))),
        std::max(min_size, 280));

    std::vector<int> sizes;
    const auto add_size = [&](double value) {
        const int size = std::clamp(static_cast<int>(std::lround(value)), min_size, max_size);
        sizes.push_back(size);
    };

    const int component_side = std::max(component_width, component_height);
    for (const double scale : {1.5, 1.8, 2.1, 2.5, 3.0, 3.6}) {
        add_size(component_side * scale);
    }
    for (const double ratio : {0.035, 0.055, 0.08, 0.11, 0.16, 0.24, 0.32, 0.38}) {
        add_size(frame_min * ratio);
    }

    std::sort(sizes.begin(), sizes.end());
    sizes.erase(
        std::unique(
            sizes.begin(),
            sizes.end(),
            [](int lhs, int rhs) { return std::abs(lhs - rhs) < 6; }),
        sizes.end());
    if (frame_min <= 1000) {
        std::reverse(sizes.begin(), sizes.end());
    }
    return sizes;
}

void detectInPaddedRoi(
    const cv::Mat& gray,
    cv::aruco::ArucoDetector& detector,
    const cv::Rect& source_rect,
    int pad,
    VisionResult& result)
{
    if (source_rect.empty()) {
        return;
    }

    cv::Mat padded;
    cv::copyMakeBorder(
        gray(source_rect),
        padded,
        pad,
        pad,
        pad,
        pad,
        cv::BORDER_CONSTANT | cv::BORDER_ISOLATED,
        cv::Scalar(255));

    for (const double scale : {1.0, 2.0}) {
        cv::Mat detector_input = padded;
        if (scale > 1.0) {
            cv::resize(padded, detector_input, cv::Size(), scale, scale, cv::INTER_CUBIC);
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        detector.detectMarkers(detector_input, marker_corners, ids);

        for (std::size_t marker_index = 0; marker_index < ids.size(); ++marker_index) {
            if (marker_index >= marker_corners.size() || marker_corners[marker_index].size() != 4) {
                continue;
            }

            std::vector<cv::Point2f> source_corners;
            source_corners.reserve(4);
            for (const auto& corner : marker_corners[marker_index]) {
                source_corners.push_back({
                    static_cast<float>(corner.x / scale - pad + source_rect.x),
                    static_cast<float>(corner.y / scale - pad + source_rect.y),
                });
            }
            if (!markerShapeLooksPlausible(gray, source_corners)) {
                continue;
            }
            appendMarkerIfUnique(result, makeMarkerObservation(ids[marker_index], source_corners));
        }
    }
}

void appendRoiFallbackMarkers(
    const cv::Mat& image,
    const cv::Mat& gray,
    cv::aruco::ArucoDetector& detector,
    int max_components,
    int max_rois,
    VisionResult& result)
{
    const cv::Mat bright = brightComponentMask(image, gray);
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int components = cv::connectedComponentsWithStats(bright, labels, stats, centroids, 8);
    const int frame_min = std::max(1, std::min(image.cols, image.rows));
    const int min_area = std::max(24, (image.cols * image.rows) / 90000);
    const int max_area = std::max(600, (image.cols * image.rows) / 120);
    const int max_component_side = std::max(36, frame_min / 8);

    struct ComponentCandidate {
        int label = -1;
        double score = 0.0;
    };
    std::vector<ComponentCandidate> component_candidates;
    component_candidates.reserve(static_cast<std::size_t>(std::max(0, components - 1)));
    const cv::Point2d frame_center(image.cols / 2.0, image.rows / 2.0);
    const bool live_frame = isLiveFrameSize(image.size());
    for (int label = 1; label < components; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
        const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
        if (area < min_area || area > max_area ||
            width < 4 || height < 4 ||
            width > max_component_side || height > max_component_side) {
            continue;
        }

        const double ratio = width / static_cast<double>(std::max(1, height));
        if (ratio < 0.15 || ratio > 6.0) {
            continue;
        }
        const double cx = centroids.at<double>(label, 0);
        const double cy = centroids.at<double>(label, 1);
        const double center_distance = std::hypot(cx - frame_center.x, cy - frame_center.y);
        const double center_score = 1.0 - std::clamp(
            center_distance / std::max(1.0, std::hypot(frame_center.x, frame_center.y)),
            0.0,
            1.0);
        const double area_score = std::clamp(
            area / static_cast<double>(std::max(1, max_area)),
            0.0,
            1.0);
        component_candidates.push_back({label, live_frame ? (0.75 * center_score + 0.25 * area_score) : area_score});
    }

    std::sort(
        component_candidates.begin(),
        component_candidates.end(),
        [](const auto& lhs, const auto& rhs) { return lhs.score > rhs.score; });

    int processed_components = 0;
    int attempted_rois = 0;
    const int component_limit = live_frame && max_components > 0
        ? std::max(1, max_components)
        : static_cast<int>(component_candidates.size());
    const int roi_limit = live_frame && max_rois > 0 ? std::max(1, max_rois) : std::numeric_limits<int>::max();

    for (const auto& component : component_candidates) {
        if (processed_components >= component_limit || attempted_rois >= roi_limit) {
            break;
        }
        const int label = component.label;
        const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
        const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
        const double cx = centroids.at<double>(label, 0);
        const double cy = centroids.at<double>(label, 1);
        const auto sizes = fallbackRoiSizes(frame_min, width, height);
        ++processed_components;
        for (const int size : sizes) {
            const int shift = std::max(4, size / 4);
            const std::array<int, 5> offsets {-shift, -shift / 2, 0, shift / 2, shift};
            for (const int dx : offsets) {
                for (const int dy : offsets) {
                    if (attempted_rois >= roi_limit) {
                        return;
                    }
                    const int x = static_cast<int>(std::lround(cx + dx - size / 2.0));
                    const int y = static_cast<int>(std::lround(cy + dy - size / 2.0));
                    const cv::Rect source_rect =
                        cv::Rect(x, y, size, size) & cv::Rect(0, 0, image.cols, image.rows);
                    if (source_rect.width < size / 2 || source_rect.height < size / 2) {
                        continue;
                    }
                    if (darkRatioInRect(gray, source_rect) < 0.30) {
                        continue;
                    }
                    ++attempted_rois;
                    detectInPaddedRoi(
                        gray,
                        detector,
                        source_rect,
                        std::max(12, size / 2),
                        result);
                }
            }
        }
    }
}

void appendTargetedFallbackMarkers(
    const cv::Mat& image,
    const cv::Mat& gray,
    cv::aruco::ArucoDetector& detector,
    const std::vector<MarkerObservation>& seed_markers,
    int max_rois,
    VisionResult& result)
{
    int attempted_rois = 0;
    const int roi_limit = max_rois > 0 ? std::max(1, max_rois) : std::numeric_limits<int>::max();
    const int frame_min = std::max(1, std::min(image.cols, image.rows));
    for (const auto& seed : seed_markers) {
        const float side = markerApproxSidePx(seed);
        std::vector<int> sizes;
        for (const double scale : {1.8, 2.2, 2.8, 3.4}) {
            sizes.push_back(std::clamp(
                static_cast<int>(std::lround(side * scale)),
                std::max(44, static_cast<int>(std::lround(frame_min * 0.035))),
                std::min(280, static_cast<int>(std::lround(frame_min * 0.38)))));
        }
        for (const double ratio : {0.18, 0.24, 0.30}) {
            sizes.push_back(std::clamp(
                static_cast<int>(std::lround(frame_min * ratio)),
                44,
                std::min(280, static_cast<int>(std::lround(frame_min * 0.38)))));
        }
        std::sort(sizes.begin(), sizes.end());
        sizes.erase(std::unique(sizes.begin(), sizes.end()), sizes.end());
        if (frame_min <= 1000) {
            std::reverse(sizes.begin(), sizes.end());
        }

        for (const int size : sizes) {
            const int shift = std::max(4, size / 5);
            const std::array<int, 5> offsets {-shift, -shift / 2, 0, shift / 2, shift};
            for (const int dx : offsets) {
                for (const int dy : offsets) {
                    if (attempted_rois >= roi_limit) {
                        return;
                    }
                    const int x = static_cast<int>(std::lround(seed.center_px.x + dx - size / 2.0f));
                    const int y = static_cast<int>(std::lround(seed.center_px.y + dy - size / 2.0f));
                    const cv::Rect source_rect =
                        cv::Rect(x, y, size, size) & cv::Rect(0, 0, image.cols, image.rows);
                    if (source_rect.width < size / 2 || source_rect.height < size / 2) {
                        continue;
                    }
                    if (darkRatioInRect(gray, source_rect) < 0.30) {
                        continue;
                    }
                    ++attempted_rois;
                    detectInPaddedRoi(
                        gray,
                        detector,
                        source_rect,
                        std::max(12, size / 2),
                        result);
                }
            }
        }
    }
}

} // namespace

ArucoDetector::ArucoDetector(const common::ArucoConfig& config)
    : config_(config)
{
}

VisionResult ArucoDetector::detect(
    const cv::Mat& image,
    std::uint32_t frame_seq,
    std::int64_t timestamp_ms) const
{
    VisionResult result;
    result.frame_seq = frame_seq;
    result.timestamp_ms = timestamp_ms;
    result.width = image.cols;
    result.height = image.rows;

    if (image.empty()) {
        return result;
    }

    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    cv::aruco::DetectorParameters parameters;
    parameters.minMarkerPerimeterRate = config_.min_marker_perimeter_rate;
    parameters.maxMarkerPerimeterRate = config_.max_marker_perimeter_rate;
    parameters.adaptiveThreshWinSizeMin = config_.adaptive_thresh_win_size_min;
    parameters.adaptiveThreshWinSizeMax = config_.adaptive_thresh_win_size_max;
    parameters.adaptiveThreshWinSizeStep = config_.adaptive_thresh_win_size_step;

    const auto dictionary = cv::aruco::getPredefinedDictionary(dictionaryFromName(config_.dictionary));
    cv::aruco::ArucoDetector detector(dictionary, parameters);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    detector.detectMarkers(gray, marker_corners, ids);

    std::vector<MarkerObservation> partial_markers;
    appendDetectedMarkers(marker_corners, ids, image.size(), partial_markers, result);

    if (config_.roi_fallback_enabled && (!partial_markers.empty() || result.markers.empty())) {
        if (!partial_markers.empty()) {
            appendTargetedFallbackMarkers(
                image,
                gray,
                detector,
                partial_markers,
                config_.fallback_max_rois,
                result);
        }
        if (result.markers.empty() && shouldRunFullFallback(config_, frame_seq, image.size())) {
            const int fallback_max_rois = isLiveFrameSize(image.size())
                ? config_.full_fallback_max_rois
                : config_.fallback_max_rois;
            appendRoiFallbackMarkers(
                image,
                gray,
                detector,
                config_.fallback_max_components,
                fallback_max_rois,
                result);
        }
    }

    return result;
}

std::string ArucoDetector::dictionaryName() const
{
    return config_.dictionary;
}

} // namespace onboard::vision
