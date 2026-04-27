#include "vision/ArucoDetector.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <map>
#include <numeric>

namespace onboard::vision {
namespace {

cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryFromName(const std::string& name)
{
    static const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dictionaries = {
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

    for (std::size_t marker_index = 0; marker_index < ids.size(); ++marker_index) {
        if (marker_index >= marker_corners.size() || marker_corners[marker_index].size() != 4) {
            continue;
        }

        MarkerObservation marker;
        marker.id = ids[marker_index];
        for (std::size_t corner_index = 0; corner_index < marker.corners_px.size(); ++corner_index) {
            marker.corners_px[corner_index] = toPoint(marker_corners[marker_index][corner_index]);
        }

        for (const auto& corner : marker.corners_px) {
            marker.center_px.x += corner.x;
            marker.center_px.y += corner.y;
        }
        marker.center_px.x /= static_cast<float>(marker.corners_px.size());
        marker.center_px.y /= static_cast<float>(marker.corners_px.size());
        marker.orientation_deg = orientationDeg(marker.corners_px);
        result.markers.push_back(marker);
    }

    return result;
}

std::string ArucoDetector::dictionaryName() const
{
    return config_.dictionary;
}

} // namespace onboard::vision
