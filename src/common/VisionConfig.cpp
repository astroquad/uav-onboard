#include "common/VisionConfig.hpp"

#include <toml++/toml.hpp>

namespace onboard::common {
namespace {

std::string joinConfigPath(const std::string& config_dir)
{
    if (config_dir.empty()) {
        return "config/vision.toml";
    }
    const char last = config_dir.back();
    if (last == '/' || last == '\\') {
        return config_dir + "vision.toml";
    }
    return config_dir + "/vision.toml";
}

} // namespace

VisionConfig loadVisionConfig(const std::string& config_dir)
{
    VisionConfig config;

    toml::table table;
    try {
        table = toml::parse_file(joinConfigPath(config_dir));
    } catch (const toml::parse_error&) {
        return config;
    }

    if (const auto camera = table["camera"]) {
        config.camera.device = camera["device"].value_or(config.camera.device);
        config.camera.width = camera["width"].value_or(config.camera.width);
        config.camera.height = camera["height"].value_or(config.camera.height);
        config.camera.fps = camera["fps"].value_or(config.camera.fps);
    }

    if (const auto video = table["video"]) {
        config.video.width = video["width"].value_or(config.video.width);
        config.video.height = video["height"].value_or(config.video.height);
        config.video.fps = video["fps"].value_or(config.video.fps);
        config.video.jpeg_quality = video["jpeg_quality"].value_or(config.video.jpeg_quality);
        config.video.port = video["port"].value_or(config.video.port);
    }

    if (const auto aruco = table["aruco"]) {
        config.aruco.dictionary = aruco["dictionary"].value_or(config.aruco.dictionary);
        config.aruco.marker_size_mm = aruco["marker_size_mm"].value_or(config.aruco.marker_size_mm);
        config.aruco.min_marker_perimeter_rate =
            aruco["min_marker_perimeter_rate"].value_or(config.aruco.min_marker_perimeter_rate);
        config.aruco.max_marker_perimeter_rate =
            aruco["max_marker_perimeter_rate"].value_or(config.aruco.max_marker_perimeter_rate);
        config.aruco.adaptive_thresh_win_size_min =
            aruco["adaptive_thresh_win_size_min"].value_or(config.aruco.adaptive_thresh_win_size_min);
        config.aruco.adaptive_thresh_win_size_max =
            aruco["adaptive_thresh_win_size_max"].value_or(config.aruco.adaptive_thresh_win_size_max);
        config.aruco.adaptive_thresh_win_size_step =
            aruco["adaptive_thresh_win_size_step"].value_or(config.aruco.adaptive_thresh_win_size_step);
    }

    if (const auto line = table["line"]) {
        config.line.enabled = line["enabled"].value_or(config.line.enabled);
        config.line.mode = line["mode"].value_or(config.line.mode);
        config.line.offset_normalized = line["offset_normalized"].value_or(config.line.offset_normalized);
        config.line.roi_top_ratio = line["roi_top_ratio"].value_or(config.line.roi_top_ratio);
        config.line.lookahead_y_ratio = line["lookahead_y_ratio"].value_or(config.line.lookahead_y_ratio);
        config.line.threshold = line["threshold"].value_or(config.line.threshold);
        config.line.min_area_px = line["min_area_px"].value_or(config.line.min_area_px);
        config.line.morph_kernel = line["morph_kernel"].value_or(config.line.morph_kernel);
        config.line.max_contour_points = line["max_contour_points"].value_or(config.line.max_contour_points);
        config.line.confidence_min = line["confidence_min"].value_or(config.line.confidence_min);
        config.line.min_line_width_px = line["min_line_width_px"].value_or(config.line.min_line_width_px);
        config.line.max_line_width_ratio = line["max_line_width_ratio"].value_or(config.line.max_line_width_ratio);
        config.line.intersection_threshold =
            line["intersection_threshold"].value_or(config.line.intersection_threshold);
    }

    return config;
}

} // namespace onboard::common
