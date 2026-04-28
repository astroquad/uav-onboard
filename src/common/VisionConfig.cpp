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
        config.video.send_fps = video["send_fps"].value_or(config.video.send_fps);
        config.video.chunk_pacing_us = video["chunk_pacing_us"].value_or(config.video.chunk_pacing_us);
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
        config.line.mask_strategy = line["mask_strategy"].value_or(config.line.mask_strategy);
        config.line.offset_normalized = line["offset_normalized"].value_or(config.line.offset_normalized);
        config.line.roi_top_ratio = line["roi_top_ratio"].value_or(config.line.roi_top_ratio);
        config.line.lookahead_y_ratio = line["lookahead_y_ratio"].value_or(config.line.lookahead_y_ratio);
        config.line.lookahead_band_ratio =
            line["lookahead_band_ratio"].value_or(config.line.lookahead_band_ratio);
        config.line.threshold = line["threshold"].value_or(config.line.threshold);
        config.line.local_contrast_blur =
            line["local_contrast_blur"].value_or(config.line.local_contrast_blur);
        config.line.local_contrast_threshold =
            line["local_contrast_threshold"].value_or(config.line.local_contrast_threshold);
        config.line.min_area_px = line["min_area_px"].value_or(config.line.min_area_px);
        config.line.morph_kernel = line["morph_kernel"].value_or(config.line.morph_kernel);
        config.line.morph_open_kernel = line["morph_open_kernel"].value_or(config.line.morph_open_kernel);
        config.line.morph_close_kernel = line["morph_close_kernel"].value_or(config.line.morph_close_kernel);
        config.line.morph_dilate_kernel = line["morph_dilate_kernel"].value_or(config.line.morph_dilate_kernel);
        config.line.line_run_merge_gap_px =
            line["line_run_merge_gap_px"].value_or(config.line.line_run_merge_gap_px);
        config.line.max_contour_points = line["max_contour_points"].value_or(config.line.max_contour_points);
        config.line.confidence_min = line["confidence_min"].value_or(config.line.confidence_min);
        config.line.min_line_width_px = line["min_line_width_px"].value_or(config.line.min_line_width_px);
        config.line.max_line_width_ratio = line["max_line_width_ratio"].value_or(config.line.max_line_width_ratio);
        config.line.process_width = line["process_width"].value_or(config.line.process_width);
        config.line.max_candidates = line["max_candidates"].value_or(config.line.max_candidates);
        config.line.filter_enabled = line["filter_enabled"].value_or(config.line.filter_enabled);
        config.line.filter_ema_alpha = line["filter_ema_alpha"].value_or(config.line.filter_ema_alpha);
        config.line.filter_confidence_alpha_min =
            line["filter_confidence_alpha_min"].value_or(config.line.filter_confidence_alpha_min);
        config.line.filter_min_confidence =
            line["filter_min_confidence"].value_or(config.line.filter_min_confidence);
        config.line.filter_max_offset_jump_ratio =
            line["filter_max_offset_jump_ratio"].value_or(config.line.filter_max_offset_jump_ratio);
        config.line.filter_max_offset_velocity_ratio =
            line["filter_max_offset_velocity_ratio"].value_or(config.line.filter_max_offset_velocity_ratio);
        config.line.filter_max_angle_jump_deg =
            line["filter_max_angle_jump_deg"].value_or(config.line.filter_max_angle_jump_deg);
        config.line.filter_hold_frames = line["filter_hold_frames"].value_or(config.line.filter_hold_frames);
        config.line.filter_reacquire_frames =
            line["filter_reacquire_frames"].value_or(config.line.filter_reacquire_frames);
        config.line.intersection_threshold =
            line["intersection_threshold"].value_or(config.line.intersection_threshold);
    }

    return config;
}

} // namespace onboard::common
