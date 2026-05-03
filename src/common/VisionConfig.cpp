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
        config.camera.device = camera["index"].value_or(config.camera.device);
        config.camera.width = camera["width"].value_or(config.camera.width);
        config.camera.height = camera["height"].value_or(config.camera.height);
        config.camera.fps = camera["fps"].value_or(config.camera.fps);
        config.camera.jpeg_quality = camera["jpeg_quality"].value_or(config.camera.jpeg_quality);
        config.camera.sensor_model = camera["sensor_model"].value_or(config.camera.sensor_model);
        config.camera.codec = camera["codec"].value_or(config.camera.codec);
        config.camera.autofocus_mode =
            camera["autofocus_mode"].value_or(config.camera.autofocus_mode);
        config.camera.autofocus_range =
            camera["autofocus_range"].value_or(config.camera.autofocus_range);
        config.camera.autofocus_speed =
            camera["autofocus_speed"].value_or(config.camera.autofocus_speed);
        config.camera.autofocus_window =
            camera["autofocus_window"].value_or(config.camera.autofocus_window);
        config.camera.lens_position =
            camera["lens_position"].value_or(config.camera.lens_position);
        config.camera.exposure = camera["exposure"].value_or(config.camera.exposure);
        config.camera.shutter_us = camera["shutter_us"].value_or(config.camera.shutter_us);
        config.camera.gain = camera["gain"].value_or(config.camera.gain);
        config.camera.ev = camera["ev"].value_or(config.camera.ev);
        config.camera.awb = camera["awb"].value_or(config.camera.awb);
        config.camera.awbgains = camera["awbgains"].value_or(config.camera.awbgains);
        config.camera.metering = camera["metering"].value_or(config.camera.metering);
        config.camera.denoise = camera["denoise"].value_or(config.camera.denoise);
        config.camera.sharpness = camera["sharpness"].value_or(config.camera.sharpness);
        config.camera.contrast = camera["contrast"].value_or(config.camera.contrast);
        config.camera.brightness = camera["brightness"].value_or(config.camera.brightness);
        config.camera.saturation = camera["saturation"].value_or(config.camera.saturation);
        config.camera.roi = camera["roi"].value_or(config.camera.roi);
        config.camera.tuning_file = camera["tuning_file"].value_or(config.camera.tuning_file);
        config.camera.hflip = camera["hflip"].value_or(config.camera.hflip);
        config.camera.vflip = camera["vflip"].value_or(config.camera.vflip);
        config.camera.rotation = camera["rotation"].value_or(config.camera.rotation);
    }

    if (const auto video = table["video"]) {
        config.video.width = video["width"].value_or(config.video.width);
        config.video.height = video["height"].value_or(config.video.height);
        config.video.fps = video["fps"].value_or(config.video.fps);
        config.video.jpeg_quality = video["jpeg_quality"].value_or(config.video.jpeg_quality);
        config.video.send_fps = video["send_fps"].value_or(config.video.send_fps);
        config.video.chunk_pacing_us = video["chunk_pacing_us"].value_or(config.video.chunk_pacing_us);
        config.video.port = video["port"].value_or(config.video.port);

        config.debug_video.send_fps = video["send_fps"].value_or(config.debug_video.send_fps);
        config.debug_video.jpeg_quality =
            video["jpeg_quality"].value_or(config.debug_video.jpeg_quality);
        config.debug_video.chunk_pacing_us =
            video["chunk_pacing_us"].value_or(config.debug_video.chunk_pacing_us);
    }

    if (const auto debug_video = table["debug_video"]) {
        config.debug_video.enabled =
            debug_video["enabled"].value_or(config.debug_video.enabled);
        config.debug_video.send_fps =
            debug_video["send_fps"].value_or(config.debug_video.send_fps);
        config.debug_video.jpeg_quality =
            debug_video["jpeg_quality"].value_or(config.debug_video.jpeg_quality);
        config.debug_video.chunk_pacing_us =
            debug_video["chunk_pacing_us"].value_or(config.debug_video.chunk_pacing_us);
        config.debug_video.send_width =
            debug_video["send_width"].value_or(config.debug_video.send_width);
        config.debug_video.send_height =
            debug_video["send_height"].value_or(config.debug_video.send_height);
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
        config.line.white_v_min = line["white_v_min"].value_or(config.line.white_v_min);
        config.line.white_s_max = line["white_s_max"].value_or(config.line.white_s_max);
        config.line.dark_v_max = line["dark_v_max"].value_or(config.line.dark_v_max);
        config.line.min_area_px = line["min_area_px"].value_or(config.line.min_area_px);
        config.line.morph_kernel = line["morph_kernel"].value_or(config.line.morph_kernel);
        config.line.morph_open_kernel = line["morph_open_kernel"].value_or(config.line.morph_open_kernel);
        config.line.morph_close_kernel = line["morph_close_kernel"].value_or(config.line.morph_close_kernel);
        config.line.morph_dilate_kernel = line["morph_dilate_kernel"].value_or(config.line.morph_dilate_kernel);
        config.line.fill_close_kernel = line["fill_close_kernel"].value_or(config.line.fill_close_kernel);
        config.line.fill_dilate_kernel = line["fill_dilate_kernel"].value_or(config.line.fill_dilate_kernel);
        config.line.dark_fill_close_kernel =
            line["dark_fill_close_kernel"].value_or(config.line.dark_fill_close_kernel);
        config.line.dark_fill_dilate_kernel =
            line["dark_fill_dilate_kernel"].value_or(config.line.dark_fill_dilate_kernel);
        config.line.line_run_merge_gap_px =
            line["line_run_merge_gap_px"].value_or(config.line.line_run_merge_gap_px);
        config.line.max_contour_points = line["max_contour_points"].value_or(config.line.max_contour_points);
        config.line.confidence_min = line["confidence_min"].value_or(config.line.confidence_min);
        config.line.min_line_width_px = line["min_line_width_px"].value_or(config.line.min_line_width_px);
        config.line.max_line_width_ratio = line["max_line_width_ratio"].value_or(config.line.max_line_width_ratio);
        config.line.dark_max_line_width_ratio =
            line["dark_max_line_width_ratio"].value_or(config.line.dark_max_line_width_ratio);
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

    if (const auto decision = table["intersection_decision"]) {
        config.intersection_decision.enabled =
            decision["enabled"].value_or(config.intersection_decision.enabled);
        config.intersection_decision.fps_assumption =
            decision["fps_assumption"].value_or(config.intersection_decision.fps_assumption);
        config.intersection_decision.cruise_window_frames =
            decision["cruise_window_frames"].value_or(config.intersection_decision.cruise_window_frames);
        config.intersection_decision.turn_confirm_frames =
            decision["turn_confirm_frames"].value_or(config.intersection_decision.turn_confirm_frames);
        config.intersection_decision.cooldown_frames =
            decision["cooldown_frames"].value_or(config.intersection_decision.cooldown_frames);
        config.intersection_decision.min_cross_branch_frames =
            decision["min_cross_branch_frames"].value_or(config.intersection_decision.min_cross_branch_frames);
        config.intersection_decision.min_t_branch_frames =
            decision["min_t_branch_frames"].value_or(config.intersection_decision.min_t_branch_frames);
        config.intersection_decision.min_l_branch_frames =
            decision["min_l_branch_frames"].value_or(config.intersection_decision.min_l_branch_frames);
        config.intersection_decision.min_branch_score =
            decision["min_branch_score"].value_or(config.intersection_decision.min_branch_score);
        config.intersection_decision.high_confidence_score =
            decision["high_confidence_score"].value_or(config.intersection_decision.high_confidence_score);
        config.intersection_decision.candidate_min_frames =
            decision["candidate_min_frames"].value_or(config.intersection_decision.candidate_min_frames);
        config.intersection_decision.turn_confirm_required =
            decision["turn_confirm_required"].value_or(config.intersection_decision.turn_confirm_required);
        config.intersection_decision.record_node_once_frames =
            decision["record_node_once_frames"].value_or(config.intersection_decision.record_node_once_frames);
        config.intersection_decision.turn_zone_y_min =
            decision["turn_zone_y_min"].value_or(config.intersection_decision.turn_zone_y_min);
        config.intersection_decision.turn_zone_y_max =
            decision["turn_zone_y_max"].value_or(config.intersection_decision.turn_zone_y_max);
        config.intersection_decision.late_zone_y =
            decision["late_zone_y"].value_or(config.intersection_decision.late_zone_y);
        config.intersection_decision.min_prearm_frames =
            decision["min_prearm_frames"].value_or(config.intersection_decision.min_prearm_frames);
        config.intersection_decision.front_missing_frames =
            decision["front_missing_frames"].value_or(config.intersection_decision.front_missing_frames);
        config.intersection_decision.node_advance_min_frames =
            decision["node_advance_min_frames"].value_or(config.intersection_decision.node_advance_min_frames);
    }

    return config;
}

} // namespace onboard::common
