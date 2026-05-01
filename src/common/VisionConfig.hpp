#pragma once

#include <string>

namespace onboard::common {

struct CameraConfig {
    int device = 0;
    int width = 960;
    int height = 720;
    int fps = 12;
    int jpeg_quality = 45;
    std::string sensor_model = "imx519";
    std::string codec = "mjpeg";
    std::string autofocus_mode = "manual";
    std::string autofocus_range = "normal";
    std::string autofocus_speed = "normal";
    std::string autofocus_window;
    double lens_position = 0.67;
    std::string exposure = "sport";
    int shutter_us = 0;
    double gain = 0.0;
    double ev = 0.0;
    std::string awb = "auto";
    std::string awbgains;
    std::string metering;
    std::string denoise = "cdn_fast";
    double sharpness = 0.0;
    double contrast = 0.0;
    double brightness = 0.0;
    double saturation = 0.0;
    std::string roi;
    std::string tuning_file;
    bool hflip = false;
    bool vflip = false;
    int rotation = 0;
};

struct VideoStreamConfig {
    int width = 640;
    int height = 480;
    int fps = 12;
    int jpeg_quality = 45;
    int send_fps = 5;
    int chunk_pacing_us = 150;
    int port = 5600;
};

struct DebugVideoConfig {
    bool enabled = false;
    int send_fps = 5;
    int jpeg_quality = 40;
    int chunk_pacing_us = 150;
    int send_width = 0;
    int send_height = 0;
};

struct ArucoConfig {
    std::string dictionary = "DICT_4X4_50";
    double marker_size_mm = 80.0;
    double min_marker_perimeter_rate = 0.03;
    double max_marker_perimeter_rate = 4.0;
    int adaptive_thresh_win_size_min = 3;
    int adaptive_thresh_win_size_max = 23;
    int adaptive_thresh_win_size_step = 10;
};

struct LineConfig {
    bool enabled = true;
    std::string mode = "light_on_dark";
    std::string mask_strategy = "white_fill";
    bool offset_normalized = false;
    double roi_top_ratio = 0.08;
    double lookahead_y_ratio = 0.55;
    double lookahead_band_ratio = 0.06;
    int threshold = 0;
    int local_contrast_blur = 31;
    int local_contrast_threshold = 10;
    int white_v_min = 145;
    int white_s_max = 90;
    int min_area_px = 250;
    int morph_kernel = 5;
    int morph_open_kernel = 1;
    int morph_close_kernel = 7;
    int morph_dilate_kernel = 1;
    int fill_close_kernel = 11;
    int fill_dilate_kernel = 3;
    int line_run_merge_gap_px = 16;
    int max_contour_points = 48;
    double confidence_min = 0.25;
    int min_line_width_px = 8;
    double max_line_width_ratio = 0.22;
    int process_width = 480;
    int max_candidates = 8;
    bool filter_enabled = true;
    double filter_ema_alpha = 0.35;
    double filter_confidence_alpha_min = 0.35;
    double filter_min_confidence = 0.25;
    double filter_max_offset_jump_ratio = 0.16;
    double filter_max_offset_velocity_ratio = 0.08;
    double filter_max_angle_jump_deg = 90.0;
    int filter_hold_frames = 3;
    int filter_reacquire_frames = 3;
    double intersection_threshold = 0.8;
};

struct IntersectionDecisionConfig {
    bool enabled = true;
    int fps_assumption = 12;
    int cruise_window_frames = 6;
    int turn_confirm_frames = 8;
    int cooldown_frames = 8;
    int min_cross_branch_frames = 2;
    int min_t_branch_frames = 2;
    int min_l_branch_frames = 3;
    double min_branch_score = 0.72;
    double high_confidence_score = 0.85;
    int candidate_min_frames = 2;
    int turn_confirm_required = 3;
    int record_node_once_frames = 18;
    double turn_zone_y_min = 0.42;
    double turn_zone_y_max = 0.68;
    double late_zone_y = 0.78;
    int min_prearm_frames = 2;
    int front_missing_frames = 2;
    int node_advance_min_frames = 4;
};

struct VisionConfig {
    CameraConfig camera;
    VideoStreamConfig video;
    DebugVideoConfig debug_video;
    ArucoConfig aruco;
    LineConfig line;
    IntersectionDecisionConfig intersection_decision;
};

VisionConfig loadVisionConfig(const std::string& config_dir);

} // namespace onboard::common
