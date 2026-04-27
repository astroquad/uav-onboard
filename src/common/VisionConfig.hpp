#pragma once

#include <string>

namespace onboard::common {

struct CameraConfig {
    int device = 0;
    int width = 640;
    int height = 480;
    int fps = 30;
};

struct VideoStreamConfig {
    int width = 640;
    int height = 480;
    int fps = 15;
    int jpeg_quality = 50;
    int port = 5600;
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
    std::string mode = "dark_on_light";
    bool offset_normalized = false;
    double roi_top_ratio = 0.35;
    double lookahead_y_ratio = 0.70;
    int threshold = 90;
    int min_area_px = 250;
    int morph_kernel = 5;
    int max_contour_points = 80;
    double confidence_min = 0.05;
    double intersection_threshold = 0.8;
};

struct VisionConfig {
    CameraConfig camera;
    VideoStreamConfig video;
    ArucoConfig aruco;
    LineConfig line;
};

VisionConfig loadVisionConfig(const std::string& config_dir);

} // namespace onboard::common
