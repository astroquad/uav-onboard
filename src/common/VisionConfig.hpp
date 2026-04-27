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

struct VisionConfig {
    CameraConfig camera;
    VideoStreamConfig video;
    ArucoConfig aruco;
};

VisionConfig loadVisionConfig(const std::string& config_dir);

} // namespace onboard::common
