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
    int jpeg_quality = 70;
    int port = 5600;
};

struct VisionConfig {
    CameraConfig camera;
    VideoStreamConfig video;
};

VisionConfig loadVisionConfig(const std::string& config_dir);

} // namespace onboard::common
