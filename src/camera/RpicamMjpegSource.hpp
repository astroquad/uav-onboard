#pragma once

#include "camera/CameraFrame.hpp"

#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

namespace onboard::camera {

struct RpicamOptions {
    int camera_index = 0;
    int width = 640;
    int height = 480;
    int fps = 15;
    int jpeg_quality = 50;
    std::string codec = "mjpeg";
    std::string autofocus_mode;
    std::string autofocus_range;
    std::string autofocus_speed;
    std::string autofocus_window;
    double lens_position = -1.0;
    std::string exposure;
    int shutter_us = 0;
    double gain = 0.0;
    double ev = 0.0;
    std::string awb;
    std::string awbgains;
    std::string metering;
    std::string denoise;
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

class RpicamMjpegSource {
public:
    RpicamMjpegSource() = default;
    ~RpicamMjpegSource();

    RpicamMjpegSource(const RpicamMjpegSource&) = delete;
    RpicamMjpegSource& operator=(const RpicamMjpegSource&) = delete;

    bool open(const RpicamOptions& options);
    bool readFrame(CameraFrame& frame);
    void close();
    std::string lastError() const;

private:
    bool extractFrame(CameraFrame& frame);

    FILE* pipe_ = nullptr;
    RpicamOptions options_;
    std::uint32_t next_frame_id_ = 1;
    std::vector<std::uint8_t> buffer_;
    std::string last_error_;
};

} // namespace onboard::camera
