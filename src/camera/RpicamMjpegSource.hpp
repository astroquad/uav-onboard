#pragma once

#include "camera/CameraFrame.hpp"

#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

namespace onboard::camera {

struct RpicamOptions {
    int width = 640;
    int height = 480;
    int fps = 15;
    int jpeg_quality = 50;
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
