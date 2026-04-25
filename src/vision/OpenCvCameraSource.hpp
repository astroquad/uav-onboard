#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <string>

namespace onboard::vision {

struct CameraOpenOptions {
    int device_index = 0;
    int width = 640;
    int height = 480;
    int fps = 30;
};

class OpenCvCameraSource {
public:
    bool open(const CameraOpenOptions& options);
    bool read(cv::Mat& frame);
    void close();
    bool isOpen() const;
    std::string lastError() const;

private:
    cv::VideoCapture capture_;
    mutable std::string last_error_;
};

} // namespace onboard::vision
