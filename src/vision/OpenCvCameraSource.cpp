#include "vision/OpenCvCameraSource.hpp"

#include <sstream>

namespace onboard::vision {

bool OpenCvCameraSource::open(const CameraOpenOptions& options)
{
    close();
    if (!capture_.open(options.device_index, cv::CAP_V4L2)) {
        if (!capture_.open(options.device_index)) {
            std::ostringstream error;
            error << "failed to open camera device " << options.device_index;
            last_error_ = error.str();
            return false;
        }
    }

    if (options.width > 0) {
        capture_.set(cv::CAP_PROP_FRAME_WIDTH, options.width);
    }
    if (options.height > 0) {
        capture_.set(cv::CAP_PROP_FRAME_HEIGHT, options.height);
    }
    if (options.fps > 0) {
        capture_.set(cv::CAP_PROP_FPS, options.fps);
    }

    return true;
}

bool OpenCvCameraSource::read(cv::Mat& frame)
{
    if (!capture_.isOpened()) {
        last_error_ = "camera is not open";
        return false;
    }
    if (!capture_.read(frame) || frame.empty()) {
        last_error_ = "failed to read a non-empty camera frame";
        return false;
    }
    return true;
}

void OpenCvCameraSource::close()
{
    if (capture_.isOpened()) {
        capture_.release();
    }
}

bool OpenCvCameraSource::isOpen() const
{
    return capture_.isOpened();
}

std::string OpenCvCameraSource::lastError() const
{
    return last_error_;
}

} // namespace onboard::vision
