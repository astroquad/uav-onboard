#include "vision/FakeFrameSource.hpp"

#include "common/Time.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace onboard::vision {

bool FakeFrameSource::open(const FrameSourceOptions& options)
{
    config_ = options.vision;
    next_frame_id_ = 1;
    open_ = true;
    last_error_.clear();
    return true;
}

bool FakeFrameSource::read(Frame& frame)
{
    if (!open_) {
        last_error_ = "fake frame source is not open";
        return false;
    }

    const int width = std::max(1, config_.camera.width);
    const int height = std::max(1, config_.camera.height);
    frame = {};
    frame.frame_id = next_frame_id_++;
    frame.timestamp_ms = common::unixTimestampMs();
    frame.width = width;
    frame.height = height;
    frame.image_bgr = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

    const double angle_rad = config_.source.fake_line_angle_deg * 3.14159265358979323846 / 180.0;
    const double center_x = width * 0.5 + config_.source.fake_line_offset_px;
    const double dx = std::tan(angle_rad) * height * 0.5;
    const int line_width = std::max(12, width / 24);
    cv::line(
        frame.image_bgr,
        cv::Point(static_cast<int>(center_x - dx), 0),
        cv::Point(static_cast<int>(center_x + dx), height - 1),
        cv::Scalar(255, 255, 255),
        line_width,
        cv::LINE_8);
    return true;
}

void FakeFrameSource::close()
{
    open_ = false;
}

std::string FakeFrameSource::lastError() const
{
    return last_error_;
}

} // namespace onboard::vision
