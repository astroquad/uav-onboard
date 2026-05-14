#include "vision/RpicamFrameSource.hpp"

#include <opencv2/imgcodecs.hpp>

#include <utility>

namespace onboard::vision {
namespace {

camera::RpicamOptions toRpicamOptions(const common::VisionConfig& config)
{
    camera::RpicamOptions options;
    options.camera_index = config.camera.device;
    options.width = config.camera.width;
    options.height = config.camera.height;
    options.fps = config.camera.fps;
    options.jpeg_quality = config.camera.jpeg_quality;
    options.codec = config.camera.codec;
    options.autofocus_mode = config.camera.autofocus_mode;
    options.autofocus_range = config.camera.autofocus_range;
    options.autofocus_speed = config.camera.autofocus_speed;
    options.autofocus_window = config.camera.autofocus_window;
    options.lens_position = config.camera.lens_position;
    options.focus_absolute = config.camera.focus_absolute;
    options.focus_device = config.camera.focus_device;
    options.exposure = config.camera.exposure;
    options.shutter_us = config.camera.shutter_us;
    options.gain = config.camera.gain;
    options.ev = config.camera.ev;
    options.awb = config.camera.awb;
    options.awbgains = config.camera.awbgains;
    options.metering = config.camera.metering;
    options.denoise = config.camera.denoise;
    options.sharpness = config.camera.sharpness;
    options.contrast = config.camera.contrast;
    options.brightness = config.camera.brightness;
    options.saturation = config.camera.saturation;
    options.roi = config.camera.roi;
    options.tuning_file = config.camera.tuning_file;
    options.hflip = config.camera.hflip;
    options.vflip = config.camera.vflip;
    options.rotation = config.camera.rotation;
    return options;
}

} // namespace

bool RpicamFrameSource::open(const FrameSourceOptions& options)
{
    last_error_.clear();
    if (!camera_.open(toRpicamOptions(options.vision))) {
        last_error_ = camera_.lastError();
        return false;
    }
    return true;
}

bool RpicamFrameSource::read(Frame& frame)
{
    camera::CameraFrame camera_frame;
    if (!camera_.readFrame(camera_frame)) {
        last_error_ = camera_.lastError();
        return false;
    }

    const cv::Mat encoded(
        1,
        static_cast<int>(camera_frame.jpeg_data.size()),
        CV_8UC1,
        camera_frame.jpeg_data.data());
    cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (image.empty()) {
        last_error_ = "failed to decode rpicam MJPEG frame";
        return false;
    }

    frame = {};
    frame.frame_id = camera_frame.frame_id;
    frame.timestamp_ms = camera_frame.timestamp_ms;
    frame.width = camera_frame.width;
    frame.height = camera_frame.height;
    frame.jpeg_data = std::move(camera_frame.jpeg_data);
    frame.image_bgr = std::move(image);
    return true;
}

void RpicamFrameSource::close()
{
    camera_.close();
}

std::string RpicamFrameSource::lastError() const
{
    return last_error_;
}

} // namespace onboard::vision
