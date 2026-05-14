#include "vision/GazeboCameraSource.hpp"

#include "common/Time.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <utility>

#if defined(ONBOARD_HAVE_GAZEBO_TRANSPORT)
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>
#endif

namespace onboard::vision {
namespace {

#if defined(ONBOARD_HAVE_GAZEBO_TRANSPORT)
bool convertImageMessage(const gz::msgs::Image& message, cv::Mat& image_bgr)
{
    const int width = static_cast<int>(message.width());
    const int height = static_cast<int>(message.height());
    if (width <= 0 || height <= 0 || message.data().empty()) {
        return false;
    }

    int channels = 0;
    int type = CV_8UC1;
    int conversion = -1;
    switch (message.pixel_format_type()) {
    case gz::msgs::RGB_INT8:
        channels = 3;
        type = CV_8UC3;
        conversion = cv::COLOR_RGB2BGR;
        break;
    case gz::msgs::BGR_INT8:
        channels = 3;
        type = CV_8UC3;
        break;
    case gz::msgs::RGBA_INT8:
        channels = 4;
        type = CV_8UC4;
        conversion = cv::COLOR_RGBA2BGR;
        break;
    case gz::msgs::BGRA_INT8:
        channels = 4;
        type = CV_8UC4;
        conversion = cv::COLOR_BGRA2BGR;
        break;
    case gz::msgs::L_INT8:
        channels = 1;
        type = CV_8UC1;
        conversion = cv::COLOR_GRAY2BGR;
        break;
    default:
        return false;
    }

    const std::size_t step = message.step() > 0
        ? static_cast<std::size_t>(message.step())
        : static_cast<std::size_t>(width * channels);
    if (message.data().size() < step * static_cast<std::size_t>(height)) {
        return false;
    }

    cv::Mat source(
        height,
        width,
        type,
        const_cast<char*>(message.data().data()),
        step);
    if (conversion >= 0) {
        cv::cvtColor(source, image_bgr, conversion);
    } else {
        image_bgr = source.clone();
    }
    return !image_bgr.empty();
}
#endif

} // namespace

struct GazeboCameraSource::Impl {
    FrameSourceOptions options;
    mutable std::mutex mutex;
    std::condition_variable condition;
    Frame latest;
    bool have_latest = false;
    bool open = false;
    std::string last_error;
    std::uint32_t next_frame_id = 1;

#if defined(ONBOARD_HAVE_GAZEBO_TRANSPORT)
    gz::transport::Node node;

    void onImage(const gz::msgs::Image& message)
    {
        cv::Mat image;
        if (!convertImageMessage(message, image)) {
            std::lock_guard<std::mutex> lock(mutex);
            last_error = "unsupported or invalid Gazebo camera image format";
            return;
        }

        Frame frame;
        frame.frame_id = next_frame_id++;
        frame.timestamp_ms = common::unixTimestampMs();
        frame.width = image.cols;
        frame.height = image.rows;
        frame.image_bgr = std::move(image);

        {
            std::lock_guard<std::mutex> lock(mutex);
            latest = std::move(frame);
            have_latest = true;
        }
        condition.notify_one();
    }
#endif
};

GazeboCameraSource::GazeboCameraSource()
    : impl_(std::make_unique<Impl>())
{
}

GazeboCameraSource::~GazeboCameraSource() = default;

bool GazeboCameraSource::open(const FrameSourceOptions& options)
{
    close();
    impl_->options = options;
    impl_->last_error.clear();
    impl_->have_latest = false;
    impl_->next_frame_id = 1;

#if defined(ONBOARD_HAVE_GAZEBO_TRANSPORT)
    const auto& topic = options.vision.source.gazebo_topic;
    if (topic.empty()) {
        impl_->last_error = "Gazebo camera topic is empty";
        return false;
    }
    if (!impl_->node.Subscribe(topic, &Impl::onImage, impl_.get())) {
        impl_->last_error = "failed to subscribe Gazebo camera topic: " + topic;
        return false;
    }
    impl_->open = true;
    return true;
#else
    (void)options;
    impl_->last_error = "Gazebo transport support was not available at build time";
    return false;
#endif
}

bool GazeboCameraSource::read(Frame& frame)
{
    std::unique_lock<std::mutex> lock(impl_->mutex);
    if (!impl_->open) {
        impl_->last_error = "Gazebo camera source is not open";
        return false;
    }

    const auto timeout = std::chrono::milliseconds(
        std::max(1, impl_->options.vision.source.read_timeout_ms));
    if (!impl_->condition.wait_for(lock, timeout, [this]() { return impl_->have_latest; })) {
        impl_->last_error = "timed out waiting for Gazebo camera frame";
        return false;
    }

    frame = std::move(impl_->latest);
    impl_->latest = {};
    impl_->have_latest = false;
    return true;
}

void GazeboCameraSource::close()
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->open = false;
    impl_->have_latest = false;
}

std::string GazeboCameraSource::lastError() const
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->last_error;
}

} // namespace onboard::vision
