#include "app/LatestVideoSender.hpp"

#include "app/DebugVideoThrottle.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace onboard::app {

bool LatestVideoSender::start(
    const std::string& ip,
    std::uint16_t port,
    const common::DebugVideoConfig& config,
    int camera_fps)
{
    if (!streamer_.open(ip, port)) {
        last_error_ = streamer_.lastError();
        return false;
    }
    config_ = config;
    streamer_.setChunkPacingUs(config.chunk_pacing_us);
    // Spread each frame's chunks across 60% of the send period so shallow
    // or bufferbloated paths (LTE uplinks, DERP relays) see a smooth packet
    // stream instead of ~20-packet line-rate bursts they tail-drop.
    const int send_fps = effectiveDebugVideoFps(config.send_fps, camera_fps);
    streamer_.setFrameSpreadUs(600000 / std::max(1, send_fps));
    streamer_.setFecGroupSize(config.fec_group_size);
    running_ = true;
    worker_ = std::thread([this]() { workerLoop(); });
    return true;
}

void LatestVideoSender::submit(VideoSubmission submission)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_) {
        ++dropped_frames_;
    }
    latest_ = std::move(submission);
    condition_.notify_one();
}

void LatestVideoSender::noteSkippedFrame()
{
    skipped_frames_.fetch_add(1);
}

void LatestVideoSender::stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        latest_.reset();
    }
    condition_.notify_one();
    if (worker_.joinable()) {
        worker_.join();
    }
    streamer_.close();
}

std::string LatestVideoSender::takeLastError()
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::string output = std::move(last_error_);
    last_error_.clear();
    return output;
}

std::uint64_t LatestVideoSender::sentFrames() const
{
    return sent_frames_.load();
}

std::uint64_t LatestVideoSender::droppedFrames() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_frames_;
}

std::uint64_t LatestVideoSender::skippedFrames() const
{
    return skipped_frames_.load();
}

std::uint64_t LatestVideoSender::chunksSent() const
{
    return chunks_sent_.load();
}

std::uint64_t LatestVideoSender::sendFailures() const
{
    return send_failures_.load();
}

int LatestVideoSender::lastChunkCount() const
{
    return last_chunk_count_.load();
}

double LatestVideoSender::lastSendMs() const
{
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return last_send_ms_;
}

void LatestVideoSender::workerLoop()
{
    while (true) {
        std::optional<VideoSubmission> submission;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this]() { return !running_ || latest_.has_value(); });
            if (!running_ && !latest_) {
                return;
            }
            submission = std::move(latest_);
            latest_.reset();
        }

        if (!submission) {
            continue;
        }

        camera::CameraFrame wire;
        if (!prepareWireFrame(*submission, wire)) {
            send_failures_.fetch_add(1);
            continue;
        }

        const auto send_started = std::chrono::steady_clock::now();
        const bool sent = streamer_.sendFrame(wire);
        const auto send_finished = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            last_send_ms_ = std::chrono::duration<double, std::milli>(
                send_finished - send_started).count();
        }
        last_chunk_count_.store(streamer_.lastChunkCount());
        if (sent) {
            ++sent_frames_;
            chunks_sent_.fetch_add(static_cast<std::uint64_t>(
                std::max(0, streamer_.lastChunkCount())));
            continue;
        }

        send_failures_.fetch_add(1);
        setError(streamer_.lastError());
    }
}

bool LatestVideoSender::prepareWireFrame(
    VideoSubmission& submission,
    camera::CameraFrame& wire)
{
    wire.frame_id = submission.frame.frame_id;
    wire.timestamp_ms = submission.frame.timestamp_ms;
    wire.width = submission.frame.width;
    wire.height = submission.frame.height;

    const bool want_resize = config_.send_width > 0 || config_.send_height > 0;
    if (!want_resize) {
        if (!submission.frame.jpeg_data.empty()) {
            wire.jpeg_data = std::move(submission.frame.jpeg_data);
            return true;
        }
        if (submission.image_bgr.empty()) {
            setError("video frame has neither JPEG data nor a decoded image");
            return false;
        }
        return encodeJpeg(submission.image_bgr, wire);
    }

    cv::Mat image = submission.image_bgr;
    if (image.empty()) {
        if (submission.frame.jpeg_data.empty()) {
            setError("video frame has neither JPEG data nor a decoded image");
            return false;
        }
        image = cv::imdecode(submission.frame.jpeg_data, cv::IMREAD_UNCHANGED);
        if (image.empty()) {
            setError("failed to decode JPEG for debug video downscale");
            return false;
        }
    }

    int target_width = config_.send_width;
    int target_height = config_.send_height;
    if (target_width <= 0) {
        target_width = static_cast<int>(std::lround(
            image.cols * static_cast<double>(target_height) / image.rows));
    }
    if (target_height <= 0) {
        target_height = static_cast<int>(std::lround(
            image.rows * static_cast<double>(target_width) / image.cols));
    }
    target_width = std::max(1, target_width);
    target_height = std::max(1, target_height);

    // Downscale only; when the source is already at or below the target,
    // forward the original camera JPEG untouched.
    if (target_width >= image.cols && target_height >= image.rows) {
        if (!submission.frame.jpeg_data.empty()) {
            wire.jpeg_data = std::move(submission.frame.jpeg_data);
            return true;
        }
        return encodeJpeg(image, wire);
    }

    cv::Mat resized;
    cv::resize(
        image,
        resized,
        cv::Size(target_width, target_height),
        0.0,
        0.0,
        cv::INTER_AREA);
    return encodeJpeg(resized, wire);
}

bool LatestVideoSender::encodeJpeg(const cv::Mat& image, camera::CameraFrame& wire)
{
    const std::vector<int> params {
        cv::IMWRITE_JPEG_QUALITY,
        std::clamp(config_.jpeg_quality, 1, 100),
    };
    std::vector<unsigned char> encoded;
    if (!cv::imencode(".jpg", image, encoded, params)) {
        setError("failed to JPEG-encode debug video frame");
        return false;
    }
    wire.width = image.cols;
    wire.height = image.rows;
    wire.jpeg_data.assign(encoded.begin(), encoded.end());
    return true;
}

void LatestVideoSender::setError(const std::string& error)
{
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = error;
}

} // namespace onboard::app
