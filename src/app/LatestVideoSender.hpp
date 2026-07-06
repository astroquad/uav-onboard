#pragma once

#include "camera/CameraFrame.hpp"
#include "common/VisionConfig.hpp"
#include "video/UdpMjpegStreamer.hpp"

#include <opencv2/core.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace onboard::app {

// One debug-video frame handed to the sender. jpeg_data (pre-encoded camera
// frame) and image_bgr (decoded image, BGR or grayscale) are both optional,
// but at least one must be set. The submitted Mat must not be mutated after
// submit(); every frame source allocates a fresh Mat per frame.
struct VideoSubmission {
    camera::CameraFrame frame;
    cv::Mat image_bgr;
};

// Best-effort latest-frame-wins UDP MJPEG sender. submit() only swaps the
// pending frame under a mutex; all heavy work (optional downscale via
// debug_video.send_width/send_height, JPEG encode, chunked sendto with
// pacing) runs on a worker thread so the vision/mission loop never blocks
// on the GCS link.
class LatestVideoSender {
public:
    bool start(
        const std::string& ip,
        std::uint16_t port,
        const common::DebugVideoConfig& config);
    void submit(VideoSubmission submission);
    void noteSkippedFrame();
    void stop();

    std::string takeLastError();
    std::uint64_t sentFrames() const;
    std::uint64_t droppedFrames() const;
    std::uint64_t skippedFrames() const;
    std::uint64_t chunksSent() const;
    std::uint64_t sendFailures() const;
    int lastChunkCount() const;
    double lastSendMs() const;

private:
    void workerLoop();
    // Applies the configured downscale/re-encode; returns false (with
    // last_error_ set) when the submission cannot produce a JPEG.
    bool prepareWireFrame(VideoSubmission& submission, camera::CameraFrame& wire);
    bool encodeJpeg(const cv::Mat& image, camera::CameraFrame& wire);
    void setError(const std::string& error);

    video::UdpMjpegStreamer streamer_;
    common::DebugVideoConfig config_;
    std::atomic<bool> running_ {false};
    std::thread worker_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::optional<VideoSubmission> latest_;
    std::string last_error_;
    std::uint64_t dropped_frames_ = 0;
    std::atomic<std::uint64_t> sent_frames_ {0};
    std::atomic<std::uint64_t> skipped_frames_ {0};
    std::atomic<std::uint64_t> chunks_sent_ {0};
    std::atomic<std::uint64_t> send_failures_ {0};
    std::atomic<int> last_chunk_count_ {0};
    mutable std::mutex stats_mutex_;
    double last_send_ms_ = 0.0;
};

} // namespace onboard::app
