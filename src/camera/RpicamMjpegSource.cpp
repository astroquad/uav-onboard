#include "camera/RpicamMjpegSource.hpp"

#include "common/Time.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <sstream>

#ifdef _WIN32
#define popen _popen
#define pclose _pclose
#endif

namespace onboard::camera {
namespace {

constexpr const char* kRpicamStderrLog = "/tmp/astroquad_rpicam_vid.log";

std::string shellQuote(const std::string& value)
{
    std::string quoted = "'";
    for (const char ch : value) {
        if (ch == '\'') {
            quoted += "'\\''";
        } else {
            quoted += ch;
        }
    }
    quoted += "'";
    return quoted;
}

void appendStringOption(std::ostringstream& command, const char* name, const std::string& value)
{
    if (!value.empty()) {
        command << ' ' << name << ' ' << shellQuote(value);
    }
}

void appendDoubleOption(std::ostringstream& command, const char* name, double value)
{
    if (value != 0.0) {
        command << ' ' << name << ' ' << std::setprecision(6) << value;
    }
}

std::string buildCommand(const RpicamOptions& options)
{
    std::ostringstream command;
    command << "rpicam-vid"
            << " --verbose 0"
            << " -t 0"
            << " --nopreview"
            << " --camera " << options.camera_index
            << " --codec " << (options.codec.empty() ? "mjpeg" : options.codec)
            << " --quality " << options.jpeg_quality
            << " --width " << options.width
            << " --height " << options.height
            << " --framerate " << options.fps
            << " -o -";

    appendStringOption(command, "--autofocus-mode", options.autofocus_mode);
    appendStringOption(command, "--autofocus-range", options.autofocus_range);
    appendStringOption(command, "--autofocus-speed", options.autofocus_speed);
    appendStringOption(command, "--autofocus-window", options.autofocus_window);
    if (options.lens_position >= 0.0 &&
        (options.autofocus_mode.empty() || options.autofocus_mode == "manual")) {
        command << " --lens-position " << std::setprecision(6) << options.lens_position;
    }
    appendStringOption(command, "--exposure", options.exposure);
    if (options.shutter_us > 0) {
        command << " --shutter " << options.shutter_us;
    }
    appendDoubleOption(command, "--gain", options.gain);
    appendDoubleOption(command, "--ev", options.ev);
    appendStringOption(command, "--awb", options.awb);
    appendStringOption(command, "--awbgains", options.awbgains);
    appendStringOption(command, "--metering", options.metering);
    appendStringOption(command, "--denoise", options.denoise);
    appendDoubleOption(command, "--sharpness", options.sharpness);
    appendDoubleOption(command, "--contrast", options.contrast);
    appendDoubleOption(command, "--brightness", options.brightness);
    appendDoubleOption(command, "--saturation", options.saturation);
    appendStringOption(command, "--roi", options.roi);
    appendStringOption(command, "--tuning-file", options.tuning_file);
    if (options.hflip) {
        command << " --hflip";
    }
    if (options.vflip) {
        command << " --vflip";
    }
    if (options.rotation != 0) {
        command << " --rotation " << options.rotation;
    }
#ifndef _WIN32
    command << " 2>" << kRpicamStderrLog;
#endif
    return command.str();
}

const char* pipeReadMode()
{
#ifdef _WIN32
    return "rb";
#else
    return "r";
#endif
}

} // namespace

RpicamMjpegSource::~RpicamMjpegSource()
{
    close();
}

bool RpicamMjpegSource::open(const RpicamOptions& options)
{
    close();
    options_ = options;
    buffer_.clear();

    const std::string command = buildCommand(options);
    pipe_ = popen(command.c_str(), pipeReadMode());
    if (!pipe_) {
        last_error_ = "failed to start rpicam-vid: ";
        last_error_ += std::strerror(errno);
        return false;
    }
    return true;
}

bool RpicamMjpegSource::readFrame(CameraFrame& frame)
{
    if (!pipe_) {
        last_error_ = "rpicam source is not open";
        return false;
    }

    while (true) {
        if (extractFrame(frame)) {
            return true;
        }

        std::array<std::uint8_t, 4096> chunk {};
        const std::size_t read_count = std::fread(chunk.data(), 1, chunk.size(), pipe_);
        if (read_count == 0) {
            if (std::feof(pipe_)) {
                last_error_ = "rpicam-vid ended; check ";
                last_error_ += kRpicamStderrLog;
            } else {
                last_error_ = "failed to read rpicam-vid output; check ";
                last_error_ += kRpicamStderrLog;
            }
            return false;
        }
        buffer_.insert(buffer_.end(), chunk.begin(), chunk.begin() + static_cast<std::ptrdiff_t>(read_count));

        if (buffer_.size() > 4 * 1024 * 1024) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(buffer_.size() / 2));
        }
    }
}

void RpicamMjpegSource::close()
{
    if (pipe_) {
        pclose(pipe_);
        pipe_ = nullptr;
    }
}

std::string RpicamMjpegSource::lastError() const
{
    return last_error_;
}

bool RpicamMjpegSource::extractFrame(CameraFrame& frame)
{
    auto start = buffer_.end();
    if (buffer_.size() < 2) {
        return false;
    }

    for (std::size_t index = 0; index + 1 < buffer_.size(); ++index) {
        if (buffer_[index] == 0xff && buffer_[index + 1] == 0xd8) {
            start = buffer_.begin() + static_cast<std::ptrdiff_t>(index);
            break;
        }
    }
    if (start == buffer_.end()) {
        if (buffer_.size() > 1) {
            buffer_.erase(buffer_.begin(), buffer_.end() - 1);
        }
        return false;
    }

    auto end = buffer_.end();
    const auto start_index = static_cast<std::size_t>(start - buffer_.begin());
    for (std::size_t index = start_index + 2; index + 1 < buffer_.size(); ++index) {
        if (buffer_[index] == 0xff && buffer_[index + 1] == 0xd9) {
            end = buffer_.begin() + static_cast<std::ptrdiff_t>(index + 2);
            break;
        }
    }
    if (end == buffer_.end()) {
        if (start != buffer_.begin()) {
            buffer_.erase(buffer_.begin(), start);
        }
        return false;
    }

    frame.frame_id = next_frame_id_++;
    frame.timestamp_ms = onboard::common::unixTimestampMs();
    frame.width = options_.width;
    frame.height = options_.height;
    frame.jpeg_data.assign(start, end);
    buffer_.erase(buffer_.begin(), end);
    last_error_.clear();
    return true;
}

} // namespace onboard::camera
