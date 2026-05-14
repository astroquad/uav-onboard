#pragma once

#include "vision/FrameSource.hpp"

#include <string>

namespace onboard::vision {

class FakeFrameSource final : public FrameSource {
public:
    bool open(const FrameSourceOptions& options) override;
    bool read(Frame& frame) override;
    void close() override;
    std::string lastError() const override;

private:
    common::VisionConfig config_;
    std::uint32_t next_frame_id_ = 1;
    bool open_ = false;
    std::string last_error_;
};

} // namespace onboard::vision
