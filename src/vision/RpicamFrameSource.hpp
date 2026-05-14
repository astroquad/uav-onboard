#pragma once

#include "camera/RpicamMjpegSource.hpp"
#include "vision/FrameSource.hpp"

#include <string>

namespace onboard::vision {

class RpicamFrameSource final : public FrameSource {
public:
    bool open(const FrameSourceOptions& options) override;
    bool read(Frame& frame) override;
    void close() override;
    std::string lastError() const override;

private:
    camera::RpicamMjpegSource camera_;
    std::string last_error_;
};

} // namespace onboard::vision
