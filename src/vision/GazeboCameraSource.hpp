#pragma once

#include "vision/FrameSource.hpp"

#include <memory>
#include <string>

namespace onboard::vision {

class GazeboCameraSource final : public FrameSource {
public:
    GazeboCameraSource();
    ~GazeboCameraSource() override;

    GazeboCameraSource(const GazeboCameraSource&) = delete;
    GazeboCameraSource& operator=(const GazeboCameraSource&) = delete;

    bool open(const FrameSourceOptions& options) override;
    bool read(Frame& frame) override;
    void close() override;
    std::string lastError() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace onboard::vision
