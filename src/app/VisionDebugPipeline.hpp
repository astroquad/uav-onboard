#pragma once

#include "common/NetworkConfig.hpp"
#include "common/VisionConfig.hpp"

namespace onboard::app {

struct VisionDebugPipelineOptions {
    common::NetworkConfig network;
    common::VisionConfig vision;
    int count = 0;
    bool send_video = true;
    bool send_telemetry = true;
    bool enable_aruco = true;
    bool enable_line = true;
};

class VisionDebugPipeline {
public:
    int run(const VisionDebugPipelineOptions& options);
};

} // namespace onboard::app
