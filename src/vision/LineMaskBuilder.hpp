#pragma once

#include "common/VisionConfig.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace onboard::vision {

enum class LinePolarity {
    LightOnDark,
    DarkOnLight,
};

struct LineMaskGeometry {
    int source_width = 0;
    int source_height = 0;
    int roi_top = 0;
    int roi_height = 0;
    int work_width = 0;
    int work_height = 0;
    double scale_x = 1.0;
    double scale_y = 1.0;
};

struct LineMaskCandidate {
    cv::Mat mask;
    LinePolarity polarity = LinePolarity::LightOnDark;
};

struct LineMaskFrame {
    LineMaskGeometry geometry;
    std::vector<LineMaskCandidate> masks;
};

class LineMaskBuilder {
public:
    explicit LineMaskBuilder(const common::LineConfig& config);

    LineMaskFrame build(const cv::Mat& image) const;

private:
    common::LineConfig config_;
};

cv::Point toSourcePoint(const cv::Point& point, const LineMaskGeometry& geometry);
cv::Point toWorkPoint(float source_x, float source_y, const LineMaskGeometry& geometry);
int scaledLineKernelSize(int configured_kernel, const LineMaskGeometry& geometry);

} // namespace onboard::vision
