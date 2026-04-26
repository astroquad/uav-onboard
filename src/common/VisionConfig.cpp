#include "common/VisionConfig.hpp"

#include <toml++/toml.hpp>

namespace onboard::common {
namespace {

std::string joinConfigPath(const std::string& config_dir)
{
    if (config_dir.empty()) {
        return "config/vision.toml";
    }
    const char last = config_dir.back();
    if (last == '/' || last == '\\') {
        return config_dir + "vision.toml";
    }
    return config_dir + "/vision.toml";
}

} // namespace

VisionConfig loadVisionConfig(const std::string& config_dir)
{
    VisionConfig config;

    toml::table table;
    try {
        table = toml::parse_file(joinConfigPath(config_dir));
    } catch (const toml::parse_error&) {
        return config;
    }

    if (const auto camera = table["camera"]) {
        config.camera.device = camera["device"].value_or(config.camera.device);
        config.camera.width = camera["width"].value_or(config.camera.width);
        config.camera.height = camera["height"].value_or(config.camera.height);
        config.camera.fps = camera["fps"].value_or(config.camera.fps);
    }

    if (const auto video = table["video"]) {
        config.video.width = video["width"].value_or(config.video.width);
        config.video.height = video["height"].value_or(config.video.height);
        config.video.fps = video["fps"].value_or(config.video.fps);
        config.video.jpeg_quality = video["jpeg_quality"].value_or(config.video.jpeg_quality);
        config.video.port = video["port"].value_or(config.video.port);
    }

    return config;
}

} // namespace onboard::common
