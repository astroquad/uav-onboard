#pragma once

#include "autopilot/AutopilotState.hpp"
#include "autopilot/MavlinkTransport.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace onboard::autopilot {

class AutopilotMavlinkAdapter {
public:
    AutopilotMavlinkAdapter(std::unique_ptr<MavlinkTransport> transport, MavlinkIds ids);

    const AutopilotState& state() const { return state_; }
    const std::string& transportName() const { return transport_->name(); }

    void waitHeartbeat(std::chrono::seconds timeout);
    void requestDefaultStreams();
    void setGuidedMode(std::chrono::seconds timeout);
    void setLandMode(std::chrono::seconds timeout);
    void arm(std::chrono::seconds timeout);
    void disarm(std::chrono::seconds timeout);
    void takeoff(double target_altitude_m);
    void sendBodyVelocity(const BodyVelocityCommand& command);
    void sendLocalNedPositionTarget(const LocalNedPositionTargetCommand& command);
    bool waitAltitudeReached(double target_altitude_m, double ratio, std::chrono::seconds timeout);
    bool waitDisarmed(std::chrono::seconds timeout);
    bool poll(int timeout_ms);
    std::optional<double> bestAltitudeM() const;

private:
    void sendCommandLong(
        std::uint16_t command,
        float p1 = 0.0f,
        float p2 = 0.0f,
        float p3 = 0.0f,
        float p4 = 0.0f,
        float p5 = 0.0f,
        float p6 = 0.0f,
        float p7 = 0.0f);
    void requestMessageInterval(std::uint32_t message_id, double rate_hz);
    void setMode(std::uint32_t mode, const std::string& name, std::chrono::seconds timeout);
    void processMessage(const mavlink_message_t& message);

    std::unique_ptr<MavlinkTransport> transport_;
    MavlinkIds ids_;
    AutopilotState state_;
};

} // namespace onboard::autopilot
