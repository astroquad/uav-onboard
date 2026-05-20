#pragma once

#include <optional>

namespace onboard::mission {

struct AltitudePolicyConfig {
    double vertiport_altitude_m = 1.3;
    double cruise_altitude_m = 2.0;
    double ramp_rate_mps = 0.4;
    // Trigger thresholds for off-pad detection.
    double off_pad_rangefinder_min_m = 1.8;
    double off_pad_rangefinder_hold_s = 1.0;
    double off_pad_local_distance_m = 1.5;
};

enum class AltitudePhase {
    OnPad,           // 1.3m hold while on the vertiport.
    Ramping,         // Transitioning 1.3 -> 2.0m.
    Cruise,          // 2.0m hold over grid.
};

struct AltitudePolicyInput {
    double now_s = 0.0;
    std::optional<double> rangefinder_m;
    double distance_from_pad_m = 0.0;
    bool off_pad_requested = false;  // set by mission when OFF_PAD_FORWARD starts
};

struct AltitudePolicyOutput {
    AltitudePhase phase = AltitudePhase::OnPad;
    double target_altitude_m = 1.3;
    bool off_pad_confirmed = false;  // true once we believe we are off the vertiport surface
};

class AltitudePolicy {
public:
    explicit AltitudePolicy(AltitudePolicyConfig config);

    AltitudePolicyOutput update(const AltitudePolicyInput& input);
    void reset();

    AltitudePhase phase() const { return phase_; }
    double currentSetpointM() const { return current_setpoint_m_; }

private:
    AltitudePolicyConfig config_;
    AltitudePhase phase_ = AltitudePhase::OnPad;
    double current_setpoint_m_ = 1.3;
    double off_pad_hold_start_s_ = -1.0;
    bool off_pad_confirmed_ = false;
};

} // namespace onboard::mission
